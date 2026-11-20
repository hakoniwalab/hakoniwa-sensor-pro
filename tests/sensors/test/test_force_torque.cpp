// Force-torque adapter without MuJoCo (U1, U2, U3): timing, every error stage measured
// against independently computed expectations, tare, flags, seeds, reset and the config.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "hakoniwa/mujoco_sensors/force_torque_sensor_adapter.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStep = 1'000'000;  // 1 ms
constexpr double kPi = 3.14159265358979323846;

std::string write_config(const std::string& name, const std::string& body)
{
    const std::string path = ::testing::TempDir() + "/ft_" + name + ".json";
    std::ofstream(path) << body;
    return path;
}

// Range 100 N / 200 N / 5 N m / 10 N m and the given extra keys.
std::string config(const std::string& name, const std::string& extra, double rate = 500)
{
    return write_config(name, "{\"type\":\"force_torque\",\"update_rate\":" + std::to_string(rate) +
                                  ",\"range\":{\"fxy\":100,\"fz\":200,\"txy\":5,\"tz\":10}" +
                                  (extra.empty() ? "" : "," + extra) + "}");
}

std::unique_ptr<sa::IForceTorqueSensorAdapter> make(const std::string& path, std::uint64_t seed = 7,
                                                    std::uint64_t step = kStep, const char* sensor = "ft")
{
    sa::ForceTorqueAdapterSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = step;
    setup.vehicle_name = "arm";
    setup.sensor_name = sensor;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    auto adapter = ms::create_force_torque_sensor_adapter(setup, error);
    EXPECT_TRUE(adapter) << error;
    return adapter;
}

std::string make_error(const std::string& path)
{
    sa::ForceTorqueAdapterSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStep;
    std::string error;
    auto adapter = ms::create_force_torque_sensor_adapter(setup, error);
    EXPECT_FALSE(adapter) << path;
    return error;
}

double axis(const sa::Wrench& w, int i)
{
    const double v[6] = {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z};
    return v[i];
}

sa::Wrench wrench(std::array<double, 6> a)
{
    sa::Wrench w;
    w.force = {a[0], a[1], a[2]};
    w.torque = {a[3], a[4], a[5]};
    return w;
}

// Runs n steps with truth(k) and collects the sample of every step.
std::vector<sa::ForceTorqueSample> run(sa::IForceTorqueSensorAdapter& a, std::uint64_t n,
                                       const std::function<sa::Wrench(std::uint64_t)>& truth,
                                       std::uint64_t first = 0)
{
    std::vector<sa::ForceTorqueSample> out;
    std::string error;
    for (std::uint64_t k = first; k < first + n; ++k) {
        sa::ForceTorqueTruth t;
        t.sim_time_ns = k * kStep;
        t.wrench = truth(k);
        sa::ForceTorqueSample s;
        EXPECT_TRUE(a.step(t, s, error)) << error;
        out.push_back(s);
    }
    return out;
}

const double kRange[6] = {100, 100, 200, 5, 5, 10};

double stddev(const std::vector<double>& v)
{
    double m = 0;
    for (double x : v) m += x;
    m /= static_cast<double>(v.size());
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

}  // namespace

// T-FTS-U2-010 [FTS-ADP-030] [FTS-ADP-300] Ideal: the truth on the period boundaries, held between
TEST(ForceTorque, IdealPassesTruthOnPeriod)
{
    auto a = make(config("ideal", ""));
    const auto s = run(*a, 10, [](std::uint64_t k) { return wrench({1.0 * k, 2, 3, 0.1, 0.2, 0.3}); });
    for (std::uint64_t k = 0; k < 10; ++k) {
        EXPECT_EQ(s[k].updated, k % 2 == 0) << k;  // 500 Hz on 1 ms steps
        const std::uint64_t measured = k - k % 2;
        EXPECT_EQ(s[k].sample_time_ns, measured * kStep);
        EXPECT_DOUBLE_EQ(s[k].wrench.force.x, 1.0 * measured);
        EXPECT_DOUBLE_EQ(s[k].wrench.torque.z, 0.3);
        EXPECT_EQ(s[k].flags, 0u);
    }
}

// T-FTS-U2-020 [FTS-ADP-020] Wrong time is an error and leaves the state unchanged
TEST(ForceTorque, WrongTimeIsAnErrorWithoutSideEffects)
{
    const std::string path = config("time", "\"noise\":{\"definition\":\"1sigma\",\"rate_hz\":500,\"fxy\":1,\"fz\":1,\"txy\":0.1,\"tz\":0.1}");
    auto a = make(path);
    auto b = make(path);
    auto truth = [](std::uint64_t) { return wrench({1, 2, 3, 0.1, 0.2, 0.3}); };
    run(*a, 3, truth);
    run(*b, 3, truth);
    sa::ForceTorqueTruth t;
    t.sim_time_ns = 7 * kStep;
    sa::ForceTorqueSample s;
    std::string error;
    EXPECT_FALSE(a->step(t, s, error));
    t.sim_time_ns = 1 * kStep;
    EXPECT_FALSE(a->step(t, s, error));
    const auto sa_ = run(*a, 6, truth, 3);
    const auto sb = run(*b, 6, truth, 3);
    for (std::size_t i = 0; i < sa_.size(); ++i) EXPECT_DOUBLE_EQ(sa_[i].wrench.force.z, sb[i].wrench.force.z);
}

// T-FTS-U2-030 [FTS-ADP-120] Delay by whole steps; time-0 truth before the history is long enough
TEST(ForceTorque, DelayByWholeSteps)
{
    auto a = make(config("delay", "\"delay\":0.0035", 1000));  // 3 steps
    const auto s = run(*a, 10, [](std::uint64_t k) { return wrench({10.0 + k, 0, 0, 0, 0, 0}); });
    for (std::uint64_t k = 0; k < 10; ++k) {
        const double expected = k < 3 ? 10.0 : 10.0 + static_cast<double>(k - 3);
        EXPECT_DOUBLE_EQ(s[k].wrench.force.x, expected) << k;
    }
}

// T-FTS-U2-040 [FTS-ADP-130] Low-pass: steady DC passes; at the cut-off the gain is 1/sqrt(2)
TEST(ForceTorque, LowpassDcAndCutoffGain)
{
    auto dc = make(config("lp_dc", "\"lowpass_cutoff_hz\":20", 1000));
    const auto s = run(*dc, 50, [](std::uint64_t) { return wrench({42, 0, 0, 0, 0, 0}); });
    for (const auto& x : s) EXPECT_NEAR(x.wrench.force.x, 42.0, 1e-9);

    const double fc = 20.0;
    auto a = make(config("lp_sine", "\"lowpass_cutoff_hz\":20", 1000));
    const auto out = run(*a, 4000, [fc](std::uint64_t k) {
        return wrench({50.0 * std::sin(2 * kPi * fc * static_cast<double>(k) * 1e-3), 0, 0, 0, 0, 0});
    });
    double peak = 0;
    for (std::size_t k = 2000; k < out.size(); ++k) peak = std::max(peak, std::fabs(out[k].wrench.force.x));
    EXPECT_NEAR(peak / 50.0, 1.0 / std::sqrt(2.0), 0.005);
}

// T-FTS-U2-050 [FTS-ADP-210] Nonlinearity: zero at 0 and the range ends, curve 4u(1-|u|), within NL
TEST(ForceTorque, NonlinearityShape)
{
    auto a = make(config("nl", "\"nonlinearity\":\"2%\""));
    auto at = [&](double fraction) {
        std::string reset_error;
        a->reset(reset_error);
        std::array<double, 6> v{};
        for (int i = 0; i < 6; ++i) v[i] = fraction * kRange[i];
        const auto s = run(*a, 1, [&](std::uint64_t) { return wrench(v); });
        std::array<double, 6> dev{};
        for (int i = 0; i < 6; ++i) dev[i] = axis(s[0].wrench, i) - v[i];
        return dev;
    };
    const auto zero = at(0.0), end = at(1.0), half = at(0.5), quarter = at(0.25), neg = at(-0.5);
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(zero[i], 0.0, 1e-12);
        EXPECT_NEAR(end[i], 0.0, 1e-9 * kRange[i]);
        EXPECT_LE(std::fabs(half[i]), 0.02 * kRange[i] + 1e-12);
        EXPECT_NEAR(quarter[i], 0.75 * half[i], 1e-9 * kRange[i]);  // 4*0.25*0.75 = 0.75
        EXPECT_NEAR(neg[i], -half[i], 1e-9 * kRange[i]);            // odd curve
    }
}

// T-FTS-U2-060 [FTS-ADP-220] Calibration matrix on range-normalised values, within S and X
TEST(ForceTorque, CalibrationMatrixBoundsAndLinearity)
{
    auto a = make(config("cal", "\"scale_error\":\"1%\",\"crosstalk\":\"5%\""));
    std::string error;
    for (int j = 0; j < 6; ++j) {
        for (double frac : {1.0, 0.5}) {
            a->reset(error);
            std::array<double, 6> v{};
            v[j] = frac * kRange[j];
            const auto s = run(*a, 1, [&](std::uint64_t) { return wrench(v); });
            for (int i = 0; i < 6; ++i) {
                const double y = axis(s[0].wrench, i);
                if (i == j) EXPECT_LE(std::fabs(y - v[j]), 0.01 * frac * kRange[i] + 1e-12);
                else EXPECT_LE(std::fabs(y), 0.05 * frac * kRange[i] + 1e-12);
            }
        }
    }
    // Linear: the response to half the load is half the response to the load (inside the range,
    // so the clamp does not cut it).
    a->reset(error);
    const auto full = run(*a, 1, [](std::uint64_t) { return wrench({80, 0, 0, 0, 0, 0}); });
    a->reset(error);
    const auto half = run(*a, 1, [](std::uint64_t) { return wrench({40, 0, 0, 0, 0, 0}); });
    for (int i = 0; i < 6; ++i) EXPECT_NEAR(axis(half[0].wrench, i), 0.5 * axis(full[0].wrench, i), 1e-12);
}

// T-FTS-U2-070 [FTS-ADP-240] Bias: a constant offset within the bound, drawn once
TEST(ForceTorque, BiasConstantWithinBound)
{
    auto a = make(config("bias", "\"bias\":{\"fxy\":2,\"fz\":\"1%\",\"txy\":0.1,\"tz\":0.2}"));
    const auto s = run(*a, 20, [](std::uint64_t) { return wrench({0, 0, 0, 0, 0, 0}); });
    const double bound[6] = {2, 2, 2, 0.1, 0.1, 0.2};
    for (int i = 0; i < 6; ++i) {
        EXPECT_LE(std::fabs(axis(s[0].wrench, i)), bound[i]);
        EXPECT_GT(std::fabs(axis(s[0].wrench, i)), 0.0);
        for (const auto& x : s) EXPECT_DOUBLE_EQ(axis(x.wrench, i), axis(s[0].wrench, i));
    }
}

// T-FTS-U2-080 [FTS-ADP-250] Drift: random walk with sigma D per hour (increment sigma D*sqrt(T/3600))
TEST(ForceTorque, DriftIncrementSigma)
{
    auto a = make(config("drift", "\"drift_per_hour\":{\"fxy\":60,\"fz\":60,\"txy\":6,\"tz\":6}", 1000));
    const auto s = run(*a, 40000, [](std::uint64_t) { return wrench({0, 0, 0, 0, 0, 0}); });
    std::vector<double> inc;
    for (std::size_t k = 1; k < s.size(); ++k) inc.push_back(s[k].wrench.force.z - s[k - 1].wrench.force.z);
    const double expected = 60.0 * std::sqrt(1e-3 / 3600.0);
    EXPECT_NEAR(stddev(inc), expected, 0.03 * expected);
}

// T-FTS-U2-090 [FTS-ADP-310..340] Noise sigma converted by sqrt(B/B0)
TEST(ForceTorque, NoiseBandwidthConversion)
{
    struct Case {
        const char* name;
        std::string extra;
        double rate;
        double expected;
    };
    // sigma0 = 0.1 N measured at 100 Hz (B0 = 50 Hz).
    const std::string noise = "\"noise\":{\"definition\":\"1sigma\",\"rate_hz\":100,\"fxy\":0.1,\"fz\":0.1,\"txy\":0.01,\"tz\":0.01}";
    const std::string noise6 = "\"noise\":{\"definition\":\"6sigma_pp\",\"bandwidth_hz\":50,\"fxy\":0.6,\"fz\":0.6,\"txy\":0.06,\"tz\":0.06}";
    const std::vector<Case> cases = {
        {"n_plain", noise, 500, 0.1 * std::sqrt(250.0 / 50.0)},
        {"n_lowpass", noise + ",\"lowpass_cutoff_hz\":100", 500, 0.1 * std::sqrt(100.0 / 50.0)},
        {"n_same", noise, 100, 0.1},
        {"n_6sigma", noise6, 100, 0.1},
    };
    for (const auto& c : cases) {
        auto a = make(config(c.name, c.extra, c.rate));
        const auto s = run(*a, 200000, [](std::uint64_t) { return wrench({0, 0, 0, 0, 0, 0}); });
        std::vector<double> v;
        for (const auto& x : s)
            if (x.updated) v.push_back(x.wrench.force.y);
        ASSERT_GE(v.size(), 20000u) << c.name;
        EXPECT_NEAR(stddev(v), c.expected, 0.03 * c.expected) << c.name;
    }
}

// T-FTS-U2-100 [FTS-ADP-270] [FTS-ADP-110] Clamp with the saturated flag; overload on the truth
TEST(ForceTorque, ClampAndOverloadFlags)
{
    auto a = make(config("flags", "\"overload\":{\"fxy\":\"150%\",\"fz\":300,\"txy\":10,\"tz\":20}"));
    // k=0 within range; k=1 (not a sample step) spikes Fx to 200 N (> 150 N overload) only in the truth;
    // k=2 back to normal; k=4 Fz 250 N: clamped at 200 (saturated) but below the 300 N overload.
    const auto s = run(*a, 6, [](std::uint64_t k) {
        if (k == 1) return wrench({200, 0, 0, 0, 0, 0});
        if (k == 4) return wrench({0, 0, 250, 0, 0, 0});
        return wrench({10, 0, 0, 0, 0, 0});
    });
    EXPECT_EQ(s[0].flags, 0u);
    EXPECT_EQ(s[2].flags, static_cast<std::uint32_t>(sa::kForceTorqueFlagOverloaded));
    EXPECT_DOUBLE_EQ(s[2].wrench.force.x, 10.0);
    EXPECT_EQ(s[4].flags, static_cast<std::uint32_t>(sa::kForceTorqueFlagSaturated));
    EXPECT_DOUBLE_EQ(s[4].wrench.force.z, 200.0);
    EXPECT_EQ(s[5].flags, s[4].flags);  // held
}

// T-FTS-U2-110 [FTS-ADP-410..440] Tare averages the window; clear and reset forget it; drift continues
TEST(ForceTorque, TareAveragesWindow)
{
    const std::string path = config("tare", "\"noise\":{\"definition\":\"1sigma\",\"rate_hz\":1000,\"fxy\":1,\"fz\":1,\"txy\":0.1,\"tz\":0.1}", 500);
    auto a = make(path);
    std::string error;
    EXPECT_FALSE(a->tare(error));
    auto truth = [](std::uint64_t) { return wrench({0, 0, -19.6, 0, 0, 0}); };
    const auto before = run(*a, 400, truth);
    // Independent expectation: the mean of the last 0.1 s (50 samples at 500 Hz) of untared outputs.
    std::vector<double> samples;
    for (const auto& x : before)
        if (x.updated) samples.push_back(x.wrench.force.z);
    double mean = 0;
    for (std::size_t i = samples.size() - 50; i < samples.size(); ++i) mean += samples[i];
    mean /= 50.0;
    ASSERT_TRUE(a->tare(error)) << error;
    auto b = make(path);  // same seed: same raw values, no tare
    run(*b, 400, truth);
    const auto after = run(*a, 2, truth, 400);
    const auto raw = run(*b, 2, truth, 400);
    EXPECT_NEAR(after[0].wrench.force.z, raw[0].wrench.force.z - mean, 1e-12);
    a->clear_tare();
    b->clear_tare();
    const auto cleared = run(*a, 2, truth, 402);
    const auto raw2 = run(*b, 2, truth, 402);
    EXPECT_DOUBLE_EQ(cleared[0].wrench.force.z, raw2[0].wrench.force.z);
    ASSERT_TRUE(a->reset(error));
    EXPECT_FALSE(a->tare(error));
}

// T-FTS-U2-120 [FTS-ADP-290] Resolution rounds halves away from zero, after the tare
TEST(ForceTorque, ResolutionRounding)
{
    auto a = make(config("res", "\"resolution\":{\"fxy\":0.5,\"fz\":0.5,\"txy\":0.01,\"tz\":0.01}"));
    std::string error;
    const std::vector<std::pair<double, double>> cases = {{0.24, 0.0}, {0.25, 0.5}, {-0.25, -0.5}, {0.74, 0.5}, {1.0, 1.0}};
    for (const auto& [in, out] : cases) {
        a->reset(error);
        const auto s = run(*a, 1, [in = in](std::uint64_t) { return wrench({in, 0, 0, 0, 0, 0}); });
        EXPECT_DOUBLE_EQ(s[0].wrench.force.x, out) << in;
    }
}

// T-FTS-U2-130 [FTS-ADP-040] [FTS-ADP-050] [FTS-ADP-060] Seeds, independent streams, reset replays
TEST(ForceTorque, SeedsStreamsAndReset)
{
    const std::string both = config("seed_both", "\"bias\":{\"fxy\":1,\"fz\":1,\"txy\":0.1,\"tz\":0.1},\"noise\":{\"definition\":\"1sigma\",\"rate_hz\":500,\"fxy\":1,\"fz\":1,\"txy\":0.1,\"tz\":0.1}");
    const std::string bias_only = config("seed_bias", "\"bias\":{\"fxy\":1,\"fz\":1,\"txy\":0.1,\"tz\":0.1}");
    auto zero = [](std::uint64_t) { return wrench({0, 0, 0, 0, 0, 0}); };
    auto a = make(both, 11), b = make(both, 11), c = make(both, 12), d = make(bias_only, 11);
    const auto sa_ = run(*a, 20, zero), sb = run(*b, 20, zero), sc = run(*c, 20, zero), sd = run(*d, 1, zero);
    for (std::size_t k = 0; k < 20; ++k) EXPECT_DOUBLE_EQ(sa_[k].wrench.force.x, sb[k].wrench.force.x);
    EXPECT_NE(sa_[0].wrench.force.x, sc[0].wrench.force.x);
    // The bias stream does not depend on whether noise is on: mean of a's samples ~ d's bias.
    double mean = 0;
    std::size_t n = 0;
    auto e = make(both, 11);
    const auto long_run = run(*e, 40000, zero);
    for (const auto& x : long_run)
        if (x.updated) mean += x.wrench.force.x, ++n;
    mean /= static_cast<double>(n);
    EXPECT_NEAR(mean, sd[0].wrench.force.x, 4.0 / std::sqrt(static_cast<double>(n)));
    // Reset replays the same sequence.
    std::string error;
    ASSERT_TRUE(a->reset(error));
    const auto again = run(*a, 20, zero);
    for (std::size_t k = 0; k < 20; ++k) EXPECT_DOUBLE_EQ(again[k].wrench.force.x, sa_[k].wrench.force.x);
}

// T-FTS-U3-010 [FTS-ADP-510] [FTS-ADP-540] Configuration errors at creation
TEST(ForceTorque, ConfigErrors)
{
    const std::string base = "\"type\":\"force_torque\",\"update_rate\":500,\"range\":{\"fxy\":100,\"fz\":200,\"txy\":5,\"tz\":10}";
    EXPECT_NE(make_error(write_config("e_unknown", "{" + base + ",\"nois\":{}}")).find("unknown key"), std::string::npos);
    EXPECT_FALSE(make_error(write_config("e_type", "{\"type\":\"imu\",\"update_rate\":500}")).empty());
    EXPECT_FALSE(make_error(write_config("e_range", "{\"type\":\"force_torque\",\"update_rate\":500,\"range\":{\"fxy\":100}}")).empty());
    EXPECT_FALSE(make_error(write_config("e_zero", "{\"type\":\"force_torque\",\"update_rate\":500,\"range\":{\"fxy\":0,\"fz\":1,\"txy\":1,\"tz\":1}}")).empty());
    EXPECT_FALSE(make_error(write_config("e_neg", "{" + base + ",\"bias\":{\"fxy\":-1}}")).empty());
    EXPECT_FALSE(make_error(write_config("e_pct", "{" + base + ",\"crosstalk\":\"5 percent\"}")).empty());
    EXPECT_FALSE(make_error(write_config("e_def", "{" + base + ",\"noise\":{\"definition\":\"rms\",\"rate_hz\":100,\"fxy\":1}}")).empty());
    EXPECT_FALSE(make_error(write_config("e_rate", "{" + base + ",\"noise\":{\"definition\":\"1sigma\",\"fxy\":1}}")).empty());
    EXPECT_FALSE(make_error(write_config("e_lp", "{" + base + ",\"lowpass_cutoff_hz\":600}")).empty());
}

// T-FTS-U3-020 [FTS-ADP-550] [FTS-ADP-610] Capabilities follow the configuration; log columns
TEST(ForceTorque, CapabilitiesAndLog)
{
    auto ideal = make(config("cap_ideal", ""));
    auto caps = ideal->capabilities();
    EXPECT_FALSE(caps.noise || caps.bias || caps.drift || caps.calibration_matrix || caps.nonlinearity || caps.lowpass ||
                 caps.delay || caps.resolution || caps.overload);
    EXPECT_TRUE(caps.range_clamp && caps.tare && caps.deterministic_with_seed && caps.thread_safe_across_instances);
    EXPECT_FALSE(caps.overload_offset || caps.temperature || caps.hysteresis);

    auto full = make(std::string(HAKO_SENSORS_CONFIG_DIR) + "/force_torque/onrobot_hex_e_qc.json");
    caps = full->capabilities();
    EXPECT_TRUE(caps.noise && caps.calibration_matrix && caps.nonlinearity && caps.lowpass && caps.resolution &&
                caps.overload);
    EXPECT_EQ(full->log_columns().size(), 18u);
    EXPECT_EQ(full->log_columns()[0].name, "true_fx");
    std::vector<double> values;
    full->log_values(values);
    EXPECT_EQ(values.size(), 18u);
}

// T-FTS-U3-030 [FTS-ADP-520] [FTS-ADP-540] Per-axis keys: an axis key takes precedence over its group;
// a configuration with groups only reads as before; range and overload need every axis
TEST(ForceTorque, PerAxisKeys)
{
    // Noise: fx overrides fxy, fy keeps the group; tx and ty both by axis.
    const std::string noise = "\"noise\":{\"definition\":\"1sigma\",\"rate_hz\":500,\"fxy\":0.1,\"fx\":0.3,"
                              "\"fz\":0.2,\"tx\":0.01,\"ty\":0.04,\"tz\":0.02}";
    auto a = make(config("axis_noise", noise));
    const auto s = run(*a, 40000, [](std::uint64_t) { return wrench({0, 0, 0, 0, 0, 0}); });
    std::vector<double> v[6];
    for (const auto& x : s)
        if (x.updated)
            for (int i = 0; i < 6; ++i) v[i].push_back(axis(x.wrench, i));
    const double expected[6] = {0.3, 0.1, 0.2, 0.01, 0.04, 0.02};
    for (int i = 0; i < 6; ++i) EXPECT_NEAR(stddev(v[i]), expected[i], 0.03 * expected[i]) << i;

    // Groups only and the same values by axis give the same sequence (the same seed).
    auto g = make(config("axis_group", "\"bias\":{\"fxy\":0.5,\"fz\":0.5,\"txy\":0.05,\"tz\":0.05}"));
    auto p = make(config("axis_split", "\"bias\":{\"fx\":0.5,\"fy\":0.5,\"fz\":0.5,\"tx\":0.05,\"ty\":0.05,\"tz\":0.05}"));
    const auto sg = run(*g, 4, [](std::uint64_t) { return wrench({0, 0, 0, 0, 0, 0}); });
    const auto sp = run(*p, 4, [](std::uint64_t) { return wrench({0, 0, 0, 0, 0, 0}); });
    for (int i = 0; i < 6; ++i) EXPECT_EQ(axis(sg[0].wrench, i), axis(sp[0].wrench, i)) << i;

    // Range by axis: Fx clips at 50 N, Fy (group 100 N) does not at 80 N.
    auto r = make(write_config("axis_range", "{\"type\":\"force_torque\",\"update_rate\":500,"
                                             "\"range\":{\"fxy\":100,\"fx\":50,\"fz\":200,\"txy\":5,\"tz\":10}}"));
    const auto sr = run(*r, 1, [](std::uint64_t) { return wrench({80, 80, 0, 0, 0, 0}); });
    EXPECT_DOUBLE_EQ(sr[0].wrench.force.x, 50.0);
    EXPECT_DOUBLE_EQ(sr[0].wrench.force.y, 80.0);

    // Errors: a pair half given, an unknown axis key, a % range.
    EXPECT_NE(make_error(write_config("axis_half", "{\"type\":\"force_torque\",\"update_rate\":500,"
                                                   "\"range\":{\"fx\":50,\"fz\":200,\"txy\":5,\"tz\":10}}"))
                  .find("range.fy"),
              std::string::npos);
    EXPECT_NE(make_error(config("axis_unknown", "\"bias\":{\"fq\":1}")).find("unknown key"), std::string::npos);
    EXPECT_FALSE(make_error(write_config("axis_pct", "{\"type\":\"force_torque\",\"update_rate\":500,"
                                                    "\"range\":{\"fxy\":\"50%\",\"fz\":200,\"txy\":5,\"tz\":10}}"))
                     .empty());
    EXPECT_NE(make_error(config("axis_overload", "\"overload\":{\"fx\":\"150%\",\"fz\":300,\"txy\":10,\"tz\":20}"))
                  .find("overload.fy"),
              std::string::npos);
}
