// Black-box tests for the force-torque vibration model (FTV-010..620).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "hakoniwa/mujoco_sensors/force_torque_vibration.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

HAKO_SENSOR_HOST_MODEL_DECLARE(hakoniwa);

namespace {

constexpr std::uint64_t kStepNs = 1'000'000;  // 1 ms
constexpr double kStepSeconds = 1e-3;
constexpr double kPi = 3.14159265358979323846;

std::string write_config(const std::string& name, const std::string& body)
{
    const std::string path = ::testing::TempDir() + "/ft_vibration_" + name + ".json";
    std::ofstream(path) << body;
    return path;
}

std::unique_ptr<sa::IForceTorqueVibration> make(const std::string& path, std::uint64_t seed = 7,
                                                std::uint64_t step = kStepNs)
{
    sa::ForceTorqueVibrationSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = step;
    setup.vehicle_name = "arm";
    setup.vehicle_index = 2;
    setup.sensor_name = "wrist";
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    auto model = ms::create_force_torque_vibration(setup, error);
    EXPECT_TRUE(model) << error;
    return model;
}

std::string make_error(const std::string& path)
{
    sa::ForceTorqueVibrationSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStepNs;
    std::string error;
    auto model = ms::create_force_torque_vibration(setup, error);
    EXPECT_FALSE(model) << path;
    return error;
}

double axis(const sa::Wrench& w, int i)
{
    const double values[6] = {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z};
    return values[i];
}

std::vector<sa::Wrench> run(sa::IForceTorqueVibration& model, std::size_t count,
                            const std::vector<double>& velocity, std::size_t first = 0)
{
    std::vector<sa::Wrench> result;
    result.reserve(count);
    std::string error;
    for (std::size_t k = first; k < first + count; ++k) {
        sa::Wrench out;
        EXPECT_TRUE(model.step(static_cast<std::uint64_t>(k) * kStepNs, velocity, out, error)) << error;
        result.push_back(out);
    }
    return result;
}

std::vector<double> values(const std::vector<sa::Wrench>& samples, int which)
{
    std::vector<double> result;
    result.reserve(samples.size());
    for (const auto& sample : samples) result.push_back(axis(sample, which));
    return result;
}

double stddev(const std::vector<double>& samples, std::size_t discard = 0)
{
    const auto first = samples.begin() + static_cast<std::ptrdiff_t>(discard);
    const double mean = std::accumulate(first, samples.end(), 0.0) /
                        static_cast<double>(samples.end() - first);
    double sum = 0.0;
    for (auto it = first; it != samples.end(); ++it) sum += (*it - mean) * (*it - mean);
    return std::sqrt(sum / static_cast<double>(samples.end() - first - 1));
}

// Independently scan integer-frequency DFT bins; sufficient for the deliberately integer-Hz test tones.
double peak_frequency(const std::vector<double>& samples, int low_hz, int high_hz)
{
    double best_power = -1.0;
    int best_hz = low_hz;
    for (int f = low_hz; f <= high_hz; ++f) {
        double re = 0.0, im = 0.0;
        for (std::size_t k = 0; k < samples.size(); ++k) {
            const double phase = 2.0 * kPi * static_cast<double>(f) * kStepSeconds * static_cast<double>(k);
            re += samples[k] * std::cos(phase);
            im -= samples[k] * std::sin(phase);
        }
        const double power = re * re + im * im;
        if (power > best_power) {
            best_power = power;
            best_hz = f;
        }
    }
    return static_cast<double>(best_hz);
}

std::string order_config(const std::string& name, double gear, double order, double exponent,
                         double amplitude, double deadband = 0.01, bool resonance = false,
                         const std::string& extra = "")
{
    return write_config(name,
        "{\"type\":\"force_torque_vibration\",\"f_ref_hz\":50,\"joints\":[{\"name\":\"joint\","
        "\"gear_ratio\":" + std::to_string(gear) + ",\"deadband_hz\":" + std::to_string(deadband) +
        ",\"orders\":[{\"order\":" + std::to_string(order) + ",\"exponent\":" +
        std::to_string(exponent) + ",\"resonance\":" + (resonance ? "true" : "false") +
        ",\"amplitude\":{\"fx\":" + std::to_string(amplitude) + "}}]}]" + extra + "}");
}

}  // namespace

// T-FTV-U1-010 [FTV-060] With no configured component every output axis is exactly zero
TEST(ForceTorqueVibration, EmptyModelIsExactlyZero)
{
    auto model = make(write_config("empty", R"({"type":"force_torque_vibration","joints":[
      {"name":"j","gear_ratio":100,"orders":[]}]})"));
    ASSERT_TRUE(model);
    std::string error;
    for (std::size_t k = 0; k < 20; ++k) {
        sa::Wrench out;
        const double velocity = (k % 2 == 0) ? -123.0 : 456.0;
        ASSERT_TRUE(model->step(k * kStepNs, {velocity}, out, error)) << error;
        for (int i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(axis(out, i), 0.0);
    }
}

// T-FTV-U1-020 [FTV-020] Wrong velocity count and NaN fail without advancing any state
TEST(ForceTorqueVibration, InvalidVelocityIsAnErrorWithoutSideEffects)
{
    const auto path = order_config("bad_velocity", 10, 2, 0, 3);
    auto tested = make(path), fresh = make(path);
    ASSERT_TRUE(tested && fresh);
    sa::Wrench out;
    std::string error;
    EXPECT_FALSE(tested->step(0, {}, out, error));
    EXPECT_FALSE(error.empty());
    error.clear();
    EXPECT_FALSE(tested->step(0, {std::numeric_limits<double>::quiet_NaN()}, out, error));
    EXPECT_FALSE(error.empty());
    const auto a = run(*tested, 100, {2 * kPi});
    const auto b = run(*fresh, 100, {2 * kPi});
    for (std::size_t k = 0; k < a.size(); ++k)
        for (int i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(axis(a[k], i), axis(b[k], i));
}

// T-FTV-U1-030 [FTV-030] Wrong step times fail with messages and leave phase and call time unchanged
TEST(ForceTorqueVibration, WrongTimeIsAnErrorWithoutSideEffects)
{
    const auto path = order_config("bad_time", 10, 2, 0, 3);
    auto tested = make(path), fresh = make(path);
    ASSERT_TRUE(tested && fresh);
    sa::Wrench out;
    std::string error;
    EXPECT_FALSE(tested->step(kStepNs, {2 * kPi}, out, error));
    EXPECT_FALSE(error.empty());
    const auto first_a = run(*tested, 10, {2 * kPi});
    const auto first_b = run(*fresh, 10, {2 * kPi});
    error.clear();
    EXPECT_FALSE(tested->step(9 * kStepNs, {2 * kPi}, out, error));  // repeated time
    EXPECT_FALSE(error.empty());
    const auto rest_a = run(*tested, 90, {2 * kPi}, 10);
    const auto rest_b = run(*fresh, 90, {2 * kPi}, 10);
    EXPECT_EQ(values(first_a, 0), values(first_b, 0));
    EXPECT_EQ(values(rest_a, 0), values(rest_b, 0));
}

// T-FTV-U2-010 [FTV-110..140] Order frequency, amplitude, axes, speed scaling and dead-band taper
TEST(ForceTorqueVibration, OrderTracksMotorSpeedAndAmplitudeLaw)
{
    // N=10 and qdot=2*pi rad/s gives fm=10 Hz; order 2 gives a 20 Hz output.
    auto constant = make(order_config("order", 10, 2, 0, 3));
    ASSERT_TRUE(constant);
    const auto x = values(run(*constant, 4000, {2 * kPi}), 0);
    EXPECT_EQ(peak_frequency(x, 10, 30), 20.0);
    const double peak = *std::max_element(x.begin(), x.end(), [](double a, double b) {
        return std::fabs(a) < std::fabs(b);
    });
    EXPECT_NEAR(std::fabs(peak), 3.0, 2e-3);  // 4000 samples cover 80 cycles densely.
    auto axes_model = make(order_config("order_axes", 10, 2, 0, 3));
    ASSERT_TRUE(axes_model);
    for (const auto& out : run(*axes_model, 100, {2 * kPi}))
        for (int i = 1; i < 6; ++i) EXPECT_DOUBLE_EQ(axis(out, i), 0.0);

    auto scaled = make(order_config("order_scaled", 10, 2, 1, 3));
    ASSERT_TRUE(scaled);
    const auto y = values(run(*scaled, 4000, {2 * kPi}), 0);
    const double scaled_peak = std::fabs(*std::max_element(y.begin(), y.end(), [](double a, double b) {
        return std::fabs(a) < std::fabs(b);
    }));
    EXPECT_NEAR(scaled_peak, 3.0 * 10.0 / 50.0, 2e-3);

    auto below = make(order_config("order_dead", 10, 1, 0, 3, 5));
    ASSERT_TRUE(below);
    const auto z = values(run(*below, 5000, {0.2 * kPi}), 0);  // fm=1 Hz: dead-band factor 1/5.
    const double below_peak = std::fabs(*std::max_element(z.begin(), z.end(), [](double a, double b) {
        return std::fabs(a) < std::fabs(b);
    }));
    EXPECT_NEAR(below_peak, 3.0 / 5.0, 2e-3);
    auto stopped = make(order_config("order_stopped", 10, 1, 0, 3, 5));
    ASSERT_TRUE(stopped);
    for (const auto& out : run(*stopped, 20, {0.0})) EXPECT_DOUBLE_EQ(out.force.x, 0.0);
}

// T-FTV-U2-020 [FTV-110] A velocity change changes phase slope, never phase itself
TEST(ForceTorqueVibration, OrderPhaseIsContinuousAcrossVelocityStep)
{
    constexpr double amplitude = 3.0;
    constexpr double gear = 10.0;
    constexpr double order = 2.0;
    auto model = make(order_config("continuity", gear, order, 0, amplitude));
    ASSERT_TRUE(model);
    const auto before = run(*model, 500, {2 * kPi});
    const auto after = run(*model, 500, {4 * kPi}, 500);
    const double delta = std::fabs(after.front().force.x - before.back().force.x);
    const double phase_step = order * gear * 4.0 * kPi * kStepSeconds;
    EXPECT_LE(delta, amplitude * phase_step + 1e-12);  // |sin(x+d)-sin(x)| <= |d|.
}

// T-FTV-U3-010 [FTV-210..240] A normalized mode has its requested sigma, centre and activity growth
TEST(ForceTorqueVibration, StructuralModeStatisticsAndActivity)
{
    const std::string common =
        "\"f_ref_hz\":50,\"joints\":[{\"name\":\"j\",\"gear_ratio\":10,\"orders\":[]}],"
        "\"modes\":[{\"frequency_hz\":20,\"damping\":0.08,\"at_rest\":{\"fz\":2},"
        "\"per_activity\":{\"fz\":3}}]";
    auto rest = make(write_config("mode_rest", "{\"type\":\"force_torque_vibration\"," + common + "}"));
    auto moving = make(write_config("mode_moving", "{\"type\":\"force_torque_vibration\"," + common + "}"));
    ASSERT_TRUE(rest && moving);
    const auto r = values(run(*rest, 30000, {0.0}), 2);
    const auto m = values(run(*moving, 30000, {10 * kPi}), 2);  // fm=50 Hz, activity=1.
    EXPECT_NEAR(stddev(r, 2000), 2.0, 0.06);  // 28k correlated samples: 3% tolerance.
    EXPECT_NEAR(peak_frequency(r, 10, 30), 20.0, 2.0);
    EXPECT_NEAR(stddev(m, 2000), 5.0, 0.15);
}

// T-FTV-U4-010 [FTV-310/320] Broadband converts one-sided density to per-step sigma and vanishes at rest
TEST(ForceTorqueVibration, BroadbandDensityAndZeroActivity)
{
    const std::string body = R"({"type":"force_torque_vibration","f_ref_hz":50,
        "joints":[{"name":"j","gear_ratio":10,"orders":[]}],
        "broadband":{"exponent":1,"density_per_activity":{"fx":0.02}}})";
    auto moving = make(write_config("broad_moving", body));
    auto stopped = make(write_config("broad_stopped", body));
    ASSERT_TRUE(moving && stopped);
    const double activity = 1.0;  // qdot=10*pi -> fm=50 Hz, divided by f_ref=50 Hz.
    const double expected = std::sqrt(0.02 * activity / (2.0 * kStepSeconds));
    EXPECT_NEAR(stddev(values(run(*moving, 30000, {10 * kPi}), 0)), expected, 0.025 * expected);
    for (const auto& out : run(*stopped, 100, {0.0})) EXPECT_DOUBLE_EQ(out.force.x, 0.0);
}

// T-FTV-U2-030 [FTV-160] Resonance gain is 1/(2*zeta) on resonance and approximately one far away
TEST(ForceTorqueVibration, OptionalResonanceGain)
{
    const std::string mode =
        ",\"modes\":[{\"frequency_hz\":20,\"damping\":0.1,\"at_rest\":{},\"per_activity\":{}}]";
    auto resonant = make(order_config("resonant", 10, 2, 0, 1, 0.01, true, mode));
    auto plain = make(order_config("resonant_plain", 10, 2, 0, 1, 0.01, false, mode));
    ASSERT_TRUE(resonant && plain);
    const auto a = values(run(*resonant, 2000, {2 * kPi}), 0);
    const auto b = values(run(*plain, 2000, {2 * kPi}), 0);
    EXPECT_NEAR(stddev(a) / stddev(b), 1.0 / (2.0 * 0.1), 1e-10);

    auto far = make(order_config("far", 10, 2, 0, 1, 0.01, true, mode));
    auto far_plain = make(order_config("far_plain", 10, 2, 0, 1, 0.01, false, mode));
    ASSERT_TRUE(far && far_plain);
    const auto c = values(run(*far, 10000, {0.02 * kPi}), 0);       // order frequency 0.2 Hz.
    const auto d = values(run(*far_plain, 10000, {0.02 * kPi}), 0);
    EXPECT_NEAR(stddev(c) / stddev(d), 1.0, 0.001);
}

// T-FTV-U1-040 [FTV-040] Reset reproduces phases, filter state and all random draws
TEST(ForceTorqueVibration, ResetReproducesSequenceFromCreation)
{
    const auto path = write_config("reset", R"({"type":"force_torque_vibration",
      "joints":[{"name":"j","gear_ratio":10,"orders":[{"order":2,"amplitude":{"fx":1}}]}],
      "modes":[{"frequency_hz":20,"damping":0.1,"at_rest":{"fz":1},"per_activity":{}}],
      "broadband":{"density_per_activity":{"fy":0.01}}})");
    auto model = make(path);
    ASSERT_TRUE(model);
    const auto first = run(*model, 1000, {2 * kPi});
    std::string error;
    ASSERT_TRUE(model->reset(error)) << error;
    const auto again = run(*model, 1000, {2 * kPi});
    for (std::size_t k = 0; k < first.size(); ++k)
        for (int i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(axis(first[k], i), axis(again[k], i));
}

// T-FTV-U5-010 [FTV-610/620] Seeds reproduce sequences, differ across seeds, and streams are separate
TEST(ForceTorqueVibration, SeedsAndComponentStreamsAreIndependent)
{
    const std::string order_only = R"({"type":"force_torque_vibration","joints":[{"name":"j",
      "gear_ratio":10,"orders":[{"order":2,"amplitude":{"fx":1}}]}]})";
    const std::string with_mode = R"({"type":"force_torque_vibration","joints":[{"name":"j",
      "gear_ratio":10,"orders":[{"order":2,"amplitude":{"fx":1}}]}],
      "modes":[{"frequency_hz":20,"damping":0.1,"at_rest":{"fz":1},"per_activity":{}}]})";
    const auto p1 = write_config("seed_order_1", order_only);
    const auto p2 = write_config("seed_order_mode", with_mode);
    auto a = make(p1, 17), b = make(p1, 17), different = make(p1, 18), extra = make(p2, 17);
    ASSERT_TRUE(a && b && different && extra);
    const auto av = run(*a, 1000, {2 * kPi});
    const auto bv = run(*b, 1000, {2 * kPi});
    const auto dv = run(*different, 1000, {2 * kPi});
    const auto ev = run(*extra, 1000, {2 * kPi});
    bool seed_differs = false;
    for (std::size_t k = 0; k < av.size(); ++k) {
        EXPECT_DOUBLE_EQ(av[k].force.x, bv[k].force.x);
        EXPECT_DOUBLE_EQ(av[k].force.x, ev[k].force.x);  // adding a mode cannot perturb vibration_phase.
        seed_differs = seed_differs || av[k].force.x != dv[k].force.x;
    }
    EXPECT_TRUE(seed_differs);
}

// T-FTV-U6-010 [FTV-450/510] Every listed malformed configuration is rejected with a message
TEST(ForceTorqueVibration, ConfigurationErrorsAreRejected)
{
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"unknown", R"({"type":"force_torque_vibration","mystery":1})"},
        {"missing_type", R"({})"},
        {"wrong_type", R"({"type":"force_torque"})"},
        {"missing_name", R"({"type":"force_torque_vibration","joints":[{"gear_ratio":1,"orders":[]}]})"},
        {"missing_gear", R"({"type":"force_torque_vibration","joints":[{"name":"j","orders":[]}]})"},
        {"missing_order", R"({"type":"force_torque_vibration","joints":[{"name":"j","gear_ratio":1,"orders":[{"amplitude":{}}]}]})"},
        {"missing_frequency", R"({"type":"force_torque_vibration","modes":[{"damping":0.1,"at_rest":{},"per_activity":{}}]})"},
        {"missing_damping", R"({"type":"force_torque_vibration","modes":[{"frequency_hz":10,"at_rest":{},"per_activity":{}}]})"},
        {"zero_gear", R"({"type":"force_torque_vibration","joints":[{"name":"j","gear_ratio":0,"orders":[]}]})"},
        {"zero_damping", R"({"type":"force_torque_vibration","modes":[{"frequency_hz":10,"damping":0,"at_rest":{},"per_activity":{}}]})"},
        {"unit_damping", R"({"type":"force_torque_vibration","modes":[{"frequency_hz":10,"damping":1,"at_rest":{},"per_activity":{}}]})"},
        {"nyquist", R"({"type":"force_torque_vibration","modes":[{"frequency_hz":500,"damping":0.1,"at_rest":{},"per_activity":{}}]})"},
        {"negative_amplitude", R"({"type":"force_torque_vibration","joints":[{"name":"j","gear_ratio":1,"orders":[{"order":1,"amplitude":{"fx":-1}}]}]})"},
        {"duplicate", R"({"type":"force_torque_vibration","joints":[{"name":"j","gear_ratio":1,"orders":[]},{"name":"j","gear_ratio":2,"orders":[]}]})"},
        {"percent", R"({"type":"force_torque_vibration","joints":[{"name":"j","gear_ratio":1,"orders":[{"order":1,"amplitude":{"fx":"1%"}}]}]})"},
    };
    for (const auto& test : cases) EXPECT_FALSE(make_error(write_config(test.first, test.second)).empty()) << test.first;
}

// T-FTV-U6-020 [FTV-450] An explicit axis overrides its group value
TEST(ForceTorqueVibration, AxisValueOverridesAxisGroup)
{
    const auto path = write_config("axis_override", R"({"type":"force_torque_vibration","joints":[{
      "name":"j","gear_ratio":10,"orders":[{"order":2,"amplitude":{"fxy":1,"fx":3}}]}]})");
    auto model = make(path);
    ASSERT_TRUE(model);
    const auto out = run(*model, 2000, {2 * kPi});
    EXPECT_NEAR(stddev(values(out, 0)) / stddev(values(out, 1)), 3.0, 1e-10);
}

// T-FTV-U1-050 [FTV-020] joint_names preserves configuration order
TEST(ForceTorqueVibration, JointNamesAreInConfigurationOrder)
{
    auto model = make(write_config("names", R"({"type":"force_torque_vibration","joints":[
      {"name":"shoulder","gear_ratio":100,"orders":[]},
      {"name":"elbow","gear_ratio":80,"orders":[]},
      {"name":"wrist","gear_ratio":50,"orders":[]}]})"));
    ASSERT_TRUE(model);
    EXPECT_EQ(model->joint_names(), (std::vector<std::string>{"shoulder", "elbow", "wrist"}));
}

// T-FTV-U1-060 [FTV-050] The hakoniwa host-model entry describes and creates this model
TEST(ForceTorqueVibration, HakoniwaHostModelEntryExists)
{
    const auto* implementation = HAKO_SENSOR_HOST_MODEL_ENTRY(hakoniwa)();
    ASSERT_NE(implementation, nullptr);
    EXPECT_EQ(implementation->version, sa::kHostModelImplementationVersion);
    ASSERT_NE(implementation->name, nullptr);
    EXPECT_STREQ(implementation->name, "hakoniwa");
    ASSERT_NE(implementation->create_force_torque_vibration, nullptr);

    sa::ForceTorqueVibrationSetup setup;
    setup.config_path = write_config("entry", R"({"type":"force_torque_vibration"})");
    setup.physics_step_ns = kStepNs;
    std::string error;
    auto model = implementation->create_force_torque_vibration(setup, error);
    EXPECT_TRUE(model) << error;
}
