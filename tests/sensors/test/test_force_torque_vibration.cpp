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

#include "hakoniwa/mujoco_sensors/force_torque_sensor_adapter.hpp"
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

// T-FTV-U4-010 [FTV-310] [FTV-320] Broadband converts one-sided density to per-step sigma and vanishes at rest
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

// T-FTV-U5-010 [FTV-610] [FTV-620] Seeds reproduce sequences, differ across seeds, and streams are separate
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

// T-FTV-U6-010 [FTV-450] [FTV-510] Every listed malformed configuration is rejected with a message
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

namespace {

// The order phase theta of a single order-1 component on fx: with a constant velocity the motor phase grows by c per step,
// so y_k = A sin(c k + theta'), theta' = theta plus a fixed offset of the step convention. Over a whole number of periods
// the projections give A cos(theta') and A sin(theta').
struct PhaseFit {
    double amplitude;
    double phase;
};

PhaseFit fit_phase(const std::vector<double>& y, double c, std::size_t first)
{
    double s = 0.0, co = 0.0;
    for (std::size_t k = 0; k < y.size(); ++k) {
        const double a = c * static_cast<double>(first + k);
        s += y[k] * std::sin(a);
        co += y[k] * std::cos(a);
    }
    const double n = static_cast<double>(y.size());
    return {std::hypot(2.0 * s / n, 2.0 * co / n), std::atan2(2.0 * co / n, 2.0 * s / n)};
}

std::string one_order(const std::string& name, const std::string& joints)
{
    return write_config(name, R"({"type":"force_torque_vibration","f_ref_hz":50,"joints":[)" + joints + "]}");
}

const char* kJointA = R"({"name":"a","gear_ratio":10,"deadband_hz":0.01,"orders":[{"order":1,"amplitude":{"fx":1}}]})";

}  // namespace

// T-FTV-U1-070 [FTV-010] A model is created from the setup fields; a missing or unreadable file fails with a message
TEST(ForceTorqueVibration, CreationFromSetupFieldsAndFileErrors)
{
    auto model = make(one_order("create", kJointA), 3, kStepNs);
    ASSERT_TRUE(model);
    EXPECT_EQ(model->joint_names(), std::vector<std::string>{"a"});
    EXPECT_FALSE(make_error(::testing::TempDir() + "/ft_vibration_no_such_file.json").empty());
    EXPECT_FALSE(make_error(write_config("not_json", "{ this is not json")).empty());
    // The vehicle index and the sensor name are inputs of the stream derivation: changing either changes the draws.
    const auto path = one_order("create_names", kJointA);
    auto base = make(path, 3);
    sa::ForceTorqueVibrationSetup other;
    other.config_path = path;
    other.physics_step_ns = kStepNs;
    other.vehicle_name = "arm";
    other.vehicle_index = 2;
    other.sensor_name = "elbow";
    other.has_random_seed = true;
    other.random_seed = 3;
    std::string error;
    auto renamed = ms::create_force_torque_vibration(other, error);
    ASSERT_TRUE(base && renamed) << error;
    const auto x = values(run(*base, 1000, {2 * kPi}), 0);
    const auto y = values(run(*renamed, 1000, {2 * kPi}), 0);
    EXPECT_NE(x, y);
}

// T-FTV-U2-040 [FTV-150] The order phase is drawn once at creation, uniform on (-pi, pi), joints then orders then axes
TEST(ForceTorqueVibration, OrderPhaseDrawnOnceUniformInDrawOrder)
{
    const double c = 10.0 * 2.0 * kPi * kStepSeconds;  // gear 10, qdot 2*pi rad/s: 10 motor turns per second.
    const auto path = one_order("phase", kJointA);

    // Drawn once: the phase fitted on the first and on the next 10 periods is the same, with amplitude 1.
    auto once = make(path, 11);
    ASSERT_TRUE(once);
    const auto y = values(run(*once, 2000, {2 * kPi}), 0);
    const auto p1 = fit_phase(std::vector<double>(y.begin(), y.begin() + 1000), c, 0);
    const auto p2 = fit_phase(std::vector<double>(y.begin() + 1000, y.end()), c, 1000);
    EXPECT_NEAR(p1.amplitude, 1.0, 1e-9);
    EXPECT_NEAR(std::remainder(p1.phase - p2.phase, 2.0 * kPi), 0.0, 1e-9);

    // Uniform over seeds: the mean resultant length of 400 phases stays near 0 (about 0.044 expected; a phase drawn
    // from a narrower range would give a large value) and each quadrant holds about a quarter.
    double sx = 0.0, sy = 0.0;
    int quadrant[4] = {0, 0, 0, 0};
    for (std::uint64_t seed = 1; seed <= 400; ++seed) {
        auto m = make(path, seed);
        ASSERT_TRUE(m);
        const auto fit = fit_phase(values(run(*m, 1000, {2 * kPi}), 0), c, 0);
        sx += std::cos(fit.phase);
        sy += std::sin(fit.phase);
        ++quadrant[(fit.phase >= 0 ? 0 : 2) + ((std::abs(fit.phase) <= kPi / 2) ? 0 : 1)];
    }
    EXPECT_LT(std::hypot(sx, sy) / 400.0, 0.15);
    for (int q : quadrant) {
        EXPECT_GT(q, 60);
        EXPECT_LT(q, 140);
    }

    // Draw order: components appended after joint a (another order of a, another joint) leave a's draws unchanged;
    // a joint placed before a takes the first draws, so a's phase changes.
    const char* kOrder2 = R"({"name":"a","gear_ratio":10,"deadband_hz":0.01,"orders":[{"order":1,"amplitude":{"fx":1}},
        {"order":3,"amplitude":{"fy":1}}]})";
    const char* kJointB = R"({"name":"b","gear_ratio":7,"deadband_hz":0.01,"orders":[{"order":2,"amplitude":{"fz":1}}]})";
    auto alone = make(path, 5);
    auto appended = make(one_order("phase_appended", std::string(kOrder2) + "," + kJointB), 5);
    auto prepended = make(one_order("phase_prepended", std::string(kJointB) + "," + kJointA), 5);
    ASSERT_TRUE(alone && appended && prepended);
    const auto a = values(run(*alone, 1000, {2 * kPi}), 0);
    const auto b = values(run(*appended, 1000, {2 * kPi, 0.0}), 0);
    const auto p = values(run(*prepended, 1000, {0.0, 2 * kPi}), 0);
    for (std::size_t k = 0; k < a.size(); ++k) EXPECT_DOUBLE_EQ(a[k], b[k]);
    EXPECT_NE(a, p);
}

// T-FTV-U1-080 [FTV-170] load_gain {positive, negative, lag_s}: the model then needs joint loads; values are checked
TEST(ForceTorqueVibration, LoadGainConfiguration)
{
    auto with = make(one_order("load_ok",
        R"({"name":"a","gear_ratio":10,"load_gain":{"positive":1.5,"negative":0.7,"lag_s":0},"orders":[]})"));
    auto without = make(one_order("load_none", kJointA));
    ASSERT_TRUE(with && without);
    EXPECT_TRUE(with->needs_joint_load());
    EXPECT_FALSE(without->needs_joint_load());
    sa::Wrench out;
    std::string error;
    EXPECT_FALSE(with->step(0, {1.0}, out, error));        // loads unknown: refused (FTV-022)
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(with->step(0, {1.0}, {2.0}, out, error)) << error;
    const std::vector<std::pair<std::string, std::string>> bad = {
        {"load_no_negative", R"({"positive":1})"},
        {"load_no_positive", R"({"negative":1})"},
        {"load_zero_positive", R"({"positive":0,"negative":1})"},
        {"load_negative_lag", R"({"positive":1,"negative":1,"lag_s":-0.1})"},
        {"load_unknown_key", R"({"positive":1,"negative":1,"gain":2})"},
    };
    for (const auto& b : bad)
        EXPECT_FALSE(make_error(one_order(b.first,
            R"({"name":"a","gear_ratio":10,"load_gain":)" + b.second + R"(,"orders":[]})")).empty()) << b.first;
}

// T-FTV-U6-030 [FTV-410] [FTV-420] [FTV-430] [FTV-440] Every documented key is accepted; an unknown key is rejected at
// every level (top, joint, order, mode, broadband)
TEST(ForceTorqueVibration, ConfigurationKeysAtEveryLevel)
{
    const std::string order = R"({"order":2,"exponent":0.8,"resonance":true,"amplitude":{"fxy":0.1,"fz":0.05,"txy":0.01,"tz":0.001}})";
    const std::string joint = R"({"name":"a","gear_ratio":100,"deadband_hz":20,"run_log_sigma":0.3,"run_group":"gear",
        "load_gain":{"positive":1.4,"negative":0.9,"lag_s":0.7},"orders":[)" + order + "]}";
    const std::string mode = R"({"frequency_hz":12,"damping":0.1,"at_rest":{"fz":0.05},"per_activity":{"fz":0.02},
        "per_motion":{"fx":0.03},"motion_ramp":0.2,"at_rest_while_moving":0.3,"frame":"world","run_log_sigma":0.2,"run_group":"arm"})";
    const std::string broadband = R"({"exponent":1,"density_per_activity":{"fxy":1e-4},"run_log_sigma":0.1,"run_group":"arm"})";
    auto full = [&](const std::string& j, const std::string& m, const std::string& b, const std::string& top) {
        return R"({"type":"force_torque_vibration","f_ref_hz":50,"joints":[)" + j + R"(],"modes":[)" + m +
               R"(],"broadband":)" + b + top + "}";
    };
    auto model = make(write_config("keys_full", full(joint, mode, broadband, "")));
    ASSERT_TRUE(model);
    EXPECT_TRUE(model->needs_joint_load());
    EXPECT_TRUE(model->needs_sensor_orientation());

    auto with_extra = [](std::string object) { return object.insert(1, R"("bogus":1,)"); };
    const std::string order_extra = R"({"name":"a","gear_ratio":100,"orders":[)" + with_extra(order) + "]}";
    const std::vector<std::pair<std::string, std::string>> unknown = {
        {"keys_top", full(joint, mode, broadband, R"(,"bogus":1)")},
        {"keys_joint", full(with_extra(joint), mode, broadband, "")},
        {"keys_order", full(order_extra, mode, broadband, "")},
        {"keys_mode", full(joint, with_extra(mode), broadband, "")},
        {"keys_broadband", full(joint, mode, with_extra(broadband), "")},
    };
    for (const auto& u : unknown) EXPECT_FALSE(make_error(write_config(u.first, u.second)).empty()) << u.first;
}

// T-FTV-U5-020 [FTV-620] The model's streams are independent of the force-torque adapter's: stepping an adapter of the
// same seed, vehicle and sensor name in between does not change the vibration sequence
TEST(ForceTorqueVibration, StreamsIndependentOfForceTorqueAdapter)
{
    const auto path = write_config("indep", R"({"type":"force_torque_vibration","joints":[{"name":"a","gear_ratio":10,
      "orders":[{"order":2,"amplitude":{"fx":1}}]}],"modes":[{"frequency_hz":20,"damping":0.1,"at_rest":{"fz":1},
      "per_activity":{}}],"broadband":{"density_per_activity":{"fy":0.01}}})");
    const auto ft_path = write_config("indep_ft", R"({"type":"force_torque","update_rate":1000,
      "range":{"fxy":100,"fz":100,"txy":10,"tz":10},"noise":{"definition":"1sigma","rate_hz":1000,"fxy":0.1,"fz":0.1,"txy":0.01,"tz":0.01},
      "bias":{"fxy":0.5,"fz":0.5,"txy":0.05,"tz":0.05}})");
    auto alone = make(path, 9);
    auto shared = make(path, 9);
    sa::ForceTorqueAdapterSetup setup;
    setup.config_path = ft_path;
    setup.physics_step_ns = kStepNs;
    setup.vehicle_name = "arm";
    setup.vehicle_index = 2;
    setup.sensor_name = "wrist";
    setup.has_random_seed = true;
    setup.random_seed = 9;
    std::string error;
    auto adapter = ms::create_force_torque_sensor_adapter(setup, error);
    ASSERT_TRUE(alone && shared && adapter) << error;
    const auto a = run(*alone, 500, {2 * kPi});
    for (std::uint64_t k = 0; k < 500; ++k) {
        sa::ForceTorqueTruth t;
        t.sim_time_ns = k * kStepNs;
        sa::ForceTorqueSample s;
        ASSERT_TRUE(adapter->step(t, s, error)) << error;
        sa::Wrench w;
        ASSERT_TRUE(shared->step(k * kStepNs, {2 * kPi}, w, error)) << error;
        for (int i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(axis(a[k], i), axis(w, i)) << "step " << k << " axis " << i;
    }
}
