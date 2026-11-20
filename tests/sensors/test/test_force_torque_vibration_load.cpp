// Tests of the load-direction gain of the force-torque vibration model (FTV-022, FTV-024, FTV-170..185, FTV-510).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "hakoniwa/mujoco_sensors/force_torque_vibration.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStepNs = 1'000'000;  // 1 ms
constexpr double kPi = 3.14159265358979323846;

std::string write_config(const std::string& name, const std::string& body)
{
    const std::string path = ::testing::TempDir() + "/ft_vibration_load_" + name + ".json";
    std::ofstream(path) << body;
    return path;
}

std::unique_ptr<sa::IForceTorqueVibration> make(const std::string& path)
{
    sa::ForceTorqueVibrationSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStepNs;
    setup.has_random_seed = true;
    setup.random_seed = 3;
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
    EXPECT_FALSE(ms::create_force_torque_vibration(setup, error)) << path;
    return error;
}

// One joint, gear 10, order 1 of amplitude 1 N on Fx: 10 Hz at 2 pi rad/s.
std::string config(const std::string& load_gain)
{
    return R"({"type":"force_torque_vibration","joints":[{"name":"a","gear_ratio":10)" + load_gain +
           R"(,"orders":[{"order":1,"amplitude":{"fx":1}}]}]})";
}

// Peak |Fx| over steps [first, first + count) with the given velocity and load.
double peak(sa::IForceTorqueVibration& m, int first, int count, double velocity, double load)
{
    double p = 0;
    std::string error;
    for (int k = first; k < first + count; ++k) {
        sa::Wrench w;
        EXPECT_TRUE(m.step(static_cast<std::uint64_t>(k) * kStepNs, {velocity}, {load}, w, error)) << error;
        p = std::max(p, std::fabs(w.force.x));
    }
    return p;
}

TEST(ForceTorqueVibrationLoad, WithoutLoadGainLoadsAreIgnored)
{
    // [FTV-022] [FTV-024] a model without load_gain: both steps give the same output
    auto a = make(write_config("plain_a", config("")));
    auto b = make(write_config("plain_b", config("")));
    EXPECT_FALSE(a->needs_joint_load());
    std::string error;
    for (int k = 0; k < 200; ++k) {
        sa::Wrench wa, wb;
        ASSERT_TRUE(a->step(k * kStepNs, {2 * kPi}, wa, error)) << error;
        ASSERT_TRUE(b->step(k * kStepNs, {2 * kPi}, {k < 100 ? 5.0 : -5.0}, wb, error)) << error;
        ASSERT_EQ(wa.force.x, wb.force.x);
    }
}

TEST(ForceTorqueVibrationLoad, LoadGainNeedsLoads)
{
    // [FTV-022] [FTV-024] the step without loads is refused, with no change of state
    auto m = make(write_config("needs", config(R"(,"load_gain":{"positive":0.5,"negative":2})")));
    EXPECT_TRUE(m->needs_joint_load());
    sa::Wrench w;
    std::string error;
    EXPECT_FALSE(m->step(0, {2 * kPi}, w, error));
    EXPECT_NE(error.find("load"), std::string::npos);
    EXPECT_FALSE(m->step(0, {2 * kPi}, {}, w, error));                    // wrong count
    EXPECT_FALSE(m->step(0, {2 * kPi}, {std::nan("")}, w, error));       // not finite
    EXPECT_TRUE(m->step(0, {2 * kPi}, {1.0}, w, error)) << error;        // time 0 still expected
}

TEST(ForceTorqueVibrationLoad, GainFollowsTheSignOfPowerWithoutLag)
{
    // [FTV-175] [FTV-180] [FTV-185] lag 0: P = tau * qdot > 0 -> positive, < 0 -> negative, at once
    auto m = make(write_config("nolag", config(R"(,"load_gain":{"positive":0.5,"negative":2})")));
    const double v = 2 * kPi;
    EXPECT_NEAR(peak(*m, 0, 300, v, +3.0), 0.5, 2e-3);
    EXPECT_NEAR(peak(*m, 300, 300, v, -3.0), 2.0, 5e-3);
    // turning the other way flips the sign of the power: tau > 0 with qdot < 0 is negative
    EXPECT_NEAR(peak(*m, 600, 300, -v, +3.0), 2.0, 5e-3);
    EXPECT_NEAR(peak(*m, 900, 300, -v, -3.0), 0.5, 2e-3);
}

TEST(ForceTorqueVibrationLoad, ZeroPowerKeepsTheDirectionAndGainStartsAtOne)
{
    // [FTV-175] P = 0 keeps the previous direction; [FTV-180] g = 1 until the first direction
    auto m = make(write_config("zero", config(R"(,"load_gain":{"positive":0.5,"negative":2,"lag_s":0.5})")));
    const double v = 2 * kPi;
    EXPECT_NEAR(peak(*m, 0, 300, v, 0.0), 1.0, 2e-3);   // no direction yet: gain 1
    EXPECT_NEAR(peak(*m, 300, 300, v, -1.0), 2.0, 5e-3); // first direction: the target at once
    EXPECT_NEAR(peak(*m, 600, 300, v, 0.0), 2.0, 5e-3);  // zero power keeps it
}

TEST(ForceTorqueVibrationLoad, FirstOrderLag)
{
    // [FTV-180] after a switch from + to -, g approaches the target as 1 - exp(-t / T). The ratio to the same model
    // without load_gain (same phases) is the gain itself at every step.
    auto lagged = make(write_config("lag", config(R"(,"load_gain":{"positive":1,"negative":3,"lag_s":0.2})")));
    auto plain = make(write_config("lag_ref", config("")));
    const double v = 2 * kPi;
    std::string error;
    std::vector<double> gain;
    for (int k = 0; k < 1500; ++k) {
        sa::Wrench a, b;
        const double load = k < 100 ? +1.0 : -1.0;
        ASSERT_TRUE(lagged->step(k * kStepNs, {v}, {load}, a, error)) << error;
        ASSERT_TRUE(plain->step(k * kStepNs, {v}, b, error)) << error;
        gain.push_back(std::fabs(b.force.x) > 0.2 ? a.force.x / b.force.x : std::nan(""));
    }
    auto at = [&](int k) {  // nearest step with a usable ratio
        for (int d = 0; d < 20; ++d) {
            if (!std::isnan(gain[k - d])) return std::make_pair(k - d, gain[k - d]);
        }
        return std::make_pair(k, std::nan(""));
    };
    EXPECT_NEAR(at(99).second, 1.0, 1e-9);  // positive: g = 1
    const auto [k1, g1] = at(299);
    // the switch is applied from step 100 on: after n steps g = 3 - 2 (1 - a)^n, a = 1 - exp(-h / T)
    const double expected = 3.0 - 2.0 * std::exp(-(k1 - 99) * 0.001 / 0.2);
    EXPECT_NEAR(g1, expected, 1e-6);
    const auto [k2, g2] = at(1499);  // 7 time constants later: 3 - 2 e^-7
    EXPECT_NEAR(g2, 3.0 - 2.0 * std::exp(-(k2 - 99) * 0.001 / 0.2), 1e-6);
}

TEST(ForceTorqueVibrationLoad, ResetReturnsTheGainToOne)
{
    // [FTV-040] [FTV-180]
    auto m = make(write_config("reset", config(R"(,"load_gain":{"positive":0.5,"negative":2})")));
    const double v = 2 * kPi;
    EXPECT_NEAR(peak(*m, 0, 300, v, -1.0), 2.0, 5e-3);
    std::string error;
    ASSERT_TRUE(m->reset(error)) << error;
    EXPECT_NEAR(peak(*m, 0, 300, v, 0.0), 1.0, 2e-3);
}

TEST(ForceTorqueVibrationLoad, ConfigurationErrors)
{
    // [FTV-510]
    EXPECT_NE(make_error(write_config("missing", config(R"(,"load_gain":{"positive":1})"))).find("negative"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("zero", config(R"(,"load_gain":{"positive":0,"negative":1})"))).find("positive"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("lagneg", config(R"(,"load_gain":{"positive":1,"negative":1,"lag_s":-1})")))
                  .find("lag_s"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("unknown", config(R"(,"load_gain":{"positive":1,"negative":1,"x":1})"))).find("x"),
              std::string::npos);
}

TEST(ForceTorqueVibrationLoad, DescriptorVersion)
{
    // the host-model descriptor follows the header version (2 since the joint loads, 3 since the sensor orientation)
    EXPECT_EQ(sa::kHostModelImplementationVersion, 3u);
}

}  // namespace
