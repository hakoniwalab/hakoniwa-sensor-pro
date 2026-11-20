// Tests of a mode's level while moving (FTV-230, FTV-235, FTV-360, FTV-510).
#include <gtest/gtest.h>

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
    const std::string path = ::testing::TempDir() + "/ft_vibration_motion_" + name + ".json";
    std::ofstream(path) << body;
    return path;
}

std::unique_ptr<sa::IForceTorqueVibration> make(const std::string& path, std::uint64_t seed = 5)
{
    sa::ForceTorqueVibrationSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStepNs;
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
    EXPECT_FALSE(ms::create_force_torque_vibration(setup, error)) << path;
    return error;
}

// Gear ratio 10, f_ref 50 Hz: activity = 10 |qdot| / (2 pi 50) = |qdot| / (10 pi).
double qdot_for_activity(double activity) { return activity * 10.0 * kPi; }

std::string mode_config(const std::string& extra)
{
    return R"({"type":"force_torque_vibration","joints":[{"name":"a","gear_ratio":10}],
      "modes":[{"frequency_hz":20,"damping":0.1)" + extra + "}]}";
}

// Standard deviation of Fz over n steps (after a settling of 2000 steps) at a constant activity.
double sd_fz(sa::IForceTorqueVibration& m, double activity, int n = 40000)
{
    const double v = qdot_for_activity(activity);
    std::string error;
    double s = 0, s2 = 0;
    for (int k = 0; k < n + 2000; ++k) {
        sa::Wrench w;
        EXPECT_TRUE(m.step(static_cast<std::uint64_t>(k) * kStepNs, {v}, w, error)) << error;
        if (k < 2000) continue;
        s += w.force.z;
        s2 += w.force.z * w.force.z;
    }
    const double mean = s / n;
    return std::sqrt(s2 / n - mean * mean);
}

TEST(ForceTorqueVibrationMotion, LevelDoesNotDependOnSpeed)
{
    // [FTV-230] per_motion: the same level at activity 0.1 and 1.0 (both above the ramp 0.02)
    const auto path = write_config("speed", mode_config(R"(,"per_motion":{"fz":0.3})"));
    auto slow = make(path);
    auto fast = make(path);
    const double a = sd_fz(*slow, 0.1), b = sd_fz(*fast, 1.0);
    EXPECT_NEAR(a, 0.3, 0.02);
    EXPECT_NEAR(b, 0.3, 0.02);
}

TEST(ForceTorqueVibrationMotion, RampFromRest)
{
    // [FTV-235] below the ramp the level rises linearly: activity 0.005 with ramp 0.02 gives a quarter
    auto quarter = make(write_config("ramp", mode_config(R"(,"per_motion":{"fz":0.4})")));
    EXPECT_NEAR(sd_fz(*quarter, 0.005), 0.1, 0.01);
    auto custom = make(write_config("ramp_custom", mode_config(R"(,"per_motion":{"fz":0.4},"motion_ramp":0.01)")));
    EXPECT_NEAR(sd_fz(*custom, 0.005), 0.2, 0.015);
    auto rest = make(write_config("rest", mode_config(R"(,"per_motion":{"fz":0.4})")));
    EXPECT_NEAR(sd_fz(*rest, 0.0, 5000), 0.0, 1e-12);  // nothing at rest without at_rest
}

TEST(ForceTorqueVibrationMotion, AddsToAtRestAndPerActivity)
{
    // [FTV-230] sigma = s0 + s1 activity + s2 min(1, activity / ramp)
    auto m = make(write_config("sum", mode_config(R"(,"at_rest":{"fz":0.1},"per_activity":{"fz":0.2},"per_motion":{"fz":0.3})")));
    EXPECT_NEAR(sd_fz(*m, 0.5), 0.1 + 0.2 * 0.5 + 0.3, 0.03);
}

TEST(ForceTorqueVibrationMotion, ConfigWithoutPerMotionIsUnchanged)
{
    // compatibility: a mode without per_motion gives the same output as before (per_motion 0)
    auto a = make(write_config("plain", mode_config(R"(,"at_rest":{"fz":0.1},"per_activity":{"fz":0.2})")));
    auto b = make(write_config("zero", mode_config(R"(,"at_rest":{"fz":0.1},"per_activity":{"fz":0.2},"per_motion":{"fz":0},"motion_ramp":0.5)")));
    std::string error;
    for (int k = 0; k < 500; ++k) {
        sa::Wrench wa, wb;
        ASSERT_TRUE(a->step(k * kStepNs, {qdot_for_activity(0.3)}, wa, error));
        ASSERT_TRUE(b->step(k * kStepNs, {qdot_for_activity(0.3)}, wb, error));
        ASSERT_EQ(wa.force.z, wb.force.z);
    }
}

TEST(ForceTorqueVibrationMotion, RunMultiplierScalesTheLevelWhileMoving)
{
    // [FTV-360] the level while moving is scaled by the run multiplier; at_rest is not
    const std::string with = mode_config(R"(,"at_rest":{"fy":0.2},"per_motion":{"fz":0.3},"run_log_sigma":1.0)");
    const std::string without = mode_config(R"(,"at_rest":{"fy":0.2},"per_motion":{"fz":0.3})");
    auto a = make(write_config("run_a", with), 9);
    auto b = make(write_config("run_b", without), 9);
    std::string error;
    double ratio = std::nan("");
    for (int k = 0; k < 2000; ++k) {
        sa::Wrench wa, wb;
        ASSERT_TRUE(a->step(k * kStepNs, {qdot_for_activity(0.5)}, wa, error));
        ASSERT_TRUE(b->step(k * kStepNs, {qdot_for_activity(0.5)}, wb, error));
        ASSERT_EQ(wa.force.y, wb.force.y);                       // at_rest unchanged
        if (std::fabs(wb.force.z) > 1e-3) {
            const double r = wa.force.z / wb.force.z;              // the multiplier, constant over the run
            if (std::isnan(ratio)) ratio = r;
            ASSERT_NEAR(r, ratio, 1e-9);
        }
    }
    EXPECT_GT(std::fabs(std::log(ratio)), 1e-3);
}

TEST(ForceTorqueVibrationMotion, ConfigurationErrors)
{
    // [FTV-510]
    EXPECT_NE(make_error(write_config("ramp0", mode_config(R"(,"per_motion":{"fz":0.3},"motion_ramp":0)"))).find("motion_ramp"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("rampneg", mode_config(R"(,"motion_ramp":-1)"))).find("motion_ramp"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("neg", mode_config(R"(,"per_motion":{"fz":-0.1})"))).find("per_motion"),
              std::string::npos);
}

}  // namespace
