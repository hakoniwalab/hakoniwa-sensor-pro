// Tests of the share of the level at rest kept while moving (FTV-232) and of modes in world axes with the
// sensor orientation (FTV-026, FTV-027, FTV-250, FTV-255, FTV-510).
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
    const std::string path = ::testing::TempDir() + "/ft_vibration_world_" + name + ".json";
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

// Gear ratio 10, f_ref 50 Hz: activity = |qdot| / (10 pi).
double qdot_for_activity(double activity) { return activity * 10.0 * kPi; }

std::string mode_config(const std::string& extra)
{
    return R"({"type":"force_torque_vibration","joints":[{"name":"a","gear_ratio":10}],
      "modes":[{"frequency_hz":20,"damping":0.1)" + extra + "}]}";
}

sa::Quaternion about_y(double angle)  // rotation from the world to the sensor axes
{
    return {std::cos(angle / 2), 0.0, std::sin(angle / 2), 0.0};
}

struct Sd {
    double fx, fy, fz, tx, ty, tz;
};

// Standard deviations over n steps (after 2000 settling steps) at a constant activity and orientation.
Sd run(sa::IForceTorqueVibration& m, double activity, const sa::Quaternion* q, int n = 40000)
{
    const double v = qdot_for_activity(activity);
    std::string error;
    double s[6] = {}, s2[6] = {};
    for (int k = 0; k < n + 2000; ++k) {
        sa::Wrench w;
        const auto t = static_cast<std::uint64_t>(k) * kStepNs;
        const bool ok = q ? m.step(t, {v}, {}, *q, w, error) : m.step(t, {v}, w, error);
        EXPECT_TRUE(ok) << error;
        if (k < 2000) continue;
        const double a[6] = {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z};
        for (int i = 0; i < 6; ++i) {
            s[i] += a[i];
            s2[i] += a[i] * a[i];
        }
    }
    double r[6];
    for (int i = 0; i < 6; ++i) r[i] = std::sqrt(s2[i] / n - (s[i] / n) * (s[i] / n));
    return {r[0], r[1], r[2], r[3], r[4], r[5]};
}

}  // namespace

TEST(ForceTorqueVibrationWorld, AtRestShareWhileMoving)
{
    // [FTV-232] k = 0: the level at rest is gone once moving; k = 0.5: half of it; at rest it is unchanged
    auto gone = make(write_config("k0", mode_config(R"(,"at_rest":{"fz":0.2},"at_rest_while_moving":0)")));
    EXPECT_NEAR(run(*gone, 0.0, nullptr).fz, 0.2, 0.01);
    gone->reset(*new std::string);
    EXPECT_NEAR(run(*gone, 0.5, nullptr).fz, 0.0, 1e-12);
    auto half = make(write_config("k05", mode_config(R"(,"at_rest":{"fz":0.2},"at_rest_while_moving":0.5)")));
    EXPECT_NEAR(run(*half, 0.5, nullptr).fz, 0.1, 0.005);
    // below the ramp (activity 0.01 of 0.02): 1 - (1 - 0) * 0.5 = half of the level
    auto ramp = make(write_config("ramp", mode_config(R"(,"at_rest":{"fz":0.2},"at_rest_while_moving":0)")));
    EXPECT_NEAR(run(*ramp, 0.01, nullptr).fz, 0.1, 0.005);
}

TEST(ForceTorqueVibrationWorld, DefaultShareKeepsTheOutput)
{
    // [FTV-232] without the key, and with k = 1, the output is the same as before, sample by sample
    auto a = make(write_config("plain", mode_config(R"(,"at_rest":{"fz":0.1},"per_motion":{"fx":0.3})")));
    auto b = make(write_config("k1", mode_config(R"(,"at_rest":{"fz":0.1},"per_motion":{"fx":0.3},"at_rest_while_moving":1)")));
    std::string error;
    for (int k = 0; k < 3000; ++k) {
        sa::Wrench wa, wb;
        const double v = qdot_for_activity(k < 1500 ? 0.0 : 0.4);
        ASSERT_TRUE(a->step(static_cast<std::uint64_t>(k) * kStepNs, {v}, wa, error));
        ASSERT_TRUE(b->step(static_cast<std::uint64_t>(k) * kStepNs, {v}, wb, error));
        ASSERT_EQ(wa.force.x, wb.force.x);
        ASSERT_EQ(wa.force.z, wb.force.z);
    }
}

TEST(ForceTorqueVibrationWorld, WorldModeFollowsTheOrientation)
{
    // [FTV-250] [FTV-255] a world mode along world y; the sensor turned about its y axis keeps it on y,
    // a sensor turned about x by 90 degrees sees it on z
    const std::string cfg = mode_config(R"(,"frame":"world","per_motion":{"fy":0.3})");
    auto m = make(write_config("y", cfg));
    const sa::Quaternion turned_y = about_y(1.0);
    Sd s = run(*m, 0.5, &turned_y);
    EXPECT_NEAR(s.fy, 0.3, 0.015);
    EXPECT_LT(s.fx, 1e-12);
    EXPECT_LT(s.fz, 1e-12);
    auto n = make(write_config("x90", cfg));
    const sa::Quaternion about_x{std::cos(kPi / 4), std::sin(kPi / 4), 0.0, 0.0};
    s = run(*n, 0.5, &about_x);
    EXPECT_NEAR(s.fz, 0.3, 0.015);
    EXPECT_LT(s.fy, 1e-9);
}

TEST(ForceTorqueVibrationWorld, WorldModeSplitsWithTheAngle)
{
    // [FTV-255] world x seen by a sensor turned by theta about y: on x with cos(theta) and on z with sin(theta)
    const double theta = kPi / 3;
    auto m = make(write_config("split", mode_config(R"(,"frame":"world","per_motion":{"fx":0.4,"tx":0.02})")));
    const sa::Quaternion q = about_y(theta);
    const Sd s = run(*m, 0.5, &q);
    EXPECT_NEAR(s.fx, 0.4 * std::fabs(std::cos(theta)), 0.02);
    EXPECT_NEAR(s.fz, 0.4 * std::fabs(std::sin(theta)), 0.02);
    EXPECT_NEAR(s.tx, 0.02 * std::fabs(std::cos(theta)), 0.001);  // the torque turns the same way
    EXPECT_NEAR(s.tz, 0.02 * std::fabs(std::sin(theta)), 0.001);
}

TEST(ForceTorqueVibrationWorld, SensorModeIgnoresTheOrientation)
{
    // [FTV-026] [FTV-250] a sensor mode gives the same samples with any orientation, and through the older steps
    const std::string cfg = mode_config(R"(,"per_motion":{"fz":0.3})");
    auto a = make(write_config("s1", cfg));
    auto b = make(write_config("s2", cfg));
    EXPECT_FALSE(a->needs_sensor_orientation());
    std::string error;
    for (int k = 0; k < 2000; ++k) {
        sa::Wrench wa, wb;
        const auto t = static_cast<std::uint64_t>(k) * kStepNs;
        const double v = qdot_for_activity(0.3);
        ASSERT_TRUE(a->step(t, {v}, wa, error));
        ASSERT_TRUE(b->step(t, {v}, {}, about_y(0.1 * k), wb, error)) << error;
        ASSERT_EQ(wa.force.z, wb.force.z);
    }
}

TEST(ForceTorqueVibrationWorld, OrientationIsRequiredAndChecked)
{
    // [FTV-027] a world mode refuses the steps without the orientation, with no change of state
    auto m = make(write_config("need", mode_config(R"(,"frame":"world","per_motion":{"fy":0.3})")));
    EXPECT_TRUE(m->needs_sensor_orientation());
    sa::Wrench w;
    std::string error;
    EXPECT_FALSE(m->step(0, {1.0}, w, error));
    EXPECT_NE(error.find("orientation"), std::string::npos);
    EXPECT_FALSE(m->step(0, {1.0}, {}, w, error));
    // [FTV-026] not a unit quaternion, or not finite
    EXPECT_FALSE(m->step(0, {1.0}, {}, sa::Quaternion{1.0, 0.1, 0.0, 0.0}, w, error));
    EXPECT_FALSE(m->step(0, {1.0}, {}, sa::Quaternion{std::nan(""), 0.0, 0.0, 0.0}, w, error));
    // still at step 0
    EXPECT_TRUE(m->step(0, {1.0}, {}, sa::Quaternion{}, w, error)) << error;
}

TEST(ForceTorqueVibrationWorld, ConfigurationErrors)
{
    // [FTV-510]
    EXPECT_NE(make_error(write_config("frame_bad", mode_config(R"(,"frame":"tool")"))).find("frame"), std::string::npos);
    EXPECT_NE(make_error(write_config("frame_type", mode_config(R"(,"frame":1)"))).find("frame"), std::string::npos);
    EXPECT_NE(make_error(write_config("share_hi", mode_config(R"(,"at_rest_while_moving":1.5)"))).find("at_rest_while_moving"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("share_lo", mode_config(R"(,"at_rest_while_moving":-0.1)"))).find("at_rest_while_moving"),
              std::string::npos);
}
