// Tests of the run-to-run variation of the force-torque vibration model (FTV-350..380, FTV-510).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <numeric>
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
    const std::string path = ::testing::TempDir() + "/ft_vibration_run_" + name + ".json";
    std::ofstream(path) << body;
    return path;
}

std::unique_ptr<sa::IForceTorqueVibration> make(const std::string& path, std::uint64_t seed)
{
    sa::ForceTorqueVibrationSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStepNs;
    setup.vehicle_name = "arm";
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
    EXPECT_FALSE(ms::create_force_torque_vibration(setup, error)) << path;
    return error;
}

// Runs the model and returns the largest |value| per axis (Fx, Fy, Fz, Tx).
std::vector<double> peaks(sa::IForceTorqueVibration& model, const std::vector<double>& velocity, int steps = 1000)
{
    std::vector<double> p(4, 0.0);
    std::string error;
    for (int k = 0; k < steps; ++k) {
        sa::Wrench w;
        EXPECT_TRUE(model.step(static_cast<std::uint64_t>(k) * kStepNs, velocity, w, error)) << error;
        p[0] = std::max(p[0], std::fabs(w.force.x));
        p[1] = std::max(p[1], std::fabs(w.force.y));
        p[2] = std::max(p[2], std::fabs(w.force.z));
        p[3] = std::max(p[3], std::fabs(w.torque.x));
    }
    return p;
}

// Two joints, each with one order (gear 10, order 1: 10 Hz at 2*pi rad/s) of amplitude 1 on its own axis.
std::string two_joints(const std::string& run_a, const std::string& run_b)
{
    return R"({"type":"force_torque_vibration","joints":[
      {"name":"a","gear_ratio":10)" + run_a + R"(,"orders":[{"order":1,"amplitude":{"fx":1}}]},
      {"name":"b","gear_ratio":10)" + run_b + R"(,"orders":[{"order":1,"amplitude":{"fy":1}}]}]})";
}

const std::vector<double> kTurning{2 * kPi, 2 * kPi};

TEST(ForceTorqueVibrationRun, ZeroSigmaIsTheModelWithoutVariation)
{
    // [FTV-350] sigma 0 (given or not) gives m = 1
    auto plain = make(write_config("plain", two_joints("", "")), 5);
    auto zero = make(write_config("zero", two_joints(R"(,"run_log_sigma":0)", R"(,"run_log_sigma":0,"run_group":"g")")), 5);
    std::string error;
    for (int k = 0; k < 500; ++k) {
        sa::Wrench a, b;
        ASSERT_TRUE(plain->step(k * kStepNs, kTurning, a, error));
        ASSERT_TRUE(zero->step(k * kStepNs, kTurning, b, error));
        ASSERT_EQ(a.force.x, b.force.x);
        ASSERT_EQ(a.force.y, b.force.y);
    }
}

TEST(ForceTorqueVibrationRun, MultiplierIsLogNormalWithTheConfiguredSigma)
{
    // [FTV-350] [FTV-360] over runs (seeds), ln(order amplitude) ~ N(0, sigma)
    const auto path = write_config("lognormal", two_joints(R"(,"run_log_sigma":0.5)", ""));
    std::vector<double> logs;
    for (std::uint64_t seed = 1; seed <= 400; ++seed) {
        auto m = make(path, seed);
        const auto p = peaks(*m, kTurning);
        logs.push_back(std::log(p[0]));
        EXPECT_NEAR(p[1], 1.0, 2e-3);  // the other joint has no variation
    }
    const double mean = std::accumulate(logs.begin(), logs.end(), 0.0) / logs.size();
    double var = 0;
    for (double v : logs) var += (v - mean) * (v - mean);
    const double sd = std::sqrt(var / (logs.size() - 1));
    EXPECT_NEAR(mean, 0.0, 0.08);  // 4 standard errors (0.5 / sqrt(400) = 0.025)
    EXPECT_NEAR(sd, 0.5, 0.07);
}

TEST(ForceTorqueVibrationRun, GroupSharesOneDrawWithItsOwnSigma)
{
    // [FTV-370] same z, own sigma: ln m_b = 2 ln m_a when sigma_b = 2 sigma_a
    const auto path = write_config("group", two_joints(R"(,"run_log_sigma":0.3,"run_group":"drive")",
                                                       R"(,"run_log_sigma":0.6,"run_group":"drive")"));
    int varied = 0;
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        auto m = make(path, seed);
        const auto p = peaks(*m, kTurning);
        EXPECT_NEAR(std::log(p[1]), 2.0 * std::log(p[0]), 5e-3) << seed;
        if (std::fabs(std::log(p[0])) > 0.01) ++varied;
    }
    EXPECT_GE(varied, 18);  // the shared multiplier is not 1
}

TEST(ForceTorqueVibrationRun, UngroupedComponentsDrawSeparately)
{
    // [FTV-370] without a group each component has its own z
    const auto path = write_config("separate", two_joints(R"(,"run_log_sigma":0.5)", R"(,"run_log_sigma":0.5)"));
    int differ = 0;
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        auto m = make(path, seed);
        const auto p = peaks(*m, kTurning);
        if (std::fabs(std::log(p[0]) - std::log(p[1])) > 0.01) ++differ;
    }
    EXPECT_GE(differ, 18);
}

TEST(ForceTorqueVibrationRun, ChangingOneSigmaKeepsTheOtherMultiplier)
{
    // [FTV-380] a z is drawn for every component, so joint a's multiplier does not depend on joint b's sigma
    auto with0 = make(write_config("other0", two_joints(R"(,"run_log_sigma":0.5)", R"(,"run_log_sigma":0)")), 9);
    auto with1 = make(write_config("other1", two_joints(R"(,"run_log_sigma":0.5)", R"(,"run_log_sigma":1.0)")), 9);
    EXPECT_NEAR(peaks(*with0, kTurning)[0], peaks(*with1, kTurning)[0], 1e-12);
}

TEST(ForceTorqueVibrationRun, LevelAtRestIsNotScaled)
{
    // [FTV-360] a mode's at_rest is not scaled; at rest (activity 0) the output equals sigma 0
    const std::string base = R"({"type":"force_torque_vibration","joints":[{"name":"a","gear_ratio":10}],
      "modes":[{"frequency_hz":20,"damping":0.1,"at_rest":{"fz":0.2},"per_activity":{"fz":1})";
    auto s0 = make(write_config("rest0", base + "}]}"), 4);
    auto s2 = make(write_config("rest2", base + R"(,"run_log_sigma":2})" + "]}"), 4);
    std::string error;
    for (int k = 0; k < 300; ++k) {
        sa::Wrench a, b;
        ASSERT_TRUE(s0->step(k * kStepNs, {0.0}, a, error));
        ASSERT_TRUE(s2->step(k * kStepNs, {0.0}, b, error));
        ASSERT_EQ(a.force.z, b.force.z);
    }
}

TEST(ForceTorqueVibrationRun, BroadbandAndModeGrowthAreScaled)
{
    // [FTV-360] growth s1 and sigma_b scale with m; with a shared group both scale by the same factor
    const std::string cfg = R"({"type":"force_torque_vibration","joints":[{"name":"a","gear_ratio":10}],
      "modes":[{"frequency_hz":20,"damping":0.1,"per_activity":{"fz":1},"run_log_sigma":0.7,"run_group":"g"}],
      "broadband":{"density_per_activity":{"fy":1e-4},"run_log_sigma":0.7,"run_group":"g"}})";
    const std::string ref = R"({"type":"force_torque_vibration","joints":[{"name":"a","gear_ratio":10}],
      "modes":[{"frequency_hz":20,"damping":0.1,"per_activity":{"fz":1}}],
      "broadband":{"density_per_activity":{"fy":1e-4}}})";
    auto varied = make(write_config("grow", cfg), 11);
    auto plain = make(write_config("grow_ref", ref), 11);
    std::string error;
    std::vector<double> rz, ry;
    for (int k = 0; k < 2000; ++k) {
        sa::Wrench a, b;
        ASSERT_TRUE(varied->step(k * kStepNs, {2 * kPi}, a, error));
        ASSERT_TRUE(plain->step(k * kStepNs, {2 * kPi}, b, error));
        if (std::fabs(b.force.z) > 1e-6) rz.push_back(a.force.z / b.force.z);
        if (std::fabs(b.force.y) > 1e-9) ry.push_back(a.force.y / b.force.y);
    }
    ASSERT_FALSE(rz.empty());
    ASSERT_FALSE(ry.empty());
    // the random sequences are the same streams, so the ratio is the constant multiplier
    for (double r : rz) EXPECT_NEAR(r, rz.front(), 1e-9);
    for (double r : ry) EXPECT_NEAR(r, rz.front(), 1e-9);
    EXPECT_GT(std::fabs(std::log(rz.front())), 1e-6);
}

TEST(ForceTorqueVibrationRun, ResetDrawsTheSameMultipliers)
{
    // [FTV-380] [FTV-040]
    auto m = make(write_config("reset", two_joints(R"(,"run_log_sigma":0.8)", R"(,"run_log_sigma":0.8)")), 3);
    const auto first = peaks(*m, kTurning);
    std::string error;
    ASSERT_TRUE(m->reset(error)) << error;
    const auto again = peaks(*m, kTurning);
    EXPECT_EQ(first, again);
}

TEST(ForceTorqueVibrationRun, ConfigurationErrors)
{
    // [FTV-510]
    EXPECT_NE(make_error(write_config("neg", two_joints(R"(,"run_log_sigma":-0.1)", ""))).find("run_log_sigma"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("empty", two_joints(R"(,"run_group":"")", ""))).find("run_group"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("type", two_joints(R"(,"run_group":3)", ""))).find("run_group"),
              std::string::npos);
    EXPECT_NE(make_error(write_config("bb", R"({"type":"force_torque_vibration","broadband":{"run_log_sigma":"x"}})"))
                  .find("run_log_sigma"),
              std::string::npos);
}

}  // namespace
