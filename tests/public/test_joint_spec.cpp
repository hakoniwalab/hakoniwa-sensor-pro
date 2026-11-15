#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/mujoco_sensors/joint_sensor_adapter.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStepNs = 1'000'000;
constexpr double kPi = 3.141592653589793238462643383279502884;

std::string write_config(const std::string& name, const std::string& extra)
{
    const std::string path = ::testing::TempDir() + "/public_joint_" + name + ".json";
    std::ofstream(path) << "{\"type\":\"joint\",\"update_rate\":1000"
                       << (extra.empty() ? "" : "," + extra) << "}";
    return path;
}

std::unique_ptr<sa::IJointSensorAdapter> make_adapter(const std::string& path, std::uint64_t seed = 5678)
{
    sa::JointSensorAdapterSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStepNs;
    setup.vehicle_name = "public-test-arm";
    setup.sensor_name = "public-test-joint";
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    auto adapter = ms::create_joint_sensor_adapter(setup, error);
    EXPECT_NE(adapter, nullptr) << error;
    return adapter;
}

sa::JointSensorTruth truth(std::uint64_t time_ns, double position, double velocity = 0.0,
                           double actuator_torque = 0.0, double transmitted_torque = 0.0)
{
    sa::JointSensorTruth value;
    value.sim_time_ns = time_ns;
    value.position = position;
    value.velocity = velocity;
    value.actuator_torque = actuator_torque;
    value.transmitted_torque = transmitted_torque;
    return value;
}

sa::JointSensorSample step(sa::IJointSensorAdapter& adapter, const sa::JointSensorTruth& input)
{
    sa::JointSensorSample sample;
    std::string error;
    EXPECT_TRUE(adapter.step(input, sample, error)) << error;
    return sample;
}

void expect_same_sample(const sa::JointSensorSample& left, const sa::JointSensorSample& right)
{
    EXPECT_EQ(left.updated, right.updated);
    EXPECT_EQ(left.sample_time_ns, right.sample_time_ns);
    EXPECT_DOUBLE_EQ(left.position, right.position);
    EXPECT_DOUBLE_EQ(left.velocity, right.velocity);
    EXPECT_DOUBLE_EQ(left.effort, right.effort);
    EXPECT_EQ(left.has_effort, right.has_effort);
    EXPECT_EQ(left.encoder_count, right.encoder_count);
    EXPECT_EQ(left.flags, right.flags);
}

}  // namespace

// [JNT-ADP-020]
TEST(PublicJointSpec, RejectsOffGridRepeatedAndBackwardTimesWithoutChangingState)
{
    const std::string path = write_config(
        "time", "\"encoder\":{\"bits\":24,\"noise_counts\":2},"
                "\"torque\":{\"source\":\"sensor\",\"range\":100,\"noise\":0.25}");
    auto checked = make_adapter(path);
    auto reference = make_adapter(path);
    ASSERT_NE(checked, nullptr);
    ASSERT_NE(reference, nullptr);

    sa::JointSensorSample ignored;
    std::string error;
    EXPECT_FALSE(checked->step(truth(kStepNs / 2, 0.1, 0.2, 0.3, 0.4), ignored, error));
    expect_same_sample(step(*checked, truth(0, 0.1, 0.2, 0.3, 0.4)),
                       step(*reference, truth(0, 0.1, 0.2, 0.3, 0.4)));

    EXPECT_FALSE(checked->step(truth(0, 9.0, 9.0, 9.0, 9.0), ignored, error));
    expect_same_sample(step(*checked, truth(kStepNs, 0.2, 0.3, 0.4, 0.5)),
                       step(*reference, truth(kStepNs, 0.2, 0.3, 0.4, 0.5)));

    EXPECT_FALSE(checked->step(truth(kStepNs / 2, 8.0, 8.0, 8.0, 8.0), ignored, error));
    EXPECT_FALSE(checked->step(truth(kStepNs + kStepNs / 2, 7.0, 7.0, 7.0, 7.0), ignored, error));
    expect_same_sample(step(*checked, truth(2 * kStepNs, 0.3, 0.4, 0.5, 0.6)),
                       step(*reference, truth(2 * kStepNs, 0.3, 0.4, 0.5, 0.6)));
}

// [JNT-ADP-110]
TEST(PublicJointSpec, FloorsDelayAtStepBoundariesAndUsesTimeZeroHistory)
{
    struct DelayCase {
        const char* name;
        const char* delay;
        std::uint64_t steps;
    };
    const std::array<DelayCase, 3> cases = {{{"below", "0.001999", 1},
                                             {"exact", "0.002", 2},
                                             {"above", "0.002001", 2}}};
    for (const auto& test_case : cases) {
        auto adapter = make_adapter(write_config(test_case.name, "\"delay\":" + std::string(test_case.delay)));
        ASSERT_NE(adapter, nullptr) << test_case.name;
        for (std::uint64_t k = 0; k < 6; ++k) {
            const auto sample = step(*adapter, truth(k * kStepNs, 10.0 + static_cast<double>(k),
                                                     20.0 + static_cast<double>(k)));
            const std::uint64_t source = k < test_case.steps ? 0 : k - test_case.steps;
            EXPECT_DOUBLE_EQ(sample.position, 10.0 + static_cast<double>(source)) << test_case.name << " k=" << k;
            EXPECT_DOUBLE_EQ(sample.velocity, 20.0 + static_cast<double>(source)) << test_case.name << " k=" << k;
        }
    }
}

// [JNT-ADP-520]
TEST(PublicJointSpec, AcceptsEquivalentAngleVelocityAndFractionUnits)
{
    const std::array<std::string, 4> angle_forms = {
        "0.017453292519943295", "\"1deg\"", "\"60arcmin\"", "\"3600arcsec\""};
    std::vector<std::unique_ptr<sa::IJointSensorAdapter>> angle_adapters;
    for (std::size_t i = 0; i < angle_forms.size(); ++i) {
        angle_adapters.push_back(make_adapter(write_config(
            "angle_" + std::to_string(i), "\"encoder\":{\"bits\":40,\"offset\":" + angle_forms[i] + "}")));
        ASSERT_NE(angle_adapters.back(), nullptr);
    }
    for (std::uint64_t k = 0; k < 4; ++k) {
        const auto input = truth(k * kStepNs, 0.25 + 0.01 * static_cast<double>(k));
        const auto expected = step(*angle_adapters[0], input);
        for (std::size_t i = 1; i < angle_adapters.size(); ++i)
            expect_same_sample(expected, step(*angle_adapters[i], input));
    }

    auto radians_per_second = make_adapter(write_config(
        "velocity_rad", "\"velocity\":{\"method\":\"true\",\"lsb\":6.283185307179586}"));
    auto revolutions_per_minute = make_adapter(write_config(
        "velocity_rpm", "\"velocity\":{\"method\":\"true\",\"lsb\":\"60rpm\"}"));
    ASSERT_NE(radians_per_second, nullptr);
    ASSERT_NE(revolutions_per_minute, nullptr);
    for (std::uint64_t k = 0; k < 4; ++k) {
        const auto input = truth(k * kStepNs, 0.0, 8.0);
        const auto from_radians = step(*radians_per_second, input);
        EXPECT_DOUBLE_EQ(from_radians.velocity, 2.0 * kPi);
        expect_same_sample(from_radians, step(*revolutions_per_minute, input));
    }

    auto percent = make_adapter(write_config(
        "fraction_percent", "\"torque\":{\"source\":\"sensor\",\"range\":100,\"scale_error\":\"5%\"}"));
    auto fraction = make_adapter(write_config(
        "fraction_number", "\"torque\":{\"source\":\"sensor\",\"range\":100,\"scale_error\":0.05}"));
    ASSERT_NE(percent, nullptr);
    ASSERT_NE(fraction, nullptr);
    for (std::uint64_t k = 0; k < 4; ++k) {
        const auto input = truth(k * kStepNs, 0.0, 0.0, 0.0, 20.0 + static_cast<double>(k));
        expect_same_sample(step(*percent, input), step(*fraction, input));
    }
}
