#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/mujoco_sensors/force_torque_sensor_adapter.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStepNs = 1'000'000;

std::string write_config(const std::string& name, const std::string& extra)
{
    const std::string path = ::testing::TempDir() + "/public_ft_" + name + ".json";
    std::ofstream(path) << "{\"type\":\"force_torque\",\"update_rate\":1000,"
                          "\"range\":{\"fxy\":200,\"fz\":400,\"txy\":20,\"tz\":40}"
                       << (extra.empty() ? "" : "," + extra) << "}";
    return path;
}

std::unique_ptr<sa::IForceTorqueSensorAdapter> make_adapter(const std::string& path, std::uint64_t seed = 1234)
{
    sa::ForceTorqueAdapterSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStepNs;
    setup.vehicle_name = "public-test-arm";
    setup.sensor_name = "public-test-ft";
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    auto adapter = ms::create_force_torque_sensor_adapter(setup, error);
    EXPECT_NE(adapter, nullptr) << error;
    return adapter;
}

sa::Wrench wrench(const std::array<double, 6>& values)
{
    sa::Wrench result;
    result.force = {values[0], values[1], values[2]};
    result.torque = {values[3], values[4], values[5]};
    return result;
}

double axis(const sa::Wrench& value, std::size_t index)
{
    const std::array<double, 6> values = {value.force.x, value.force.y, value.force.z,
                                          value.torque.x, value.torque.y, value.torque.z};
    return values[index];
}

sa::ForceTorqueSample step(sa::IForceTorqueSensorAdapter& adapter, std::uint64_t index,
                           const std::array<double, 6>& values)
{
    sa::ForceTorqueTruth truth;
    truth.sim_time_ns = index * kStepNs;
    truth.wrench = wrench(values);
    sa::ForceTorqueSample sample;
    std::string error;
    EXPECT_TRUE(adapter.step(truth, sample, error)) << error;
    return sample;
}

void expect_same_wrench(const sa::Wrench& left, const sa::Wrench& right)
{
    for (std::size_t i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(axis(left, i), axis(right, i)) << "axis " << i;
}

double mean(const std::vector<double>& values)
{
    double total = 0.0;
    for (double value : values) total += value;
    return total / static_cast<double>(values.size());
}

double sample_standard_deviation(const std::vector<double>& values, double average)
{
    double squared_error = 0.0;
    for (double value : values) squared_error += (value - average) * (value - average);
    return std::sqrt(squared_error / static_cast<double>(values.size() - 1));
}

}  // namespace

// [FTS-ADP-230]
TEST(PublicForceTorqueSpec, ConvertsNormalisedValuesBackToPhysicalUnits)
{
    auto adapter = make_adapter(write_config("physical_units", ""));
    ASSERT_NE(adapter, nullptr);

    // Each truth is u = 0.25 of its axis range, so y = u*R is
    // {50 N, 50 N, 100 N, 5 N m, 5 N m, 10 N m}.
    const std::array<double, 6> expected = {50.0, 50.0, 100.0, 5.0, 5.0, 10.0};
    const auto sample = step(*adapter, 0, expected);
    for (std::size_t i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(axis(sample.wrench, i), expected[i]) << "axis " << i;
}

// [FTS-ADP-260]
TEST(PublicForceTorqueSpec, AddsConfiguredGaussianNoise)
{
    constexpr std::size_t kSamples = 25'000;
    constexpr double kTruth = 40.0;
    constexpr double kSigma = 2.0;
    auto adapter = make_adapter(write_config(
        "noise", "\"noise\":{\"definition\":\"1sigma\",\"rate_hz\":1000,"
                 "\"fxy\":2,\"fz\":2,\"txy\":0.2,\"tz\":0.2}"));
    ASSERT_NE(adapter, nullptr);

    std::vector<double> values;
    values.reserve(kSamples);
    for (std::size_t i = 0; i < kSamples; ++i) {
        const auto sample = step(*adapter, i, {kTruth, 0.0, 0.0, 0.0, 0.0, 0.0});
        ASSERT_TRUE(sample.updated);
        values.push_back(sample.wrench.force.x);
    }

    // rate_hz/2 and update_rate/2 are both 500 Hz, hence section 2.4 gives sigma = 2 N.
    const double average = mean(values);
    EXPECT_LT(std::fabs(average - kTruth), 4.0 * kSigma / std::sqrt(static_cast<double>(kSamples)));
    EXPECT_NEAR(sample_standard_deviation(values, average), kSigma, 0.03 * kSigma);
}

// [FTS-ADP-280]
TEST(PublicForceTorqueSpec, SubtractsTheStoredTare)
{
    auto adapter = make_adapter(write_config("tare", ""));
    ASSERT_NE(adapter, nullptr);

    const auto tare_sample = step(*adapter, 0, {30.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(tare_sample.wrench.force.x, 30.0);
    std::string error;
    ASSERT_TRUE(adapter->tare(error)) << error;

    // The stored tare is 30 N, so the next pre-tare value 45 N becomes 45 - 30 = 15 N.
    const auto measured = step(*adapter, 1, {45.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(measured.wrench.force.x, 15.0);
}

// [FTS-ADP-520]
TEST(PublicForceTorqueSpec, AcceptsEquivalentAxisGroupFormsAndScalarNonlinearity)
{
    auto percent_bias = make_adapter(write_config(
        "bias_percent", "\"bias\":{\"fxy\":\"25%\",\"fz\":\"25%\","
                        "\"txy\":\"25%\",\"tz\":\"25%\"}"));
    auto numeric_bias = make_adapter(write_config(
        "bias_number", "\"bias\":{\"fxy\":50,\"fz\":100,\"txy\":5,\"tz\":10}"));
    ASSERT_NE(percent_bias, nullptr);
    ASSERT_NE(numeric_bias, nullptr);
    for (std::uint64_t i = 0; i < 4; ++i) {
        const auto from_percent = step(*percent_bias, i, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        const auto from_number = step(*numeric_bias, i, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        expect_same_wrench(from_percent.wrench, from_number.wrench);
    }

    auto scalar = make_adapter(write_config("nonlinearity_scalar", "\"nonlinearity\":\"2%\""));
    auto grouped = make_adapter(write_config(
        "nonlinearity_groups", "\"nonlinearity\":{\"fxy\":\"2%\",\"fz\":\"2%\","
                               "\"txy\":\"2%\",\"tz\":\"2%\"}"));
    ASSERT_NE(scalar, nullptr);
    ASSERT_NE(grouped, nullptr);
    const std::array<double, 6> truth = {50.0, -50.0, 100.0, -5.0, 5.0, -10.0};
    expect_same_wrench(step(*scalar, 0, truth).wrench, step(*grouped, 0, truth).wrench);
}

// [FTS-ADP-530]
TEST(PublicForceTorqueSpec, AcceptsEquivalentCalibrationFractionForms)
{
    auto percent = make_adapter(write_config(
        "calibration_percent", "\"scale_error\":\"5%\",\"crosstalk\":\"5%\""));
    auto fraction = make_adapter(write_config(
        "calibration_fraction", "\"scale_error\":0.05,\"crosstalk\":0.05"));
    ASSERT_NE(percent, nullptr);
    ASSERT_NE(fraction, nullptr);

    const std::array<double, 6> truth = {20.0, -30.0, 40.0, -2.0, 3.0, -4.0};
    for (std::uint64_t i = 0; i < 4; ++i)
        expect_same_wrench(step(*percent, i, truth).wrench, step(*fraction, i, truth).wrench);
}
