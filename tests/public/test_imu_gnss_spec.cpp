#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hakoniwa/imu_gnss_sensors/hakoniwa_imu_sensor_adapter.hpp"
#include "hakoniwa/imu_gnss_sensors/hakoniwa_mag_sensor_adapter.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace hs = hakoniwa::drone_pro::sensors;

namespace {

using Json = nlohmann::json;

constexpr std::uint64_t kStepNs = 1'000'000;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr std::uint64_t kSeed = 0x5a17c3u;

Json identity_sensitivity()
{
    return {{"scaleErrorPct", {0.0, 0.0, 0.0}},
            {"crossAxisPct", {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}}};
}

Json imu_config(Json gyro, Json accel)
{
    return {{"components", {{"sensors", {{"imu", Json::object()}, {"gyro", std::move(gyro)},
                                           {"acc", std::move(accel)}}}}}};
}

Json mag_config(Json mag)
{
    return {{"components", {{"sensors", {{"imu", Json::object()}, {"mag", std::move(mag)}}}}}};
}

std::string write_config(const std::string& name, const Json& config)
{
    const std::string path = ::testing::TempDir() + "/public_imu_gnss_" + name + ".json";
    std::ofstream out(path);
    EXPECT_TRUE(out.good());
    out << config;
    EXPECT_TRUE(out.good());
    return path;
}

std::unique_ptr<sa::IImuSensorAdapter> make_imu(const std::string& name, const Json& gyro, const Json& accel)
{
    sa::ImuAdapterSetup setup;
    setup.config_path = write_config(name, imu_config(gyro, accel));
    setup.vehicle_name = "public-spec-drone";
    setup.vehicle_index = 2;
    setup.physics_step_ns = kStepNs;
    setup.has_random_seed = true;
    setup.random_seed = kSeed;
    std::string error;
    auto adapter = hs::create_hakoniwa_imu_sensor_adapter(setup, error);
    EXPECT_NE(adapter, nullptr) << error;
    return adapter;
}

std::unique_ptr<sa::IMagSensorAdapter> make_mag(const std::string& name, const Json& mag)
{
    sa::MagAdapterSetup setup;
    setup.config_path = write_config(name, mag_config(mag));
    setup.vehicle_name = "public-spec-drone";
    setup.vehicle_index = 2;
    setup.physics_step_ns = kStepNs;
    setup.has_random_seed = true;
    setup.random_seed = kSeed;
    std::string error;
    auto adapter = hs::create_hakoniwa_mag_sensor_adapter(setup, error);
    EXPECT_NE(adapter, nullptr) << error;
    return adapter;
}

sa::ImuTruth imu_truth(std::uint64_t step)
{
    const double k = static_cast<double>(step);
    sa::ImuTruth truth;
    truth.sim_time_ns = step * kStepNs;
    truth.angular_velocity = {0.123456789 + 0.01 * k, -0.234567891 - 0.02 * k,
                              0.345678912 + 0.03 * k};
    truth.specific_force = {1.234567891 + 0.04 * k, -2.345678912 - 0.05 * k,
                            9.456789123 + 0.06 * k};
    return truth;
}

sa::ImuSample step_imu(sa::IImuSensorAdapter& adapter, std::uint64_t step, const sa::ImuTruth& truth)
{
    sa::ImuTruth timed = truth;
    timed.sim_time_ns = step * kStepNs;
    sa::ImuSample sample;
    std::string error;
    EXPECT_TRUE(adapter.step(timed, sample, error)) << error;
    return sample;
}

sa::MagSample step_mag(sa::IMagSensorAdapter& adapter, std::uint64_t step)
{
    const double k = static_cast<double>(step);
    sa::MagTruth truth;
    truth.sim_time_ns = step * kStepNs;
    truth.magnetic_field = {21345.678901 + 0.7 * k, -4567.891234 - 0.5 * k,
                            48765.432109 + 0.3 * k};
    sa::MagSample sample;
    std::string error;
    EXPECT_TRUE(adapter.step(truth, sample, error)) << error;
    return sample;
}

void expect_vector_eq(const sa::Vector3& actual, const sa::Vector3& expected)
{
    EXPECT_EQ(actual.x, expected.x);
    EXPECT_EQ(actual.y, expected.y);
    EXPECT_EQ(actual.z, expected.z);
}

bool is_multiple(double value, double step)
{
    const double multiple = std::round(value / step);
    return std::fabs(value - multiple * step) <= std::fabs(step) * 1e-8;
}

}  // namespace

// T-COM-U2-020. [SEN-COM-140] The adapter creates a value only on a sample boundary and returns that same
// value, unchanged, on every step until the next boundary. The second half of the requirement ("control and
// logs read the same value") is the host's: through this interface the host receives one sample per step
// and logs that sample itself (hakoniwa-sensor-adapter docs/spec/sensor-log.md), so there is no second read
// inside the adapter to compare.
TEST(PublicImuGnssSpec, T_COM_U2_020_ValueCreatedOnBoundaryAndHeldUntilTheNext)
{
    Json gyro = {{"updatePeriodMsec", 4.0}, {"noise", {0.02, 0.03, 0.04}}};
    Json accel = {{"updatePeriodMsec", 4.0}, {"noise", {0.2, 0.3, 0.4}}};
    auto adapter = make_imu("held_sample", gyro, accel);
    ASSERT_NE(adapter, nullptr);

    const sa::ImuSample sample = step_imu(*adapter, 0, imu_truth(0));
    ASSERT_TRUE(sample.gyro.updated);
    ASSERT_TRUE(sample.accel.updated);

    // Steps 1, 2, 3: the truth changes, the noise generator would draw new values if it were asked,
    // but no value is created: the boundary sample comes back bit for bit.
    for (std::uint64_t k = 1; k < 4; ++k) {
        const sa::ImuSample held = step_imu(*adapter, k, imu_truth(k));
        EXPECT_FALSE(held.gyro.updated);
        EXPECT_FALSE(held.accel.updated);
        expect_vector_eq(held.gyro.value, sample.gyro.value);
        expect_vector_eq(held.accel.value, sample.accel.value);
        EXPECT_EQ(held.gyro.sample_time_ns, sample.gyro.sample_time_ns);
        EXPECT_EQ(held.accel.sample_time_ns, sample.accel.sample_time_ns);
    }
    // Step 4 is the next boundary: a new value.
    const sa::ImuSample next = step_imu(*adapter, 4, imu_truth(4));
    EXPECT_TRUE(next.gyro.updated);
    EXPECT_TRUE(next.accel.updated);
}

// T-COM-I1-050. [SEN-COM-210]
TEST(PublicImuGnssSpec, T_COM_I1_050_UpdatePeriodMeasuresAndEmitsEveryFourSteps)
{
    Json gyro = {{"updatePeriodMsec", 4.0}, {"noise", {0.01, 0.02, 0.03}}};
    Json accel = {{"updatePeriodMsec", 4.0}, {"noise", {0.04, 0.05, 0.06}}};
    auto adapter = make_imu("four_ms_period", gyro, accel);
    ASSERT_NE(adapter, nullptr);

    sa::ImuSample previous;
    sa::ImuSample boundary_zero;
    for (std::uint64_t k = 0; k <= 8; ++k) {
        const sa::ImuSample sample = step_imu(*adapter, k, imu_truth(k));
        const bool boundary = k % 4 == 0;
        EXPECT_EQ(sample.gyro.updated, boundary) << "step " << k;
        EXPECT_EQ(sample.accel.updated, boundary) << "step " << k;
        EXPECT_EQ(sample.gyro.sample_time_ns, (k - k % 4) * kStepNs) << "step " << k;
        EXPECT_EQ(sample.accel.sample_time_ns, (k - k % 4) * kStepNs) << "step " << k;
        if (k == 0) boundary_zero = sample;
        if (!boundary) {
            expect_vector_eq(sample.gyro.value, previous.gyro.value);
            expect_vector_eq(sample.accel.value, previous.accel.value);
        } else if (k != 0) {
            EXPECT_NE(sample.gyro.value.x, previous.gyro.value.x);
            EXPECT_NE(sample.accel.value.x, previous.accel.value.x);
        }
        previous = sample;
    }
    EXPECT_NE(previous.gyro.value.x, boundary_zero.gyro.value.x);
    EXPECT_NE(previous.accel.value.x, boundary_zero.accel.value.x);
}

// Planned default-compatibility check. [SEN-GYR-140]
TEST(PublicImuGnssSpec, GyroOmittedSensitivityMatchesExplicitIdentity)
{
    Json omitted_gyro = {{"noise", {0.01, 0.02, 0.03}}};
    Json identity_gyro = omitted_gyro;
    identity_gyro["sensitivity"] = identity_sensitivity();
    Json accel = {{"noise", {0.04, 0.05, 0.06}}};
    auto omitted = make_imu("gyro_sensitivity_omitted", omitted_gyro, accel);
    auto identity = make_imu("gyro_sensitivity_identity", identity_gyro, accel);
    ASSERT_NE(omitted, nullptr);
    ASSERT_NE(identity, nullptr);

    for (std::uint64_t k = 0; k < 64; ++k) {
        const sa::ImuTruth truth = imu_truth(k);
        const sa::ImuSample omitted_sample = step_imu(*omitted, k, truth);
        const sa::ImuSample identity_sample = step_imu(*identity, k, truth);
        expect_vector_eq(omitted_sample.gyro.value, identity_sample.gyro.value);
    }
}

// Planned default-compatibility check. [SEN-GYR-210]
TEST(PublicImuGnssSpec, GyroOmittedResolutionDoesNotRound)
{
    constexpr double kGridRadPerSec = 1e-6;
    const double grid_deg_per_sec = kGridRadPerSec * 180.0 / kPi;
    Json unrounded_gyro = {{"noise", {0.01, 0.02, 0.03}}};
    Json rounded_gyro = unrounded_gyro;
    rounded_gyro["resolutionDegPerSec"] = grid_deg_per_sec;
    Json accel = {{"noise", {0.04, 0.05, 0.06}}};
    auto unrounded = make_imu("gyro_resolution_omitted", unrounded_gyro, accel);
    auto rounded = make_imu("gyro_resolution_set", rounded_gyro, accel);
    ASSERT_NE(unrounded, nullptr);
    ASSERT_NE(rounded, nullptr);

    std::array<bool, 3> off_grid{};
    for (std::uint64_t k = 0; k < 64; ++k) {
        const sa::ImuTruth truth = imu_truth(k);
        const sa::Vector3 raw = step_imu(*unrounded, k, truth).gyro.value;
        const sa::Vector3 quantized = step_imu(*rounded, k, truth).gyro.value;
        const std::array<double, 3> raw_axes = {raw.x, raw.y, raw.z};
        const std::array<double, 3> quantized_axes = {quantized.x, quantized.y, quantized.z};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            off_grid[axis] = off_grid[axis] || !is_multiple(raw_axes[axis], kGridRadPerSec);
            EXPECT_TRUE(is_multiple(quantized_axes[axis], kGridRadPerSec)) << "axis " << axis << ", step " << k;
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) EXPECT_TRUE(off_grid[axis]) << "axis " << axis;
}

// T-ACC-U2-180. [SEN-ACC-120]
TEST(PublicImuGnssSpec, AccelOmittedSensitivityMatchesExplicitIdentity)
{
    Json gyro = {{"noise", {0.01, 0.02, 0.03}}};
    Json omitted_accel = {{"noise", {0.04, 0.05, 0.06}}};
    Json identity_accel = omitted_accel;
    identity_accel["sensitivity"] = identity_sensitivity();
    auto omitted = make_imu("accel_sensitivity_omitted", gyro, omitted_accel);
    auto identity = make_imu("accel_sensitivity_identity", gyro, identity_accel);
    ASSERT_NE(omitted, nullptr);
    ASSERT_NE(identity, nullptr);

    for (std::uint64_t k = 0; k < 64; ++k) {
        const sa::ImuTruth truth = imu_truth(k);
        const sa::ImuSample omitted_sample = step_imu(*omitted, k, truth);
        const sa::ImuSample identity_sample = step_imu(*identity, k, truth);
        expect_vector_eq(omitted_sample.accel.value, identity_sample.accel.value);
    }
}

// T-ACC-U2-180. [SEN-ACC-190]
TEST(PublicImuGnssSpec, AccelOmittedResolutionDoesNotRound)
{
    constexpr double kGridMps2 = 1e-6;
    Json gyro = {{"noise", {0.01, 0.02, 0.03}}};
    Json unrounded_accel = {{"noise", {0.04, 0.05, 0.06}}};
    Json rounded_accel = unrounded_accel;
    rounded_accel["resolutionMps2"] = kGridMps2;
    auto unrounded = make_imu("accel_resolution_omitted", gyro, unrounded_accel);
    auto rounded = make_imu("accel_resolution_set", gyro, rounded_accel);
    ASSERT_NE(unrounded, nullptr);
    ASSERT_NE(rounded, nullptr);

    std::array<bool, 3> off_grid{};
    for (std::uint64_t k = 0; k < 64; ++k) {
        const sa::ImuTruth truth = imu_truth(k);
        const sa::Vector3 raw = step_imu(*unrounded, k, truth).accel.value;
        const sa::Vector3 quantized = step_imu(*rounded, k, truth).accel.value;
        const std::array<double, 3> raw_axes = {raw.x, raw.y, raw.z};
        const std::array<double, 3> quantized_axes = {quantized.x, quantized.y, quantized.z};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            off_grid[axis] = off_grid[axis] || !is_multiple(raw_axes[axis], kGridMps2);
            EXPECT_TRUE(is_multiple(quantized_axes[axis], kGridMps2)) << "axis " << axis << ", step " << k;
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) EXPECT_TRUE(off_grid[axis]) << "axis " << axis;
}

// T-MAG-U2-160. [SEN-MAG-190]
TEST(PublicImuGnssSpec, MagnetometerOmittedResolutionDoesNotRound)
{
    constexpr double kGridNt = 1e-3;
    Json unrounded_config = {{"noise", {20.0, 30.0, 40.0}}};
    Json rounded_config = unrounded_config;
    rounded_config["resolutionNT"] = kGridNt;
    auto unrounded = make_mag("mag_resolution_omitted", unrounded_config);
    auto rounded = make_mag("mag_resolution_set", rounded_config);
    ASSERT_NE(unrounded, nullptr);
    ASSERT_NE(rounded, nullptr);

    std::array<bool, 3> off_grid{};
    for (std::uint64_t k = 0; k < 64; ++k) {
        const sa::Vector3 raw = step_mag(*unrounded, k).value;
        const sa::Vector3 quantized = step_mag(*rounded, k).value;
        const std::array<double, 3> raw_axes = {raw.x, raw.y, raw.z};
        const std::array<double, 3> quantized_axes = {quantized.x, quantized.y, quantized.z};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            off_grid[axis] = off_grid[axis] || !is_multiple(raw_axes[axis], kGridNt);
            EXPECT_TRUE(is_multiple(quantized_axes[axis], kGridNt)) << "axis " << axis << ", step " << k;
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) EXPECT_TRUE(off_grid[axis]) << "axis " << axis;
}
