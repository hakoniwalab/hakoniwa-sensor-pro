#include <gtest/gtest.h>
#include "hakoniwa/imu_gnss_sensors/sensor_models.hpp"
#include "hakoniwa/imu_gnss_sensors/sensor_delay_line.hpp"
#include <cmath>
#include <limits>
#include <random>
#include <array>
#include <vector>
#include <deque>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace hako::aircraft::impl;
namespace
{
constexpr double pi = 3.14159265358979323846;
void expect_vec(const Vector3 &a, const Vector3 &b, double e = 1e-12)
{
    EXPECT_NEAR(a.x, b.x, e);
    EXPECT_NEAR(a.y, b.y, e);
    EXPECT_NEAR(a.z, b.z, e);
}
Matrix3 product(const Matrix3 &a, const Matrix3 &b)
{
    Matrix3 r{};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                r.v[i][j] += a.v[i][k] * b.v[k][j];
    return r;
}
Matrix3 transpose(const Matrix3 &a)
{
    Matrix3 r{};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.v[i][j] = a.v[j][i];
    return r;
}
void expect_matrix(const Matrix3 &a, const Matrix3 &b, double e = 1e-9)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            EXPECT_NEAR(a.v[i][j], b.v[i][j], e);
}
double determinant(const Matrix3 &m)
{
    return m.v[0][0] * (m.v[1][1] * m.v[2][2] - m.v[1][2] * m.v[2][1]) -
           m.v[0][1] * (m.v[1][0] * m.v[2][2] - m.v[1][2] * m.v[2][0]) +
           m.v[0][2] * (m.v[1][0] * m.v[2][1] - m.v[1][1] * m.v[2][0]);
}
struct VectorStatistics
{
    Vector3 mean{};
    Vector3 variance{};
    double correlation_xy{};
    double correlation_xz{};
    double correlation_yz{};
};
[[maybe_unused]] VectorStatistics vector_statistics(const std::vector<Vector3> &values)
{
    VectorStatistics result{};
    for (const auto &value : values)
    {
        result.mean.x += value.x;
        result.mean.y += value.y;
        result.mean.z += value.z;
    }
    result.mean.x /= values.size();
    result.mean.y /= values.size();
    result.mean.z /= values.size();
    double covariance_xy = 0;
    double covariance_xz = 0;
    double covariance_yz = 0;
    for (const auto &value : values)
    {
        double x = value.x - result.mean.x;
        double y = value.y - result.mean.y;
        double z = value.z - result.mean.z;
        result.variance.x += x * x;
        result.variance.y += y * y;
        result.variance.z += z * z;
        covariance_xy += x * y;
        covariance_xz += x * z;
        covariance_yz += y * z;
    }
    result.variance.x /= values.size();
    result.variance.y /= values.size();
    result.variance.z /= values.size();
    covariance_xy /= values.size();
    covariance_xz /= values.size();
    covariance_yz /= values.size();
    result.correlation_xy = covariance_xy / std::sqrt(result.variance.x * result.variance.y);
    result.correlation_xz = covariance_xz / std::sqrt(result.variance.x * result.variance.z);
    result.correlation_yz = covariance_yz / std::sqrt(result.variance.y * result.variance.z);
    return result;
}
[[maybe_unused]] double component(const Vector3 &value, int axis)
{
    if (axis == 0)
        return value.x;
    if (axis == 1)
        return value.y;
    return value.z;
}
OrientationDeg matrix_to_orientation(const Matrix3 &m)
{
    double p = std::asin(std::clamp(-m.v[2][0], -1.0, 1.0));
    double cp = std::cos(p);
    double r = 0, y = 0;
    if (std::abs(cp) > 1e-10)
    {
        r = std::atan2(m.v[2][1], m.v[2][2]);
        y = std::atan2(m.v[1][0], m.v[0][0]);
    }
    else
    {
        y = std::atan2(-m.v[0][1], m.v[1][1]);
    }
    return {r * 180 / pi, p * 180 / pi, y * 180 / pi};
}
bool independently_px4_representable(const Matrix3 &wanted)
{
    for (int i = 0; i <= 40; i++)
    {
        if (i == 38)
            continue;
        auto off = matrix_to_orientation(product(wanted, transpose(px4_rotation_matrix(i))));
        if (std::abs(off.roll) <= 45 + 1e-9 && std::abs(off.pitch) <= 45 + 1e-9 && std::abs(off.yaw) <= 45 + 1e-9)
            return true;
    }
    return false;
}
void expect_non_finite_orientations_rejected(double value)
{
    const OrientationDeg values[] = {{value, 0, 0}, {0, value, 0}, {0, 0, value}};
    for (const auto &orientation : values)
    {
        EXPECT_FALSE(make_px4_board_rotation(orientation));
        EXPECT_THROW(make_ardupilot_board_rotation(orientation), std::invalid_argument);
    }
}
} // namespace

TEST(SensorModels, T_COM_U1_010_MountingRotation)
{
    Matrix3 yaw90{};
    yaw90.v = {{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}};
    expect_matrix(mounting_rotation({0, 0, 90}), yaw90);
    Matrix3 pitch90{};
    pitch90.v = {{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}};
    expect_matrix(mounting_rotation({0, 90, 0}), pitch90);
    Matrix3 roll90{};
    roll90.v = {{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}};
    expect_matrix(mounting_rotation({90, 0, 0}), roll90);
    Matrix3 yaw45{};
    double q = std::sqrt(.5);
    yaw45.v = {{{q, -q, 0}, {q, q, 0}, {0, 0, 1}}};
    expect_matrix(mounting_rotation({0, 0, 45}), yaw45);
    auto combination = mounting_rotation({90, 90, 90});
    Matrix3 expected_combination{};
    expected_combination.v = {{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}};
    expect_matrix(combination, expected_combination);
    auto orthogonal = product(transpose(combination), combination);
    Matrix3 identity{};
    identity.v = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    expect_matrix(orthogonal, identity);
    EXPECT_NEAR(determinant(combination), 1.0, 1e-12);
}
TEST(SensorModels, T_COM_U1_020_VectorRotation)
{
    auto r = mounting_rotation({0, 0, 90});
    auto s = body_to_sensor(r, {1, 0, 0});
    expect_vec(s, {0, -1, 0});
    expect_vec(sensor_to_body(r, s), {1, 0, 0});
}
TEST(SensorModels, T_COM_U1_030_RigidBodyPoint)
{
    Matrix3 i{};
    i.v = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    double w = 300 * pi / 180;
    auto x = rigid_body_point_state({1, 2, 3}, {4, 5, 6}, {0, 0, 0}, i, {0, 0, w}, {0, 0, 0}, {.05, 0, 0});
    expect_vec(x.position_world, {1.05, 2, 3});
    expect_vec(x.velocity_world, {4, 5 + .05 * w, 6});
    expect_vec(x.acceleration_body, {-.05 * w * w, 0, 0});
    EXPECT_NEAR(std::abs(x.acceleration_body.x), 1.370778389, 1e-9);
}
TEST(SensorModels, T_COM_U1_040_DelayLine)
{
    EXPECT_EQ(SensorDelayLine<int>::steps_for(0, 0.01), 0u);
    EXPECT_EQ(SensorDelayLine<int>::steps_for(10, 0.01), 1u);
    EXPECT_EQ(SensorDelayLine<int>::steps_for(15, 0.01), 2u);
    SensorDelayLine<int> d;
    d.set_delay_steps(2);
    EXPECT_EQ(d.apply(1), 1);
    EXPECT_EQ(d.apply(2), 1);
    EXPECT_EQ(d.apply(3), 1);
    EXPECT_EQ(d.apply(4), 2);
}
TEST(SensorModels, T_COM_U1_060_Clamp)
{
    for (auto [in, out] : std::vector<std::pair<double, double>>{{-2, -1}, {-1, -1}, {0, 0}, {1, 1}, {2, 1}})
        EXPECT_DOUBLE_EQ(clamp_to_range(in, -1, 1), out);
}
TEST(SensorModels, T_COM_U1_070_DerivedSeed)
{
    auto s = derive_sensor_seed(123456789u, "Drone", 7, "gyro", "drift");
    EXPECT_EQ(s, 3971137832u);
    EXPECT_NE(s, derive_sensor_seed(123456788u, "Drone", 7, "gyro", "drift"));
    EXPECT_NE(s, derive_sensor_seed(123456789u, "DroneX", 7, "gyro", "drift"));
    EXPECT_NE(s, derive_sensor_seed(123456789u, "Drone", 8, "gyro", "drift"));
    EXPECT_NE(s, derive_sensor_seed(123456789u, "Drone", 7, "acc", "drift"));
    EXPECT_NE(s, derive_sensor_seed(123456789u, "Drone", 7, "gyro", "noise"));
    EXPECT_EQ(derive_sensor_seed(123456789u, "Drone", 7, "gyro"), 3064411982u);
}
TEST(SensorModels, T_COM_U1_080_GaussMarkovStatistics)
{
    GaussMarkov g(1, .1, .001, 42);
    constexpr int n = 500000, lag = 100;
    std::vector<double> x;
    x.reserve(n);
    for (int i = 0; i < n; i++)
        x.push_back(g.step());
    double mean = 0;
    for (double v : x)
        mean += v;
    mean /= n;
    double var = 0, cov = 0;
    for (double v : x)
        var += (v - mean) * (v - mean);
    var /= n;
    for (int i = lag; i < n; i++)
        cov += (x[i] - mean) * (x[i - lag] - mean);
    cov /= (n - lag);
    EXPECT_NEAR(var, 1, .1);
    EXPECT_NEAR(cov / var, std::exp(-1.0), .05);
}
TEST(SensorModels, T_COM_U1_090_InitialStationaryDistribution)
{
    constexpr int n = 20000;
    double mean = 0, sum2 = 0;
    for (int i = 0; i < n; i++)
    {
        GaussMarkov g(2, .1, .001, i);
        double x = g.value();
        mean += x;
        sum2 += x * x;
    }
    mean /= n;
    double sd = std::sqrt(sum2 / n - mean * mean);
    EXPECT_LT(std::abs(mean), 4 * 2 / std::sqrt(double(n)));
    EXPECT_NEAR(sd, 2, .06);
}
TEST(SensorModels, T_COM_U1_100_AutopilotPosition)
{
    expect_vec(autopilot_position({.05, 0, 0}, {.01, 0, .02}), {.04, 0, -.02});
}
TEST(SensorModels, T_COM_U1_110_Px4BoardRotation)
{
    for (int i = 0; i <= 40; i++)
    {
        if (i == 38)
            continue;
        auto wanted = px4_rotation_matrix(i);
        auto c = make_px4_board_rotation(matrix_to_orientation(wanted));
        ASSERT_TRUE(c);
        EXPECT_NE(c->rotation, 38);
        expect_matrix(product(mounting_rotation(c->offset), px4_rotation_matrix(c->rotation)), wanted, 1e-6);
        EXPECT_LE(std::abs(c->offset.roll), 45 + 1e-9);
        EXPECT_LE(std::abs(c->offset.pitch), 45 + 1e-9);
        EXPECT_LE(std::abs(c->offset.yaw), 45 + 1e-9);
    }
    EXPECT_THROW(px4_rotation_matrix(38), std::out_of_range);
    std::mt19937 rng(19);
    std::uniform_real_distribution<double> d(-40, 40);
    for (int n = 0; n < 1000; n++)
    {
        int rotation = n % 40;
        if (rotation >= 38)
            ++rotation;
        auto wanted = product(mounting_rotation({d(rng), d(rng), d(rng)}), px4_rotation_matrix(rotation));
        auto c = make_px4_board_rotation(matrix_to_orientation(wanted));
        ASSERT_TRUE(c);
        EXPECT_NE(c->rotation, 38);
        expect_matrix(product(mounting_rotation(c->offset), px4_rotation_matrix(c->rotation)), wanted, 1e-6);
        EXPECT_LE(std::abs(c->offset.roll), 45 + 1e-9);
        EXPECT_LE(std::abs(c->offset.pitch), 45 + 1e-9);
        EXPECT_LE(std::abs(c->offset.yaw), 45 + 1e-9);
    }
    for (int r = -180; r <= 180; r += 45)
        for (int p = -90; p <= 90; p += 45)
            for (int y = -180; y <= 180; y += 45)
            {
                OrientationDeg o{double(r), double(p), double(y)};
                auto wanted = mounting_rotation(o);
                auto c = make_px4_board_rotation(o);
                if (c)
                {
                    EXPECT_NE(c->rotation, 38);
                    expect_matrix(product(mounting_rotation(c->offset), px4_rotation_matrix(c->rotation)), wanted,
                                  1e-6);
                    EXPECT_LE(std::abs(c->offset.roll), 45 + 1e-9);
                    EXPECT_LE(std::abs(c->offset.pitch), 45 + 1e-9);
                    EXPECT_LE(std::abs(c->offset.yaw), 45 + 1e-9);
                }
                else
                {
                    EXPECT_FALSE(independently_px4_representable(wanted));
                }
            }
    expect_non_finite_orientations_rejected(std::numeric_limits<double>::quiet_NaN());
    expect_non_finite_orientations_rejected(std::numeric_limits<double>::infinity());
    expect_non_finite_orientations_rejected(-std::numeric_limits<double>::infinity());
}
TEST(SensorModels, T_COM_U1_115_Px4BoardRotationSharedCases)
{
    std::ifstream input(std::string(HAKO_TEST_PROJECT_ROOT) + "/src/test/data/px4_board_rotation_cases.json");
    ASSERT_TRUE(input.is_open());
    const auto cases = nlohmann::json::parse(input);
    for (const auto& item : cases)
    {
        const auto& o = item.at("orientation");
        auto result = make_px4_board_rotation({o[0].get<double>(), o[1].get<double>(), o[2].get<double>()});
        if (item.value("error", false))
        {
            EXPECT_FALSE(result) << item.at("name").get<std::string>();
            continue;
        }
        ASSERT_TRUE(result) << item.at("name").get<std::string>();
        EXPECT_EQ(item.at("rotation").get<int>(), result->rotation);
        const auto& offset = item.at("offset");
        EXPECT_NEAR(offset[0].get<double>(), result->offset.roll, 1e-9);
        EXPECT_NEAR(offset[1].get<double>(), result->offset.pitch, 1e-9);
        EXPECT_NEAR(offset[2].get<double>(), result->offset.yaw, 1e-9);
    }
}
TEST(SensorModels, T_COM_U1_120_ArduPilotBoardRotation)
{
    std::ifstream input(std::string(HAKO_TEST_PROJECT_ROOT) + "/src/test/data/ardupilot_board_rotation_cases.json");
    ASSERT_TRUE(input.good());
    nlohmann::json cases;
    input >> cases;
    for (const auto &test_case : cases)
    {
        const auto a = test_case.at("orientation");
        OrientationDeg angles{a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
        auto c = make_ardupilot_board_rotation(angles);
        EXPECT_EQ(c.custom_enabled, test_case.value("custom", false));
        EXPECT_NE(c.orientation, 38);
        if (c.custom_enabled)
            expect_matrix(mounting_rotation(c.custom), mounting_rotation(angles), 1e-6);
        else
        {
            EXPECT_EQ(c.orientation, test_case.at("rotation").get<int>());
            expect_matrix(ardupilot_rotation_matrix(c.orientation), mounting_rotation(angles), 1e-6);
        }
    }
    EXPECT_THROW(ardupilot_rotation_matrix(38), std::out_of_range);
    EXPECT_THROW(ardupilot_rotation_matrix(41), std::out_of_range);
    std::mt19937 rng(23);
    std::uniform_real_distribution<double> d(-179, 179);
    for (int i = 0; i < 1000; i++)
    {
        OrientationDeg o{d(rng), d(rng) / 2, d(rng)};
        auto c = make_ardupilot_board_rotation(o);
        EXPECT_NE(c.orientation, 38);
        Matrix3 recovered = c.custom_enabled ? mounting_rotation(c.custom) : ardupilot_rotation_matrix(c.orientation);
        expect_matrix(recovered, mounting_rotation(o), 1e-6);
    }
}
TEST(SensorModels, T_COM_U1_125_AutopilotRotationMatricesUseKnownValues)
{
    Matrix3 yaw90{};
    yaw90.v = {{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}};
    expect_matrix(px4_rotation_matrix(2), yaw90);
    expect_matrix(ardupilot_rotation_matrix(2), yaw90);

    Matrix3 roll180{};
    roll180.v = {{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};
    expect_matrix(px4_rotation_matrix(8), roll180);
    expect_matrix(ardupilot_rotation_matrix(8), roll180);

    Matrix3 pitch90{};
    pitch90.v = {{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}};
    expect_matrix(px4_rotation_matrix(24), pitch90);
    expect_matrix(ardupilot_rotation_matrix(24), pitch90);

    const double q = std::sqrt(.5);
    Matrix3 roll45{};
    roll45.v = {{{1, 0, 0}, {0, q, -q}, {0, q, q}}};
    expect_matrix(ardupilot_rotation_matrix(42), roll45);

    Matrix3 roll315{};
    roll315.v = {{{1, 0, 0}, {0, q, q}, {0, -q, q}}};
    expect_matrix(ardupilot_rotation_matrix(43), roll315);
}
TEST(SensorModels, T_COM_U1_130_SDFormatConversion)
{
    EXPECT_NEAR(sdformat_dynamic_bias_stddev(.01, 3600), .01 * std::sqrt(2.0 / 3600), 1e-15);
}
TEST(SensorModels, T_COM_U1_140_ResolutionRounding)
{
    EXPECT_DOUBLE_EQ(round_to_resolution(-1.5, 1), -2);
    EXPECT_DOUBLE_EQ(round_to_resolution(-.5, 1), -1);
    EXPECT_DOUBLE_EQ(round_to_resolution(.49, 1), 0);
    EXPECT_DOUBLE_EQ(round_to_resolution(.5, 1), 1);
    EXPECT_DOUBLE_EQ(round_to_resolution(1.5, 1), 2);
    EXPECT_DOUBLE_EQ(round_to_resolution(.0305, .061), .061);
}

namespace
{
GyroModelConfig gyro_cfg(const nlohmann::json &gyro = nlohmann::json::object(),
                         const nlohmann::json &mount = nlohmann::json::object())
{
    return parse_gyro_model_config({{"components", {{"sensors", {{"imu", {{"mount", mount}}}, {"gyro", gyro}}}}}});
}
GyroModel gyro(const GyroModelConfig &c, double dt = .001, int samples = 1, double delay = 0, std::uint32_t seed = 123)
{
    return GyroModel(c, dt, samples, delay, seed, "Drone", 0);
}
[[maybe_unused]] double rad(double d)
{
    return d * pi / 180;
}
} // namespace

namespace
{
AccelerationModelConfig acc_cfg(const nlohmann::json &acc = nlohmann::json::object(),
                                const nlohmann::json &mount = nlohmann::json::object())
{
    return parse_acceleration_model_config(
        {{"components", {{"sensors", {{"imu", {{"mount", mount}}}, {"acc", acc}}}}}});
}
GeomagnetModelConfig mag_cfg(const nlohmann::json &mag = nlohmann::json::object(),
                            const nlohmann::json &mount = nlohmann::json::object());

TEST(SensorModels, T_COM_U2_070_UpdatePeriodSamplesAndHolds)
{
    auto gyro_periodic = gyro(gyro_cfg({{"updatePeriodMsec", 4}}));
    AccelerationModel acc_periodic(acc_cfg({{"updatePeriodMsec", 4}}), .001, 1, 0, 123, "Drone", 0);
    GeomagnetModel mag_periodic(mag_cfg({{"updatePeriodMsec", 4}}), .001, 1, 0, 123, "Drone", 0);
    auto gyro_every_step = gyro(gyro_cfg());
    for (int step = 0; step < 9; ++step)
    {
        const double input = step + 1;
        const double periodic = 1 + 4 * (step / 4);
        expect_vec(gyro_periodic.step({input, 0, 0}), {periodic, 0, 0});
        expect_vec(acc_periodic.step({input, 0, 0}, {}, {}), {periodic, 0, 0});
        expect_vec(mag_periodic.step({input, 0, 0}), {periodic, 0, 0});
        expect_vec(gyro_every_step.step({input, 0, 0}), {input, 0, 0});
    }

    auto noisy = gyro(gyro_cfg({{"updatePeriodMsec", 4}, {"noise", .1}}));
    const auto first = noisy.step({});
    for (int step = 1; step < 4; ++step)
        expect_vec(noisy.step({}), first, 0);
    const auto second = noisy.step({});
    EXPECT_TRUE(first.x != second.x || first.y != second.y || first.z != second.z);
    for (int step = 1; step < 4; ++step)
        expect_vec(noisy.step({}), second, 0);
    noisy.reset();
    expect_vec(noisy.step({}), first, 0);
}

TEST(SensorModels, T_COM_U3_070_SampleCountValidation)
{
    for (const auto &value : std::vector<nlohmann::json>{4, 0, 1.5})
    {
        EXPECT_THROW(gyro_cfg({{"sampleCount", value}}), std::runtime_error);
        EXPECT_THROW(acc_cfg({{"sampleCount", value}}), std::runtime_error);
        EXPECT_THROW(mag_cfg({{"sampleCount", value}}), std::runtime_error);
    }
    EXPECT_NO_THROW(gyro_cfg({{"sampleCount", 1}}));
    EXPECT_NO_THROW(acc_cfg({{"sampleCount", 1}}));
    EXPECT_NO_THROW(mag_cfg({{"sampleCount", 1}}));
}
GeomagnetModelConfig mag_cfg(const nlohmann::json &mag, const nlohmann::json &mount)
{
    return parse_geomagnet_model_config(
        {{"components", {{"sensors", {{"imu", {{"mount", mount}}}, {"mag", mag}}}}}});
}
nlohmann::json density_root(const std::string &sensor, const nlohmann::json &value,
                            const std::optional<int> &period_msec = std::nullopt)
{
    nlohmann::json sensor_config{{"noiseDensity", value}};
    if (period_msec)
        sensor_config["updatePeriodMsec"] = *period_msec;
    return {{"simulation", {{"timeStep", .001}}},
            {"components", {{"sensors", {{sensor, sensor_config}}}}}};
}
}

TEST(SensorModels, T_COM_U1_150_NoiseDensityConversion)
{
    const auto with_period = parse_gyro_model_config(density_root("gyro", .0003, 4));
    const auto one_msec_period = parse_gyro_model_config(density_root("gyro", .0003, 1));
    EXPECT_NEAR(with_period.noise_stddev.x, .0003 / std::sqrt(.004), 1e-15);
    EXPECT_NEAR(one_msec_period.noise_stddev.x, .0003 / std::sqrt(.001), 1e-15);
    EXPECT_NEAR(.1 * pi / 180.0 / 60.0, 2.908882086657216e-5, 1e-16);
    ASSERT_EQ(with_period.notice_settings.size(), 1U);
    EXPECT_NE(with_period.notice_settings[0].find("gyro.noiseDensity -> noise = ["), std::string::npos);
}

TEST(SensorModels, T_COM_U2_060_NoiseDensityMatchesNoiseBitForBit)
{
    constexpr double dt = .004;
    const Vector3 density{.0003, .0004, .0005};
    const Vector3 sigma{density.x / std::sqrt(dt), density.y / std::sqrt(dt), density.z / std::sqrt(dt)};
    const nlohmann::json density_json{density.x, density.y, density.z};
    const nlohmann::json sigma_json{sigma.x, sigma.y, sigma.z};

    GyroModel gyro_density(parse_gyro_model_config(density_root("gyro", density_json, 4)), dt, 1, 0, 42,
                           "Drone", 0);
    GyroModel gyro_noise(gyro_cfg({{"noise", sigma_json}}), dt, 1, 0, 42, "Drone", 0);
    expect_vec(gyro_density.step({}), gyro_noise.step({}), 0);

    AccelerationModel acc_density(parse_acceleration_model_config(density_root("acc", density_json, 4)),
                                  dt, 1, 0, 42, "Drone", 0);
    AccelerationModel acc_noise(acc_cfg({{"noise", sigma_json}}), dt, 1, 0, 42, "Drone", 0);
    expect_vec(acc_density.step({}, {}, {}), acc_noise.step({}, {}, {}), 0);

    GeomagnetModel mag_density(parse_geomagnet_model_config(density_root("mag", density_json, 4)),
                               dt, 1, 0, 42, "Drone", 0);
    GeomagnetModel mag_noise(mag_cfg({{"noise", sigma_json}}), dt, 1, 0, 42, "Drone", 0);
    expect_vec(mag_density.step({}), mag_noise.step({}), 0);
}

TEST(SensorModels, T_COM_U3_060_NoiseDensityValidationAndForms)
{
    auto both = density_root("gyro", .1);
    both["components"]["sensors"]["gyro"]["noise"] = .1;
    EXPECT_THROW(parse_gyro_model_config(both), std::runtime_error);

    auto nonfinite = density_root("acc", .1);
    nonfinite["components"]["sensors"]["acc"]["noiseDensity"] =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(parse_acceleration_model_config(nonfinite), std::runtime_error);

    EXPECT_THROW(parse_gyro_model_config(density_root("gyro", .1)), std::runtime_error);

    expect_vec(parse_gyro_model_config(density_root("gyro", .1, 1)).noise_stddev,
               {.1 / std::sqrt(.001), .1 / std::sqrt(.001), .1 / std::sqrt(.001)});
    expect_vec(parse_gyro_model_config(density_root("gyro", {.1, .2, .3}, 1)).noise_stddev,
               {.1 / std::sqrt(.001), .2 / std::sqrt(.001), .3 / std::sqrt(.001)});
}

TEST(SensorModels, T_ACC_U2_010_Through_070_PhysicsAndMount)
{
    AccelerationModel base(acc_cfg(), .001, 1, 0, 10, "Drone", 0);
    expect_vec(base.step({0, 0, -9.81}, {}, {}), {0, 0, -9.81});
    expect_vec(base.step({1, 0, -9.81}, {}, {}), {1, 0, -9.81});
    expect_vec(base.step({0, 0, 0}, {}, {}), {});
    double w = 300 * pi / 180;
    auto position = acc_cfg(nlohmann::json::object(), {{"positionM", {.05, 0, 0}}});
    AccelerationModel centripetal(position, .001, 1, 0, 10, "Drone", 0);
    expect_vec(centripetal.step({0, 0, -9.81}, {0, 0, w}, {}), {-.05 * w * w, 0, -9.81}, 1e-9);
    auto mounted = acc_cfg(nlohmann::json::object(), {{"positionM", {.05, 0, 0}},
                                {"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}});
    AccelerationModel rotated(mounted, .001, 1, 0, 10, "Drone", 0);
    expect_vec(rotated.step({1, 0, -9.81}, {}, {}), {0, -1, -9.81}, 1e-9);
    expect_vec(rotated.step({0, 0, -9.81}, {0, 0, w}, {}), {0, .05 * w * w, -9.81}, 1e-9);
}

TEST(SensorModels, T_ACC_U2_080_Through_100_Order)
{
    auto c = acc_cfg({{"resolutionMps2", 1.0}, {"range", {{"fullScaleG", 2.0}}}});
    AccelerationModel resolution(c, .001, 1, 0, 1, "Drone", 0);
    EXPECT_DOUBLE_EQ(resolution.step({19.6, 0, 0}, {}, {}).x, 2 * 9.80665);
    c = acc_cfg({{"zeroGOffset", {{"fixedMg", {20, 0, 0}}}}, {"range", {{"fullScaleG", 2.0}}}});
    AccelerationModel offset(c, .001, 1, 0, 1, "Drone", 0);
    EXPECT_DOUBLE_EQ(offset.step({19.5, 0, 0}, {}, {}).x, 2 * 9.80665);
    c = acc_cfg({{"sensitivity", {{"scaleErrorPct", {1, 0, 0}},
                                    {"crossAxisPct", {{0, 0, 1}, {2, 0, 0}, {0, 0, 0}}}}}},
                {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}});
    AccelerationModel sensitivity(c, .001, 1, 0, 1, "Drone", 0);
    expect_vec(sensitivity.step({2, 0, -9.81}, {}, {}), {-.0981, -2, -9.81}, 1e-9);
}

TEST(SensorModels, T_ACC_U2_130_150_170_OffsetAndRange)
{
    auto c = acc_cfg({{"zeroGOffset", {{"fixedMg", {20, -15, 40}}}}});
    AccelerationModel m(c, .001, 1, 0, 1, "Drone", 0);
    expect_vec(m.step({0, 0, -9.81}, {}, {}), {.196133, -.14709975, -9.417734}, 1e-9);
    c = acc_cfg({{"range", {{"fullScaleG", 1.0}}}});
    AccelerationModel range(c, .001, 1, 0, 1, "Drone", 0);
    EXPECT_DOUBLE_EQ(range.step({0, 0, -9.81}, {}, {}).z, -9.80665);
}

TEST(SensorModels, T_ACC_U2_110_120_140_NoiseDriftAndReset)
{
    auto c = acc_cfg({{"noise", {0.1, 0.2, 0}},
                      {"zeroGOffset", {{"randomStdMg", 10}}},
                      {"drift", {{"mode", "gauss-markov"}, {"stddevMg", 5},
                                  {"correlationTimeSec", .1}}}});
    AccelerationModel m(c, .001, 1, 2, 42, "Drone", 0);
    std::vector<Vector3> first;
    for (int i = 0; i < 200; ++i)
        first.push_back(m.step({double(i), -double(i), -9.80665}, {}, {}));
    m.reset();
    for (int i = 0; i < 200; ++i)
        expect_vec(m.step({double(i), -double(i), -9.80665}, {}, {}), first[i]);

    AccelerationModel noise(acc_cfg({{"noise", {0.1, 0.2, 0}}}), .001, 1, 0, 42, "Drone", 0);
    std::vector<Vector3> values;
    values.reserve(20000);
    for (int i = 0; i < 20000; ++i)
        values.push_back(noise.step({}, {}, {}));
    auto statistics = vector_statistics(values);
    EXPECT_NEAR(std::sqrt(statistics.variance.x), .1, .003);
    EXPECT_NEAR(std::sqrt(statistics.variance.y), .2, .006);
    EXPECT_DOUBLE_EQ(statistics.variance.z, 0);
    EXPECT_LT(std::abs(statistics.correlation_xy), .03);
}

// [SEN-ACC-303] The recorded offset is the value drawn at the reset, measured from the output (not the 30 mg).
TEST(SensorModels, T_ACC_U1_070_RecordsTheStartupDraw)
{
    AccelerationModel m(acc_cfg({{"zeroGOffset", {{"randomStdMg", 30}}}}), .001, 1, 0, 7, "Drone", 0);
    const auto output = m.step({0, 0, -9.81}, {}, {});
    const auto recorded = m.zero_g_offset_mps2();
    expect_vec(recorded, {output.x, output.y, output.z + 9.81}, 1e-12);
    EXPECT_GT(std::abs(recorded.x) + std::abs(recorded.y) + std::abs(recorded.z), 0.0);
    EXPECT_NE(std::abs(recorded.x), 30 * .001 * 9.80665);
    m.reset();
    expect_vec(m.zero_g_offset_mps2(), recorded, 0.0);
    AccelerationModel fixed(acc_cfg({{"zeroGOffset", {{"fixedMg", {20, -15, 40}}}}}), .001, 1, 0, 7, "Drone", 0);
    expect_vec(fixed.zero_g_offset_mps2(), {.196133, -.14709975, .392266}, 1e-9);
}

TEST(SensorModels, T_COM_U2_030_AccelerationOffsetDoesNotChangeNoiseSequence)
{
    AccelerationModel a(acc_cfg({{"noise", {1.0, 0, 0}}}), .001, 1, 0, 42, "Drone", 0);
    AccelerationModel b(acc_cfg({{"noise", {1.0, 0, 0}},
                                 {"zeroGOffset", {{"randomStdMg", 10}}}}),
                        .001, 1, 0, 42, "Drone", 0);
    const auto offset = b.step({}, {}, {}).x - a.step({}, {}, {}).x;
    for (int i = 0; i < 100; ++i)
        EXPECT_NEAR(b.step({}, {}, {}).x - a.step({}, {}, {}).x, offset, 1e-15);
}

TEST(SensorModels, T_ACC_U3_010_Through_060_ConfigValidation)
{
    EXPECT_EQ(acc_cfg({{"noise", .1}}).noise_stddev.x, .1);
    expect_vec(acc_cfg({{"noise", {.1, .2, -.1}}}).noise_stddev, {.1, .2, -.1});
    EXPECT_THROW(acc_cfg({{"noise", {1, 2}}}), std::runtime_error);
    EXPECT_THROW(acc_cfg({{"zeroGOffset", {{"fixedMg", {0, 0, 0}}, {"randomStdMg", 1}}}}),
                 std::runtime_error);
    EXPECT_THROW(acc_cfg({{"drift", {{"mode", "random-walk"}}}}), std::runtime_error);
    EXPECT_THROW(acc_cfg({{"resolutionMps2", 0}}), std::runtime_error);
    EXPECT_THROW(acc_cfg({{"range", {{"fullScaleG", -1}}}}), std::runtime_error);
    EXPECT_THROW(acc_cfg({{"mount", nlohmann::json::object()}}), std::runtime_error);
}

TEST(SensorModels, T_MAG_U2_010_Through_090_FieldMountAndOrder)
{
    GeomagnetModel base(mag_cfg(), .001, 1, 0, 1, "Drone", 0);
    expect_vec(base.step({25000, 0, 43301.270}), {25000, 0, 43301.270});
    auto c = mag_cfg(nlohmann::json::object(),
                     {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}});
    GeomagnetModel mounted(c, .001, 1, 0, 1, "Drone", 0);
    expect_vec(mounted.step({50000, 0, 0}), {0, -50000, 0}, 1e-9);
    c = mag_cfg({{"softIron", {{"diagonal", {2, 1, 1}}}}, {"hardIronNT", {1000, 0, 0}}},
                {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}});
    GeomagnetModel order(c, .001, 1, 0, 1, "Drone", 0);
    expect_vec(order.step({50000, 0, 0}), {1000, -50000, 0}, 1e-9);
    c = mag_cfg({{"resolutionNT", 10000}, {"range", {{"fullScaleGauss", .245}}}});
    GeomagnetModel range(c, .001, 1, 0, 1, "Drone", 0);
    EXPECT_DOUBLE_EQ(range.step({26000, 0, 0}).x, 24500);
}

TEST(SensorModels, T_MAG_U2_120_Through_150_SoftIronResolutionRange)
{
    auto c = mag_cfg({{"softIron", {{"diagonal", {1.02, .98, 1.01}},
                                      {"offDiagonal", {.01, -.02, .005}}}},
                      {"hardIronNT", {3000, -1500, 800}}});
    GeomagnetModel soft(c, .001, 1, 0, 1, "Drone", 0);
    expect_vec(soft.step({25000, 0, 43301.270}), {27633.9746, -1033.49365, 44034.2827}, 1e-3);
    c = mag_cfg({{"resolutionNT", 24}});
    GeomagnetModel resolution(c, .001, 1, 0, 1, "Drone", 0);
    expect_vec(resolution.step({36, -36, 11}), {48, -48, 0});
    c = mag_cfg({{"range", {{"fullScaleGauss", .5}}}});
    GeomagnetModel range(c, .001, 1, 0, 1, "Drone", 0);
    expect_vec(range.step({-60000, 0, 60000}), {-50000, 0, 50000});
}

TEST(SensorModels, T_MAG_U2_020_030_050_060_FieldCases)
{
    GeomagnetModel m(mag_cfg(), .001, 1, 0, 1, "Drone", 0);
    expect_vec(m.step({49240.38765, 8682.40888, 0}), {49240.38765, 8682.40888, 0});
    expect_vec(m.step({43301.27019, 0, 25000}), {43301.27019, 0, 25000});
    expect_vec(m.step({0, -50000, 0}), {0, -50000, 0});
    expect_vec(m.step({25000, 0, -43301.27019}), {25000, 0, -43301.27019});
    for (int i = 0; i < 600; ++i)
        expect_vec(m.step({123, 456, 789}), {123, 456, 789});
}

TEST(SensorModels, T_MAG_U2_100_110_NoiseDriftAndReset)
{
    auto c = mag_cfg({{"noise", {50, 100, 0}},
                      {"drift", {{"mode", "gauss-markov"}, {"stddevNT", 100},
                                  {"correlationTimeSec", .1}}}});
    GeomagnetModel replay(c, .001, 1, 2, 42, "Drone", 0);
    std::vector<Vector3> first;
    for (int i = 0; i < 200; ++i) first.push_back(replay.step({double(i), 0, 0}));
    replay.reset();
    for (int i = 0; i < 200; ++i) expect_vec(replay.step({double(i), 0, 0}), first[i]);

    GeomagnetModel noise(mag_cfg({{"noise", {50, 100, 0}}}), .001, 1, 0, 42, "Drone", 0);
    std::vector<Vector3> values;
    values.reserve(20000);
    for (int i = 0; i < 20000; ++i) values.push_back(noise.step({}));
    auto statistics = vector_statistics(values);
    EXPECT_NEAR(std::sqrt(statistics.variance.x), 50, 1.5);
    EXPECT_NEAR(std::sqrt(statistics.variance.y), 100, 3);
    EXPECT_DOUBLE_EQ(statistics.variance.z, 0);
    EXPECT_LT(std::abs(statistics.correlation_xy), .03);
}


TEST(SensorModels, T_COM_U2_030_GeomagnetAxisNoiseStreamsAreIndependent)
{
    GeomagnetModel a(mag_cfg({{"noise", {1.0, 1.0, 0}}}), .001, 1, 0, 42, "Drone", 0);
    GeomagnetModel b(mag_cfg({{"noise", {1.0, 0.0, 0}}}), .001, 1, 0, 42, "Drone", 0);
    for (int i = 0; i < 100; ++i)
        EXPECT_DOUBLE_EQ(a.step({}).x, b.step({}).x);
}

TEST(SensorModels, T_MAG_U3_020_Through_070_ConfigValidation)
{
    EXPECT_EQ(mag_cfg({{"noise", 50}}).noise_stddev.x, 50);
    EXPECT_THROW(mag_cfg({{"noise", {1, 2}}}), std::runtime_error);
    EXPECT_THROW(mag_cfg({{"softIron", {{"diagonal", {1, 1, 0}}}}}), std::runtime_error);
    EXPECT_NO_THROW(mag_cfg({{"softIron", {{"diagonal", {.0001, .0001, 1}}}}}));
    EXPECT_THROW(mag_cfg({{"mount", nlohmann::json::object()}}), std::runtime_error);
    EXPECT_THROW(mag_cfg({{"drift", {{"mode", "random-walk"}}}}), std::runtime_error);
    EXPECT_THROW(mag_cfg({{"range", {{"fullScaleGauss", 0}}}}), std::runtime_error);
}

#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
TEST(SensorModels, T_COM_U2_010_ImuMountRotatesAccelerationGyroAndGeomagnetEqually)
{
    const nlohmann::json mount{
        {"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}};
    AccelerationModel acceleration(acc_cfg(nlohmann::json::object(), mount), .001, 1, 0, 1,
                                   "common-mount", 0);
    GyroModel gyroscope(gyro_cfg(nlohmann::json::object(), mount), .001, 1, 0, 1,
                        "common-mount", 0);
    GeomagnetModel geomagnet(mag_cfg(nlohmann::json::object(), mount), .001, 1, 0, 1,
                             "common-mount", 0);

    expect_vec(acceleration.step({1, 0, -9.81}, {}, {}), {0, -1, -9.81}, 1e-12);
    expect_vec(gyroscope.step({1, 0, 0}), {0, -1, 0}, 1e-12);
    expect_vec(geomagnet.step({1, 0, 0}), {0, -1, 0}, 1e-12);
}

TEST(SensorModels, T_ACC_U2_160_SensorAxisIsClampedAfterYaw45Rotation)
{
    const auto config = acc_cfg(
        {{"range", {{"fullScaleG", 2.0}}}},
        {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 45}}}});
    AccelerationModel model(config, .001, 1, 0, 1, "acc-range", 0);

    // Independent values from the design row: only sensor x saturates at 2 g.
    expect_vec(model.step({15, 15, -9.81}, {}, {}), {19.6133, 0, -9.81}, 1e-9);
}

TEST(SensorModels, T_GPS_U1_020_LongDistanceWgs84ApproximationBounds)
{
    struct Reference
    {
        double origin_latitude_deg;
        double origin_altitude_m;
        double north_and_east_m;
        double reference_latitude_deg;
        double reference_longitude_deg;
    };
    // These reference positions were produced independently by adding local north/east vectors
    // in WGS-84 ECEF and converting the result back to geodetic coordinates (PROJ 9.5).
    constexpr std::array<Reference, 12> references{{
        {0.0, 0.0, 1000, 0.0090436945827403227, 139.0089831527676},
        {0.0, 0.0, 2121.3203435596424, 0.019184571905234551, 139.01905614416867},
        {35.7, 0.0, 1000, 35.709012274638845, 139.01105048628867},
        {35.7, 0.0, 2121.3203435596424, 35.719116697377473, 139.02344457906301},
        {60.0, 0.0, 1000, 60.008974448351339, 139.01792600162372},
        {60.0, 0.0, 2121.3203435596424, 60.019034769795454, 139.03803834451423},
        {0.0, 1000.0, 1000, 0.0090422673309763345, 139.00898174455946},
        {0.0, 1000.0, 2121.3203435596424, 0.019181544247692146, 139.0190531569084},
        {35.7, 1000.0, 1000, 35.70901085728709, 139.0110487557838},
        {35.7, 1000.0, 2121.3203435596424, 35.719113691107289, 139.02344090718165},
        {60.0, 1000.0, 1000, 60.008973042871474, 139.01792319782854},
        {60.0, 1000.0, 2121.3203435596424, 60.019031789236038, 139.03803239315297},
    }};
    const auto distance_m = [](double lat1, double lon1, double lat2, double lon2) {
        constexpr double mean_earth_radius_m = 6371008.8;
        const double p1 = lat1 * pi / 180.0;
        const double p2 = lat2 * pi / 180.0;
        const double dp = (lat2 - lat1) * pi / 180.0;
        const double dl = (lon2 - lon1) * pi / 180.0;
        const double h = std::sin(dp / 2) * std::sin(dp / 2) +
                         std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
        return 2 * mean_earth_radius_m * std::asin(std::sqrt(h));
    };

    for (const auto &reference : references)
    {
        Matrix3 body_to_world{};
        body_to_world.v = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        GpsModel model(parse_gps_model_config(nlohmann::json::object()), .001, 1, 0, 1,
                       "gps-long-distance", 0,
                       reference.origin_latitude_deg, 139.0, reference.origin_altitude_m);
        const auto value = model.step({reference.north_and_east_m, reference.north_and_east_m, 0},
                                      {}, {}, body_to_world);
        const double error_m = distance_m(value.latitude_deg, value.longitude_deg,
                                          reference.reference_latitude_deg,
                                          reference.reference_longitude_deg);
        const double limit_m = reference.origin_latitude_deg == 0.0 ? .01
                               : reference.origin_latitude_deg == 35.7 ? .6 : 1.4;
        EXPECT_LE(error_m, limit_m) << "origin latitude=" << reference.origin_latitude_deg
                                    << ", altitude=" << reference.origin_altitude_m
                                    << ", north/east=" << reference.north_and_east_m;
    }
}

TEST(SensorModels, T_COM_U3_020_MissingMountDefaultsToZeroPositionAndOrientation)
{
    const auto gyro_config = gyro_cfg();
    const auto acceleration_config = acc_cfg();
    const auto geomagnet_config = mag_cfg();
    const auto barometric_config = parse_barometric_model_config(nlohmann::json::object());
    const auto gps_config = parse_gps_model_config(nlohmann::json::object());

    expect_vec(gyro_config.mount_position_m, {});
    EXPECT_DOUBLE_EQ(gyro_config.mount_orientation_deg.roll, 0);
    EXPECT_DOUBLE_EQ(gyro_config.mount_orientation_deg.pitch, 0);
    EXPECT_DOUBLE_EQ(gyro_config.mount_orientation_deg.yaw, 0);
    expect_vec(acceleration_config.mount_position_m, {});
    EXPECT_DOUBLE_EQ(acceleration_config.mount_orientation_deg.roll, 0);
    EXPECT_DOUBLE_EQ(acceleration_config.mount_orientation_deg.pitch, 0);
    EXPECT_DOUBLE_EQ(acceleration_config.mount_orientation_deg.yaw, 0);
    EXPECT_DOUBLE_EQ(geomagnet_config.mount_orientation_deg.roll, 0);
    EXPECT_DOUBLE_EQ(geomagnet_config.mount_orientation_deg.pitch, 0);
    EXPECT_DOUBLE_EQ(geomagnet_config.mount_orientation_deg.yaw, 0);
    expect_vec(barometric_config.mount_position_m, {});
    expect_vec(gps_config.mount_position_m, {});
}

TEST(SensorModels, T_COM_U3_030_GyroLocalMountIsRejected)
{
    EXPECT_THROW(gyro_cfg({{"mount", nlohmann::json::object()}}), std::runtime_error);
}

TEST(SensorModels, T_COM_U3_040_UnknownGyroFieldWarnsAndKnownValuesRemainReadable)
{
    testing::internal::CaptureStderr();
    const auto config = gyro_cfg({{"noise", .25}, {"fooBar", 7}});
    const auto warning = testing::internal::GetCapturedStderr();

    expect_vec(config.noise_stddev, {.25, .25, .25});
    EXPECT_NE(warning.find("WARNING: unknown sensor setting components.sensors.gyro.fooBar"),
              std::string::npos);
}

TEST(SensorModels, T_COM_U3_050_SensorNumericFieldsRejectWrongJsonTypes)
{
    const std::array<nlohmann::json, 3> wrong_types{
        nlohmann::json("not-a-number"), nlohmann::json::array(), nlohmann::json::object()};
    for (const auto &value : wrong_types)
    {
        EXPECT_THROW(gyro_cfg({{"updatePeriodMsec", value}}), std::runtime_error);
        EXPECT_THROW(acc_cfg({{"updatePeriodMsec", value}}), std::runtime_error);
        EXPECT_THROW(mag_cfg({{"updatePeriodMsec", value}}), std::runtime_error);
        EXPECT_THROW(parse_barometric_model_config(
                         {{"components", {{"sensors", {{"baro", {{"updatePeriodMsec", value}}}}}}}}),
                     std::runtime_error);
        EXPECT_THROW(parse_gps_model_config(
                         {{"components", {{"sensors", {{"gps", {{"updatePeriodMsec", value}}}}}}}}),
                     std::runtime_error);
    }
}

TEST(SensorModels, T_GYR_U2_001_DisallowedMountHasOnlyIntendedError)
{
    testing::internal::CaptureStderr();
    EXPECT_THROW(gyro_cfg({{"mount", nlohmann::json::object()}}), std::runtime_error);
    const auto output = testing::internal::GetCapturedStderr();
    EXPECT_EQ(output.find("unknown sensor setting"), std::string::npos);
}

TEST(SensorModels, T_GYR_U2_002_NoticeListsConfiguredNewSettings)
{
    const nlohmann::json root = {
        {"components", {{"sensors", {{"imu", {{"mount", {{"positionM", {0, 0, 0}}}}}},
                                      {"gyro", {{"resolutionDegPerSec", 0.1}}}}}}}};
    const auto lines = sensor_model_v2_notice_lines(parse_gyro_model_config(root), "drone1", 0);
    EXPECT_EQ(lines, (std::vector<std::string>{"INFO: sensor model v2 [drone1#0] is enabled",
                                               "INFO: sensor model v2 [drone1#0] setting: imu.mount.positionM",
                                               "INFO: sensor model v2 [drone1#0] setting: gyro.resolutionDegPerSec"}));
}

TEST(SensorModels, T_GYR_U2_003_NoticeReportsNoNewSettings)
{
    const auto config = parse_gyro_model_config(nlohmann::json::object());
    const auto lines = sensor_model_v2_notice_lines(config, "plain", 3);
    EXPECT_EQ(lines, (std::vector<std::string>{"INFO: sensor model v2 [plain#3] is enabled",
                                               "INFO: sensor model v2 [plain#3]: no new settings configured"}));
}

TEST(SensorModels, T_GYR_U2_004_NoticeIsEmittedOnceForEachAircraft)
{
    const nlohmann::json root = {
        {"components", {{"sensors", {{"gyro", {{"resolutionDegPerSec", 0.1}}}}}}}};
    const auto config = parse_gyro_model_config(root);
    testing::internal::CaptureStderr();
    announce_sensor_model_v2(config, "notice-drone", 0);
    announce_sensor_model_v2(config, "notice-drone", 0);
    announce_sensor_model_v2(config, "notice-drone", 1);
    const auto output = testing::internal::GetCapturedStderr();
    const auto first_notice = output.find("sensor model v2 [notice-drone#0] is enabled");
    ASSERT_NE(first_notice, std::string::npos);
    EXPECT_EQ(output.find("sensor model v2 [notice-drone#0] is enabled", first_notice + 1), std::string::npos);
    EXPECT_NE(output.find("sensor model v2 [notice-drone#0] setting: gyro.resolutionDegPerSec"),
              std::string::npos);
    EXPECT_NE(output.find("sensor model v2 [notice-drone#1] is enabled"), std::string::npos);
    EXPECT_NE(output.find("sensor model v2 [notice-drone#1] setting: gyro.resolutionDegPerSec"),
              std::string::npos);
}

TEST(SensorModels, T_GYR_U2_010_AxesAndUnits)
{
    auto m = gyro(gyro_cfg());
    expect_vec(m.step({.1, -.2, .3}), {.1, -.2, .3});
}
TEST(SensorModels, T_GYR_U2_020_Yaw90)
{
    auto m = gyro(gyro_cfg(nlohmann::json::object(),
                           {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}}));
    expect_vec(m.step({1, 0, 0}), {0, -1, 0});
}
TEST(SensorModels, T_COM_U3_010_OrientationRequiresExactObjectShape)
{
    EXPECT_THROW(gyro_cfg(nlohmann::json::object(), {{"orientationDeg", {0, 0, 90}}}), std::runtime_error);
    EXPECT_THROW(gyro_cfg(nlohmann::json::object(),
                          {{"orientationDeg", {{"roll", 0}, {"pitch", 0}}}}),
                 std::runtime_error);
    EXPECT_THROW(gyro_cfg(nlohmann::json::object(),
                          {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}, {"extra", 1}}}}),
                 std::runtime_error);
    EXPECT_THROW(gyro_cfg(nlohmann::json::object(),
                          {{"orientationDeg", {{"roll", "zero"}, {"pitch", 0}, {"yaw", 90}}}}),
                 std::runtime_error);
}
TEST(SensorModels, T_GYR_U2_030_PositionIgnored)
{
    auto a = gyro(gyro_cfg(nlohmann::json::object(), {{"positionM", {0, 0, 0}}})),
         b = gyro(gyro_cfg(nlohmann::json::object(), {{"positionM", {.3, -.2, .1}}}));
    expect_vec(a.step({.1, .2, .3}), b.step({.1, .2, .3}));
}
TEST(SensorModels, T_GYR_U2_040_RotateBeforeClamp)
{
    auto m = gyro(gyro_cfg({{"range", {{"fullScaleDegPerSec", 250}}}},
                           {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 45}}}}));
    auto v = m.step({rad(200), rad(200), 0});
    EXPECT_NEAR(v.x, rad(250), 1e-12);
    EXPECT_NEAR(v.y, 0, 1e-12);
}
TEST(SensorModels, T_GYR_U2_050_ResolutionBeforeClamp)
{
    auto m = gyro(gyro_cfg({{"resolutionDegPerSec", 10}, {"range", {{"fullScaleDegPerSec", 245}}}}));
    EXPECT_NEAR(m.step({rad(248), 0, 0}).x, rad(245), 1e-12);
}
TEST(SensorModels, T_GYR_U2_060_NoiseStatistics)
{
    auto m = gyro(gyro_cfg({{"noise", {.01, .02, 0}}}));
    constexpr int n = 20000;
    double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    for (int i = 0; i < n; i++)
    {
        auto v = m.step({});
        sx += v.x;
        sy += v.y;
        sxx += v.x * v.x;
        syy += v.y * v.y;
        sxy += v.x * v.y;
        EXPECT_DOUBLE_EQ(v.z, 0);
    }
    double mx = sx / n, my = sy / n, vx = sxx / n - mx * mx, vy = syy / n - my * my, cov = sxy / n - mx * my;
    EXPECT_LT(std::abs(mx), 4 * .01 / std::sqrt(n));
    EXPECT_LT(std::abs(my), 4 * .02 / std::sqrt(n));
    EXPECT_NEAR(std::sqrt(vx), .01, .0003);
    EXPECT_NEAR(std::sqrt(vy), .02, .0006);
    EXPECT_LT(std::abs(cov / std::sqrt(vx * vy)), .03);
}
TEST(SensorModels, T_GYR_U2_070_ArdupilotTriangle)
{
    auto c = gyro_cfg({{"drift", {{"mode", "ardupilot-sitl"}, {"speedDegPerSecPerMin", .5}, {"timeMin", 5}}}});
    auto m = gyro(c, 150);
    for (double expected : {1.25, 2.5, 1.25, 0.0})
    {
        auto v = m.step({});
        EXPECT_NEAR(v.x, rad(expected), 1e-12);
        EXPECT_DOUBLE_EQ(v.x, v.y);
        EXPECT_DOUBLE_EQ(v.y, v.z);
    }
}
TEST(SensorModels, T_GYR_U2_080_RandomWalkReproducible)
{
    auto c = gyro_cfg({{"drift", {{"mode", "random-walk"}, {"randomWalkDegPerSecSqrtSec", .01}, {"seed", 77}}}});
    auto a = gyro(c), b = gyro(c);
    for (int i = 0; i < 100; i++)
        expect_vec(a.step({}), b.step({}));

    constexpr int seed_count = 2000;
    constexpr int first_step = 100;
    constexpr int second_step = 400;
    constexpr double dt = .01;
    std::vector<Vector3> at_first_time;
    std::vector<Vector3> at_second_time;
    at_first_time.reserve(seed_count);
    at_second_time.reserve(seed_count);
    for (int seed = 0; seed < seed_count; seed++)
    {
        std::uint32_t drift_seed = static_cast<std::uint32_t>(seed) * 2654435761u;
        auto seeded_config =
            gyro_cfg({{"drift", {{"mode", "random-walk"}, {"randomWalkDegPerSecSqrtSec", .01}, {"seed", drift_seed}}}});
        auto model = gyro(seeded_config, dt);
        for (int step = 1; step <= second_step; step++)
        {
            auto value = model.step({});
            if (step == first_step)
                at_first_time.push_back(value);
            if (step == second_step)
                at_second_time.push_back(value);
        }
    }
    double density = rad(.01);
    auto first = vector_statistics(at_first_time);
    auto second = vector_statistics(at_second_time);
    for (double variance : {first.variance.x, first.variance.y, first.variance.z})
        EXPECT_NEAR(variance, density * density * first_step * dt, .1 * density * density * first_step * dt);
    for (double variance : {second.variance.x, second.variance.y, second.variance.z})
        EXPECT_NEAR(variance, density * density * second_step * dt, .1 * density * density * second_step * dt);
    for (double correlation : {first.correlation_xy, first.correlation_xz, first.correlation_yz,
                               second.correlation_xy, second.correlation_xz, second.correlation_yz})
        EXPECT_LT(std::abs(correlation), .05);
}
TEST(SensorModels, T_GYR_U2_081_ZeroSeedReproducible)
{
    auto c = gyro_cfg({{"noise", .01}, {"drift", {{"mode", "random-walk"}, {"randomWalkDegPerSecSqrtSec", .01}}}});
    GyroModel a(c, .001, 1, 0, 0u, "Drone", 0), b(c, .001, 1, 0, 0u, "Drone", 0), other(c, .001, 1, 0, 1u, "Drone", 0);
    bool differs = false;
    for (int i = 0; i < 100; i++)
    {
        auto av = a.step({}), bv = b.step({}), ov = other.step({});
        expect_vec(av, bv);
        differs = differs || av.x != ov.x || av.y != ov.y || av.z != ov.z;
    }
    EXPECT_TRUE(differs);
}
TEST(SensorModels, T_COM_U2_040_ResetRestoresGyroSequence)
{
    auto c = gyro_cfg({{"noise", .01},
                       {"zeroRateOffset", {{"randomStdDegPerSec", .1}}},
                       {"drift", {{"mode", "random-walk"}, {"randomWalkDegPerSecSqrtSec", .01}}}});
    auto m = GyroModel(c, .001, 1, 2, 0u, "Drone", 0);
    std::vector<Vector3> first;
    for (int i = 0; i < 100; i++)
        first.push_back(m.step({double(i), -double(i), .5 * double(i)}));
    m.reset();
    for (int i = 0; i < 100; i++)
        expect_vec(m.step({double(i), -double(i), .5 * double(i)}), first[i]);
}
TEST(SensorModels, T_COM_U2_030_GyroAxisNoiseStreamsAreIndependent)
{
    auto a = gyro(gyro_cfg({{"noise", {1.0, 1.0, 1.0}}}));
    auto b = gyro(gyro_cfg({{"noise", {1.0, 0.0, 1.0}}}));
    for (int i = 0; i < 100; ++i)
        EXPECT_DOUBLE_EQ(a.step({}).x, b.step({}).x);
}
TEST(SensorModels, T_GYR_U2_090_GaussMarkovStatistics)
{
    auto c = gyro_cfg({{"drift", {{"mode", "gauss-markov"}, {"stddevDegPerSec", .005}, {"correlationTimeSec", .1}}}});
    auto m = gyro(c);
    constexpr int n = 500000, lag = 100;
    std::vector<Vector3> values;
    values.reserve(n);
    for (int i = 0; i < n; i++)
        values.push_back(m.step({}));
    auto statistics = vector_statistics(values);
    double s = rad(.005);
    for (double variance : {statistics.variance.x, statistics.variance.y, statistics.variance.z})
        EXPECT_NEAR(variance, s * s, .1 * s * s);
    for (int axis = 0; axis < 3; axis++)
    {
        double covariance = 0;
        for (int i = lag; i < n; i++)
            covariance += (component(values[i], axis) - component(statistics.mean, axis)) *
                          (component(values[i - lag], axis) - component(statistics.mean, axis));
        covariance /= n - lag;
        EXPECT_NEAR(covariance / component(statistics.variance, axis), std::exp(-1.0), .05);
    }
}
TEST(SensorModels, T_GYR_U2_100_DriftInSensorAxes)
{
    auto c = gyro_cfg({{"drift", {{"mode", "ardupilot-sitl"}, {"speedDegPerSecPerMin", 1}, {"timeMin", 5}}}},
                      {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}});
    auto m = gyro(c, 60);
    auto sensor = m.step({});
    expect_vec(sensor, {rad(1), rad(1), rad(1)});
    auto body = sensor_to_body(mounting_rotation({0, 0, 90}), sensor);
    expect_vec(body, {-rad(1), rad(1), rad(1)});
}
TEST(SensorModels, T_GYR_U2_110_RangeBoundaries)
{
    auto c = gyro_cfg({{"range", {{"fullScaleDegPerSec", 250}}}});
    for (auto p : std::vector<std::pair<double, double>>{{-300, -250}, {-250, -250}, {0, 0}, {250, 250}, {300, 250}})
    {
        auto m = gyro(c);
        EXPECT_NEAR(m.step({rad(p.first), 0, 0}).x, rad(p.second), 1e-12);
    }
}
TEST(SensorModels, T_GYR_U2_120_SensorAxisClamp)
{
    auto m = gyro(gyro_cfg({{"range", {{"fullScaleDegPerSec", 250}}}},
                           {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 45}}}}));
    auto v = m.step({rad(200), rad(200), 0});
    EXPECT_NEAR(v.x, rad(250), 1e-12);
    EXPECT_NEAR(v.y, 0, 1e-12);
}
TEST(SensorModels, T_GYR_U2_130_SensitivityAfterRotation)
{
    auto c =
        gyro_cfg({{"sensitivity", {{"scaleErrorPct", {1, 0, 0}}, {"crossAxisPct", {{0, 0, 0}, {2, 0, 0}, {0, 0, 0}}}}}},
                 {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 90}}}});
    auto m = gyro(c);
    auto v = m.step({0, 1, 0});
    EXPECT_NEAR(v.x, 1.01, 1e-12);
    EXPECT_NEAR(v.y, .02, 1e-12);
    EXPECT_NEAR(v.z, 0, 1e-12);
}
TEST(SensorModels, T_GYR_U2_140_FixedZeroRate)
{
    auto m = gyro(gyro_cfg({{"zeroRateOffset", {{"fixedDegPerSec", {.5, -1, .2}}}}}), 1);
    for (int i = 0; i < 100; i++)
        expect_vec(m.step({}), {rad(.5), rad(-1), rad(.2)});
}
TEST(SensorModels, T_GYR_U2_150_RandomZeroRate)
{
    constexpr int n = 20000;
    std::vector<Vector3> values;
    values.reserve(n);
    for (int i = 1; i <= n; i++)
    {
        auto m = GyroModel(gyro_cfg({{"zeroRateOffset", {{"randomStdDegPerSec", 1}}}}), .001, 1, 0, i, "Drone", 0);
        auto value = m.step({});
        expect_vec(m.step({}), value);
        values.push_back(value);
    }
    auto statistics = vector_statistics(values);
    for (double mean : {statistics.mean.x, statistics.mean.y, statistics.mean.z})
        EXPECT_LT(std::abs(mean), 4 * rad(1) / std::sqrt(n));
    for (double variance : {statistics.variance.x, statistics.variance.y, statistics.variance.z})
        EXPECT_NEAR(std::sqrt(variance), rad(1), .03 * rad(1));
    EXPECT_LT(std::abs(statistics.correlation_xy), .03);
    EXPECT_LT(std::abs(statistics.correlation_xz), .03);
    EXPECT_LT(std::abs(statistics.correlation_yz), .03);
}
TEST(SensorModels, T_GYR_U3_010_NoiseParsing)
{
    auto scalar = gyro_cfg({{"noise", .1}}).noise_stddev;
    expect_vec(scalar, {.1, .1, .1});
    auto vector = gyro_cfg({{"noise", {.1, .2, -1}}}).noise_stddev;
    expect_vec(vector, {.1, .2, -1});
    auto negative = gyro(gyro_cfg({{"noise", {-1, -1, -1}}}));
    expect_vec(negative.step({.1, .2, .3}), {.1, .2, .3});
    EXPECT_THROW(gyro_cfg({{"noise", {1, 2}}}), std::runtime_error);
    EXPECT_THROW(gyro_cfg({{"noise", "bad"}}), std::runtime_error);
}
TEST(SensorModels, T_GYR_U3_020_ZeroRateParsing)
{
    EXPECT_EQ(gyro_cfg().zero_rate_mode, GyroModelConfig::ZeroRateMode::None);
    EXPECT_EQ(gyro_cfg({{"zeroRateOffset", {{"fixedDegPerSec", {1, 2, 3}}}}}).zero_rate_mode,
              GyroModelConfig::ZeroRateMode::Fixed);
    EXPECT_EQ(gyro_cfg({{"zeroRateOffset", {{"randomStdDegPerSec", 1}}}}).zero_rate_mode,
              GyroModelConfig::ZeroRateMode::Random);
    EXPECT_THROW(gyro_cfg({{"zeroRateOffset", {{"fixedDegPerSec", {1, 2, 3}}, {"randomStdDegPerSec", 1}}}}),
                 std::runtime_error);
}

namespace
{
BarometricModelConfig baro_cfg(const nlohmann::json &baro = nlohmann::json::object())
{
    return parse_barometric_model_config({{"components", {{"sensors", {{"baro", baro}}}}}});
}
}

TEST(SensorModels, T_BAR_U1_010_Through_030_StandardAtmosphere)
{
    const std::vector<std::tuple<double, double, double>> cases{
        {-500, 107478.0, 18.25}, {-100, 102532.1, 15.65}, {0, 101325.0, 15.0},
        {100, 100129.5, 14.35}, {1000, 89876.3, 8.5}, {11000, 22700.0, -56.38},
        {15000, 12111.8, -56.5}, {20000, 5529.3, -56.5}};
    for (const auto &[height, pressure, temperature] : cases)
    {
        const auto value = standard_atmosphere(height);
        EXPECT_NEAR(value.pressure_pa, pressure, .05);
        EXPECT_NEAR(value.temperature_c, temperature, .005);
    }
    // [SEN-BAR-095] The layers meet at 11,000 m of geopotential height (11,019.07 m geometric).
    const double tropopause = 6356766.0 * 11000.0 / (6356766.0 - 11000.0);
    EXPECT_LT(std::abs(standard_atmosphere(tropopause - .001).pressure_pa -
                       standard_atmosphere(tropopause + .001).pressure_pa), .01);
}

TEST(SensorModels, T_BAR_U1_040_050_110_InverseAtmosphere)
{
    for (double height : {-500.0, 0.0, 100.0, 1000.0, 10999.0, 11000.0, 11001.0, 15000.0, 19999.0})
        EXPECT_NEAR(pressure_altitude(standard_atmosphere(height).pressure_pa), height, 1e-6);
    EXPECT_DOUBLE_EQ(pressure_altitude(standard_atmosphere(20000).pressure_pa), 20000);
}

TEST(SensorModels, T_BAR_U1_060_AtmosphereBoundaryIsMonotonicAndInvertible)
{
    for (double pressure : {50000.0, 110000.0})
        for (double temperature : {-60.0, 50.0})
        {
            double previous = std::numeric_limits<double>::infinity();
            for (int height = -500; height <= 20000; ++height)
            {
                const double value = standard_atmosphere(height, pressure, temperature).pressure_pa;
                EXPECT_LT(value, previous);
                EXPECT_NEAR(pressure_altitude(value, pressure, temperature), height, 1e-6);
                previous = value;
            }
        }
}

TEST(SensorModels, T_BAR_U1_130_AtmosphereClampsAboveCeiling)
{
    const auto at_ceiling = standard_atmosphere(20000);
    const auto above_ceiling = standard_atmosphere(25000);
    EXPECT_NEAR(at_ceiling.pressure_pa, 5529.3, .05);
    EXPECT_DOUBLE_EQ(above_ceiling.pressure_pa, at_ceiling.pressure_pa);
    EXPECT_DOUBLE_EQ(above_ceiling.temperature_c, at_ceiling.temperature_c);
    EXPECT_DOUBLE_EQ(pressure_altitude(at_ceiling.pressure_pa), 20000);
}

TEST(SensorModels, T_BAR_U2_010_020_030_AtmosphereOutputs)
{
    for (const auto &[altitude, sensor_z, pressure] :
         std::vector<std::tuple<double, double, double>>{{0, 0, 101325.0}, {0, -100, 100129.5},
                                                        {1000, 0, 89876.3}, {0, -1000, 89876.3},
                                                        {1000, 1000, 101325.0}, {11000, 0, 22700.0},
                                                        {15000, 0, 12111.8}})
    {
        BarometricModel model(baro_cfg(), .001, 1, 0, 1, "Drone", 0);
        const auto value = model.step(altitude - sensor_z, 101325, 15);
        EXPECT_NEAR(value.pressure_pa, pressure, .05);
        EXPECT_NEAR(value.pressure_altitude_m, altitude - sensor_z, 1e-9);
    }
}

TEST(SensorModels, T_BAR_U2_050_MountOrientationIsIgnoredByConfiguration)
{
    const auto plain = baro_cfg({{"mount", {{"orientationDeg", {{"roll", 0}, {"pitch", 0}, {"yaw", 0}}}}}});
    const auto rotated = baro_cfg({{"mount", {{"orientationDeg", {{"roll", 30}, {"pitch", -45}, {"yaw", 90}}}}}});
    EXPECT_DOUBLE_EQ(plain.mount_position_m.x, rotated.mount_position_m.x);
    EXPECT_DOUBLE_EQ(plain.mount_position_m.y, rotated.mount_position_m.y);
    EXPECT_DOUBLE_EQ(plain.mount_position_m.z, rotated.mount_position_m.z);
}

TEST(SensorModels, T_BAR_U2_060_080_110_OrderAndOutputs)
{
    BarometricModel offset(baro_cfg({{"zeroOffset", {{"fixedM", 1000}}}}), .001, 1, 0, 1, "Drone", 0);
    const auto value = offset.step(0, 101325, 15);
    EXPECT_NEAR(value.pressure_pa, 89876.3, .05);
    EXPECT_NEAR(value.pressure_altitude_m, 1000, 1e-9);
    EXPECT_NEAR(value.temperature_c, 15.0, 1e-12);
    BarometricModel ordered(baro_cfg({{"resolutionPa", 1000}, {"range", {{"maxHPa", 1005}}}}),
                             .001, 1, 0, 1, "Drone", 0);
    const auto limited = ordered.step(0, 101325, 15);
    EXPECT_DOUBLE_EQ(limited.pressure_pa, 100500);
    EXPECT_NEAR(limited.pressure_altitude_m, 68.90, .01);

    BarometricModel clamped(baro_cfg({{"zeroOffset", {{"fixedM", 1000}}},
                                      {"range", {{"minHPa", 950}}}}),
                             .001, 1, 0, 1, "Drone", 0);
    const auto clamped_value = clamped.step(0, 101325, 15);
    EXPECT_DOUBLE_EQ(clamped_value.pressure_pa, 95000);
    EXPECT_NEAR(clamped_value.pressure_altitude_m, 540.38, .01);
    EXPECT_NEAR(clamped_value.temperature_c, 15.0, 1e-12);
}

TEST(SensorModels, T_BAR_U2_070_120_NoisePeriodAndReset)
{
    BarometricModel model(baro_cfg({{"noise", 1}, {"updatePeriodMsec", 4}}),
                           .001, 1, 0, 42, "Drone", 0);
    const auto first = model.step(0, 101325, 15);
    for (int i = 0; i < 3; ++i)
        EXPECT_DOUBLE_EQ(model.step(10, 101325, 15).pressure_altitude_m, first.pressure_altitude_m);
    model.reset();
    EXPECT_DOUBLE_EQ(model.step(0, 101325, 15).pressure_altitude_m, first.pressure_altitude_m);
}

TEST(SensorModels, T_BAR_U2_090_100_ResolutionRangeAndAltitude)
{
    BarometricModel rounded(baro_cfg({{"resolutionPa", 100}}), .001, 1, 0, 1, "Drone", 0);
    const auto rounded_value = rounded.step(100, 101325, 15);
    EXPECT_DOUBLE_EQ(rounded_value.pressure_pa, 100100);
    EXPECT_NEAR(rounded_value.pressure_altitude_m, 102.48, .01);

    for (const auto &[height, pressure, altitude] :
         std::vector<std::tuple<double, double, double>>{{0, 100000, 110.89}, {510, 95346.9, 510},
                                                        {1000, 95000, 540.38}})
    {
        BarometricModel ranged(baro_cfg({{"range", {{"minHPa", 950}, {"maxHPa", 1000}}}}),
                                .001, 1, 0, 1, "Drone", 0);
        const auto value = ranged.step(height, 101325, 15);
        EXPECT_NEAR(value.pressure_pa, pressure, .05);
        EXPECT_NEAR(value.pressure_altitude_m, altitude, .01);
    }
}

TEST(SensorModels, T_BAR_U2_120_NoiseStatistics)
{
    constexpr int count = 20000;
    BarometricModel model(baro_cfg({{"noise", 1}}), .001, 1, 0, 42, "Drone", 0);
    double altitude_sum = 0, pressure_sum = 0;
    std::vector<double> altitudes, pressures;
    altitudes.reserve(count);
    pressures.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        const auto value = model.step(0, 101325, 15);
        altitudes.push_back(value.pressure_altitude_m);
        pressures.push_back(value.pressure_pa);
        altitude_sum += value.pressure_altitude_m;
        pressure_sum += value.pressure_pa;
    }
    const double altitude_mean = altitude_sum / count;
    const double pressure_mean = pressure_sum / count;
    double altitude_variance = 0, pressure_variance = 0;
    for (int i = 0; i < count; ++i)
    {
        altitude_variance += std::pow(altitudes[i] - altitude_mean, 2);
        pressure_variance += std::pow(pressures[i] - pressure_mean, 2);
    }
    EXPECT_LT(std::abs(altitude_mean), 4.0 / std::sqrt(count));
    EXPECT_NEAR(std::sqrt(altitude_variance / count), 1, .03);
    EXPECT_NEAR(std::sqrt(pressure_variance / count), 12.02, .03 * 12.02);
    for (double noise : {0.0, -1.0})
    {
        BarometricModel constant(baro_cfg({{"noise", noise}}), .001, 1, 0, 42, "Drone", 0);
        EXPECT_DOUBLE_EQ(constant.step(0, 101325, 15).pressure_pa, constant.step(0, 101325, 15).pressure_pa);
    }
}

TEST(SensorModels, T_COM_U2_030_BarometerOffsetDoesNotChangeNoiseSequence)
{
    BarometricModel a(baro_cfg({{"noise", 1}}), .001, 1, 0, 42, "Drone", 0);
    BarometricModel b(baro_cfg({{"noise", 1}, {"zeroOffset", {{"randomStdM", 4}}}}),
                       .001, 1, 0, 42, "Drone", 0);
    const auto offset = b.step(0, 101325, 15).pressure_altitude_m
        - a.step(0, 101325, 15).pressure_altitude_m;
    for (int i = 0; i < 100; ++i)
        EXPECT_NEAR(b.step(0, 101325, 15).pressure_altitude_m
                    - a.step(0, 101325, 15).pressure_altitude_m, offset, 1e-9);
}

TEST(SensorModels, T_BAR_U2_130_GaussMarkovStatistics)
{
    constexpr int count = 500000;
    constexpr int lag = 100;
    BarometricModel model(baro_cfg({{"drift", {{"mode", "gauss-markov"}, {"stddevM", 1},
                                                {"correlationTimeSec", .1}}}}),
                           .001, 1, 0, 42, "Drone", 0);
    std::vector<double> values;
    values.reserve(count);
    double sum = 0;
    for (int i = 0; i < count; ++i)
    {
        const double value = model.step(0, 101325, 15).pressure_altitude_m;
        values.push_back(value);
        sum += value;
    }
    const double mean = sum / count;
    double variance = 0, covariance = 0;
    for (int i = 0; i < count; ++i) variance += std::pow(values[i] - mean, 2);
    for (int i = lag; i < count; ++i) covariance += (values[i] - mean) * (values[i - lag] - mean);
    variance /= count;
    covariance /= count - lag;
    EXPECT_NEAR(variance, 1, .1);
    EXPECT_NEAR(covariance / variance, std::exp(-1.0), .05);
}

TEST(SensorModels, T_BAR_U2_140_150_ZeroOffsetIsStableAndDistributed)
{
    BarometricModel fixed(baro_cfg({{"zeroOffset", {{"fixedM", 1.5}}}}), .001, 1, 0, 1, "Drone", 0);
    for (int i = 0; i < 1000; ++i) EXPECT_NEAR(fixed.step(0, 101325, 15).pressure_altitude_m, 1.5, 1e-9);
    constexpr int count = 20000;
    double sum = 0, square_sum = 0;
    for (int seed = 1; seed <= count; ++seed)
    {
        BarometricModel random(baro_cfg({{"zeroOffset", {{"randomStdM", 4}}}}),
                                .001, 1, 0, seed, "Drone", 0);
        const double first = random.step(0, 101325, 15).pressure_altitude_m;
        EXPECT_DOUBLE_EQ(random.step(0, 101325, 15).pressure_altitude_m, first);
        sum += first;
        square_sum += first * first;
    }
    const double mean = sum / count;
    EXPECT_LT(std::abs(mean), 16.0 / std::sqrt(count));
    EXPECT_NEAR(std::sqrt(square_sum / count - mean * mean), 4, .12);
}

TEST(SensorModels, T_BAR_U2_160_DelayAppliesToHeightButNotWeather)
{
    BarometricModel model(baro_cfg(), .001, 1, 100, 1, "Drone", 0);
    for (int i = 0; i < 1000; ++i) model.step(0, 101325, 15);
    for (int i = 0; i < 100; ++i) EXPECT_NEAR(model.step(10, 101325, 15).pressure_pa, 101325, .05);
    EXPECT_NEAR(model.step(10, 101325, 15).pressure_pa, 101204.9, .05);
    const auto weather = model.step(10, 100325, 25);
    EXPECT_NEAR(weather.pressure_pa, 100210.1, .05);
    EXPECT_NEAR(weather.temperature_c, 24.935, .005);
}

TEST(SensorModels, T_BAR_U2_190_200_WarningsAndInvalidAtmosphereTransitions)
{
    BarometricModel model(baro_cfg(), .001, 1, 0, 1, "Drone", 0);
    testing::internal::CaptureStderr();
    for (int i = 0; i < 1000; ++i) model.step(25000, 101325, 15);
    model.step(30000, 101325, 15);
    const auto ceiling_output = testing::internal::GetCapturedStderr();
    const auto ceiling_warning = ceiling_output.find("exceeds 20000 m");
    ASSERT_NE(ceiling_warning, std::string::npos);
    EXPECT_EQ(ceiling_output.find("exceeds 20000 m", ceiling_warning + 1), std::string::npos);

    BarometricModel atmosphere(baro_cfg(), .001, 1, 0, 1, "Drone", 0);
    EXPECT_DOUBLE_EQ(atmosphere.step(0, 100000, 15).pressure_pa, 100000);
    testing::internal::CaptureStderr();
    EXPECT_DOUBLE_EQ(atmosphere.step(0, 0, 15).pressure_pa, 100000);
    EXPECT_DOUBLE_EQ(atmosphere.step(0, -1, 15).pressure_pa, 100000);
    EXPECT_NEAR(atmosphere.step(0, 100000, -201.65).temperature_c, 15, 1e-12);
    EXPECT_NEAR(atmosphere.step(0, 100000, -201.60).temperature_c, -201.60, 1e-12);
    atmosphere.step(0, 0, 15);
    const auto invalid_output = testing::internal::GetCapturedStderr();
    const auto first_warning = invalid_output.find("invalid sea-level atmosphere");
    ASSERT_NE(first_warning, std::string::npos);
    const auto second_warning = invalid_output.find("invalid sea-level atmosphere", first_warning + 1);
    ASSERT_NE(second_warning, std::string::npos);
    EXPECT_EQ(invalid_output.find("invalid sea-level atmosphere", second_warning + 1), std::string::npos);

    BarometricModel initially_invalid(baro_cfg(), .001, 1, 0, 1, "Drone", 0);
    testing::internal::CaptureStderr();
    const auto fallback = initially_invalid.step(0, 0, 15);
    testing::internal::GetCapturedStderr();
    EXPECT_DOUBLE_EQ(fallback.pressure_pa, 101325);
    EXPECT_DOUBLE_EQ(fallback.temperature_c, 15);
}

TEST(SensorModels, T_BAR_U3_010_Through_040_ConfigValidation)
{
    EXPECT_DOUBLE_EQ(baro_cfg({{"noise", 1}}).noise_stddev_m, 1);
    EXPECT_DOUBLE_EQ(baro_cfg({{"noise", 0}}).noise_stddev_m, 0);
    EXPECT_DOUBLE_EQ(baro_cfg({{"noise", -1}}).noise_stddev_m, -1);
    EXPECT_DOUBLE_EQ(baro_cfg().noise_stddev_m, 0);
    EXPECT_THROW(baro_cfg({{"noise", "bad"}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"noise", {1, 2, 3}}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"noise", std::numeric_limits<double>::quiet_NaN()}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"noise", 1}, {"noiseDensity", 1}, {"updatePeriodMsec", 1}}),
                 std::runtime_error);
    EXPECT_THROW(baro_cfg({{"noiseDensity", 1}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"sampleCount", 2}}), std::runtime_error);
    EXPECT_EQ(baro_cfg().zero_offset_mode, BarometricModelConfig::ZeroOffsetMode::None);
    EXPECT_EQ(baro_cfg({{"zeroOffset", {{"fixedM", 1}}}}).zero_offset_mode,
              BarometricModelConfig::ZeroOffsetMode::Fixed);
    EXPECT_EQ(baro_cfg({{"zeroOffset", {{"randomStdM", 1}}}}).zero_offset_mode,
              BarometricModelConfig::ZeroOffsetMode::Random);
    EXPECT_THROW(baro_cfg({{"zeroOffset", {{"fixedM", 1}, {"randomStdM", 1}}}}),
                 std::runtime_error);
    const auto drift = baro_cfg({{"drift", {{"mode", "gauss-markov"}, {"stddevM", 1},
                                              {"correlationTimeSec", 2}}}});
    EXPECT_DOUBLE_EQ(drift.drift_stddev_m, 1);
    EXPECT_DOUBLE_EQ(drift.correlation_time_sec, 2);
    EXPECT_THROW(baro_cfg({{"drift", {{"mode", "random-walk"}}}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"drift", {{"mode", "ardupilot-sitl"}}}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"drift", {{"mode", "gauss-markov"}, {"stddevM", -1},
                                      {"correlationTimeSec", 1}}}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"drift", {{"mode", "gauss-markov"}, {"stddevM", 1},
                                      {"correlationTimeSec", 0}}}}), std::runtime_error);
    EXPECT_DOUBLE_EQ(baro_cfg().resolution_pa, 0);
    EXPECT_THROW(baro_cfg({{"resolutionPa", 0}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"resolutionPa", -1}}), std::runtime_error);
    EXPECT_THROW(baro_cfg({{"resolutionPa", std::numeric_limits<double>::quiet_NaN()}}),
                 std::runtime_error);
    EXPECT_FALSE(baro_cfg({{"range", {{"minHPa", 950}}}}).maximum_pressure_pa.has_value());
    EXPECT_FALSE(baro_cfg({{"range", {{"maxHPa", 1000}}}}).minimum_pressure_pa.has_value());
    EXPECT_THROW(baro_cfg({{"range", {{"minHPa", 1000}, {"maxHPa", 1000}}}}),
                 std::runtime_error);
    EXPECT_THROW(baro_cfg({{"range", {{"minHPa", 1001}, {"maxHPa", 1000}}}}),
                 std::runtime_error);
    EXPECT_THROW(baro_cfg({{"range", {{"minHPa", 0}}}}), std::runtime_error);
}

TEST(SensorModels, T_BAR_U2_NotificationIncludesBaroAndIsOncePerAircraft)
{
    const auto gyro = parse_gyro_model_config(nlohmann::json::object());
    const auto acceleration = parse_acceleration_model_config(nlohmann::json::object());
    const auto geomagnet = parse_geomagnet_model_config(nlohmann::json::object());
    const auto barometric = baro_cfg({{"resolutionPa", .1}});
    const auto lines = sensor_model_v2_notice_lines(gyro, acceleration, geomagnet, barometric,
                                                     "baro-notice", 0);
    EXPECT_NE(std::find(lines.begin(), lines.end(),
                        "INFO: sensor model v2 [baro-notice#0] setting: baro.resolutionPa"), lines.end());
    testing::internal::CaptureStderr();
    announce_sensor_model_v2(gyro, acceleration, geomagnet, barometric, "baro-notice-once", 0);
    announce_sensor_model_v2(gyro, acceleration, geomagnet, barometric, "baro-notice-once", 0);
    const auto output = testing::internal::GetCapturedStderr();
    const auto first = output.find("sensor model v2 [baro-notice-once#0] is enabled");
    ASSERT_NE(first, std::string::npos);
    EXPECT_EQ(output.find("sensor model v2 [baro-notice-once#0] is enabled", first + 1), std::string::npos);
    EXPECT_NE(output.find("setting: baro.resolutionPa"), std::string::npos);
}

namespace
{
// The tests below sample every 1 ms physics step; v2 settings need the period written (SEN-GPS-175).
nlohmann::json with_gps_period(nlohmann::json gps)
{
    if (gps.is_object() && !gps.contains("updatePeriodMsec"))
        gps["updatePeriodMsec"] = 1;
    return gps;
}
GpsModelConfig gps_cfg(const nlohmann::json &gps = nlohmann::json::object())
{
    return parse_gps_model_config({{"components", {{"sensors", {{"gps", with_gps_period(gps)}}}}}});
}
GpsModelConfig gps_cfg_as_written(const nlohmann::json &gps)
{
    return parse_gps_model_config({{"components", {{"sensors", {{"gps", gps}}}}}});
}
Matrix3 identity_matrix()
{
    Matrix3 r{};
    r.v = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    return r;
}
Vector3 gps_error_m(const GpsModelValue &value, double latitude, double longitude, double altitude)
{
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    const double phi = latitude * pi / 180.0;
    const double denominator = 1.0 - e2 * std::sin(phi) * std::sin(phi);
    const double meridian = a * (1.0 - e2) / std::pow(denominator, 1.5);
    const double prime_vertical = a / std::sqrt(denominator);
    return {(value.latitude_deg - latitude) * pi / 180.0 * (meridian + altitude),
            (value.longitude_deg - longitude) * pi / 180.0 * (prime_vertical + altitude) * std::cos(phi),
            altitude - value.altitude_m};
}
}

TEST(SensorModels, T_GPS_U1_010_030_Wgs84LocalConversion)
{
    GpsModel model(gps_cfg(), .001, 1, 0, 1, "gps", 0, 35.7, 139.7, 100);
    const auto value = model.step({10, 10, -10}, {1, -2, 3}, {}, identity_matrix());
    constexpr double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f);
    const double phi = 35.7 * pi / 180.0, d = 1.0 - e2 * std::sin(phi) * std::sin(phi);
    const double m = a * (1.0 - e2) / std::pow(d, 1.5), n = a / std::sqrt(d);
    EXPECT_NEAR(value.latitude_deg, 35.7 + 10 / (m + 100) * 180 / pi, 1e-12);
    EXPECT_NEAR(value.longitude_deg, 139.7 + 10 / ((n + 100) * std::cos(phi)) * 180 / pi, 1e-12);
    EXPECT_DOUBLE_EQ(value.altitude_m, 110);
    expect_vec(value.velocity_ned_mps, {1, -2, 3});
}

// [SEN-GPS-175] A GPS written with the v2 settings states its period; a legacy GPS loads as before.
TEST(SensorModels, T_GPS_U3_175_V2SettingsRequireTheUpdatePeriod)
{
    for (const char *key : {"mount", "positionNoise", "velocityNoise", "drift", "accuracy", "satellitesVisible",
                            "softDelayMsec"})
    {
        nlohmann::json gps = {{key, nlohmann::json::object()}};
        if (std::string(key) == "satellitesVisible" || std::string(key) == "softDelayMsec")
            gps[key] = 10;
        EXPECT_THROW(gps_cfg_as_written(gps), std::runtime_error) << key;
    }
    EXPECT_NO_THROW(gps_cfg_as_written({{"sampleCount", 1}, {"delayMsec", 100}}));
    EXPECT_NO_THROW(gps_cfg_as_written({{"mount", {{"positionM", {0.1, 0.0, 0.0}}}}, {"updatePeriodMsec", 100}}));
}

// T-GPS-U1-070 [SEN-GPS-270] The vehicle location for ArduPilot SITL is the body origin's truth: no lever arm, noise or delay.
TEST(SensorModels, T_GPS_U1_070_ReferenceStateIsTheBodyOriginTruth)
{
    auto config = gps_cfg({{"mount", {{"positionM", {0.3, 0.0, -0.2}}}},
                           {"positionNoise", {{"horizontalM", 0.5}, {"verticalM", 0.5}}}, {"delayMsec", 50}});
    GpsModel model(config, .001, 1, 50, 1, "gps", 0, 35.7, 139.7, 100);
    GpsModel origin(gps_cfg(), .001, 1, 0, 1, "gps", 0, 35.7, 139.7, 100);
    const auto truth = origin.step({10, 10, -10}, {1, -2, 3}, {}, identity_matrix());
    model.step({10, 10, -10}, {1, -2, 3}, {}, identity_matrix());
    const auto reference = model.reference_state({10, 10, -10}, {1, -2, 3});
    EXPECT_DOUBLE_EQ(reference.latitude_deg, truth.latitude_deg);
    EXPECT_DOUBLE_EQ(reference.longitude_deg, truth.longitude_deg);
    EXPECT_DOUBLE_EQ(reference.altitude_m, truth.altitude_m);
    expect_vec(reference.velocity_ned_mps, {1, -2, 3});
}

TEST(SensorModels, T_GPS_U2_020_030_GroundSpeedAndCourse)
{
    GpsModel model(gps_cfg(), .001, 1, 0, 1, "gps", 0, 0, 0, 0);
    auto value = model.step({}, {3, 4, 5}, {}, identity_matrix());
    EXPECT_DOUBLE_EQ(value.ground_speed_mps, 5);
    EXPECT_NEAR(value.course_deg, 53.13010235415598, 1e-12);
    EXPECT_TRUE(value.course_valid);
    value = model.step({}, {0, .0999, 0}, {}, identity_matrix());
    EXPECT_FALSE(value.course_valid);
    value = model.step({}, {0, .1, 0}, {}, identity_matrix());
    EXPECT_TRUE(value.course_valid);
    EXPECT_DOUBLE_EQ(value.course_deg, 90);
}

TEST(SensorModels, T_GPS_U2_040_050_AccuracyAndAntennaState)
{
    const auto config = gps_cfg({{"mount", {{"positionM", {0.2, 0, 0}}}},
                                 {"accuracy", {{"horizontalM", 1.5}, {"verticalM", 3.0}, {"speedMps", .3}}},
                                 {"satellitesVisible", 0}});
    GpsModel model(config, .001, 1, 0, 1, "gps", 0, 0, 0, 0);
    const auto value = model.step({}, {}, {0, 0, 1}, identity_matrix());
    EXPECT_NEAR(value.longitude_deg * 6378137.0 * pi / 180.0, 0, 1e-12);
    EXPECT_NEAR(value.velocity_ned_mps.y, .2, 1e-12);
    EXPECT_DOUBLE_EQ(value.horizontal_accuracy_m, 1.5);
    EXPECT_DOUBLE_EQ(value.vertical_accuracy_m, 3.0);
    EXPECT_DOUBLE_EQ(value.speed_accuracy_mps, .3);
    EXPECT_EQ(value.satellites_visible, 0);
}

TEST(SensorModels, T_GPS_U2_070_120_DelayThenSampling)
{
    GpsModel model(gps_cfg({{"updatePeriodMsec", 4}}), .001, 1, 3, 1, "gps", 0, 0, 0, 0);
    std::vector<double> latitude;
    for (int i = 0; i < 9; ++i)
        latitude.push_back(model.step({double(i), 0, 0}, {}, {}, identity_matrix()).latitude_deg);
    EXPECT_EQ(latitude[0], latitude[1]);
    EXPECT_EQ(latitude[1], latitude[2]);
    EXPECT_EQ(latitude[2], latitude[3]);
    EXPECT_EQ(latitude[4], latitude[5]);
    EXPECT_EQ(latitude[5], latitude[6]);
    EXPECT_EQ(latitude[6], latitude[7]);
    EXPECT_LT(latitude[4], latitude[8]);
}

TEST(SensorModels, T_GPS_U3_010_020_030_ConfigValidation)
{
    const auto defaults = gps_cfg();
    EXPECT_DOUBLE_EQ(defaults.horizontal_accuracy_m, .5);
    EXPECT_DOUBLE_EQ(defaults.vertical_accuracy_m, .75);
    EXPECT_DOUBLE_EQ(defaults.speed_accuracy_mps, .5);
    EXPECT_EQ(defaults.satellites_visible, 10);
    EXPECT_EQ(gps_cfg({{"satellitesVisible", 0}}).satellites_visible, 0);
    EXPECT_EQ(gps_cfg({{"satellitesVisible", 254}}).satellites_visible, 254);
    EXPECT_THROW(gps_cfg({{"satellitesVisible", -1}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"satellitesVisible", 255}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"satellitesVisible", 1.5}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"noiseDensity", 1}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"sampleCount", 2}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"drift", {{"mode", "gauss-markov"}, {"horizontalM", -1},
                                      {"verticalM", 1}, {"correlationTimeSec", 1}}}}), std::runtime_error);
}

TEST(SensorModels, T_GPS_U2_010_060_AxesCourseAndUnusedOrientation)
{
    const auto config = gps_cfg({{"mount", {{"orientationDeg", {{"roll", 30}, {"pitch", -20},
                                                                  {"yaw", 90}}}}}});
    GpsModel model(config, .001, 1, 0, 1, "gps-axes", 0, 35.7, 139.0, 100);
    const auto value = model.step({10, 20, -30}, {-1, 2, 3}, {}, identity_matrix());
    const auto error = gps_error_m(value, 35.7, 139.0, 100);
    EXPECT_NEAR(error.x, 10, 1e-8);
    EXPECT_NEAR(error.y, 20, 1e-8);
    EXPECT_NEAR(error.z, -30, 1e-12);
    expect_vec(value.velocity_ned_mps, {-1, 2, 3});
    EXPECT_NEAR(value.course_deg, 116.56505117707799, 1e-12);
}

TEST(SensorModels, T_GPS_U2_030_AllCardinalCourses)
{
    GpsModel model(gps_cfg(), .001, 1, 0, 1, "gps-course", 0, 0, 0, 0);
    const std::pair<Vector3, double> cases[] = {
        {{1, 0, 0}, 0}, {{0, 1, 0}, 90}, {{-1, 0, 0}, 180}, {{0, -1, 0}, 270}};
    for (const auto &[velocity, course] : cases)
        EXPECT_DOUBLE_EQ(model.step({}, velocity, {}, identity_matrix()).course_deg, course);
    const auto almost_north = model.step({}, {1, -1e-12, 0}, {}, identity_matrix());
    EXPECT_GE(almost_north.course_deg, 0);
    EXPECT_LT(almost_north.course_deg, 360);
}

TEST(SensorModels, T_GPS_U2_080_090_PositionAndVelocityNoiseStatistics)
{
    const auto config = gps_cfg({{"positionNoise", {{"horizontalM", 1.0}, {"verticalM", 2.0}}},
                                 {"velocityNoise", {{"horizontalMps", .1}, {"verticalMps", .2}}}});
    GpsModel model(config, .001, 1, 0, 123, "gps-statistics", 0, 60, 139, 1000);
    std::vector<Vector3> positions;
    std::vector<Vector3> velocities;
    positions.reserve(20000);
    velocities.reserve(20000);
    for (int i = 0; i < 20000; ++i)
    {
        const auto value = model.step({}, {}, {}, identity_matrix());
        positions.push_back(gps_error_m(value, 60, 139, 1000));
        velocities.push_back(value.velocity_ned_mps);
    }
    const auto position = vector_statistics(positions);
    const auto velocity = vector_statistics(velocities);
    for (int axis = 0; axis < 3; ++axis)
    {
        const double position_sigma = axis == 2 ? 2.0 : 1.0;
        const double velocity_sigma = axis == 2 ? .2 : .1;
        EXPECT_LT(std::abs(component(position.mean, axis)), 4 * position_sigma / std::sqrt(20000.0));
        EXPECT_LT(std::abs(component(velocity.mean, axis)), 4 * velocity_sigma / std::sqrt(20000.0));
        EXPECT_NEAR(std::sqrt(component(position.variance, axis)), position_sigma, .03 * position_sigma);
        EXPECT_NEAR(std::sqrt(component(velocity.variance, axis)), velocity_sigma, .03 * velocity_sigma);
    }
    EXPECT_LT(std::abs(position.correlation_xy), .03);
    EXPECT_LT(std::abs(velocity.correlation_xy), .03);
}

TEST(SensorModels, T_GPS_U2_100_GaussMarkovStatistics)
{
    const auto config = gps_cfg({{"drift", {{"mode", "gauss-markov"}, {"horizontalM", 1.0},
                                              {"verticalM", 2.0}, {"correlationTimeSec", .1}}}});
    GpsModel model(config, .01, 1, 0, 456, "gps-drift", 0, 0, 0, 0);
    std::vector<Vector3> values;
    values.reserve(50000);
    for (int i = 0; i < 50000; ++i)
        values.push_back(gps_error_m(model.step({}, {}, {}, identity_matrix()), 0, 0, 0));
    const auto statistics = vector_statistics(values);
    EXPECT_NEAR(statistics.variance.x, 1, .1);
    EXPECT_NEAR(statistics.variance.y, 1, .1);
    EXPECT_NEAR(statistics.variance.z, 4, .4);
    EXPECT_LT(std::abs(statistics.correlation_xy), .03);
    for (int axis = 0; axis < 3; ++axis)
    {
        double covariance = 0;
        for (std::size_t i = 10; i < values.size(); ++i)
            covariance += component(values[i], axis) * component(values[i - 10], axis);
        covariance /= values.size() - 10;
        EXPECT_NEAR(covariance / component(statistics.variance, axis), std::exp(-1.0), .05);
    }
}

TEST(SensorModels, T_GPS_U2_110_And_U3_NoErrorAndValidationBoundaries)
{
    const nlohmann::json no_errors[] = {
        nlohmann::json::object(),
        {{"positionNoise", {{"horizontalM", 0}, {"verticalM", -1}}},
         {"velocityNoise", {{"horizontalMps", -1}, {"verticalMps", 0}}}},
        {{"drift", {{"mode", "gauss-markov"}, {"horizontalM", 0},
                     {"verticalM", 0}, {"correlationTimeSec", 1}}}}};
    for (const auto &json : no_errors)
    {
        GpsModel model(gps_cfg(json), .001, 1, 0, 1, "gps-none", 0, 0, 0, 0);
        const auto value = model.step({1, 2, 3}, {4, 5, 6}, {}, identity_matrix());
        expect_vec(gps_error_m(value, 0, 0, 0), {1, 2, 3});
        expect_vec(value.velocity_ned_mps, {4, 5, 6});
    }
    EXPECT_THROW(gps_cfg({{"positionNoise", {{"horizontalM", "bad"}}}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"velocityNoise", {{"verticalMps", INFINITY}}}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"drift", {{"mode", "other"}, {"horizontalM", 1},
                                      {"verticalM", 1}, {"correlationTimeSec", 1}}}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"drift", {{"mode", "gauss-markov"}, {"horizontalM", 1},
                                      {"verticalM", 1}, {"correlationTimeSec", 0}}}}), std::runtime_error);
    EXPECT_THROW(gps_cfg({{"accuracy", {{"horizontalM", "bad"}}}}), std::runtime_error);
}

TEST(SensorModels, T_GPS_U2_030_040_ResetAndIndependentRandomStreams)
{
    const auto first_config = gps_cfg({{"positionNoise", {{"horizontalM", 1.0}, {"verticalM", 1.0}}},
                                       {"velocityNoise", {{"horizontalMps", .1}, {"verticalMps", .1}}}});
    const auto second_config = gps_cfg({{"positionNoise", {{"horizontalM", 1.0}, {"verticalM", 1.0}}},
                                        {"velocityNoise", {{"horizontalMps", 0.0}, {"verticalMps", 0.0}}}});
    GpsModel first(first_config, .001, 1, 0, 789, "gps-reset", 0, 0, 0, 0);
    GpsModel second(second_config, .001, 1, 0, 789, "gps-reset", 0, 0, 0, 0);
    std::vector<Vector3> initial;
    for (int i = 0; i < 20; ++i)
    {
        const auto a = gps_error_m(first.step({}, {}, {}, identity_matrix()), 0, 0, 0);
        const auto b = gps_error_m(second.step({}, {}, {}, identity_matrix()), 0, 0, 0);
        expect_vec(a, b);
        initial.push_back(a);
    }
    first.reset();
    for (const auto &expected : initial)
        expect_vec(gps_error_m(first.step({}, {}, {}, identity_matrix()), 0, 0, 0), expected);
}

TEST(SensorModels, T_GPS_U2_NotificationIncludesGpsAndIsOncePerAircraft)
{
    const auto empty = nlohmann::json::object();
    const auto gps = gps_cfg({{"accuracy", {{"horizontalM", 1.5}}}, {"satellitesVisible", 12}});
    const auto lines = sensor_model_v2_notice_lines(parse_gyro_model_config(empty),
        parse_acceleration_model_config(empty), parse_geomagnet_model_config(empty),
        parse_barometric_model_config(empty), gps, "gps-notice", 0);
    EXPECT_NE(std::find(lines.begin(), lines.end(),
        "INFO: sensor model v2 [gps-notice#0] setting: gps.accuracy"), lines.end());
    EXPECT_NE(std::find(lines.begin(), lines.end(),
        "INFO: sensor model v2 [gps-notice#0] setting: gps.satellitesVisible"), lines.end());
    testing::internal::CaptureStderr();
    announce_sensor_model_v2(parse_gyro_model_config(empty), parse_acceleration_model_config(empty),
        parse_geomagnet_model_config(empty), parse_barometric_model_config(empty), gps, "gps-once", 0);
    announce_sensor_model_v2(parse_gyro_model_config(empty), parse_acceleration_model_config(empty),
        parse_geomagnet_model_config(empty), parse_barometric_model_config(empty), gps, "gps-once", 0);
    const auto output = testing::internal::GetCapturedStderr();
    const auto first = output.find("sensor model v2 [gps-once#0] is enabled");
    ASSERT_NE(first, std::string::npos);
    EXPECT_EQ(output.find("sensor model v2 [gps-once#0] is enabled", first + 1), std::string::npos);
}

// [SEN-COM-177] A period that is not a whole number of physics steps is measured at the rounded period, with a
// warning that names the sensor; a whole number of steps gives no warning.
TEST(SensorModels, T_COM_U1_160_PeriodNotWholeStepsWarns)
{
    testing::internal::CaptureStderr();
    auto every_step = gyro(gyro_cfg({{"updatePeriodMsec", 4}}), .003);
    AccelerationModel acc(acc_cfg({{"updatePeriodMsec", 4}}), .003, 1, 0, 1, "Drone", 0);
    BarometricModel baro(baro_cfg({{"updatePeriodMsec", 25}}), .003, 1, 0, 1, "Drone", 0);
    GpsModel gps(gps_cfg({{"updatePeriodMsec", 100}}), .003, 1, 0, 1, "Drone", 0, 47.0, -122.0, 100.0);
    const auto warned = testing::internal::GetCapturedStderr();
    for (const char *sensor : {"gyro", "acc", "baro", "gps"})
    {
        EXPECT_NE(warned.find(std::string("components.sensors.") + sensor + ".updatePeriodMsec"), std::string::npos)
            << sensor;
    }
    EXPECT_NE(warned.find("measuring every 1 steps (3 ms)"), std::string::npos) << warned;
    // 4 ms on a 3 ms step rounds to one step: a new value every step.
    const Vector3 first = every_step.step({1, 0, 0});
    const Vector3 second = every_step.step({2, 0, 0});
    EXPECT_EQ(first.x, 1.0);
    EXPECT_EQ(second.x, 2.0);

    testing::internal::CaptureStderr();
    auto whole = gyro(gyro_cfg({{"updatePeriodMsec", 4}}), .001);
    BarometricModel whole_baro(baro_cfg({{"updatePeriodMsec", 20}}), .0005, 1, 0, 1, "Drone", 0);
    EXPECT_EQ(testing::internal::GetCapturedStderr().find("updatePeriodMsec"), std::string::npos);
}

#else
TEST(SensorModels, T_GYR_U2_004_OffDoesNotAnnounceV2)
{
    testing::internal::CaptureStderr();
    EXPECT_NO_THROW(gyro_cfg());
    const auto output = testing::internal::GetCapturedStderr();
    EXPECT_EQ(output.find("sensor model v2"), std::string::npos);
}

#endif
