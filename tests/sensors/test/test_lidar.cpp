#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/lidar_sensor_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStepNs = 1000000;
constexpr double kPi = 3.14159265358979323846;

class TempConfig {
public:
    explicit TempConfig(const std::string& text)
    {
        static std::uint64_t serial = 0;
        path_ = std::filesystem::temp_directory_path() /
                ("hakoniwa-lidar-" + std::to_string(++serial) + ".json");
        std::ofstream(path_) << text;
    }
    ~TempConfig() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

class ScriptedCaster : public sa::IRayCaster {
public:
    struct Call {
        std::uint64_t step;
        std::vector<sa::Ray> rays;
        double max_distance;
    };

    std::uint64_t current_step{0};
    double wall_distance{2.0};
    std::function<double(const sa::Ray&)> distance = [this](const sa::Ray&) { return wall_distance; };
    std::function<std::int32_t(const sa::Ray&)> object_id =
        [](const sa::Ray&) { return 9; };
    std::vector<Call> calls;

    bool cast(const sa::Ray* input, std::size_t count, double maximum, sa::RayHit* hits,
              std::string&) override
    {
        calls.push_back({current_step, std::vector<sa::Ray>(input, input + count), maximum});
        for (std::size_t i = 0; i < count; ++i) {
            hits[i] = {};
            const double value = distance(input[i]);
            if (value >= 0.0 && value <= maximum) {
                hits[i] = {true, value, object_id(input[i]), {}};
            }
        }
        return true;
    }
};

std::string lidar_config(std::uint32_t horizontal, std::uint32_t vertical = 1,
                         const std::string& mode = "rotating", double rate = 10.0,
                         const std::string& additions = "")
{
    return "{\"type\":\"lidar\",\"update_rate\":" + std::to_string(rate) +
           ",\"scan_mode\":\"" + mode + "\",\"scan\":{" +
           "\"horizontal\":{\"samples\":" + std::to_string(horizontal) +
           ",\"min_angle\":-1.0,\"max_angle\":1.0}," +
           "\"vertical\":{\"samples\":" + std::to_string(vertical) +
           ",\"min_angle\":-0.2,\"max_angle\":0.2}}," +
           "\"range\":{\"min\":0.1,\"max\":10.0,\"resolution\":0}" + additions + "}";
}

std::unique_ptr<sa::ILidarSensorAdapter> make(const std::string& text, ScriptedCaster& caster,
                                               std::uint64_t seed = 2468,
                                               std::string* creation_error = nullptr)
{
    TempConfig config(text);
    sa::LidarAdapterSetup setup{};
    setup.config_path = config.path();
    setup.ray_caster = &caster;
    setup.physics_step_ns = kStepNs;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    setup.vehicle_name = "vehicle";
    setup.vehicle_index = 3;
    setup.sensor_name = "lidar";
    std::string error;
    auto adapter = ms::create_lidar_sensor_adapter(setup, error);
    if (creation_error) *creation_error = error;
    return adapter;
}

sa::LidarScan step(sa::ILidarSensorAdapter& adapter, ScriptedCaster& caster, std::uint64_t index,
                   const sa::SensorPoseTruth* supplied_truth = nullptr)
{
    sa::SensorPoseTruth truth = supplied_truth ? *supplied_truth : sa::SensorPoseTruth{};
    truth.sim_time_ns = index * kStepNs;
    caster.current_step = index;
    sa::LidarScan output{};
    std::string error;
    EXPECT_TRUE(adapter.step(truth, output, error)) << error;
    return output;
}

sa::LidarScan finish_scan(sa::ILidarSensorAdapter& adapter, ScriptedCaster& caster,
                          std::uint64_t first_step, std::uint64_t period_steps)
{
    sa::LidarScan output{};
    for (std::uint64_t index = first_step; index < first_step + period_steps; ++index) {
        output = step(adapter, caster, index);
    }
    return output;
}

void expect_relative_float(float actual, double expected)
{
    EXPECT_NEAR(actual, expected, std::max(1.0, std::fabs(expected)) * 1e-6);
}

TEST(Lidar, T_LID_U2_010_AngleGrid)
{
    // RIS-COM-010, RIS-LID-010
    ScriptedCaster caster;
    auto adapter = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":5,"min_angle":-1.0,"max_angle":1.0},
        "vertical":{"samples":3,"min_angle":-0.3,"max_angle":0.3}},
        "range":{"min":0.1,"max":10}})", caster);
    ASSERT_NE(adapter, nullptr);
    const auto scan = step(*adapter, caster, 0);
    EXPECT_TRUE(scan.updated);
    EXPECT_EQ(scan.horizontal_count, 5u);
    EXPECT_EQ(scan.vertical_count, 3u);
    EXPECT_DOUBLE_EQ(scan.angle_min, -1.0);
    EXPECT_DOUBLE_EQ(scan.angle_max, 1.0);
    EXPECT_DOUBLE_EQ(scan.angle_increment, 0.5);
    EXPECT_DOUBLE_EQ(scan.vertical_angle_min, -0.3);
    EXPECT_DOUBLE_EQ(scan.vertical_angle_max, 0.3);
    EXPECT_DOUBLE_EQ(scan.vertical_angle_increment, 0.3);

    auto one = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":1,"min_angle":0.75,"max_angle":1.25}},
        "range":{"min":0.1,"max":10}})", caster);
    ASSERT_NE(one, nullptr);
    const auto one_scan = step(*one, caster, 0);
    EXPECT_EQ(one_scan.horizontal_count, 1u);
    EXPECT_DOUBLE_EQ(one_scan.angle_min, 0.75);
    EXPECT_DOUBLE_EQ(one_scan.angle_max, 0.75);
    EXPECT_DOUBLE_EQ(one_scan.angle_increment, 0.0);
    EXPECT_EQ(one_scan.vertical_count, 1u);
    EXPECT_DOUBLE_EQ(one_scan.vertical_angle_min, 0.0);
    EXPECT_DOUBLE_EQ(one_scan.vertical_angle_max, 0.0);
}

TEST(Lidar, T_LID_U2_020_WallAndFluAxes)
{
    // RIS-COM-020, RIS-LID-020
    ScriptedCaster caster;
    caster.distance = [](const sa::Ray& ray) {
        return ray.direction.x > 0.0 ? 4.0 / ray.direction.x : -1.0;
    };
    auto adapter = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":5,"min_angle":-1.0471975511965976,
        "max_angle":1.0471975511965976}},"range":{"min":0.1,"max":10}})", caster);
    ASSERT_NE(adapter, nullptr);
    const auto scan = step(*adapter, caster, 0);
    const double azimuths[] = {-kPi / 3.0, -kPi / 6.0, 0.0, kPi / 6.0, kPi / 3.0};
    ASSERT_EQ(scan.horizontal_count, 5u);
    for (std::size_t i = 0; i < 5; ++i) {
        // A ray at azimuth az reaches the x = 4 m wall after 4 / cos(az) metres.
        expect_relative_float(scan.ranges[i], 4.0 / std::cos(azimuths[i]));
    }
    ASSERT_EQ(scan.point_count, 5u);
    EXPECT_LT(scan.points[0].y, 0.0F);
    EXPECT_GT(scan.points[4].y, 0.0F);

    ScriptedCaster three_dimensional;
    auto upper = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":1,"min_angle":0},
        "vertical":{"samples":2,"min_angle":-0.5235987755982988,
        "max_angle":0.5235987755982988}},"range":{"min":0.1,"max":10}})",
        three_dimensional);
    ASSERT_NE(upper, nullptr);
    const auto points = step(*upper, three_dimensional, 0);
    ASSERT_EQ(points.point_count, 2u);
    EXPECT_LT(points.points[0].z, 0.0F);
    EXPECT_GT(points.points[1].z, 0.0F);
}

TEST(Lidar, T_LID_U2_030_RowMajorVerticalGrid)
{
    // RIS-LID-030
    ScriptedCaster caster;
    caster.distance = [](const sa::Ray& ray) {
        return ray.direction.z < 0.0 ? 1.0 / -ray.direction.z : -1.0;
    };
    auto adapter = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":4,"min_angle":-1.0,"max_angle":1.0},
        "vertical":{"samples":2,"min_angle":-0.5235987755982988,
        "max_angle":0.5235987755982988}},"range":{"min":0.1,"max":10}})", caster);
    ASSERT_NE(adapter, nullptr);
    const auto scan = step(*adapter, caster, 0);
    ASSERT_EQ(scan.horizontal_count, 4u);
    ASSERT_EQ(scan.vertical_count, 2u);
    for (std::size_t h = 0; h < 4; ++h) {
        expect_relative_float(scan.ranges[h], 2.0);  // 1 / sin(30 degrees) = 2.
        EXPECT_TRUE(std::isinf(scan.ranges[4 + h]));
        EXPECT_GT(scan.ranges[4 + h], 0.0F);
    }
}

TEST(Lidar, T_LID_U2_040_RotationSchedule)
{
    // RIS-LID-050
    for (std::uint32_t horizontal : {36u, 100u, 360u}) {
        ScriptedCaster caster;
        auto adapter = make(lidar_config(horizontal, 3), caster);
        ASSERT_NE(adapter, nullptr);
        finish_scan(*adapter, caster, 0, 100);

        std::uint32_t column = 0;
        for (const auto& call : caster.calls) {
            ASSERT_EQ(call.rays.size() % 3, 0u);
            const std::uint32_t columns_in_call = static_cast<std::uint32_t>(call.rays.size() / 3);
            for (std::uint32_t local = 0; local < columns_in_call; ++local, ++column) {
                // Spec: column h is cast at floor(h * P / H), with P = 100 steps.
                EXPECT_EQ(call.step, static_cast<std::uint64_t>(column) * 100u / horizontal);
                const auto& low = call.rays[local * 3].direction;
                const auto& middle = call.rays[local * 3 + 1].direction;
                const auto& high = call.rays[local * 3 + 2].direction;
                // All vertical rows of a column occur in this one recorded call.
                EXPECT_LT(low.z, middle.z);
                EXPECT_LT(middle.z, high.z);
                EXPECT_NEAR(std::atan2(low.y, low.x), std::atan2(high.y, high.x), 1e-12);
            }
        }
        EXPECT_EQ(column, horizontal);
    }
}

TEST(Lidar, T_LID_U2_050_RotatingOutputTiming)
{
    // RIS-LID-060, RIS-LID-080
    ScriptedCaster caster;
    auto adapter = make(lidar_config(8, 1), caster);
    ASSERT_NE(adapter, nullptr);
    sa::LidarScan scan{};
    for (std::uint64_t index = 0; index < 100; ++index) {
        sa::SensorPoseTruth truth{};
        truth.position_world = {1000.0 + index, 2000.0 + index, 3000.0 + index};
        truth.orientation_world_to_sensor = {0.1 + index, 0.2 + index, 0.3 + index, 0.4 + index};
        scan = step(*adapter, caster, index, &truth);
        if (index < 99) EXPECT_FALSE(scan.updated) << index;
    }
    EXPECT_TRUE(scan.updated);
    EXPECT_EQ(scan.sample_time_ns, 0u);
    EXPECT_DOUBLE_EQ(scan.scan_time, 0.1);
    EXPECT_DOUBLE_EQ(scan.time_increment, 0.0125);
    EXPECT_EQ(scan.pose_at_end.sim_time_ns, 99u * kStepNs);
    EXPECT_DOUBLE_EQ(scan.pose_at_end.position_world.x, 1099.0);
    EXPECT_DOUBLE_EQ(scan.pose_at_end.position_world.y, 2099.0);
    EXPECT_DOUBLE_EQ(scan.pose_at_end.position_world.z, 3099.0);
    EXPECT_DOUBLE_EQ(scan.pose_at_end.orientation_world_to_sensor.w, 99.1);
    EXPECT_DOUBLE_EQ(scan.pose_at_end.orientation_world_to_sensor.z, 99.4);
}

TEST(Lidar, T_LID_U2_060_InstantScan)
{
    // RIS-LID-070
    ScriptedCaster caster;
    auto adapter = make(lidar_config(7, 2, "instant"), caster);
    ASSERT_NE(adapter, nullptr);
    const auto first = step(*adapter, caster, 0);
    EXPECT_TRUE(first.updated);
    ASSERT_EQ(caster.calls.size(), 1u);
    EXPECT_EQ(caster.calls[0].step, 0u);
    EXPECT_EQ(caster.calls[0].rays.size(), 14u);
    EXPECT_DOUBLE_EQ(first.scan_time, 0.0);
    EXPECT_DOUBLE_EQ(first.time_increment, 0.0);
    const auto held = step(*adapter, caster, 1);
    EXPECT_FALSE(held.updated);
    EXPECT_EQ(caster.calls.size(), 1u);
}

TEST(Lidar, T_LID_U2_070_PointsAndIntensity)
{
    // RIS-LID-090, RIS-LID-100
    ScriptedCaster caster;
    caster.object_id = [](const sa::Ray& ray) {
        return 20 + (ray.direction.y > 0.1 ? 2 : 0) + (ray.direction.z > 0.0 ? 1 : 0);
    };
    auto adapter = make(R"({"type":"lidar","update_rate":20,"scan_mode":"rotating",
        "scan":{"horizontal":{"samples":2,"min_angle":0,"max_angle":1.5707963267948966},
        "vertical":{"samples":2,"min_angle":-0.5235987755982988,
        "max_angle":0.5235987755982988}},"range":{"min":0.1,"max":10}})", caster);
    ASSERT_NE(adapter, nullptr);
    const auto scan = finish_scan(*adapter, caster, 0, 50);
    ASSERT_EQ(scan.point_count, 4u);
    const double root_three = std::sqrt(3.0);
    const float expected_xyz[4][3] = {
        {static_cast<float>(root_three), 0.0F, -1.0F},
        {static_cast<float>(root_three), 0.0F, 1.0F},
        {0.0F, static_cast<float>(root_three), -1.0F},
        {0.0F, static_cast<float>(root_three), 1.0F}};
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(scan.points[i].x, expected_xyz[i][0], 2e-6);
        EXPECT_NEAR(scan.points[i].y, expected_xyz[i][1], 2e-6);
        EXPECT_NEAR(scan.points[i].z, expected_xyz[i][2], 2e-6);
        EXPECT_EQ(scan.points[i].ring, i % 2);
        EXPECT_EQ(scan.points[i].object_id, 20 + (i >= 2 ? 2 : 0) + (i % 2));
        EXPECT_NEAR(scan.points[i].time_offset, i < 2 ? 0.0 : 0.025, 1e-8);
    }
    ASSERT_NE(scan.intensities, nullptr);
    for (std::size_t i = 0; i < 4; ++i) EXPECT_FLOAT_EQ(scan.intensities[i], 0.0F);
    EXPECT_FALSE(adapter->capabilities().intensity);
}

TEST(Lidar, T_LID_U2_080_SaturationFlag)
{
    // RIS-COM-160
    ScriptedCaster clipped_caster;
    clipped_caster.wall_distance = 0.99;
    auto clipped = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":64,"min_angle":-1,"max_angle":1}},
        "range":{"min":0.1,"max":1.0,"resolution":0},
        "accuracy":[{"range":{"min":0.1,"max":1.0},"stddev":0.2}]})",
        clipped_caster, 12345);
    ASSERT_NE(clipped, nullptr);
    const auto clipped_scan = step(*clipped, clipped_caster, 0);
    EXPECT_NE(clipped_scan.flags & sa::kSensorFlagSaturated, 0u);
    bool found_maximum = false;
    for (std::size_t i = 0; i < 64; ++i) found_maximum |= clipped_scan.ranges[i] == 1.0F;
    EXPECT_TRUE(found_maximum);

    ScriptedCaster clean_caster;
    clean_caster.wall_distance = 0.5;
    auto clean = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":4,"min_angle":-1,"max_angle":1}},
        "range":{"min":0.1,"max":1.0,"resolution":0}})", clean_caster);
    ASSERT_NE(clean, nullptr);
    const auto clean_scan = step(*clean, clean_caster, 0);
    EXPECT_EQ(clean_scan.flags & sa::kSensorFlagSaturated, 0u);
}

TEST(Lidar, T_LID_U2_090_HeldScanAndBufferSwap)
{
    // RIS-LID-110, RIS-LID-140
    ScriptedCaster caster;
    caster.wall_distance = 2.0;
    auto adapter = make(lidar_config(4, 1, "rotating", 250.0), caster);
    ASSERT_NE(adapter, nullptr);
    const auto first = finish_scan(*adapter, caster, 0, 4);
    ASSERT_TRUE(first.updated);
    const std::vector<float> first_values(first.ranges, first.ranges + 4);
    EXPECT_EQ(first_values, std::vector<float>(4, 2.0F));

    caster.wall_distance = 3.0;
    for (std::uint64_t index = 4; index < 7; ++index) {
        const auto held = step(*adapter, caster, index);
        EXPECT_FALSE(held.updated);
        EXPECT_EQ(std::vector<float>(held.ranges, held.ranges + 4), first_values);
    }
    const auto second = step(*adapter, caster, 7);
    EXPECT_TRUE(second.updated);
    EXPECT_EQ(std::vector<float>(second.ranges, second.ranges + 4), std::vector<float>(4, 3.0F));

    caster.wall_distance = 4.0;
    for (std::uint64_t index = 8; index < 11; ++index) {
        const auto held = step(*adapter, caster, index);
        EXPECT_FALSE(held.updated);
        EXPECT_EQ(std::vector<float>(held.ranges, held.ranges + 4), std::vector<float>(4, 3.0F));
    }
    const auto third = step(*adapter, caster, 11);
    EXPECT_TRUE(third.updated);
    EXPECT_EQ(std::vector<float>(third.ranges, third.ranges + 4), std::vector<float>(4, 4.0F));
}

TEST(Lidar, T_LID_U2_100_CastDistanceAndLog)
{
    // RIS-LID-120, RIS-LID-130
    ScriptedCaster caster;
    auto adapter = make(R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":3,"min_angle":-0.5,"max_angle":0.5}},
        "range":{"min":0.1,"max":12.5}})", caster);
    ASSERT_NE(adapter, nullptr);
    const auto scan = step(*adapter, caster, 0);
    ASSERT_EQ(caster.calls.size(), 1u);
    EXPECT_DOUBLE_EQ(caster.calls[0].max_distance, 12.5);
    ASSERT_EQ(scan.point_count, 3u);
    const auto& columns = adapter->log_columns();
    std::vector<double> values;
    adapter->log_values(values);
    ASSERT_EQ(columns.size(), 1u);
    ASSERT_EQ(values.size(), 1u);
    EXPECT_EQ(columns[0].name, "points");
    EXPECT_EQ(columns[0].unit, "count");
    EXPECT_DOUBLE_EQ(values[0], 3.0);
}

TEST(Lidar, T_LID_U3_010_GridAndModeValidation)
{
    // RIS-LID-010, RIS-LID-040
    ScriptedCaster caster;
    auto defaults = make(R"({"type":"lidar","update_rate":10,
        "scan":{"horizontal":{"samples":1,"min_angle":0}},
        "range":{"min":0.1,"max":10}})", caster);
    ASSERT_NE(defaults, nullptr);
    EXPECT_TRUE(defaults->capabilities().rotating_scan);
    EXPECT_FALSE(step(*defaults, caster, 0).updated);

    const std::vector<std::string> invalid = {
        R"({"type":"lidar","update_rate":10,"scan":{"horizontal":{"samples":0}},"range":{"min":0.1,"max":10}})",
        R"({"type":"lidar","update_rate":10,"scan":{"horizontal":{"samples":2,"min_angle":1,"max_angle":0}},"range":{"min":0.1,"max":10}})",
        R"({"type":"lidar","update_rate":10,"scan":{"horizontal":{"samples":1},"vertical":{"samples":0}},"range":{"min":0.1,"max":10}})",
        R"({"type":"lidar","update_rate":10,"scan":{"horizontal":{"samples":1},"vertical":{"samples":2,"min_angle":1,"max_angle":0}},"range":{"min":0.1,"max":10}})",
        R"({"type":"lidar","update_rate":10,"scan_mode":"sweep","scan":{"horizontal":{"samples":1}},"range":{"min":0.1,"max":10}})"};
    for (const auto& text : invalid) {
        std::string error;
        EXPECT_EQ(make(text, caster, 1, &error), nullptr);
        EXPECT_FALSE(error.empty());
    }
}

}  // namespace
