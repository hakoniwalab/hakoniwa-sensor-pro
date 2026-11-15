#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/lidar_sensor_adapter.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStepNs = 1000000;
constexpr double kPi = 3.14159265358979323846;

const char* kRoom = R"(
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1"/>
    <geom name="north" type="box" pos="5.1 0 1.5" size="0.1 5.2 1.5"/>
    <geom name="south" type="box" pos="-5.1 0 1.5" size="0.1 5.2 1.5"/>
    <geom name="west" type="box" pos="0 5.1 1.5" size="5.2 0.1 1.5"/>
    <geom name="east" type="box" pos="0 -5.1 1.5" size="5.2 0.1 1.5"/>
    <body name="vehicle" pos="0 0 1">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <site name="lidar"/>
    </body>
  </worldbody>
</mujoco>
)";

class TempConfig {
public:
    explicit TempConfig(const std::string& text)
    {
        static std::uint64_t serial = 0;
        path_ = std::filesystem::temp_directory_path() /
                ("hakoniwa-mujoco-lidar-" + std::to_string(++serial) + ".json");
        std::ofstream(path_) << text;
    }
    ~TempConfig() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

std::unique_ptr<sa::ILidarSensorAdapter> make(const std::string& text, ms::MujocoRayCaster& caster)
{
    TempConfig config(text);
    sa::LidarAdapterSetup setup{};
    setup.config_path = config.path();
    setup.ray_caster = &caster;
    setup.physics_step_ns = kStepNs;
    setup.has_random_seed = true;
    setup.random_seed = 1;
    std::string error;
    auto adapter = ms::create_lidar_sensor_adapter(setup, error);
    EXPECT_NE(adapter, nullptr) << error;
    return adapter;
}

double room_range(double azimuth, double north)
{
    const double c = std::cos(azimuth);
    const double s = std::sin(azimuth);
    double distance = std::numeric_limits<double>::infinity();
    if (c > 1e-12) distance = std::min(distance, (5.0 - north) / c);
    if (c < -1e-12) distance = std::min(distance, (-5.0 - north) / c);
    if (s > 1e-12) distance = std::min(distance, 5.0 / s);
    if (s < -1e-12) distance = std::min(distance, -5.0 / s);
    return distance;
}

TEST(Lidar, T_LID_I1_020_MujocoRoomAndMotionDistortion)
{
    // RIS-LID-090
    char message[1000] = "";
    mjSpec* specification = mj_parseXMLString(kRoom, nullptr, message, sizeof(message));
    ASSERT_NE(specification, nullptr) << message;
    mjModel* model = mj_compile(specification, nullptr);
    ASSERT_NE(model, nullptr) << message;
    mjData* data = mj_makeData(model);
    ASSERT_NE(data, nullptr);

    ms::MujocoRayCaster caster;
    std::string error;
    ASSERT_TRUE(caster.initialize(model, data, "lidar", "vehicle", error)) << error;
    const std::string horizontal = R"({"type":"lidar","update_rate":10,"scan_mode":"rotating",
        "scan":{"horizontal":{"samples":360,"min_angle":-3.14159265358979323846,
        "max_angle":3.12413936106985}},"range":{"min":0.1,"max":20}})";
    auto adapter = make(horizontal, caster);
    ASSERT_NE(adapter, nullptr);

    mj_forward(model, data);
    sa::LidarScan scan{};
    for (std::uint64_t step = 0; step < 100; ++step) {
        sa::SensorPoseTruth truth{};
        truth.sim_time_ns = step * kStepNs;
        ASSERT_TRUE(adapter->step(truth, scan, error)) << error;
    }
    ASSERT_TRUE(scan.updated);
    for (std::uint32_t h = 0; h < 360; ++h) {
        const double azimuth = -kPi + h * kPi / 180.0;
        // From the centre, the first wall is 5 / max(|cos az|, |sin az|).
        const double expected = 5.0 / std::max(std::fabs(std::cos(azimuth)), std::fabs(std::sin(azimuth)));
        EXPECT_NEAR(scan.ranges[h], expected, 1e-5) << "column " << h;
    }

    ASSERT_TRUE(adapter->reset(error)) << error;
    for (std::uint64_t step = 0; step < 100; ++step) {
        data->qpos[0] = 10.0 * static_cast<double>(step) * 0.001;
        mj_forward(model, data);
        sa::SensorPoseTruth truth{};
        truth.sim_time_ns = step * kStepNs;
        ASSERT_TRUE(adapter->step(truth, scan, error)) << error;
    }
    ASSERT_TRUE(scan.updated);
    for (std::uint32_t h = 0; h < 360; ++h) {
        // Spec: column h is cast at floor(h * P / H), with P = 100 and H = 360.
        const std::uint32_t cast_step = h * 100u / 360u;
        const double north = 10.0 * static_cast<double>(cast_step) * 0.001;
        const double azimuth = -kPi + h * kPi / 180.0;
        EXPECT_NEAR(scan.ranges[h], room_range(azimuth, north), 1e-5) << "column " << h;
    }

    const std::string three_dimensional = R"({"type":"lidar","update_rate":1000,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":4,"min_angle":-3.14159265358979323846,
        "max_angle":1.5707963267948966},"vertical":{"samples":8,
        "min_angle":-0.3490658503988659,"max_angle":0.3490658503988659}},
        "range":{"min":0.1,"max":20}})";
    data->qpos[0] = 0.0;
    mj_forward(model, data);
    auto lidar_3d = make(three_dimensional, caster);
    ASSERT_NE(lidar_3d, nullptr);
    sa::SensorPoseTruth truth{};
    ASSERT_TRUE(lidar_3d->step(truth, scan, error)) << error;
    ASSERT_TRUE(scan.updated);
    ASSERT_EQ(scan.vertical_count, 8u);
    ASSERT_EQ(scan.horizontal_count, 4u);
    for (std::uint32_t row = 0; row < 8; ++row) {
        const double elevation = (-20.0 + row * (40.0 / 7.0)) * kPi / 180.0;
        for (std::uint32_t h = 0; h < 4; ++h) {
            const double azimuth = -kPi + h * kPi / 2.0;
            const double wall = 5.0 /
                (std::cos(elevation) * std::max(std::fabs(std::cos(azimuth)), std::fabs(std::sin(azimuth))));
            double expected = wall;
            if (elevation < 0.0) {
                // The sensor is 1 m high: the floor is 1 / sin(-el) metres along the ray.
                expected = std::min(expected, 1.0 / std::sin(-elevation));
            }
            EXPECT_NEAR(scan.ranges[row * 4 + h], expected, 1e-5)
                << "row " << row << ", column " << h;
        }
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(specification);
}

}  // namespace
