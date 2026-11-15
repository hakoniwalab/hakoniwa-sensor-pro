#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"
#include "hakoniwa/mujoco_sensors/rangefinder_sensor_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr double kPi = 3.14159265358979323846;

const char* kWorld = R"(
<mujoco>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 0.1"/>
    <body name="vehicle" pos="0 0 2">
      <freejoint/>
      <geom name="enclosing_box" type="box" size="0.25 0.25 0.25" mass="1"/>
      <body name="child" pos="0 0 -0.3">
        <geom name="child_geom" type="box" size="0.1 0.1 0.1" mass="0.1"/>
      </body>
      <site name="rangefinder" euler="0 90 0"/>
    </body>
  </worldbody>
</mujoco>
)";

class TempConfig {
public:
    TempConfig()
    {
        path_ = std::filesystem::temp_directory_path() / "hakoniwa-mujoco-rangefinder-gtest.json";
        std::ofstream(path_) << R"({"type":"rangefinder","update_rate":1000,
            "range":{"min":0,"max":10,"resolution":0}})";
    }
    ~TempConfig() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

TEST(Rangefinder, T_RNG_I1_010_PitchedOverGround)
{
    // RIS-RNG-040, RIS-MJR-020
    char message[1000] = "";
    mjSpec* specification = mj_parseXMLString(kWorld, nullptr, message, sizeof(message));
    ASSERT_NE(specification, nullptr) << message;
    mjModel* model = mj_compile(specification, nullptr);
    ASSERT_NE(model, nullptr) << message;
    mjData* data = mj_makeData(model);
    ASSERT_NE(data, nullptr);

    ms::MujocoRayCaster caster;
    std::string error;
    ASSERT_TRUE(caster.initialize(model, data, "rangefinder", "vehicle", error)) << error;
    TempConfig config;
    sa::RangefinderAdapterSetup setup{};
    setup.config_path = config.path();
    setup.ray_caster = &caster;
    setup.physics_step_ns = 1000000;
    setup.has_random_seed = true;
    setup.random_seed = 1;
    auto adapter = ms::create_rangefinder_sensor_adapter(setup, error);
    ASSERT_NE(adapter, nullptr) << error;

    for (double degrees : {0.0, 10.0, 20.0, 35.0}) {
        const double radians = degrees * kPi / 180.0;
        data->qpos[3] = std::cos(radians / 2.0);
        data->qpos[4] = 0.0;
        data->qpos[5] = std::sin(radians / 2.0);
        data->qpos[6] = 0.0;
        mj_forward(model, data);
        ASSERT_TRUE(adapter->reset(error)) << error;
        sa::SensorPoseTruth truth{};
        sa::RangeSample output{};
        ASSERT_TRUE(adapter->step(truth, output, error)) << error;
        EXPECT_NEAR(output.range, 2.0 / std::cos(radians), 1e-9) << degrees << " degrees";
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(specification);
}

}  // namespace
