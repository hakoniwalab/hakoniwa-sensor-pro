#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/depth_camera_sensor_adapter.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_image_renderer.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

class TempConfig {
public:
    TempConfig(const std::string& tag, const std::string& text)
    {
        path_ = std::filesystem::temp_directory_path() / ("hakoniwa-depth-render-" + tag + ".json");
        std::ofstream(path_) << text;
    }
    ~TempConfig() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

TEST(DepthCamera, T_DEP_I1_020_MujocoRayAgainstRender)
{
    // RIS-DEP-040, RIS-MJI-040, RIS-MJI-060
    const char* world = R"(
<mujoco>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 0.1"/>
    <geom name="left_box" type="box" pos="5 1.5 1.5" size="0.7 0.8 1.5"/>
    <geom name="right_box" type="box" pos="8 -2 2" size="1 1 2"/>
    <geom name="far_wall" type="box" pos="20 0 5" size="0.1 6 5"/>
    <body name="vehicle" pos="0 0 2">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <site name="camera" euler="0 15 0"/>
    </body>
  </worldbody>
</mujoco>)";
    char message[1000] = "";
    mjSpec* specification = mj_parseXMLString(world, nullptr, message, sizeof(message));
    ASSERT_NE(specification, nullptr) << message;
    mjModel* model = mj_compile(specification, nullptr);
    ASSERT_NE(model, nullptr) << message;
    mjData* data = mj_makeData(model);
    ASSERT_NE(data, nullptr);
    mj_forward(model, data);

    std::string error;
    ms::MujocoRayCaster caster;
    ASSERT_TRUE(caster.initialize(model, data, "camera", "vehicle", error)) << error;
    ms::MujocoImageRenderer renderer;
    ASSERT_TRUE(renderer.initialize(model, data, error)) << error;

    const std::string common = R"("type":"depth_camera","update_rate":1000,)"
                               R"("depth_type":"planar","image":{"width":160,"height":120},)"
                               R"("horizontal_fov":1.2,"range":{"min":0.01,"max":30})";
    TempConfig ray_config("ray", "{" + common + R"(,"method":"ray"})");
    TempConfig render_config("render", "{" + common + R"(,"method":"render"})");
    sa::DepthCameraAdapterSetup setup{};
    setup.physics_step_ns = 1000000;
    setup.has_random_seed = true;
    setup.random_seed = 1;
    setup.ray_caster = &caster;
    setup.renderer = &renderer;
    setup.config_path = ray_config.path();
    auto ray_adapter = ms::create_depth_camera_sensor_adapter(setup, error);
    ASSERT_NE(ray_adapter, nullptr) << error;
    setup.config_path = render_config.path();
    auto render_adapter = ms::create_depth_camera_sensor_adapter(setup, error);
    ASSERT_NE(render_adapter, nullptr) << error;

    sa::DepthImage ray_image{};
    sa::DepthImage render_image{};
    const int camera_id = mj_name2id(model, mjOBJ_SITE, "camera");
    mjtNum sensor_to_world[4];
    mju_mat2Quat(sensor_to_world, data->site_xmat + 9 * camera_id);
    sa::SensorPoseTruth truth{};
    truth.position_world = {data->site_xpos[3 * camera_id], data->site_xpos[3 * camera_id + 1],
                            data->site_xpos[3 * camera_id + 2]};
    truth.orientation_world_to_sensor = {sensor_to_world[0], -sensor_to_world[1],
                                         -sensor_to_world[2], -sensor_to_world[3]};
    ASSERT_TRUE(ray_adapter->step(truth, ray_image, error)) << error;
    ASSERT_TRUE(render_adapter->step(truth, render_image, error)) << error;
    ASSERT_NE(ray_image.data, nullptr);
    ASSERT_NE(render_image.data, nullptr);

    std::size_t compared = 0;
    std::size_t edge_pixels = 0;
    std::size_t empty_pixels = 0;
    for (std::size_t i = 0; i < 160u * 120u; ++i) {
        const double by_ray = ray_image.data[i];
        const double by_render = render_image.data[i];
        if (std::isinf(by_ray)) {
            ++empty_pixels;
            EXPECT_EQ(by_ray, std::numeric_limits<float>::infinity());
            EXPECT_EQ(by_render, std::numeric_limits<float>::infinity()) << "pixel " << i;
            continue;
        }
        if (std::isinf(by_render) || std::fabs(by_ray - by_render) / by_ray > 0.05) {
            ++edge_pixels;
            continue;
        }
        ++compared;
        EXPECT_LE(std::fabs(by_ray - by_render) / by_ray, 1e-3) << "pixel " << i;
    }
    EXPECT_GT(compared, 0u);
    EXPECT_GT(empty_pixels, 0u);
    EXPECT_LT(edge_pixels, 160u * 120u * 2u / 100u);

    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(specification);
}

}  // namespace
