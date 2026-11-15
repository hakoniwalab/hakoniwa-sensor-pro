#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/depth_camera_sensor_adapter.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr double kPi = 3.14159265358979323846;

class TempConfig {
public:
    explicit TempConfig(const std::string& text)
    {
        static std::uint64_t serial = 0;
        path_ = std::filesystem::temp_directory_path() /
                ("hakoniwa-mujoco-depth-" + std::to_string(++serial) + ".json");
        std::ofstream(path_) << text;
    }
    ~TempConfig() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

class World {
public:
    explicit World(const char* xml)
    {
        char message[1000] = "";
        specification = mj_parseXMLString(xml, nullptr, message, sizeof(message));
        EXPECT_NE(specification, nullptr) << message;
        if (!specification) return;
        model = mj_compile(specification, nullptr);
        EXPECT_NE(model, nullptr) << message;
        if (!model) return;
        data = mj_makeData(model);
        EXPECT_NE(data, nullptr);
        if (data) mj_forward(model, data);
    }
    ~World()
    {
        if (data) mj_deleteData(data);
        if (model) mj_deleteModel(model);
        if (specification) mj_deleteSpec(specification);
    }

    mjSpec* specification{nullptr};
    mjModel* model{nullptr};
    mjData* data{nullptr};
};

std::unique_ptr<sa::IDepthCameraSensorAdapter> make(
    const std::string& text, World& world, ms::MujocoRayCaster& caster)
{
    std::string error;
    EXPECT_TRUE(caster.initialize(world.model, world.data, "camera", "vehicle", error)) << error;
    TempConfig config(text);
    sa::DepthCameraAdapterSetup setup{};
    setup.config_path = config.path();
    setup.ray_caster = &caster;
    setup.physics_step_ns = 1000000;
    setup.has_random_seed = true;
    setup.random_seed = 1;
    auto adapter = ms::create_depth_camera_sensor_adapter(setup, error);
    EXPECT_NE(adapter, nullptr) << error;
    return adapter;
}

sa::DepthImage frame(sa::IDepthCameraSensorAdapter& adapter)
{
    sa::DepthImage image{};
    std::string error;
    EXPECT_TRUE(adapter.step(sa::SensorPoseTruth{}, image, error)) << error;
    return image;
}

TEST(DepthCamera, T_DEP_I1_010_MujocoRayGeometry)
{
    // RIS-DEP-030
    const char* ground_world = R"(
<mujoco>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 0.1"/>
    <body name="vehicle" pos="0 0 2">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <site name="camera" euler="0 15 0"/>
    </body>
  </worldbody>
</mujoco>)";
    World ground(ground_world);
    ASSERT_NE(ground.data, nullptr);
    ms::MujocoRayCaster ground_caster;
    auto ground_camera = make(R"({"type":"depth_camera","update_rate":1000,"method":"ray",)"
                              R"("depth_type":"planar","image":{"width":16,"height":12},)"
                              R"("horizontal_fov":1.2,"range":{"min":0.01,"max":1000}})",
                              ground, ground_caster);
    ASSERT_NE(ground_camera, nullptr);
    const auto ground_image = frame(*ground_camera);
    ASSERT_NE(ground_image.data, nullptr);
    const double f = (16.0 / 2.0) / std::tan(1.2 / 2.0);
    const double cy = (12.0 - 1.0) / 2.0;
    const double pitch = 15.0 * kPi / 180.0;
    std::size_t ground_pixels = 0;
    std::size_t sky_pixels = 0;
    for (std::uint32_t v = 0; v < 12; ++v) {
        for (std::uint32_t u = 0; u < 16; ++u) {
            const double yn = (static_cast<double>(v) - cy) / f;
            // World vertical component of the unnormalized sensor ray (1, -xn, -yn).
            const double downward = std::sin(pitch) + std::cos(pitch) * yn;
            const float actual = ground_image.data[v * 16 + u];
            if (downward > 0.0) {
                ++ground_pixels;
                EXPECT_NEAR(actual, 2.0 / downward, 1e-5) << "pixel (" << u << ", " << v << ")";
            } else {
                ++sky_pixels;
                EXPECT_EQ(actual, std::numeric_limits<float>::infinity())
                    << "pixel (" << u << ", " << v << ")";
            }
        }
    }
    EXPECT_GT(ground_pixels, 0u);
    EXPECT_GT(sky_pixels, 0u);

    const char* box_world = R"(
<mujoco>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 0.1"/>
    <geom name="box" type="box" pos="3 0 1" size="0.5 0.8 1"/>
    <body name="vehicle" pos="0 0 1">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <site name="camera"/>
    </body>
  </worldbody>
</mujoco>)";
    World box(box_world);
    ASSERT_NE(box.data, nullptr);
    ms::MujocoRayCaster box_caster;
    auto box_camera = make(R"({"type":"depth_camera","update_rate":1000,"method":"ray",)"
                           R"("depth_type":"planar","image":{"width":65,"height":49},)"
                           R"("horizontal_fov":1.2,"range":{"min":0.01,"max":30}})",
                           box, box_caster);
    ASSERT_NE(box_camera, nullptr);
    const auto box_image = frame(*box_camera);
    ASSERT_NE(box_image.data, nullptr);
    const double box_f = (65.0 / 2.0) / std::tan(1.2 / 2.0);
    std::size_t face_pixels = 0;
    for (std::uint32_t u = 0; u < 65; ++u) {
        const double xn = (static_cast<double>(u) - 32.0) / box_f;
        // The front face is x = 2.5 and spans -0.8 <= y <= 0.8.
        if (std::fabs(2.5 * xn) < 0.8) {
            ++face_pixels;
            EXPECT_NEAR(box_image.data[24 * 65 + u], 2.5, 1e-5) << "column " << u;
        }
    }
    EXPECT_GT(face_pixels, 0u);
}

}  // namespace
