#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/mujoco_image_renderer.hpp"

#include <GLFW/glfw3.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

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

const char* camera_world(const char* site_euler = nullptr)
{
    static const std::string plain = R"(
<mujoco>
  <visual>
    <global offwidth="640" offheight="480"/>
    <headlight ambient="0.8 0.8 0.8" diffuse="0.2 0.2 0.2"/>
  </visual>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>
    <geom name="ground" type="plane" size="50 50 0.1" rgba="0.5 0.5 0.5 1"/>
    <geom name="red" type="box" pos="4 1.5 1" size="0.5 0.5 1" rgba="1 0 0 1"/>
    <geom name="green" type="box" pos="4 -1.5 1" size="0.5 0.5 1" rgba="0 1 0 1"/>
    <geom name="wall" type="box" pos="12 0 6" size="0.1 20 6" rgba="0 0 1 1"/>
    <body name="camera_body" pos="0 0 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="camera"/>
      <camera name="model_camera"/>
    </body>
  </worldbody>
</mujoco>)";
    static const std::string rolled = R"(
<mujoco>
  <visual>
    <global offwidth="640" offheight="480"/>
    <headlight ambient="0.8 0.8 0.8" diffuse="0.2 0.2 0.2"/>
  </visual>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>
    <geom name="ground" type="plane" size="50 50 0.1" rgba="0.5 0.5 0.5 1"/>
    <geom name="red" type="box" pos="4 1.5 1" size="0.5 0.5 1" rgba="1 0 0 1"/>
    <geom name="green" type="box" pos="4 -1.5 1" size="0.5 0.5 1" rgba="0 1 0 1"/>
    <geom name="wall" type="box" pos="12 0 6" size="0.1 20 6" rgba="0 0 1 1"/>
    <body name="camera_body" pos="0 0 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="camera" euler="180 0 0"/>
      <camera name="model_camera"/>
    </body>
  </worldbody>
</mujoco>)";
    return site_euler ? rolled.c_str() : plain.c_str();
}

sa::SensorPoseTruth site_pose(const World& world, const char* name)
{
    const int id = mj_name2id(world.model, mjOBJ_SITE, name);
    const mjtNum* position = world.data->site_xpos + 3 * id;
    const mjtNum* rotation = world.data->site_xmat + 9 * id;
    mjtNum sensor_to_world[4];
    mju_mat2Quat(sensor_to_world, rotation);
    sa::SensorPoseTruth value{};
    value.position_world = {position[0], position[1], position[2]};
    value.orientation_world_to_sensor = {sensor_to_world[0], -sensor_to_world[1],
                                         -sensor_to_world[2], -sensor_to_world[3]};
    return value;
}

sa::RenderRequest request(std::uint32_t width, std::uint32_t height, bool rgb, bool depth)
{
    sa::RenderRequest value{};
    const double f = (static_cast<double>(width) / 2.0) / std::tan(1.2 / 2.0);
    value.pose.position_world = {0.0, 0.0, 1.0};
    value.camera = {width, height, f, f, (width - 1.0) / 2.0, (height - 1.0) / 2.0,
                    0.05, 50.0};
    value.outputs = (rgb ? sa::kRenderRgb : 0u) | (depth ? sa::kRenderDepth : 0u);
    return value;
}

struct RenderedImage {
    std::vector<std::uint8_t> rgb;
    std::vector<float> depth;
};

bool render(ms::MujocoImageRenderer& renderer, sa::RenderRequest value,
            RenderedImage& result, std::string& error)
{
    const std::size_t count = static_cast<std::size_t>(value.camera.width) * value.camera.height;
    if ((value.outputs & sa::kRenderRgb) != 0) {
        result.rgb.resize(count * 3);
        value.rgb = {result.rgb.data(), result.rgb.size()};
    }
    if ((value.outputs & sa::kRenderDepth) != 0) {
        result.depth.resize(count);
        value.depth = {result.depth.data(), result.depth.size() * sizeof(float)};
    }
    sa::RenderStatus status{};
    const bool ok = renderer.render(&value, 1, &status);
    error = status.error;
    return ok;
}

const std::uint8_t* pixel(const RenderedImage& result, std::uint32_t width,
                          std::uint32_t u, std::uint32_t v)
{
    return result.rgb.data() + (static_cast<std::size_t>(v) * width + u) * 3;
}

bool dominant(const std::uint8_t* value, int channel)
{
    const int first_other = (channel + 1) % 3;
    const int second_other = (channel + 2) % 3;
    return static_cast<int>(value[channel]) > static_cast<int>(value[first_other]) + 40 &&
           static_cast<int>(value[channel]) > static_cast<int>(value[second_other]) + 40;
}

bool dominant_within_one(const RenderedImage& result, std::uint32_t width, std::uint32_t height,
                         long expected_u, long expected_v, int channel)
{
    for (long dv = -1; dv <= 1; ++dv) {
        for (long du = -1; du <= 1; ++du) {
            const long u = expected_u + du;
            const long v = expected_v + dv;
            if (u >= 0 && v >= 0 && u < static_cast<long>(width) && v < static_cast<long>(height) &&
                dominant(pixel(result, width, static_cast<std::uint32_t>(u), static_cast<std::uint32_t>(v)), channel))
                return true;
        }
    }
    return false;
}

bool grey(const std::uint8_t* value)
{
    return std::abs(static_cast<int>(value[0]) - static_cast<int>(value[1])) <= 30 &&
           std::abs(static_cast<int>(value[1]) - static_cast<int>(value[2])) <= 30;
}

template <typename T>
std::vector<T> copy_values(const T* values, std::size_t count)
{
    return std::vector<T>(values, values + count);
}

template <typename T>
void expect_bit_identical(const std::vector<T>& before, const T* after)
{
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(std::memcmp(before.data(), after, before.size() * sizeof(T)), 0);
}

void expect_box_pixels(const RenderedImage& result, std::uint32_t width, std::uint32_t height)
{
    ASSERT_FALSE(result.rgb.empty());
    // Pinhole camera, pixel centres at integer coordinates:
    // f = (w / 2) / tan(fov / 2), cx = (w - 1) / 2, cy = (h - 1) / 2,
    // u = cx - f * y / x, v = cy - f * (z - z_cam) / x.
    const double f = (static_cast<double>(width) / 2.0) / std::tan(1.2 / 2.0);
    const double cx = (static_cast<double>(width) - 1.0) / 2.0;
    const double cy = (static_cast<double>(height) - 1.0) / 2.0;
    const long red_u = std::lround(cx - f * 1.5 / 4.0);
    const long green_u = std::lround(cx - f * -1.5 / 4.0);
    const long box_v = std::lround(cy - f * (1.0 - 1.0) / 4.0);
    EXPECT_TRUE(dominant_within_one(result, width, height, red_u, box_v, 0));
    EXPECT_TRUE(dominant_within_one(result, width, height, green_u, box_v, 1));
}

TEST(MujocoImageRenderer, T_MJI_I1_010_ManagesMultipleRendererLifetimes)
{
    // RIS-MJI-010
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    std::string error;
    const auto render_request = request(64, 48, true, false);
    {
        ms::MujocoImageRenderer first;
        ms::MujocoImageRenderer second;
        ASSERT_TRUE(first.initialize(world.model, world.data, error)) << error;
        ASSERT_TRUE(second.initialize(world.model, world.data, error)) << error;
        RenderedImage first_result{};
        RenderedImage second_result{};
        EXPECT_TRUE(render(first, render_request, first_result, error)) << error;
        EXPECT_TRUE(render(second, render_request, second_result, error)) << error;
        EXPECT_FALSE(first_result.rgb.empty());
        EXPECT_FALSE(second_result.rgb.empty());
    }
    {
        ms::MujocoImageRenderer third;
        ASSERT_TRUE(third.initialize(world.model, world.data, error)) << error;
        RenderedImage result{};
        EXPECT_TRUE(render(third, render_request, result, error)) << error;
        EXPECT_FALSE(result.rgb.empty());
    }
}

TEST(MujocoImageRenderer, T_MJI_I1_020_ProjectsColoursAndImageRows)
{
    // RIS-MJI-020, RIS-MJI-050
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    ms::MujocoImageRenderer renderer;
    std::string error;
    ASSERT_TRUE(renderer.initialize(world.model, world.data, error)) << error;
    const auto capabilities = renderer.capabilities();
    EXPECT_EQ(capabilities.outputs, sa::kRenderRgb | sa::kRenderDepth);
    EXPECT_FALSE(capabilities.lens_distortion);
    const std::uint32_t width = 160;
    const std::uint32_t height = 120;
    RenderedImage result{};
    ASSERT_TRUE(render(renderer, request(width, height, true, false), result, error)) << error;
    expect_box_pixels(result, width, height);

    const double f = (static_cast<double>(width) / 2.0) / std::tan(1.2 / 2.0);
    const double cy = (static_cast<double>(height) - 1.0) / 2.0;
    // At the wall front x = 11.9, the top-row ray is z = 1 + x * cy / f,
    // below the wall top z = 12, so the wall fills the top centre pixel.
    const double top_ray_height = 1.0 + 11.9 * cy / f;
    EXPECT_LT(top_ray_height, 12.0);
    EXPECT_TRUE(dominant(pixel(result, width, width / 2, 0), 2));
    EXPECT_TRUE(grey(pixel(result, width, width / 2, height - 1)));

    World rolled_world(camera_world("180 0 0"));
    ASSERT_NE(rolled_world.data, nullptr);
    ms::MujocoImageRenderer rolled_renderer;
    ASSERT_TRUE(rolled_renderer.initialize(rolled_world.model, rolled_world.data, error)) << error;
    RenderedImage rolled_result{};
    auto rolled_request = request(width, height, true, false);
    rolled_request.pose = site_pose(rolled_world, "camera");
    ASSERT_TRUE(render(rolled_renderer, rolled_request, rolled_result, error)) << error;
    const double cx = (static_cast<double>(width) - 1.0) / 2.0;
    const long red_right = std::lround(cx - f * -1.5 / 4.0);
    const long green_left = std::lround(cx - f * 1.5 / 4.0);
    const long middle = std::lround(cy);
    EXPECT_TRUE(dominant_within_one(rolled_result, width, height, red_right, middle, 0));
    EXPECT_TRUE(dominant_within_one(rolled_result, width, height, green_left, middle, 1));
    EXPECT_TRUE(grey(pixel(rolled_result, width, width / 2, 0)));
}

TEST(MujocoImageRenderer, T_MJI_I1_030_DoesNotModifyModelOrData)
{
    // RIS-MJI-030
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    ms::MujocoImageRenderer renderer;
    std::string error;
    ASSERT_TRUE(renderer.initialize(world.model, world.data, error)) << error;

    const auto cam_fovy = copy_values(world.model->cam_fovy, static_cast<std::size_t>(world.model->ncam));
    const auto cam_pos = copy_values(world.model->cam_pos, static_cast<std::size_t>(world.model->ncam) * 3);
    const auto cam_quat = copy_values(world.model->cam_quat, static_cast<std::size_t>(world.model->ncam) * 4);
    const auto cam_poscom0 = copy_values(world.model->cam_poscom0, static_cast<std::size_t>(world.model->ncam) * 3);
    const auto cam_pos0 = copy_values(world.model->cam_pos0, static_cast<std::size_t>(world.model->ncam) * 3);
    const auto cam_mat0 = copy_values(world.model->cam_mat0, static_cast<std::size_t>(world.model->ncam) * 9);
    const auto cam_mode = copy_values(world.model->cam_mode, static_cast<std::size_t>(world.model->ncam));
    const auto cam_bodyid = copy_values(world.model->cam_bodyid, static_cast<std::size_t>(world.model->ncam));
    const auto cam_targetbodyid = copy_values(world.model->cam_targetbodyid, static_cast<std::size_t>(world.model->ncam));
    const auto cam_projection = copy_values(world.model->cam_projection, static_cast<std::size_t>(world.model->ncam));
    const auto cam_ipd = copy_values(world.model->cam_ipd, static_cast<std::size_t>(world.model->ncam));
    const auto cam_resolution = copy_values(world.model->cam_resolution, static_cast<std::size_t>(world.model->ncam) * 2);
    const auto cam_output = copy_values(world.model->cam_output, static_cast<std::size_t>(world.model->ncam));
    const auto cam_sensorsize = copy_values(world.model->cam_sensorsize, static_cast<std::size_t>(world.model->ncam) * 2);
    const auto cam_intrinsic = copy_values(world.model->cam_intrinsic, static_cast<std::size_t>(world.model->ncam) * 4);
    const auto qpos = copy_values(world.data->qpos, static_cast<std::size_t>(world.model->nq));
    const auto site_xpos = copy_values(world.data->site_xpos, static_cast<std::size_t>(world.model->nsite) * 3);
    const auto site_xmat = copy_values(world.data->site_xmat, static_cast<std::size_t>(world.model->nsite) * 9);
    const float znear = world.model->vis.map.znear;
    const float zfar = world.model->vis.map.zfar;
    const int offwidth = world.model->vis.global.offwidth;
    const int offheight = world.model->vis.global.offheight;

    sa::RenderRequest first = request(160, 120, true, true);
    first.camera.fx = first.camera.fy = 80.0 / std::tan(0.8 / 2.0);
    first.camera.near_clip = 0.02;
    first.camera.far_clip = 25.0;
    RenderedImage result{};
    ASSERT_TRUE(render(renderer, first, result, error)) << error;
    sa::RenderRequest second = request(320, 200, true, true);
    second.camera.fx = second.camera.fy = 160.0 / std::tan(1.8 / 2.0);
    second.camera.near_clip = 0.2;
    second.camera.far_clip = 80.0;
    ASSERT_TRUE(render(renderer, second, result, error)) << error;

    expect_bit_identical(cam_fovy, world.model->cam_fovy);
    expect_bit_identical(cam_pos, world.model->cam_pos);
    expect_bit_identical(cam_quat, world.model->cam_quat);
    expect_bit_identical(cam_poscom0, world.model->cam_poscom0);
    expect_bit_identical(cam_pos0, world.model->cam_pos0);
    expect_bit_identical(cam_mat0, world.model->cam_mat0);
    expect_bit_identical(cam_mode, world.model->cam_mode);
    expect_bit_identical(cam_bodyid, world.model->cam_bodyid);
    expect_bit_identical(cam_targetbodyid, world.model->cam_targetbodyid);
    expect_bit_identical(cam_projection, world.model->cam_projection);
    expect_bit_identical(cam_ipd, world.model->cam_ipd);
    expect_bit_identical(cam_resolution, world.model->cam_resolution);
    expect_bit_identical(cam_output, world.model->cam_output);
    expect_bit_identical(cam_sensorsize, world.model->cam_sensorsize);
    expect_bit_identical(cam_intrinsic, world.model->cam_intrinsic);
    expect_bit_identical(qpos, world.data->qpos);
    expect_bit_identical(site_xpos, world.data->site_xpos);
    expect_bit_identical(site_xmat, world.data->site_xmat);
    EXPECT_EQ(std::memcmp(&znear, &world.model->vis.map.znear, sizeof(znear)), 0);
    EXPECT_EQ(std::memcmp(&zfar, &world.model->vis.map.zfar, sizeof(zfar)), 0);
    EXPECT_EQ(offwidth, world.model->vis.global.offwidth);
    EXPECT_EQ(offheight, world.model->vis.global.offheight);
}

TEST(MujocoImageRenderer, T_MJI_I1_040_GrowsOffscreenBuffer)
{
    // RIS-MJI-070
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    ASSERT_EQ(world.model->vis.global.offwidth, 640);
    ASSERT_EQ(world.model->vis.global.offheight, 480);
    ms::MujocoImageRenderer renderer;
    std::string error;
    ASSERT_TRUE(renderer.initialize(world.model, world.data, error)) << error;

    const std::uint32_t width = 1280;
    const std::uint32_t height = 720;
    RenderedImage result{};
    ASSERT_TRUE(render(renderer, request(width, height, true, true), result, error)) << error;
    ASSERT_FALSE(result.rgb.empty());
    ASSERT_FALSE(result.depth.empty());
    expect_box_pixels(result, width, height);
    EXPECT_TRUE(grey(pixel(result, width, width / 2, height - 1)));
    EXPECT_TRUE(std::isfinite(result.depth[(static_cast<std::size_t>(height) - 1) * width + width / 2]));
}

TEST(MujocoImageRenderer, T_MJI_I1_050_BatchUsesEachRequestPose)
{
    // [RIS-MJI-020] [RIS-MJI-090] Two cameras are rendered in one call at the same world state,
    // and each camera comes entirely from its request pose and K.
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    ms::MujocoImageRenderer renderer;
    std::string error;
    ASSERT_TRUE(renderer.initialize(world.model, world.data, error)) << error;

    constexpr std::uint32_t width = 160;
    constexpr std::uint32_t height = 120;
    sa::RenderRequest requests[2] = {
        request(width, height, true, false),
        request(width, height, true, false),
    };
    requests[0].camera_name = "left_pose";
    requests[1].camera_name = "shifted_pose";
    requests[1].pose.position_world.y = 1.0;
    std::vector<std::uint8_t> first(width * height * 3);
    std::vector<std::uint8_t> second(width * height * 3);
    requests[0].rgb = {first.data(), first.size()};
    requests[1].rgb = {second.data(), second.size()};
    sa::RenderStatus statuses[2]{};
    ASSERT_TRUE(renderer.render(requests, 2, statuses));
    EXPECT_TRUE(statuses[0].ok);
    EXPECT_TRUE(statuses[1].ok);
    EXPECT_NE(first, second);

    const double f = requests[0].camera.fx;
    const double cx = requests[0].camera.cx;
    const double cy = requests[0].camera.cy;
    const long first_red_u = std::lround(cx - f * 1.5 / 4.0);
    const long shifted_red_u = std::lround(cx - f * 0.5 / 4.0);
    const long box_v = std::lround(cy);
    RenderedImage first_image{first, {}};
    RenderedImage second_image{second, {}};
    EXPECT_TRUE(dominant_within_one(first_image, width, height, first_red_u, box_v, 0));
    EXPECT_TRUE(dominant_within_one(second_image, width, height, shifted_red_u, box_v, 0));
}

TEST(MujocoImageRenderer, T_MJI_I1_055_SmallDestinationIsUntouched)
{
    // [RIS-MJI-120] A too-small caller destination fails before any byte is written.
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    ms::MujocoImageRenderer renderer;
    std::string error;
    ASSERT_TRUE(renderer.initialize(world.model, world.data, error)) << error;
    auto value = request(16, 12, true, false);
    value.camera_name = "small";
    std::vector<std::uint8_t> destination(16 * 12 * 3, 0xA5);
    const auto before = destination;
    value.rgb = {destination.data(), destination.size() - 1};
    sa::RenderStatus status{};
    EXPECT_FALSE(renderer.render(&value, 1, &status));
    EXPECT_FALSE(status.ok);
    EXPECT_FALSE(status.error.empty());
    EXPECT_EQ(destination, before);
}

// A host (like Drone PRO's viewer) that owns GLFW and keeps its own window current.
class HostWindow {
public:
    HostWindow()
    {
        if (!glfwInit()) return;
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(32, 32, "host", nullptr, nullptr);
        glfwDefaultWindowHints();
        if (window) glfwMakeContextCurrent(window);
    }
    ~HostWindow()
    {
        if (window) glfwDestroyWindow(window);
        glfwTerminate();
    }
    GLFWwindow* window{nullptr};
};

TEST(MujocoImageRenderer, T_MJI_I1_060_RestoresTheHostContextAndHints)
{
    // RIS-MJI-100: creating, rendering and destroying a renderer leave the host's
    // window current, and a window the host creates afterwards gets default hints.
    HostWindow host;
    ASSERT_NE(host.window, nullptr);
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    std::string error;
    {
        ms::MujocoImageRenderer renderer;
        ASSERT_TRUE(renderer.initialize(world.model, world.data, error)) << error;
        EXPECT_EQ(glfwGetCurrentContext(), host.window) << "after initialize";
        GLFWwindow* later = glfwCreateWindow(16, 16, "later", nullptr, nullptr);
        ASSERT_NE(later, nullptr);
        EXPECT_EQ(glfwGetWindowAttrib(later, GLFW_VISIBLE), GLFW_TRUE) << "the hidden hint leaked";
        glfwDestroyWindow(later);
        RenderedImage result{};
        ASSERT_TRUE(render(renderer, request(64, 48, true, true), result, error)) << error;
        EXPECT_EQ(glfwGetCurrentContext(), host.window) << "after render";
    }
    EXPECT_EQ(glfwGetCurrentContext(), host.window) << "after destroy";
}

TEST(MujocoImageRenderer, T_MJI_I1_070_LeavesHostOwnedGlfwRunning)
{
    // RIS-MJI-110: when the host initialised GLFW, destroying the last renderer does
    // not terminate it, so the host's window stays valid.
    HostWindow host;
    ASSERT_NE(host.window, nullptr);
    World world(camera_world());
    ASSERT_NE(world.data, nullptr);
    std::string error;
    {
        ms::MujocoImageRenderer renderer;
        ASSERT_TRUE(renderer.initialize(world.model, world.data, error)) << error;
    }
    glfwGetError(nullptr);
    glfwGetWindowAttrib(host.window, GLFW_VISIBLE);
    EXPECT_EQ(glfwGetError(nullptr), GLFW_NO_ERROR) << "GLFW was terminated under the host";
}

}  // namespace
