// RGB and rendered depth on a MuJoCo world, with MuJoCo's built-in renderer (needs OpenGL).
// A camera 1 m up looks forward at a red box on the left and a green box on the right, in front
// of a far blue wall, over grey ground. Checks: colours land on the expected side and rows; the
// rendered depth agrees with the ray-cast depth of the same camera pixel by pixel.
#include "hakoniwa/mujoco_sensors/depth_camera_sensor_adapter.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_image_renderer.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"
#include "hakoniwa/mujoco_sensors/rgb_camera_sensor_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

const char* kWorld = R"(
<mujoco>
  <visual><headlight ambient="0.6 0.6 0.6" diffuse="0.4 0.4 0.4"/></visual>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>
    <geom name="ground" type="plane" size="50 50 0.1" rgba="0.5 0.5 0.5 1"/>
    <geom name="red" type="box" pos="4 1.5 1" size="0.5 0.5 1" rgba="1 0 0 1"/>
    <geom name="green" type="box" pos="4 -1.5 1" size="0.5 0.5 1" rgba="0 1 0 1"/>
    <geom name="wall" type="box" pos="12 0 6" size="0.1 20 6" rgba="0 0 1 1"/>
    <body name="vehicle" pos="0 0 1">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1" rgba="1 1 1 1"/>
      <site name="camera" pos="0.25 0 0"/>
    </body>
  </worldbody>
</mujoco>
)";

void write(const char* path, const char* text) { std::ofstream(path) << text; }

}  // namespace

int main()
{
    char err[1000] = "";
    mjSpec* spec = mj_parseXMLString(kWorld, nullptr, err, sizeof(err));
    mjModel* m = spec ? mj_compile(spec, nullptr) : nullptr;
    if (!m) {
        std::fprintf(stderr, "MuJoCo: %s\n", err);
        return EXIT_FAILURE;
    }
    mjData* d = mj_makeData(m);
    mj_forward(m, d);
    std::string e;
    ms::MujocoImageRenderer renderer;
    if (!renderer.initialize(m, d, e)) {
        std::fprintf(stderr, "renderer: %s\n", e.c_str());
        return EXIT_FAILURE;
    }
    ms::MujocoRayCaster caster;
    caster.initialize(m, d, "camera", "vehicle", e);

    write("mujoco-rgb-test.json", R"({"type":"rgb_camera","update_rate":25,"image":{"width":160,"height":120},
        "horizontal_fov":1.2,"clip":{"near":0.05,"far":100}})");
    sa::RgbCameraAdapterSetup rs{};
    rs.config_path = "mujoco-rgb-test.json";
    rs.renderer = &renderer;
    auto rgb = ms::create_rgb_camera_sensor_adapter(rs, e);
    if (!rgb) {
        std::fprintf(stderr, "rgb: %s\n", e.c_str());
        return EXIT_FAILURE;
    }
    const int camera_id = mj_name2id(m, mjOBJ_SITE, "camera");
    mjtNum sensor_to_world[4];
    mju_mat2Quat(sensor_to_world, d->site_xmat + 9 * camera_id);
    sa::SensorPoseTruth truth{};
    truth.position_world = {d->site_xpos[3 * camera_id], d->site_xpos[3 * camera_id + 1],
                            d->site_xpos[3 * camera_id + 2]};
    truth.orientation_world_to_sensor = {sensor_to_world[0], -sensor_to_world[1],
                                         -sensor_to_world[2], -sensor_to_world[3]};
    sa::RgbImages images{};
    check(rgb->step(truth, images, e) && images.count == 1 && images.images[0].updated,
          "an RGB frame");
    const auto& img = images.images[0];

    // The red box is at +y (left) -> left half; green at -y (right) -> right half.
    auto px = [&img](std::uint32_t u, std::uint32_t v) { return img.data + (static_cast<std::size_t>(v) * img.width + u) * 3; };
    auto dominant = [](const std::uint8_t* p) { return p[0] > p[1] + 40 && p[0] > p[2] + 40 ? 'r'
                                                       : p[1] > p[0] + 40 && p[1] > p[2] + 40 ? 'g'
                                                       : p[2] > p[0] + 40 && p[2] > p[1] + 40 ? 'b' : '-'; };
    // Box centres at (4, +-1.5, 1): u = cx - f * y / x, v = cy - f * (z - 1) / x.
    const double f = img.info.k[0], cx = img.info.k[2], cy = img.info.k[5];
    const auto red_u = static_cast<std::uint32_t>(std::lround(cx - f * 1.5 / 3.75));
    const auto green_u = static_cast<std::uint32_t>(std::lround(cx + f * 1.5 / 3.75));
    const auto mid_v = static_cast<std::uint32_t>(std::lround(cy));
    const auto* r = px(red_u, mid_v);
    const auto* g = px(green_u, mid_v);
    const auto* sky = px(80, 2);       // above the boxes: the far blue wall
    const auto* ground = px(80, 117);  // the bottom rows: grey ground
    std::printf("red box pixel (%u, %u) = %u %u %u; green (%u, %u) = %u %u %u; top centre = %u %u %u; bottom = %u %u %u\n",
                red_u, mid_v, r[0], r[1], r[2], green_u, mid_v, g[0], g[1], g[2], sky[0], sky[1], sky[2],
                ground[0], ground[1], ground[2]);
    check(dominant(r) == 'r' && dominant(g) == 'g', "red on the left, green on the right");
    check(dominant(sky) == 'b', "the blue wall in the top rows");
    check(std::abs(ground[0] - ground[1]) < 30 && std::abs(ground[1] - ground[2]) < 30 && ground[0] > 30,
          "grey ground in the bottom rows");

    // Rendered depth against ray-cast depth, same camera.
    const char* depth_common = R"("type":"depth_camera","update_rate":25,"image":{"width":160,"height":120},
        "horizontal_fov":1.2,"depth_type":"planar","range":{"min":0.1,"max":30})";
    write("mujoco-depth-ray.json", (std::string("{") + depth_common + R"(,"method":"ray"})").c_str());
    write("mujoco-depth-render.json", (std::string("{") + depth_common + R"(,"method":"render"})").c_str());
    sa::DepthCameraAdapterSetup ds{};
    ds.ray_caster = &caster;
    ds.renderer = &renderer;
    ds.config_path = "mujoco-depth-ray.json";
    auto by_ray = ms::create_depth_camera_sensor_adapter(ds, e);
    ds.config_path = "mujoco-depth-render.json";
    auto by_render = ms::create_depth_camera_sensor_adapter(ds, e);
    if (!by_ray || !by_render) {
        std::fprintf(stderr, "depth: %s\n", e.c_str());
        return EXIT_FAILURE;
    }
    sa::DepthImage a{}, b{};
    by_ray->step(truth, a, e);
    by_render->step(truth, b, e);
    double worst = 0, worst_rel = 0;
    int compared = 0, edge = 0, mismatch_inf = 0;
    for (std::uint32_t v = 0; v < 120; ++v)
        for (std::uint32_t u = 0; u < 160; ++u) {
            const float x = a.data[v * 160 + u], y = b.data[v * 160 + u];
            if (std::isinf(x) || std::isinf(y)) {
                mismatch_inf += std::isinf(x) != std::isinf(y);
                continue;
            }
            const double diff = std::fabs(x - y);
            // Pixels on an object's edge may see the object in one method and the background in the other.
            if (diff / x > 0.05) {
                ++edge;
                continue;
            }
            ++compared;
            worst = std::max(worst, diff);
            worst_rel = std::max(worst_rel, diff / x);
        }
    std::printf("rendered vs ray-cast planar depth: %d pixels, largest difference %.3g m (%.3g of the depth); "
                "%d edge pixels, %d where only one sees nothing\n",
                compared, worst, worst_rel, edge, mismatch_inf);
    check(compared > 18000 && worst_rel < 1e-3, "the rendered depth agrees with the ray-cast depth");
    check(edge < 400 && mismatch_inf < 200, "they disagree only on edges");

    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deleteSpec(spec);
    if (failures == 0) std::printf("mujoco_rgb_camera_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
