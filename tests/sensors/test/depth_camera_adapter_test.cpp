// Depth camera adapter against a scripted world (no MuJoCo): the pinhole model and CameraInfo,
// planar and perspective depth, image orientation (top row, left column), noise, REP 117, seeds.
#include "hakoniwa/mujoco_sensors/depth_camera_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

int failures = 0;
constexpr double kPi = 3.14159265358979323846;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

class ScriptedWorld : public sa::IRayCaster {
public:
    std::function<double(const sa::Ray&)> distance = [](const sa::Ray&) { return -1.0; };
    bool cast(const sa::Ray* rays, std::size_t n, double max_distance, sa::RayHit* hits, std::string&) override
    {
        for (std::size_t i = 0; i < n; ++i) {
            const double d = distance(rays[i]);
            hits[i] = {};
            if (d >= 0 && d <= max_distance) hits[i] = {true, d, 1, {}};
        }
        return true;
    }
};

std::string write_config(const std::string& name, const std::string& text)
{
    const std::string path = "depth-test-" + name + ".json";
    std::FILE* f = std::fopen(path.c_str(), "w");
    std::fputs(text.c_str(), f);
    std::fclose(f);
    return path;
}

std::unique_ptr<sa::IDepthCameraSensorAdapter> make(const std::string& path, sa::IRayCaster* world, std::uint64_t seed = 7)
{
    sa::DepthCameraAdapterSetup s{};
    s.config_path = path;
    s.ray_caster = world;
    s.physics_step_ns = 1000000;
    s.has_random_seed = true;
    s.random_seed = seed;
    std::string e;
    auto a = ms::create_depth_camera_sensor_adapter(s, e);
    if (!a) std::fprintf(stderr, "  create: %s\n", e.c_str());
    return a;
}

sa::DepthImage frame(sa::IDepthCameraSensorAdapter& a)
{
    std::string e;
    a.reset(e);
    sa::SensorPoseTruth t{};
    sa::DepthImage img{};
    if (!a.step(t, img, e)) std::fprintf(stderr, "  step: %s\n", e.c_str());
    return img;
}

// A 9 x 7 camera, 90 degrees wide.
std::string small(const char* depth_type, const char* extra = "")
{
    return write_config(depth_type, std::string(R"({"type":"depth_camera","update_rate":25,"image":{"width":9,"height":7},
        "horizontal_fov":1.5707963267949,"depth_type":")") + depth_type + R"(","range":{"min":0.1,"max":10})" + extra + "}");
}

std::function<double(const sa::Ray&)> wall(double w)
{
    return [w](const sa::Ray& r) { return r.direction.x > 0 ? w / r.direction.x : -1.0; };
}

void pinhole_and_depth_types()
{
    ScriptedWorld world;
    world.distance = wall(4.0);
    auto planar = make(small("planar"), &world);
    auto perspective = make(small("perspective"), &world);
    if (!planar || !perspective) return check(false, "depth cameras");
    const auto a = frame(*planar);
    const auto b = frame(*perspective);
    // fx = (9 / 2) / tan(45 deg) = 4.5; cx = 4, cy = 3.
    auto near = [](double x, double y) { return std::fabs(x - y) < 1e-12; };
    check(near(a.info.k[0], 4.5) && near(a.info.k[4], 4.5) && a.info.k[2] == 4.0 && a.info.k[5] == 3.0 && a.info.k[8] == 1.0,
          "K of the pinhole camera");
    check(near(a.info.p[0], 4.5) && a.info.p[2] == 4.0 && a.info.p[6] == 3.0 && a.info.p[10] == 1.0, "P = [K | 0]");
    double worst_planar = 0, worst_perspective = 0;
    for (std::uint32_t v = 0; v < 7; ++v)
        for (std::uint32_t u = 0; u < 9; ++u) {
            const double xn = (u - 4.0) / 4.5, yn = (v - 3.0) / 4.5;
            worst_planar = std::max(worst_planar, std::fabs(a.data[v * 9 + u] - 4.0));
            worst_perspective =
                std::max(worst_perspective, std::fabs(b.data[v * 9 + u] - 4.0 * std::sqrt(1 + xn * xn + yn * yn)));
        }
    std::printf("wall 4 m: planar error %.3g m, perspective error %.3g m\n", worst_planar, worst_perspective);
    check(worst_planar < 1e-5, "planar depth of a wall facing the camera is the same everywhere");
    check(worst_perspective < 1e-5, "perspective depth is the distance along each pixel's ray");
    check(a.depth_type == sa::DepthType::kPlanar && b.depth_type == sa::DepthType::kPerspective, "depth types");
}

void image_orientation()
{
    // The floor 1 m below: only rays pointing down hit it -> the bottom rows.
    ScriptedWorld world;
    world.distance = [](const sa::Ray& r) { return r.direction.z < -1e-9 ? 1.0 / -r.direction.z : -1.0; };
    auto a = make(small("planar"), &world);
    auto img = frame(*a);
    check(std::isinf(img.data[0]) && img.data[0] > 0 && std::isfinite(img.data[6 * 9 + 4]),
          "row 0 is the top (sky), the last row the bottom (floor)");
    check(std::isinf(img.data[3 * 9 + 4]), "the centre row looks at the horizon (no hit)");
    // A pole to the left (sensor +y): only the left columns see it.
    world.distance = [](const sa::Ray& r) { return r.direction.y > 0.3 ? 2.0 / r.direction.x : -1.0; };
    img = frame(*a);
    check(std::isfinite(img.data[3 * 9 + 0]) && std::isinf(img.data[3 * 9 + 8]), "column 0 is the left");
}

void noise_and_range()
{
    ScriptedWorld world;
    world.distance = wall(2.0);
    // 2 % of the depth: sigma 0.04 m at 2 m.
    auto a = make(small("planar", R"(,"accuracy":[{"range":{"min":0.1,"max":10},"percent":2}])"), &world);
    std::vector<double> v;
    std::string e;
    sa::DepthImage img{};
    for (std::uint64_t k = 0; v.size() < 20000; ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        a->step(t, img, e);
        if (img.updated)
            for (std::uint32_t i = 0; i < 63; ++i) v.push_back(img.data[i]);
    }
    double m = 0, s = 0;
    for (double x : v) m += x;
    m /= v.size();
    for (double x : v) s += (x - m) * (x - m);
    s = std::sqrt(s / (v.size() - 1));
    std::printf("2 %% at 2 m over %zu pixels: mean %.4f sd %.4f (expected 2, 0.04)\n", v.size(), m, s);
    check(std::fabs(m - 2.0) < 0.002 && std::fabs(s / 0.04 - 1) < 0.03, "noise in percent of the depth");

    world.distance = wall(0.05);
    auto near = frame(*a);
    check(std::isinf(near.data[31]) && near.data[31] < 0, "closer than min is -inf");
    world.distance = wall(20.0);
    auto far = frame(*a);
    check(std::isinf(far.data[31]) && far.data[31] > 0, "beyond max is +inf");

    // Same seed, same image; reset repeats it.
    world.distance = wall(2.0);
    auto b = make(small("planar", R"(,"accuracy":[{"range":{"min":0.1,"max":10},"percent":2}])"), &world);
    const auto i1 = frame(*a), i2 = frame(*b);
    check(std::vector<float>(i1.data, i1.data + 63) == std::vector<float>(i2.data, i2.data + 63), "same seed, same image");

    // 25 Hz on a 1 ms step: a frame every 40 ms.
    a->reset(e);
    int frames = 0;
    for (std::uint64_t k = 0; k < 1000; ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        a->step(t, img, e);
        frames += img.updated;
    }
    check(frames == 25, "25 frames in one second");
    check(!ms::create_depth_camera_sensor_adapter(sa::DepthCameraAdapterSetup{}, e), "the ray caster is required");
    (void)kPi;
}

}  // namespace

int main()
{
    pinhole_and_depth_types();
    image_orientation();
    noise_and_range();
    if (failures == 0) std::printf("depth_camera_adapter_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
