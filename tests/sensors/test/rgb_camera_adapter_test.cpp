// RGB camera adapter against a scripted renderer (no MuJoCo): the request it makes, the image
// layout, CameraInfo, per-channel noise, clipping, seeds and the frame rate.
#include "hakoniwa/mujoco_sensors/rgb_camera_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

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

// Draws pixel (u, v) as (u, v, 128), or a flat value.
class ScriptedRenderer : public sa::IImageRenderer {
public:
    int flat{-1};
    sa::RenderRequest last{};
    bool render(const sa::RenderRequest* requests, std::size_t count,
                sa::RenderStatus* statuses) override
    {
        if (count != 1) return false;
        const auto& r = requests[0];
        last = r;
        auto* rgb = static_cast<std::uint8_t*>(r.rgb.data);
        for (std::uint32_t v = 0; v < r.camera.height; ++v)
            for (std::uint32_t u = 0; u < r.camera.width; ++u) {
                auto* p = rgb + (static_cast<std::size_t>(v) * r.camera.width + u) * 3;
                p[0] = static_cast<std::uint8_t>(flat >= 0 ? flat : u);
                p[1] = static_cast<std::uint8_t>(flat >= 0 ? flat : v);
                p[2] = static_cast<std::uint8_t>(flat >= 0 ? flat : 128);
            }
        statuses[0].ok = true;
        return true;
    }
    sa::ImageRendererCapabilities capabilities() const override
    {
        sa::ImageRendererCapabilities value{};
        value.outputs = sa::kRenderRgb;
        return value;
    }
};

std::string write_config(const std::string& name, const std::string& noise)
{
    const std::string path = "rgb-test-" + name + ".json";
    std::FILE* f = std::fopen(path.c_str(), "w");
    std::fputs((R"({"type":"rgb_camera","update_rate":25,"image":{"width":32,"height":24},
        "horizontal_fov":1.2,"clip":{"near":0.05,"far":200})" + noise + "}").c_str(), f);
    std::fclose(f);
    return path;
}

std::unique_ptr<sa::IRgbCameraSensorAdapter> make(const std::string& path, sa::IImageRenderer* r, std::uint64_t seed = 7)
{
    sa::RgbCameraAdapterSetup s{};
    s.config_path = path;
    s.renderer = r;
    s.physics_step_ns = 1000000;
    s.has_random_seed = true;
    s.random_seed = seed;
    std::string e;
    auto a = ms::create_rgb_camera_sensor_adapter(s, e);
    if (!a) std::fprintf(stderr, "  create: %s\n", e.c_str());
    return a;
}

sa::RgbImage frame(sa::IRgbCameraSensorAdapter& a)
{
    std::string e;
    a.reset(e);
    sa::RgbImages images{};
    if (!a.step(sa::SensorPoseTruth{}, images, e)) std::fprintf(stderr, "  step: %s\n", e.c_str());
    return images.count == 1 ? images.images[0] : sa::RgbImage{};
}

void layout()
{
    ScriptedRenderer r;
    auto a = make(write_config("plain", ""), &r);
    if (!a) return check(false, "RGB camera");
    const auto img = frame(*a);
    check(r.last.outputs == sa::kRenderRgb && r.last.rgb.data &&
              r.last.camera.width == 32 && r.last.camera.height == 24 &&
              r.last.camera.near_clip == 0.05 && r.last.camera.far_clip == 200,
          "asks the renderer for the configured colour image");
    check(img.updated && img.width == 32 && img.height == 24 && img.step == 96, "image size and row bytes");
    const auto* p = img.data + (5 * 32 + 7) * 3;
    check(p[0] == 7 && p[1] == 5 && p[2] == 128, "pixel (u 7, v 5) passes through without noise");
    const double f = 16.0 / std::tan(0.6);
    check(std::fabs(img.info.k[0] - f) < 1e-12 && img.info.k[2] == 15.5 && img.info.k[5] == 11.5, "CameraInfo K");
    check(img.info.sensor_to_optical.w == 0.5 && img.info.sensor_to_optical.x == 0.5, "optical frame");
}

void noise()
{
    ScriptedRenderer r;
    r.flat = 128;
    // stddev 0.02 of full scale = 5.1 levels.
    auto a = make(write_config("noise", R"(,"noise":{"mean":0.0,"stddev":0.02})"), &r);
    if (!a) return check(false, "noisy RGB camera");
    std::vector<double> v;
    std::string e;
    sa::RgbImages images{};
    for (std::uint64_t k = 0; v.size() < 50000; ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        a->step(t, images, e);
        const auto& img = images.images[0];
        if (img.updated)
            for (std::size_t i = 0; i < 32u * 24u * 3u; ++i) v.push_back(img.data[i]);
    }
    double m = 0, s = 0;
    for (double x : v) m += x;
    m /= v.size();
    for (double x : v) s += (x - m) * (x - m);
    s = std::sqrt(s / (v.size() - 1));
    // Rounding to whole levels adds 1/12 of a level squared.
    const double expect = std::sqrt(5.1 * 5.1 + 1.0 / 12);
    std::printf("RGB noise at 128 over %zu values: mean %.3f sd %.3f (expected 128, %.3f)\n", v.size(), m, s, expect);
    check(std::fabs(m - 128) < 0.1 && std::fabs(s / expect - 1) < 0.02, "per-channel noise in full-scale units");

    r.flat = 254;
    const auto bright = frame(*a);
    bool clipped = true;
    for (std::size_t i = 0; i < 32u * 24u * 3u; ++i) clipped &= bright.data[i] <= 255;
    check(clipped, "values stay within 0..255");

    auto b = make(write_config("noise", R"(,"noise":{"mean":0.0,"stddev":0.02})"), &r);
    auto c = make(write_config("noise", R"(,"noise":{"mean":0.0,"stddev":0.02})"), &r, 8);
    const auto ia = frame(*a), ib = frame(*b), ic = frame(*c);
    const std::vector<std::uint8_t> va(ia.data, ia.data + 2304), vb(ib.data, ib.data + 2304), vc(ic.data, ic.data + 2304);
    check(va == vb && va != vc, "same seed same image; another seed another image");

    a->reset(e);
    int frames = 0;
    for (std::uint64_t k = 0; k < 1000; ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        a->step(t, images, e);
        frames += images.images[0].updated;
    }
    check(frames == 25, "25 frames in one second");
    sa::RgbCameraAdapterSetup none{};
    none.config_path = write_config("plain", "");
    check(!ms::create_rgb_camera_sensor_adapter(none, e) && !e.empty(), "the renderer is required");
}

}  // namespace

int main()
{
    layout();
    noise();
    if (failures == 0) std::printf("rgb_camera_adapter_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
