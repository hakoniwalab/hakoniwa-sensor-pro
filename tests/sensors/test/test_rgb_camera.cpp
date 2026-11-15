#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/rgb_camera_sensor_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStepNs = 1000000;

std::uint32_t legacy_image_seed(const sa::SensorAdapterSetup& setup)
{
    std::uint32_t hash = 2166136261u;
    auto byte = [&hash](std::uint8_t value) {
        hash ^= value;
        hash *= 16777619u;
    };
    auto word = [&byte](std::uint32_t value) {
        for (unsigned shift = 0; shift < 32; shift += 8)
            byte(static_cast<std::uint8_t>((value >> shift) & 255));
    };
    auto text = [&byte](const std::string& value) {
        for (unsigned char character : value) byte(character);
    };
    word(static_cast<std::uint32_t>(setup.random_seed));
    text(setup.vehicle_name);
    byte(255);
    word(setup.vehicle_index);
    byte(254);
    text(setup.sensor_name);
    byte(253);
    text("image_noise");
    return hash;
}

class TempConfig {
public:
    explicit TempConfig(const std::string& text)
    {
        static std::uint64_t serial = 0;
        path_ = std::filesystem::temp_directory_path() /
                ("hakoniwa-rgb-camera-" + std::to_string(++serial) + ".json");
        std::ofstream(path_) << text;
    }
    ~TempConfig() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

class ScriptedRenderer : public sa::IImageRenderer {
public:
    int flat{-1};
    bool return_colour{true};
    std::vector<sa::RenderRequest> requests;
    std::size_t calls{0};

    bool render(const sa::RenderRequest* input, std::size_t count,
                sa::RenderStatus* statuses) override
    {
        ++calls;
        bool ok = true;
        for (std::size_t i = 0; i < count; ++i) {
            const auto& request = input[i];
            requests.push_back(request);
            statuses[i] = {};
            if (!return_colour) {
                statuses[i].error = "scripted missing colour";
                ok = false;
                continue;
            }
            auto* rgb = static_cast<std::uint8_t*>(request.rgb.data);
            for (std::uint32_t v = 0; v < request.camera.height; ++v) {
                for (std::uint32_t u = 0; u < request.camera.width; ++u) {
                    auto* pixel = rgb + (static_cast<std::size_t>(v) * request.camera.width + u) * 3;
                    pixel[0] = static_cast<std::uint8_t>(flat >= 0 ? flat : u);
                    pixel[1] = static_cast<std::uint8_t>(flat >= 0 ? flat : v);
                    pixel[2] = static_cast<std::uint8_t>(flat >= 0 ? flat : 127);
                }
            }
            statuses[i].ok = true;
        }
        return ok;
    }
    sa::ImageRendererCapabilities capabilities() const override
    {
        sa::ImageRendererCapabilities value{};
        value.outputs = sa::kRenderRgb;
        return value;
    }
};

std::unique_ptr<sa::IRgbCameraSensorAdapter> make(
    const std::string& text, ScriptedRenderer* renderer, std::uint64_t seed = 2468,
    std::string* creation_error = nullptr)
{
    TempConfig config(text);
    sa::RgbCameraAdapterSetup setup{};
    setup.config_path = config.path();
    setup.renderer = renderer;
    setup.physics_step_ns = kStepNs;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    setup.vehicle_name = "vehicle";
    setup.vehicle_index = 4;
    setup.sensor_name = "rgb_camera";
    std::string error;
    auto adapter = ms::create_rgb_camera_sensor_adapter(setup, error);
    if (creation_error) *creation_error = error;
    return adapter;
}

std::string config(const std::string& additions = "")
{
    return R"({"type":"rgb_camera","update_rate":1000,"image":{"width":64,"height":64},"horizontal_fov":1.2)" +
           additions + "}";
}

sa::RgbImage step(sa::IRgbCameraSensorAdapter& adapter, std::uint64_t index = 0)
{
    sa::SensorPoseTruth truth{};
    truth.sim_time_ns = index * kStepNs;
    sa::RgbImages images{};
    std::string error;
    EXPECT_TRUE(adapter.step(truth, images, error)) << error;
    EXPECT_EQ(images.count, 1u);
    return images.count == 1 ? images.images[0] : sa::RgbImage{};
}

sa::RgbImages step_rig(sa::IRgbCameraSensorAdapter& adapter,
                       const sa::SensorPoseTruth& truth = {})
{
    sa::RgbImages images{};
    std::string error;
    EXPECT_TRUE(adapter.step(truth, images, error)) << error;
    return images;
}

std::vector<std::uint8_t> image_bytes(const sa::RgbImage& image)
{
    return {image.data, image.data + static_cast<std::size_t>(image.step) * image.height};
}

std::vector<double> channel_values(sa::IRgbCameraSensorAdapter& adapter, std::size_t minimum_count)
{
    std::vector<double> values;
    for (std::uint64_t index = 0; values.size() < minimum_count; ++index) {
        const auto image = step(adapter, index);
        EXPECT_TRUE(image.updated);
        if (!image.updated || !image.data) return values;
        const std::size_t count = static_cast<std::size_t>(image.step) * image.height;
        for (std::size_t i = 0; i < count; ++i) values.push_back(image.data[i]);
    }
    return values;
}

double mean(const std::vector<double>& values)
{
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double sample_stddev(const std::vector<double>& values)
{
    const double average = mean(values);
    double squared_sum = 0.0;
    for (double value : values) squared_sum += (value - average) * (value - average);
    return std::sqrt(squared_sum / (values.size() - 1));
}

TEST(RgbCamera, T_RGB_U2_010_OutputAndCameraInfo)
{
    // RIS-RGB-030, RIS-RGB-050, RIS-COM-220
    ScriptedRenderer renderer;
    auto adapter = make(R"({"type":"rgb_camera","update_rate":1000,
        "image":{"width":9,"height":7},"horizontal_fov":1.5707963267948966})", &renderer);
    ASSERT_NE(adapter, nullptr);
    const auto image = step(*adapter);
    ASSERT_TRUE(image.updated);
    EXPECT_EQ(image.width, 9u);
    EXPECT_EQ(image.height, 7u);
    EXPECT_EQ(image.step, 27u);
    ASSERT_NE(image.data, nullptr);

    const auto expect_pixel = [&image](std::uint32_t u, std::uint32_t v) {
        const auto* pixel = image.data + (static_cast<std::size_t>(v) * image.width + u) * 3;
        EXPECT_EQ(pixel[0], u);
        EXPECT_EQ(pixel[1], v);
        EXPECT_EQ(pixel[2], 127u);
    };
    expect_pixel(0, 0);
    expect_pixel(8, 0);
    expect_pixel(0, 6);
    expect_pixel(8, 6);
    expect_pixel(2, 1);
    expect_pixel(4, 3);

    EXPECT_EQ(image.info.width, 9u);
    EXPECT_EQ(image.info.height, 7u);
    const std::array<double, 9> expected_k = {4.5, 0.0, 4.0, 0.0, 4.5, 3.0, 0.0, 0.0, 1.0};
    const std::array<double, 9> expected_r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    const std::array<double, 12> expected_p = {
        4.5, 0.0, 4.0, 0.0, 0.0, 4.5, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    // For width 9 and horizontal FOV 90 degrees: f = 4.5 / tan(45 degrees) = 4.5.
    for (std::size_t i = 0; i < expected_k.size(); ++i) EXPECT_NEAR(image.info.k[i], expected_k[i], 4.5e-9);
    for (std::size_t i = 0; i < expected_r.size(); ++i) EXPECT_DOUBLE_EQ(image.info.r[i], expected_r[i]);
    for (std::size_t i = 0; i < expected_p.size(); ++i) EXPECT_NEAR(image.info.p[i], expected_p[i], 4.5e-9);
    EXPECT_TRUE(image.info.distortion_model.empty());
    EXPECT_TRUE(image.info.d.empty());
    EXPECT_DOUBLE_EQ(image.info.sensor_to_optical.w, 0.5);
    EXPECT_DOUBLE_EQ(image.info.sensor_to_optical.x, 0.5);
    EXPECT_DOUBLE_EQ(image.info.sensor_to_optical.y, -0.5);
    EXPECT_DOUBLE_EQ(image.info.sensor_to_optical.z, 0.5);

    const auto capabilities = adapter->capabilities();
    EXPECT_TRUE(capabilities.requires_renderer);
    EXPECT_FALSE(capabilities.requires_ray_caster);
    EXPECT_FALSE(capabilities.thread_safe_across_instances);
}

TEST(RgbCamera, T_RGB_U2_020_RenderRequest)
{
    // RIS-RGB-010
    ScriptedRenderer configured_renderer;
    auto configured = make(R"({"type":"rgb_camera","update_rate":1000,
        "image":{"width":11,"height":5},"horizontal_fov":1.1,
        "clip":{"near":0.2,"far":50.0}})", &configured_renderer);
    ASSERT_NE(configured, nullptr);
    step(*configured);
    ASSERT_EQ(configured_renderer.requests.size(), 1u);
    const auto& request = configured_renderer.requests[0];
    EXPECT_EQ(request.outputs, sa::kRenderRgb);
    EXPECT_NE(request.rgb.data, nullptr);
    EXPECT_EQ(request.camera.width, 11u);
    EXPECT_EQ(request.camera.height, 5u);
    EXPECT_DOUBLE_EQ(request.camera.fx, 5.5 / std::tan(1.1 / 2.0));
    EXPECT_DOUBLE_EQ(request.camera.near_clip, 0.2);
    EXPECT_DOUBLE_EQ(request.camera.far_clip, 50.0);

    ScriptedRenderer default_renderer;
    auto defaults = make(R"({"type":"rgb_camera","update_rate":1000,
        "image":{"width":3,"height":2},"horizontal_fov":0.8})", &default_renderer);
    ASSERT_NE(defaults, nullptr);
    step(*defaults);
    ASSERT_EQ(default_renderer.requests.size(), 1u);
    EXPECT_EQ(default_renderer.requests[0].outputs, sa::kRenderRgb);
    EXPECT_EQ(default_renderer.requests[0].camera.width, 3u);
    EXPECT_EQ(default_renderer.requests[0].camera.height, 2u);
    EXPECT_DOUBLE_EQ(default_renderer.requests[0].camera.fx, 1.5 / std::tan(0.8 / 2.0));
    EXPECT_DOUBLE_EQ(default_renderer.requests[0].camera.near_clip, 0.05);
    EXPECT_DOUBLE_EQ(default_renderer.requests[0].camera.far_clip, 1000.0);
}

TEST(RgbCamera, T_RGB_U2_030_ChannelNoise)
{
    // RIS-RGB-020
    constexpr std::size_t minimum_count = 50000;
    constexpr double expected_sigma = 5.108163140061026;
    ScriptedRenderer zero_mean_renderer;
    zero_mean_renderer.flat = 128;
    auto zero_mean = make(config(R"(,"noise":{"mean":0.0,"stddev":0.02})"), &zero_mean_renderer, 12345);
    ASSERT_NE(zero_mean, nullptr);
    const auto zero_values = channel_values(*zero_mean, minimum_count);
    // sqrt((255 * 0.02)^2 + 1/12) = 5.108163140061026 levels.
    EXPECT_LT(std::fabs(mean(zero_values) - 128.0),
              4.0 * expected_sigma / std::sqrt(static_cast<double>(zero_values.size())));
    EXPECT_NEAR(sample_stddev(zero_values), expected_sigma, expected_sigma * 0.02);

    ScriptedRenderer shifted_renderer;
    shifted_renderer.flat = 128;
    auto shifted = make(config(R"(,"noise":{"mean":0.1,"stddev":0.02})"), &shifted_renderer, 54321);
    ASSERT_NE(shifted, nullptr);
    const auto shifted_values = channel_values(*shifted, minimum_count);
    // A mean of 0.1 full scale shifts 128 by 255 * 0.1 = 25.5 levels.
    EXPECT_LT(std::fabs(mean(shifted_values) - 153.5),
              4.0 * expected_sigma / std::sqrt(static_cast<double>(shifted_values.size())));

    ScriptedRenderer bright_renderer;
    bright_renderer.flat = 254;
    auto bright = make(config(R"(,"noise":{"mean":0.0,"stddev":0.02})"), &bright_renderer, 111);
    ASSERT_NE(bright, nullptr);
    const auto bright_values = image_bytes(step(*bright));
    EXPECT_NE(std::find(bright_values.begin(), bright_values.end(), std::uint8_t{255}), bright_values.end());
    for (std::uint8_t value : bright_values) EXPECT_LE(static_cast<unsigned>(value), 255u);

    ScriptedRenderer dark_renderer;
    dark_renderer.flat = 1;
    auto dark = make(config(R"(,"noise":{"mean":0.0,"stddev":0.02})"), &dark_renderer, 222);
    ASSERT_NE(dark, nullptr);
    const auto dark_values = image_bytes(step(*dark));
    EXPECT_NE(std::find(dark_values.begin(), dark_values.end(), std::uint8_t{0}), dark_values.end());

    ScriptedRenderer seed_renderer_a;
    ScriptedRenderer seed_renderer_b;
    ScriptedRenderer seed_renderer_c;
    seed_renderer_a.flat = 128;
    seed_renderer_b.flat = 128;
    seed_renderer_c.flat = 128;
    const auto noisy_config = config(R"(,"noise":{"mean":0.0,"stddev":0.02})");
    auto a = make(noisy_config, &seed_renderer_a, 9001);
    auto b = make(noisy_config, &seed_renderer_b, 9001);
    auto c = make(noisy_config, &seed_renderer_c, 9002);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_NE(c, nullptr);
    const auto first_a = image_bytes(step(*a));
    const auto first_b = image_bytes(step(*b));
    const auto first_c = image_bytes(step(*c));
    EXPECT_EQ(first_a, first_b);
    EXPECT_NE(first_a, first_c);
    std::string error;
    ASSERT_TRUE(a->reset(error)) << error;
    EXPECT_EQ(image_bytes(step(*a)), first_a);
}

TEST(RgbCamera, T_RGB_U2_040_MissingColourImage)
{
    // RIS-RGB-040
    ScriptedRenderer renderer;
    renderer.return_colour = false;
    auto adapter = make(config(), &renderer);
    ASSERT_NE(adapter, nullptr);
    sa::SensorPoseTruth truth{};
    sa::RgbImages images{};
    std::string error;
    EXPECT_FALSE(adapter->step(truth, images, error));
    EXPECT_FALSE(error.empty());
}

TEST(RgbCamera, T_RGB_U2_050_StereoBatchPoseAndProjection)
{
    // [RIS-RGB-060] [RIS-RGB-070] [RIS-RGB-090]
    ScriptedRenderer renderer;
    auto adapter = make(R"({"type":"rgb_camera","update_rate":1000,
        "image":{"width":9,"height":7},"horizontal_fov":1.5707963267948966,
        "cameras":[
          {"name":"left","position_m":[0,0.06,0],"orientation_deg":{"yaw":10}},
          {"name":"right","position_m":[0,-0.06,0],"orientation_deg":{"yaw":10}}
        ]})", &renderer);
    ASSERT_NE(adapter, nullptr);
    sa::SensorPoseTruth rig{};
    rig.sim_time_ns = 0;
    rig.position_world = {10.0, 20.0, 30.0};
    const auto images = step_rig(*adapter, rig);
    ASSERT_EQ(images.count, 2u);
    ASSERT_EQ(renderer.calls, 1u);
    ASSERT_EQ(renderer.requests.size(), 2u);
    EXPECT_EQ(images.images[0].camera_name, "left");
    EXPECT_EQ(images.images[1].camera_name, "right");
    EXPECT_DOUBLE_EQ(renderer.requests[0].pose.position_world.x, 10.0);
    EXPECT_DOUBLE_EQ(renderer.requests[0].pose.position_world.y, 20.06);
    EXPECT_DOUBLE_EQ(renderer.requests[1].pose.position_world.y, 19.94);
    EXPECT_NEAR(renderer.requests[0].pose.orientation_world_to_sensor.w,
                std::cos(5.0 * 3.14159265358979323846 / 180.0), 1e-12);
    EXPECT_NEAR(renderer.requests[0].pose.orientation_world_to_sensor.z,
                -std::sin(5.0 * 3.14159265358979323846 / 180.0), 1e-12);
    EXPECT_DOUBLE_EQ(images.images[0].info.p[3], 0.0);
    EXPECT_NEAR(images.images[1].info.p[3], -4.5 * 0.12, 1e-12);
    EXPECT_EQ(adapter->capabilities().max_cameras, 8u);
}

TEST(RgbCamera, T_RGB_U2_060_PerCameraNoiseStreams)
{
    // [RIS-RGB-080] Each named camera has an independent stream, reproducible from the seed.
    const std::string rig_config = R"({"type":"rgb_camera","update_rate":1000,
        "image":{"width":64,"height":64},"horizontal_fov":1.2,
        "noise":{"mean":0,"stddev":0.02},
        "cameras":[{"name":"left","position_m":[0,0.05,0]},
                   {"name":"right","position_m":[0,-0.05,0]}]})";
    ScriptedRenderer renderer_a;
    ScriptedRenderer renderer_b;
    renderer_a.flat = 128;
    renderer_b.flat = 128;
    auto a = make(rig_config, &renderer_a, 7001);
    auto b = make(rig_config, &renderer_b, 7001);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    const auto images_a = step_rig(*a);
    const auto images_b = step_rig(*b);
    ASSERT_EQ(images_a.count, 2u);
    ASSERT_EQ(images_b.count, 2u);
    const auto a_left = image_bytes(images_a.images[0]);
    const auto a_right = image_bytes(images_a.images[1]);
    const auto b_left = image_bytes(images_b.images[0]);
    const auto b_right = image_bytes(images_b.images[1]);
    EXPECT_EQ(a_left, b_left);
    EXPECT_EQ(a_right, b_right);
    EXPECT_NE(a_left, a_right);
}

TEST(RgbCamera, T_RGB_U2_070_LegacyUnnamedCameraKeepsNoiseSequence)
{
    // [RIS-RGB-060] [RIS-RGB-080] Omitting cameras preserves the old image_noise stream exactly.
    ScriptedRenderer renderer;
    renderer.flat = 128;
    constexpr std::uint64_t seed = 8123;
    auto adapter = make(config(R"(,"noise":{"mean":0,"stddev":0.02})"), &renderer, seed);
    ASSERT_NE(adapter, nullptr);
    const auto actual = image_bytes(step(*adapter));

    sa::RgbCameraAdapterSetup setup{};
    setup.has_random_seed = true;
    setup.random_seed = seed;
    setup.vehicle_name = "vehicle";
    setup.vehicle_index = 4;
    setup.sensor_name = "rgb_camera";
    std::mt19937 rng(legacy_image_seed(setup));
    std::normal_distribution<double> normal(0.0, 1.0);
    std::vector<std::uint8_t> expected(actual.size());
    for (auto& value : expected) {
        const double noisy = 128.0 + 255.0 * 0.02 * normal(rng);
        value = static_cast<std::uint8_t>(std::clamp(std::round(noisy), 0.0, 255.0));
    }
    EXPECT_EQ(actual, expected);
}

TEST(RgbCamera, T_RGB_U3_020_StereoValidation)
{
    // [RIS-RGB-090] Stereo must share orientation and have only a non-zero y offset.
    ScriptedRenderer renderer;
    std::string error;
    const std::vector<std::string> invalid = {
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},
             "horizontal_fov":1,"cameras":[{"name":"left","position_m":[0,0.05,0]},
             {"name":"right","position_m":[0,-0.05,0],"orientation_deg":{"yaw":1}}]})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},
             "horizontal_fov":1,"cameras":[{"name":"left","position_m":[0,0.05,0]},
             {"name":"right","position_m":[0.01,-0.05,0]}]})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},
             "horizontal_fov":1,"cameras":[{"name":"left"},{"name":"right"}]})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},
             "horizontal_fov":1,"cameras":[{"name":"left","position_m":[0,-0.05,0]},
             {"name":"right","position_m":[0,0.05,0]}]})",
    };
    for (const auto& text : invalid) {
        error.clear();
        EXPECT_EQ(make(text, &renderer, 1, &error), nullptr) << text;
        EXPECT_FALSE(error.empty()) << text;
    }
}

TEST(RgbCamera, T_RGB_U3_010_ConfigurationValidation)
{
    // RIS-RGB-010, RIS-RGB-020, RIS-COM-040
    ScriptedRenderer renderer;
    std::string error;
    EXPECT_EQ(make(config(), nullptr, 1, &error), nullptr);
    EXPECT_FALSE(error.empty());

    const std::vector<std::string> invalid = {
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":1.0,"clip":{"near":0,"far":10}})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":1.0,"clip":{"near":-0.1,"far":10}})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":1.0,"clip":{"near":1,"far":1}})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":1.0,"clip":{"near":2,"far":1}})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":1.0,"noise":{"stddev":-0.01}})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":0,"height":3},"horizontal_fov":1.0})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":0},"horizontal_fov":1.0})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":0})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":-0.1})",
        R"({"type":"rgb_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":3.14159265358979323846})",
        R"({"type":"depth_camera","update_rate":1000,"image":{"width":4,"height":3},"horizontal_fov":1.0})"};
    for (const auto& text : invalid) {
        error.clear();
        EXPECT_EQ(make(text, &renderer, 1, &error), nullptr) << text;
        EXPECT_FALSE(error.empty()) << text;
    }
}

}  // namespace
