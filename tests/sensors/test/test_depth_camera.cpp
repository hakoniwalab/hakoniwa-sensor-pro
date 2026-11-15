#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/depth_camera_sensor_adapter.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr double kFov90 = 1.5707963267948966;

class TempConfig {
public:
    explicit TempConfig(const std::string& text)
    {
        static std::uint64_t serial = 0;
        path_ = std::filesystem::temp_directory_path() /
                ("hakoniwa-depth-camera-" + std::to_string(++serial) + ".json");
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
        std::vector<sa::Ray> rays;
        double max_distance{0.0};
    };

    std::function<double(const sa::Ray&, std::size_t)> distance =
        [](const sa::Ray&, std::size_t) { return 2.0; };
    std::vector<Call> calls;

    bool cast(const sa::Ray* rays, std::size_t count, double max_distance, sa::RayHit* hits,
              std::string&) override
    {
        calls.push_back({std::vector<sa::Ray>(rays, rays + count), max_distance});
        for (std::size_t i = 0; i < count; ++i) {
            hits[i] = {};
            const double value = distance(rays[i], i);
            if (value >= 0.0 && value <= max_distance) hits[i] = {true, value, 7, {}};
        }
        return true;
    }
};

class ScriptedRenderer : public sa::IImageRenderer {
public:
    std::vector<sa::RenderRequest> requests;
    std::vector<float> depths;

    bool render(const sa::RenderRequest* input, std::size_t count,
                sa::RenderStatus* statuses) override
    {
        for (std::size_t i = 0; i < count; ++i) {
            requests.push_back(input[i]);
            std::copy(depths.begin(), depths.end(), static_cast<float*>(input[i].depth.data));
            statuses[i].ok = true;
        }
        return true;
    }
    sa::ImageRendererCapabilities capabilities() const override
    {
        sa::ImageRendererCapabilities value{};
        value.outputs = sa::kRenderDepth;
        return value;
    }
};

std::unique_ptr<sa::IDepthCameraSensorAdapter> make(
    const std::string& text, ScriptedCaster* caster = nullptr, ScriptedRenderer* renderer = nullptr,
    std::uint64_t seed = 2468, std::string* creation_error = nullptr)
{
    TempConfig config(text);
    sa::DepthCameraAdapterSetup setup{};
    setup.config_path = config.path();
    setup.ray_caster = caster;
    setup.renderer = renderer;
    setup.physics_step_ns = 1000000;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    auto adapter = ms::create_depth_camera_sensor_adapter(setup, error);
    if (creation_error) *creation_error = error;
    return adapter;
}

sa::DepthImage frame(sa::IDepthCameraSensorAdapter& adapter)
{
    sa::DepthImage output{};
    std::string error;
    EXPECT_TRUE(adapter.step(sa::SensorPoseTruth{}, output, error)) << error;
    return output;
}

std::string config(const std::string& additions = "")
{
    return R"({"type":"depth_camera","update_rate":1000,"image":{"width":9,"height":7},)"
           R"("horizontal_fov":1.5707963267948966,"range":{"min":1,"max":10,"resolution":0})" +
           additions + "}";
}

void expect_relative(float actual, double expected)
{
    EXPECT_NEAR(actual, expected, std::max(1e-12, std::fabs(expected) * 1e-6));
}

double average(const std::vector<double>& values)
{
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double sample_stddev(const std::vector<double>& values)
{
    const double mean = average(values);
    double squared = 0.0;
    for (double value : values) squared += (value - mean) * (value - mean);
    return std::sqrt(squared / (values.size() - 1));
}

TEST(DepthCamera, T_DEP_U2_010_OutputAndCameraInfo)
{
    // RIS-COM-220, RIS-DEP-070
    ScriptedCaster caster;
    auto adapter = make(config(), &caster);
    ASSERT_NE(adapter, nullptr);
    const auto output = frame(*adapter);
    ASSERT_TRUE(output.updated);
    EXPECT_EQ(output.sample_time_ns, 0u);
    EXPECT_EQ(output.depth_type, sa::DepthType::kPlanar);
    EXPECT_EQ(output.width, 9u);
    EXPECT_EQ(output.height, 7u);
    ASSERT_NE(output.data, nullptr);
    EXPECT_FLOAT_EQ(output.data[31], 2.0f);
    EXPECT_EQ(output.flags, 0u);

    EXPECT_EQ(output.info.width, 9u);
    EXPECT_EQ(output.info.height, 7u);
    EXPECT_TRUE(output.info.distortion_model.empty());
    EXPECT_TRUE(output.info.d.empty());
    const std::array<double, 9> expected_k = {4.5, 0, 4, 0, 4.5, 3, 0, 0, 1};
    const std::array<double, 9> expected_r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const std::array<double, 12> expected_p = {4.5, 0, 4, 0, 0, 4.5, 3, 0, 0, 0, 1, 0};
    for (std::size_t i = 0; i < expected_k.size(); ++i)
        EXPECT_NEAR(output.info.k[i], expected_k[i], std::max(1e-12, std::fabs(expected_k[i]) * 1e-9));
    EXPECT_EQ(output.info.r, expected_r);
    for (std::size_t i = 0; i < expected_p.size(); ++i)
        EXPECT_NEAR(output.info.p[i], expected_p[i], std::max(1e-12, std::fabs(expected_p[i]) * 1e-9));
    EXPECT_DOUBLE_EQ(output.info.sensor_to_optical.w, 0.5);
    EXPECT_DOUBLE_EQ(output.info.sensor_to_optical.x, 0.5);
    EXPECT_DOUBLE_EQ(output.info.sensor_to_optical.y, -0.5);
    EXPECT_DOUBLE_EQ(output.info.sensor_to_optical.z, 0.5);
}

TEST(DepthCamera, T_DEP_U2_020_PlanarAndPerspective)
{
    // RIS-DEP-010
    ScriptedCaster planar_caster;
    planar_caster.distance = [](const sa::Ray& ray, std::size_t) { return 4.0 / ray.direction.x; };
    auto planar = make(config(R"(,"depth_type":"planar")"), &planar_caster);
    ASSERT_NE(planar, nullptr);
    const auto planar_image = frame(*planar);

    ScriptedCaster perspective_caster;
    perspective_caster.distance = planar_caster.distance;
    auto perspective = make(config(R"(,"depth_type":"perspective")"), &perspective_caster);
    ASSERT_NE(perspective, nullptr);
    const auto perspective_image = frame(*perspective);

    for (std::uint32_t v = 0; v < 7; ++v) {
        for (std::uint32_t u = 0; u < 9; ++u) {
            // f = 4.5, cx = 4, cy = 3 for this literal 9 x 7, 90 degree camera.
            const double xn = (static_cast<double>(u) - 4.0) / 4.5;
            const double yn = (static_cast<double>(v) - 3.0) / 4.5;
            expect_relative(planar_image.data[v * 9 + u], 4.0);
            expect_relative(perspective_image.data[v * 9 + u],
                            4.0 * std::sqrt(1.0 + xn * xn + yn * yn));
        }
    }
}

TEST(DepthCamera, T_DEP_U2_030_RayDirections)
{
    // RIS-COM-210
    ScriptedCaster caster;
    auto adapter = make(config(), &caster);
    ASSERT_NE(adapter, nullptr);
    frame(*adapter);
    ASSERT_EQ(caster.calls.size(), 1u);
    ASSERT_EQ(caster.calls[0].rays.size(), 63u);

    struct ExpectedRay {
        std::size_t index;
        double x;
        double y;
        double z;
    };
    // Normalized (1, -(u - 4) / 4.5, -(v - 3) / 4.5), calculated by hand.
    const double corner_norm = std::sqrt(181.0);
    const std::array<ExpectedRay, 5> expected = {{{0, 9.0 / corner_norm, 8.0 / corner_norm, 6.0 / corner_norm},
                                                  {8, 9.0 / corner_norm, -8.0 / corner_norm, 6.0 / corner_norm},
                                                  {31, 1.0, 0.0, 0.0},
                                                  {54, 9.0 / corner_norm, 8.0 / corner_norm, -6.0 / corner_norm},
                                                  {62, 9.0 / corner_norm, -8.0 / corner_norm, -6.0 / corner_norm}}};
    for (const auto& value : expected) {
        const auto& ray = caster.calls[0].rays[value.index];
        EXPECT_NEAR(ray.direction.x, value.x, 1e-12);
        EXPECT_NEAR(ray.direction.y, value.y, 1e-12);
        EXPECT_NEAR(ray.direction.z, value.z, 1e-12);
    }
    EXPECT_GT(caster.calls[0].rays[0].direction.z, 0.0);
    EXPECT_GT(caster.calls[0].rays[0].direction.y, 0.0);
}

TEST(DepthCamera, T_DEP_U2_040_RayBatchAndDistance)
{
    // RIS-DEP-030, RIS-DEP-060
    ScriptedCaster planar_caster;
    auto planar = make(config(R"(,"depth_type":"planar")"), &planar_caster);
    ASSERT_NE(planar, nullptr);
    frame(*planar);
    ASSERT_EQ(planar_caster.calls.size(), 1u);
    EXPECT_EQ(planar_caster.calls[0].rays.size(), 63u);
    // A corner is normalized (9, 8, 6) / sqrt(181).
    EXPECT_NEAR(planar_caster.calls[0].max_distance, 10.0 / (9.0 / std::sqrt(181.0)), 1e-12);

    ScriptedCaster perspective_caster;
    auto perspective = make(config(R"(,"depth_type":"perspective")"), &perspective_caster);
    ASSERT_NE(perspective, nullptr);
    frame(*perspective);
    ASSERT_EQ(perspective_caster.calls.size(), 1u);
    EXPECT_EQ(perspective_caster.calls[0].rays.size(), 63u);
    EXPECT_DOUBLE_EQ(perspective_caster.calls[0].max_distance, 10.0);
}

TEST(DepthCamera, T_DEP_U2_050_RenderDepthAndRequest)
{
    // RIS-DEP-040
    ScriptedRenderer planar_renderer;
    planar_renderer.depths.assign(63, 4.0f);
    auto planar = make(config(R"(,"method":"render","depth_type":"planar")"), nullptr, &planar_renderer);
    ASSERT_NE(planar, nullptr);
    const auto planar_image = frame(*planar);
    ASSERT_EQ(planar_renderer.requests.size(), 1u);
    const auto& request = planar_renderer.requests[0];
    EXPECT_EQ(request.camera.width, 9u);
    EXPECT_EQ(request.camera.height, 7u);
    EXPECT_DOUBLE_EQ(request.camera.fx, 4.5);
    EXPECT_EQ(request.outputs, sa::kRenderDepth);
    EXPECT_EQ(request.rgb.data, nullptr);
    EXPECT_NE(request.depth.data, nullptr);
    EXPECT_DOUBLE_EQ(request.camera.near_clip, 0.5);
    EXPECT_NEAR(request.camera.far_clip, 20.0 / (9.0 / std::sqrt(181.0)), 1e-12);
    for (std::size_t i = 0; i < 63; ++i) EXPECT_FLOAT_EQ(planar_image.data[i], 4.0f);

    ScriptedRenderer posed_renderer;
    posed_renderer.depths.assign(63, 4.0f);
    auto posed = make(config(R"(,"method":"render")"), nullptr, &posed_renderer);
    ASSERT_NE(posed, nullptr);
    sa::SensorPoseTruth posed_truth{};
    posed_truth.position_world = {1.0, 2.0, 3.0};
    posed_truth.orientation_world_to_sensor = {0.5, 0.5, -0.5, 0.5};
    sa::DepthImage posed_image{};
    std::string posed_error;
    ASSERT_TRUE(posed->step(posed_truth, posed_image, posed_error)) << posed_error;
    ASSERT_EQ(posed_renderer.requests.size(), 1u);
    EXPECT_DOUBLE_EQ(posed_renderer.requests[0].pose.position_world.x, 1.0);
    EXPECT_DOUBLE_EQ(posed_renderer.requests[0].pose.position_world.y, 2.0);
    EXPECT_DOUBLE_EQ(posed_renderer.requests[0].pose.orientation_world_to_sensor.y, -0.5);

    ScriptedRenderer perspective_renderer;
    perspective_renderer.depths.assign(63, 4.0f);
    auto perspective = make(config(R"(,"method":"render","depth_type":"perspective",)"
                                   R"("clip":{"near":0.25,"far":42})"),
                            nullptr, &perspective_renderer);
    ASSERT_NE(perspective, nullptr);
    const auto perspective_image = frame(*perspective);
    ASSERT_EQ(perspective_renderer.requests.size(), 1u);
    EXPECT_DOUBLE_EQ(perspective_renderer.requests[0].camera.near_clip, 0.25);
    EXPECT_DOUBLE_EQ(perspective_renderer.requests[0].camera.far_clip, 42.0);
    for (std::uint32_t v = 0; v < 7; ++v) {
        for (std::uint32_t u = 0; u < 9; ++u) {
            const double xn = (static_cast<double>(u) - 4.0) / 4.5;
            const double yn = (static_cast<double>(v) - 3.0) / 4.5;
            expect_relative(perspective_image.data[v * 9 + u],
                            4.0 * std::sqrt(1.0 + xn * xn + yn * yn));
        }
    }

    ScriptedRenderer small_min_renderer;
    small_min_renderer.depths.assign(63, 2.0f);
    auto small_min = make(R"({"type":"depth_camera","update_rate":1000,)"
                          R"("image":{"width":9,"height":7},"horizontal_fov":1.5707963267948966,)"
                          R"("range":{"min":0.001,"max":10},"method":"render"})",
                          nullptr, &small_min_renderer);
    ASSERT_NE(small_min, nullptr);
    frame(*small_min);
    ASSERT_EQ(small_min_renderer.requests.size(), 1u);
    EXPECT_DOUBLE_EQ(small_min_renderer.requests[0].camera.near_clip, 0.01);
}

TEST(DepthCamera, T_DEP_U2_060_NoiseAndNoReturn)
{
    // RIS-DEP-050
    ScriptedCaster noisy_caster;
    noisy_caster.distance = [](const sa::Ray& ray, std::size_t) { return 2.0 / ray.direction.x; };
    const std::string noisy_config =
        R"({"type":"depth_camera","update_rate":1000,"image":{"width":200,"height":100},)"
        R"("horizontal_fov":1.0,"range":{"min":1,"max":10,"resolution":0.001},)"
        R"("accuracy":[{"range":{"min":1,"max":10},"percent":2}]})";
    auto noisy = make(noisy_config, &noisy_caster, nullptr, 13579);
    ASSERT_NE(noisy, nullptr);
    const auto noisy_image = frame(*noisy);
    std::vector<double> values(noisy_image.data, noisy_image.data + 20000);
    const double expected_sigma = std::sqrt(0.04 * 0.04 + 0.001 * 0.001 / 12.0);
    EXPECT_LT(std::fabs(average(values) - 2.0), 4.0 * expected_sigma / std::sqrt(20000.0));
    EXPECT_NEAR(sample_stddev(values), expected_sigma, expected_sigma * 0.03);

    ScriptedCaster caster;
    auto adapter = make(config(), &caster);
    ASSERT_NE(adapter, nullptr);
    std::string error;
    caster.distance = [](const sa::Ray& ray, std::size_t) { return 0.5 / ray.direction.x; };
    EXPECT_EQ(frame(*adapter).data[31], -std::numeric_limits<float>::infinity());
    ASSERT_TRUE(adapter->reset(error));
    caster.distance = [](const sa::Ray& ray, std::size_t) { return 11.0 / ray.direction.x; };
    EXPECT_EQ(frame(*adapter).data[31], std::numeric_limits<float>::infinity());
    ASSERT_TRUE(adapter->reset(error));
    caster.distance = [](const sa::Ray&, std::size_t) { return -1.0; };
    EXPECT_EQ(frame(*adapter).data[31], std::numeric_limits<float>::infinity());

    auto limited = make(config(R"(,"no_return":"limit")"), &caster);
    ASSERT_NE(limited, nullptr);
    EXPECT_FLOAT_EQ(frame(*limited).data[31], 10.0f);
    ASSERT_TRUE(limited->reset(error));
    caster.distance = [](const sa::Ray& ray, std::size_t) { return 0.5 / ray.direction.x; };
    EXPECT_FLOAT_EQ(frame(*limited).data[31], 1.0f);
}

TEST(DepthCamera, T_DEP_U2_070_Capabilities)
{
    // RIS-DEP-080
    ScriptedCaster caster;
    auto ray = make(config(), &caster);
    ASSERT_NE(ray, nullptr);
    const auto ray_capabilities = ray->capabilities();
    EXPECT_TRUE(ray_capabilities.requires_ray_caster);
    EXPECT_FALSE(ray_capabilities.requires_renderer);
    EXPECT_TRUE(ray_capabilities.thread_safe_across_instances);
    EXPECT_TRUE(ray_capabilities.planar);
    EXPECT_TRUE(ray_capabilities.perspective);

    ScriptedRenderer renderer;
    auto render = make(config(R"(,"method":"render")"), nullptr, &renderer);
    ASSERT_NE(render, nullptr);
    const auto render_capabilities = render->capabilities();
    EXPECT_FALSE(render_capabilities.requires_ray_caster);
    EXPECT_TRUE(render_capabilities.requires_renderer);
    EXPECT_FALSE(render_capabilities.thread_safe_across_instances);
    EXPECT_TRUE(render_capabilities.planar);
    EXPECT_TRUE(render_capabilities.perspective);
}

TEST(DepthCamera, T_DEP_U3_010_DefaultsAndInvalidConfiguration)
{
    // RIS-DEP-010, RIS-DEP-020
    ScriptedCaster caster;
    auto defaults = make(config(), &caster);
    ASSERT_NE(defaults, nullptr);
    EXPECT_EQ(frame(*defaults).depth_type, sa::DepthType::kPlanar);
    EXPECT_EQ(caster.calls.size(), 1u);

    std::string error;
    EXPECT_EQ(make(config(R"(,"depth_type":"radial")"), &caster, nullptr, 1, &error), nullptr);
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(make(config(), nullptr, nullptr, 1, &error), nullptr);
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(make(config(R"(,"method":"render")"), &caster, nullptr, 1, &error), nullptr);
    EXPECT_FALSE(error.empty());
    ScriptedRenderer renderer;
    EXPECT_EQ(make(config(R"(,"method":"unknown")"), &caster, &renderer, 1, &error), nullptr);
    EXPECT_FALSE(error.empty());

    const auto invalid = [&](const std::string& image, const std::string& fov) {
        return R"({"type":"depth_camera","update_rate":1000,"image":)" + image +
               R"(,"horizontal_fov":)" + fov + R"(,"range":{"min":1,"max":10}})";
    };
    for (const std::string& text : {invalid(R"({"width":9,"height":7})", "0"),
                                    invalid(R"({"width":9,"height":7})", "3.14159265358979323846"),
                                    invalid(R"({"width":0,"height":7})", "1.0"),
                                    invalid(R"({"width":9,"height":0})", "1.0")}) {
        EXPECT_EQ(make(text, &caster, nullptr, 1, &error), nullptr);
        EXPECT_FALSE(error.empty());
    }
}

}  // namespace
