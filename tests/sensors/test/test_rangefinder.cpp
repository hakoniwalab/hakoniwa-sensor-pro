#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/rangefinder_sensor_adapter.hpp"

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

class TempConfig {
public:
    explicit TempConfig(const std::string& text)
    {
        static std::uint64_t serial = 0;
        path_ = std::filesystem::temp_directory_path() /
                ("hakoniwa-rangefinder-" + std::to_string(++serial) + ".json");
        std::ofstream(path_) << text;
    }
    ~TempConfig() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

class ScriptedCaster : public sa::IRayCaster {
public:
    std::function<double(const sa::Ray&, std::size_t)> distance =
        [](const sa::Ray&, std::size_t) { return 2.0; };
    std::vector<sa::Ray> rays;
    double max_distance{0.0};

    bool cast(const sa::Ray* input, std::size_t count, double maximum, sa::RayHit* hits,
              std::string&) override
    {
        rays.assign(input, input + count);
        max_distance = maximum;
        for (std::size_t i = 0; i < count; ++i) {
            hits[i] = {};
            const double value = distance(input[i], i);
            if (value >= 0.0 && value <= maximum) hits[i] = {true, value, 4, {-1, 0, 0}};
        }
        return true;
    }
};

std::unique_ptr<sa::IRangefinderSensorAdapter> make(
    const std::string& text, ScriptedCaster* caster, std::uint64_t seed = 2468,
    std::string* creation_error = nullptr)
{
    TempConfig config(text);
    sa::RangefinderAdapterSetup setup{};
    setup.config_path = config.path();
    setup.ray_caster = caster;
    setup.physics_step_ns = 1000000;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    auto adapter = ms::create_rangefinder_sensor_adapter(setup, error);
    if (creation_error) *creation_error = error;
    return adapter;
}

sa::RangeSample sample(sa::IRangefinderSensorAdapter& adapter, std::uint64_t index = 0)
{
    sa::SensorPoseTruth truth{};
    truth.sim_time_ns = index * 1000000;
    sa::RangeSample output{};
    std::string error;
    EXPECT_TRUE(adapter.step(truth, output, error)) << error;
    return output;
}

std::string config(const std::string& additions = "")
{
    return R"({"type":"rangefinder","update_rate":1000,"radiation_type":"laser","range":{"min":1,"max":10,"resolution":0})" +
           additions + "}";
}

double mean(const std::vector<double>& values)
{
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double stddev(const std::vector<double>& values)
{
    const double average = mean(values);
    double sum = 0.0;
    for (double value : values) sum += (value - average) * (value - average);
    return std::sqrt(sum / (values.size() - 1));
}

TEST(Rangefinder, T_RNG_U2_010_OutputAndVariance)
{
    // RIS-COM-010, RIS-RNG-010, RIS-RNG-060
    ScriptedCaster caster;
    auto adapter = make(R"({"type":"rangefinder","update_rate":1000,"radiation_type":"infrared",
        "range":{"min":0.25,"max":8,"resolution":0},
        "cone":{"horizontal":0.2,"vertical":0.4,"rays":1}})", &caster);
    ASSERT_NE(adapter, nullptr);
    const auto output = sample(*adapter);
    EXPECT_TRUE(output.updated);
    EXPECT_EQ(output.radiation_type, sa::RadiationType::kInfrared);
    EXPECT_DOUBLE_EQ(output.field_of_view, 0.4);
    EXPECT_DOUBLE_EQ(output.min_range, 0.25);
    EXPECT_DOUBLE_EQ(output.max_range, 8.0);
    EXPECT_DOUBLE_EQ(output.range, 2.0);
    EXPECT_DOUBLE_EQ(output.variance, 0.0);

    auto noisy = make(R"({"type":"rangefinder","update_rate":1000,"range":{"min":0.25,"max":8},
        "accuracy":[{"range":{"min":0.25,"max":8},"stddev":0.2}]})", &caster);
    ASSERT_NE(noisy, nullptr);
    EXPECT_NEAR(sample(*noisy).variance, 0.04, 1e-12);

    caster.distance = [](const sa::Ray&, std::size_t) { return -1.0; };
    std::string error;
    ASSERT_TRUE(noisy->reset(error));
    EXPECT_DOUBLE_EQ(sample(*noisy).variance, 0.0);
}

TEST(Rangefinder, T_RNG_U2_020_RangeBoundaries)
{
    // RIS-COM-120
    ScriptedCaster caster;
    auto adapter = make(config(), &caster);
    ASSERT_NE(adapter, nullptr);
    std::string error;
    for (double truth : {1.0, std::nextafter(1.0, 2.0), std::nextafter(10.0, 9.0), 10.0}) {
        caster.distance = [truth](const sa::Ray&, std::size_t) { return truth; };
        ASSERT_TRUE(adapter->reset(error));
        EXPECT_DOUBLE_EQ(sample(*adapter).range, truth);
    }
    caster.distance = [](const sa::Ray&, std::size_t) { return std::nextafter(1.0, 0.0); };
    ASSERT_TRUE(adapter->reset(error));
    EXPECT_EQ(sample(*adapter).range, -std::numeric_limits<double>::infinity());
    caster.distance = [](const sa::Ray&, std::size_t) { return std::nextafter(10.0, 11.0); };
    ASSERT_TRUE(adapter->reset(error));
    EXPECT_EQ(sample(*adapter).range, std::numeric_limits<double>::infinity());
    caster.distance = [](const sa::Ray&, std::size_t) { return -1.0; };
    ASSERT_TRUE(adapter->reset(error));
    EXPECT_EQ(sample(*adapter).range, std::numeric_limits<double>::infinity());

    auto limited = make(config(R"(,"no_return":"limit")"), &caster);
    ASSERT_NE(limited, nullptr);
    EXPECT_DOUBLE_EQ(sample(*limited).range, 10.0);
    caster.distance = [](const sa::Ray&, std::size_t) { return 0.5; };
    ASSERT_TRUE(limited->reset(error));
    EXPECT_DOUBLE_EQ(sample(*limited).range, 1.0);
}

TEST(Rangefinder, T_RNG_U2_030_ConeGrid)
{
    // RIS-COM-020, RIS-RNG-030, RIS-RNG-040
    const std::vector<std::vector<sa::Vector3>> expected = {
        {{1, 0, 0}},
        {{.5, -.5, -.707106781186547}, {.5, .5, -.707106781186547},
         {.5, -.5, .707106781186547}, {.5, .5, .707106781186547}},
        {{.5, -.5, -.707106781186547}, {.707106781186548, 0, -.707106781186547}, {.5, .5, -.707106781186547},
         {.707106781186548, -.707106781186547, 0}, {1, 0, 0}, {.707106781186548, .707106781186547, 0},
         {.5, -.5, .707106781186547}, {.707106781186548, 0, .707106781186547}, {.5, .5, .707106781186547}},
        {{.5,-.5,-.707106781186547},{.653281482438188,-.270598050073099,-.707106781186547},{.707106781186548,0,-.707106781186547},{.653281482438188,.270598050073099,-.707106781186547},{.5,.5,-.707106781186547},
         {.653281482438188,-.653281482438188,-.382683432365090},{.853553390593274,-.353553390593274,-.382683432365090},{.923879532511287,0,-.382683432365090},{.853553390593274,.353553390593274,-.382683432365090},{.653281482438188,.653281482438188,-.382683432365090},
         {.707106781186548,-.707106781186547,0},{.923879532511287,-.382683432365090,0},{1,0,0},{.923879532511287,.382683432365090,0},{.707106781186548,.707106781186547,0},
         {.653281482438188,-.653281482438188,.382683432365090},{.853553390593274,-.353553390593274,.382683432365090},{.923879532511287,0,.382683432365090},{.853553390593274,.353553390593274,.382683432365090},{.653281482438188,.653281482438188,.382683432365090},
         {.5,-.5,.707106781186547},{.653281482438188,-.270598050073099,.707106781186547},{.707106781186548,0,.707106781186547},{.653281482438188,.270598050073099,.707106781186547},{.5,.5,.707106781186547}}};
    const int counts[] = {1, 4, 9, 25};
    for (std::size_t case_index = 0; case_index < expected.size(); ++case_index) {
        ScriptedCaster caster;
        const int count = counts[case_index];
        auto adapter = make(config(",\"cone\":{\"horizontal\":1.5707963267948966" +
                                  std::string(",\"vertical\":1.5707963267948966") +
                                  ",\"rays\":" + std::to_string(count) + "}"), &caster);
        ASSERT_NE(adapter, nullptr);
        sample(*adapter);
        ASSERT_EQ(caster.rays.size(), expected[case_index].size());
        for (std::size_t i = 0; i < caster.rays.size(); ++i) {
            EXPECT_NEAR(caster.rays[i].direction.x, expected[case_index][i].x, 1e-12);
            EXPECT_NEAR(caster.rays[i].direction.y, expected[case_index][i].y, 1e-12);
            EXPECT_NEAR(caster.rays[i].direction.z, expected[case_index][i].z, 1e-12);
        }
    }

    ScriptedCaster nearest;
    nearest.distance = [](const sa::Ray& ray, std::size_t) {
        return (ray.direction.y > 0.1 ? 1.5 : 3.0) / ray.direction.x;
    };
    auto adapter = make(config(R"(,"cone":{"horizontal":1.5707963267948966,"vertical":0,"rays":9})"), &nearest);
    ASSERT_NE(adapter, nullptr);
    EXPECT_NEAR(sample(*adapter).range, 1.5, 1.5e-9);
    EXPECT_TRUE(nearest.rays[2].direction.y > 0.0);
}

TEST(Rangefinder, T_RNG_U2_040_NoiseStatistics)
{
    // RIS-COM-140
    constexpr std::size_t n = 20000;
    ScriptedCaster caster;
    struct NoiseCase {
        double truth;
        const char* rule;
        double sigma;
    };
    for (const NoiseCase& noise : {NoiseCase{2.0, "\"stddev\":0.1", 0.1},
                                   NoiseCase{5.0, "\"percent\":2.0", 0.1}}) {
        const double truth = noise.truth;
        caster.distance = [truth](const sa::Ray&, std::size_t) { return truth; };
        auto adapter = make(config(",\"accuracy\":[{\"range\":{\"min\":1,\"max\":10}," +
                                   std::string(noise.rule) + "}]") , &caster, 99);
        ASSERT_NE(adapter, nullptr);
        std::vector<double> values;
        values.reserve(n);
        for (std::size_t i = 0; i < n; ++i) values.push_back(sample(*adapter, i).range);
        EXPECT_LT(std::fabs(mean(values) - truth), 4.0 * noise.sigma / std::sqrt(n));
        EXPECT_NEAR(stddev(values), noise.sigma, noise.sigma * 0.03);
    }
}

TEST(Rangefinder, T_RNG_U2_050_CastDistance)
{
    // RIS-RNG-050
    ScriptedCaster caster;
    auto adapter = make(config(R"(,"cone":{"horizontal":1.5707963267948966,"vertical":0,"rays":4})"), &caster);
    ASSERT_NE(adapter, nullptr);
    sample(*adapter);
    // max 10 / cos(45 degrees) = 10 sqrt(2).
    EXPECT_NEAR(caster.max_distance, 14.142135623730951, 1.414213563e-8);
}

TEST(Rangefinder, T_RNG_U2_060_TrueRangeLog)
{
    // RIS-RNG-070
    ScriptedCaster caster;
    caster.distance = [](const sa::Ray& ray, std::size_t index) {
        return (index == 1 ? 2.25 : 4.0) / ray.direction.x;
    };
    auto adapter = make(config(R"(,"cone":{"horizontal":1.0,"vertical":0,"rays":4})"), &caster);
    ASSERT_NE(adapter, nullptr);
    sample(*adapter);
    const auto& columns = adapter->log_columns();
    std::vector<double> values;
    adapter->log_values(values);
    ASSERT_EQ(columns.size(), 1u);
    ASSERT_EQ(values.size(), 1u);
    EXPECT_EQ(columns[0].name, "true_range");
    EXPECT_EQ(columns[0].unit, "m");
    EXPECT_NEAR(values[0], 2.25, 2.25e-9);
}

TEST(Rangefinder, T_RNG_U3_010_ConeDefaultsAndValidation)
{
    // RIS-RNG-020
    ScriptedCaster caster;
    auto default_cone = make(config(), &caster);
    ASSERT_NE(default_cone, nullptr);
    sample(*default_cone);
    ASSERT_EQ(caster.rays.size(), 1u);
    EXPECT_DOUBLE_EQ(caster.rays[0].direction.x, 1.0);
    EXPECT_DOUBLE_EQ(caster.rays[0].direction.y, 0.0);
    EXPECT_DOUBLE_EQ(caster.rays[0].direction.z, 0.0);
    for (const std::string& cone : {
             R"(,"cone":{"horizontal":-0.1,"vertical":0,"rays":1})",
             R"(,"cone":{"horizontal":0,"vertical":-0.1,"rays":1})",
             R"(,"cone":{"horizontal":0,"vertical":0,"rays":0})"}) {
        std::string error;
        EXPECT_EQ(make(config(cone), &caster, 1, &error), nullptr);
        EXPECT_FALSE(error.empty());
    }
}

}  // namespace
