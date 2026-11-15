// Rangefinder adapter against a scripted world (no MuJoCo): timing, REP 117, the error model
// measured against independently computed expectations, the cone, seeds and reset.
#include "hakoniwa/mujoco_sensors/rangefinder_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
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

// A world defined by a function: distance along a ray (or no hit when negative).
class ScriptedWorld : public sa::IRayCaster {
public:
    std::function<double(const sa::Ray&)> distance = [](const sa::Ray&) { return -1.0; };
    bool cast(const sa::Ray* rays, std::size_t n, double max_distance, sa::RayHit* hits, std::string&) override
    {
        for (std::size_t i = 0; i < n; ++i) {
            const double d = distance(rays[i]);
            hits[i] = {};
            if (d >= 0 && d <= max_distance) hits[i] = {true, d, 3, {-1, 0, 0}};
        }
        return true;
    }
};

// A wall perpendicular to the sensor x axis at distance w.
std::function<double(const sa::Ray&)> wall(double w)
{
    return [w](const sa::Ray& r) { return r.direction.x > 0 ? w / r.direction.x : -1.0; };
}

std::string config(const char* name) { return std::string(HAKO_SENSORS_CONFIG_DIR) + "/rangefinder/" + name; }

std::string write_config(const std::string& name, const std::string& text)
{
    const std::string path = std::string("rangefinder-test-") + name + ".json";
    std::ofstream(path) << text;
    return path;
}

std::unique_ptr<sa::IRangefinderSensorAdapter> make(const std::string& path, sa::IRayCaster* world,
                                                    std::uint64_t seed = 7)
{
    sa::RangefinderAdapterSetup s{};
    s.config_path = path;
    s.ray_caster = world;
    s.physics_step_ns = 1000000;
    s.has_random_seed = true;
    s.random_seed = seed;
    std::string e;
    auto a = ms::create_rangefinder_sensor_adapter(s, e);
    if (!a) std::fprintf(stderr, "  create: %s\n", e.c_str());
    return a;
}

// Measured ranges over n samples at a fixed truth.
std::vector<double> samples(sa::IRangefinderSensorAdapter& a, int n, std::uint32_t* flags_or = nullptr)
{
    std::vector<double> v;
    std::string e;
    for (std::uint64_t k = 0; static_cast<int>(v.size()) < n; ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        sa::RangeSample r{};
        if (!a.step(t, r, e)) {
            std::fprintf(stderr, "  step: %s\n", e.c_str());
            break;
        }
        if (r.updated) {
            v.push_back(r.range);
            if (flags_or) *flags_or |= r.flags;
        }
    }
    return v;
}

double mean(const std::vector<double>& v)
{
    double s = 0;
    for (double x : v) s += x;
    return s / v.size();
}
double sd(const std::vector<double>& v)
{
    const double m = mean(v);
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / (v.size() - 1));
}

void ideal_and_rep117()
{
    ScriptedWorld world;
    world.distance = wall(3.0);
    auto a = make(config("ideal.json"), &world);
    if (!a) return check(false, "ideal rangefinder");
    std::string e;
    int updates = 0;
    for (std::uint64_t k = 0; k < 1000; ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        sa::RangeSample r{};
        a->step(t, r, e);
        if (r.updated) {
            ++updates;
            check(r.range == 3.0 && r.min_range == 0.1 && r.max_range == 10.0, "ideal range is the wall distance");
            check(k % 100 == 0, "measures every 100 ms from time 0");
        }
    }
    check(updates == 10, "10 Hz");
    world.distance = wall(0.05);
    a->reset(e);
    const double near = samples(*a, 1)[0];
    check(std::isinf(near) && near < 0, "closer than min is -inf (REP 117)");
    world.distance = wall(12.0);
    a->reset(e);
    const double far = samples(*a, 1)[0];
    check(std::isinf(far) && far > 0, "beyond max is +inf");
    world.distance = [](const sa::Ray&) { return -1.0; };
    a->reset(e);
    const double none = samples(*a, 1)[0];
    check(std::isinf(none) && none > 0, "no hit is +inf");
    sa::SensorPoseTruth t{};
    t.sim_time_ns = 3;
    sa::RangeSample r{};
    check(!a->step(t, r, e), "a time jump is an error");
}

void error_model()
{
    ScriptedWorld world;
    // sigma 2.5 cm below 1 m; 1 % of the distance above; resolution 1 cm.
    world.distance = wall(0.5);
    auto a = make(config("laser-40m.json"), &world);
    if (!a) return check(false, "laser rangefinder");
    auto v = samples(*a, 20000);
    // Quantising N(0.5, 0.025) to 1 cm adds about 0.01^2 / 12 of variance.
    const double expect_near = std::sqrt(0.025 * 0.025 + 0.01 * 0.01 / 12);
    std::printf("laser 0.5 m: mean %.5f sd %.5f (expected 0.5, %.5f)\n", mean(v), sd(v), expect_near);
    check(std::fabs(mean(v) - 0.5) < 0.001 && std::fabs(sd(v) / expect_near - 1) < 0.03, "stddev rule");
    bool on_grid = true;
    for (double x : v) on_grid &= std::fabs(x / 0.01 - std::round(x / 0.01)) < 1e-9;
    check(on_grid, "resolution: every value on the 1 cm grid");

    world.distance = wall(20.0);
    std::string e;
    a->reset(e);
    v = samples(*a, 20000);
    const double expect_far = std::sqrt(0.2 * 0.2 + 0.01 * 0.01 / 12);
    std::printf("laser 20 m: mean %.4f sd %.4f (expected 20, %.4f = 1 %%)\n", mean(v), sd(v), expect_far);
    check(std::fabs(mean(v) - 20.0) < 0.01 && std::fabs(sd(v) / expect_far - 1) < 0.03, "percent rule");

    // Near the maximum the noise is clipped to the range and flagged.
    world.distance = wall(39.9);
    a->reset(e);
    std::uint32_t flags = 0;
    v = samples(*a, 2000, &flags);
    double hi = 0;
    for (double x : v) hi = std::max(hi, x);
    check(hi <= 40.0 && (flags & sa::kSensorFlagSaturated), "clipped at max with the saturated flag");
}

void cone()
{
    ScriptedWorld world;
    // A pole 1.5 m ahead and to the left (only rays with y > 0.05 hit it); a wall at 3 m behind it.
    world.distance = [](const sa::Ray& r) {
        if (r.direction.y > 0.05 && r.direction.x > 0) return 1.5 / r.direction.x;
        return r.direction.x > 0 ? 3.0 / r.direction.x : -1.0;
    };
    auto a = make(config("ultrasound-cone.json"), &world);
    if (!a) return check(false, "ultrasound rangefinder");
    auto v = samples(*a, 5000);
    std::printf("cone: mean %.4f sd %.4f (expected nearest projected 1.5, 0.01)\n", mean(v), sd(v));
    check(std::fabs(mean(v) - 1.5) < 0.001 && std::fabs(sd(v) / 0.01 - 1) < 0.05,
          "a cone reports the nearest hit projected on the axis");
    world.distance = [](const sa::Ray&) { return -1.0; };
    std::string e;
    a->reset(e);
    check(samples(*a, 1)[0] == 4.0, "no_return limit reports the maximum");
}

void seeds_and_reset()
{
    ScriptedWorld world;
    world.distance = wall(5.0);
    auto a = make(config("laser-40m.json"), &world, 1);
    auto b = make(config("laser-40m.json"), &world, 1);
    auto c = make(config("laser-40m.json"), &world, 2);
    const auto va = samples(*a, 200), vb = samples(*b, 200), vc = samples(*c, 200);
    check(va == vb, "same seed, same sequence");
    check(va != vc, "another seed, another sequence");
    std::string e;
    a->reset(e);
    check(samples(*a, 200) == va, "reset repeats the sequence");
    sa::RangefinderAdapterSetup s{};
    s.config_path = config("laser-40m.json");
    check(!ms::create_rangefinder_sensor_adapter(s, e) && !e.empty(), "the ray caster is required");
    s.ray_caster = &world;
    s.config_path = write_config("bad", R"({"type":"rangefinder","update_rate":10,"range":{"min":1,"max":5},
        "accuracy":[{"range":{"min":1,"max":5},"stddev":0.1,"percent":1}]})");
    check(!ms::create_rangefinder_sensor_adapter(s, e), "an accuracy entry has stddev or percent, not both");
}

}  // namespace

int main()
{
    ideal_and_rep117();
    error_model();
    cone();
    seeds_and_reset();
    if (failures == 0) std::printf("rangefinder_adapter_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
