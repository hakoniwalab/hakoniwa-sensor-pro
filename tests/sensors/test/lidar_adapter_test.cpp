// LiDAR adapter against a scripted world (no MuJoCo): the scan grid and angles, the timing of a
// rotating scan column by column, an instant scan, the error model, 3D rows, seeds and reset.
#include "hakoniwa/mujoco_sensors/lidar_sensor_adapter.hpp"

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
    std::vector<std::size_t> casts_per_call;
    bool cast(const sa::Ray* rays, std::size_t n, double max_distance, sa::RayHit* hits, std::string&) override
    {
        casts_per_call.push_back(n);
        for (std::size_t i = 0; i < n; ++i) {
            const double d = distance(rays[i]);
            hits[i] = {};
            if (d >= 0 && d <= max_distance) hits[i] = {true, d, 9, {}};
        }
        return true;
    }
};

std::string config(const char* name) { return std::string(HAKO_SENSORS_CONFIG_DIR) + "/lidar/" + name; }

std::string write_config(const std::string& name, const std::string& text)
{
    const std::string path = "lidar-test-" + name + ".json";
    std::FILE* f = std::fopen(path.c_str(), "w");
    std::fputs(text.c_str(), f);
    std::fclose(f);
    return path;
}

std::unique_ptr<sa::ILidarSensorAdapter> make(const std::string& path, sa::IRayCaster* world, std::uint64_t seed = 7)
{
    sa::LidarAdapterSetup s{};
    s.config_path = path;
    s.ray_caster = world;
    s.physics_step_ns = 1000000;
    s.has_random_seed = true;
    s.random_seed = seed;
    std::string e;
    auto a = ms::create_lidar_sensor_adapter(s, e);
    if (!a) std::fprintf(stderr, "  create: %s\n", e.c_str());
    return a;
}

// Steps until the next complete scan; returns its step index (or -1).
long next_scan(sa::ILidarSensorAdapter& a, std::uint64_t& k, sa::LidarScan& scan, long limit = 100000)
{
    std::string e;
    for (long n = 0; n < limit; ++n, ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        if (!a.step(t, scan, e)) {
            std::fprintf(stderr, "  step: %s\n", e.c_str());
            return -1;
        }
        if (scan.updated) return static_cast<long>(k++);
    }
    return -1;
}

void wall_scan_2d()
{
    // 8 rays from -180 to +135 degrees, 10 Hz; a wall 4 m ahead.
    const auto path = write_config("wall", R"({"type":"lidar","update_rate":10,"scan_mode":"rotating",
        "scan":{"horizontal":{"samples":8,"min_angle":-3.14159265358979,"max_angle":2.35619449019234}},
        "range":{"min":0.1,"max":10}})");
    ScriptedWorld world;
    world.distance = [](const sa::Ray& r) { return r.direction.x > 1e-9 ? 4.0 / r.direction.x : -1.0; };
    auto a = make(path, &world);
    if (!a) return check(false, "wall LiDAR");
    std::uint64_t k = 0;
    sa::LidarScan scan{};
    const long at = next_scan(*a, k, scan);
    check(at == 99 && scan.sample_time_ns == 0, "a rotating scan is complete at the last step of its 100 ms");
    check(scan.horizontal_count == 8 && scan.vertical_count == 1, "2D: one row of 8");
    check(std::fabs(scan.angle_increment - kPi / 4) < 1e-12, "angle increment");
    int hits = 0;
    for (std::uint32_t i = 0; i < 8; ++i) {
        const double az = scan.angle_min + i * scan.angle_increment;
        if (std::isfinite(scan.ranges[i])) {
            ++hits;
            check(std::fabs(scan.ranges[i] - 4.0 / std::cos(az)) < 1e-5, "range along each ray");
        } else {
            check(scan.ranges[i] > 0, "no return is +inf");
        }
    }
    check(hits == 3 && scan.point_count == 3, "rays at -45, 0, +45 degrees hit the wall");
    check(std::fabs(scan.points[1].x - 4.0) < 1e-5 && std::fabs(scan.points[1].y) < 1e-5, "the forward point at (4, 0)");
    check(scan.points[0].y < 0 && scan.points[2].y > 0, "positive angles are to the left (FLU)");
    check(scan.points[0].time_offset < scan.points[1].time_offset && scan.points[1].time_offset < scan.points[2].time_offset,
          "later columns carry later times");
    check(std::fabs(scan.scan_time - 0.1) < 1e-12 && std::fabs(scan.time_increment - 0.1 / 8) < 1e-12, "scan and ray times");
    const long again = next_scan(*a, k, scan);
    check(again == 199 && scan.sample_time_ns == 100000000, "the next scan 100 ms later");
}

void rotation_timing()
{
    // 360 columns over 100 steps: column h is cast at step floor(h * 100 / 360).
    ScriptedWorld world;
    auto a = make(config("lds-02.json"), &world);  // 5 Hz: 200 steps
    if (!a) return check(false, "LDS-02");
    std::uint64_t k = 0;
    sa::LidarScan scan{};
    next_scan(*a, k, scan);
    std::size_t total = 0, most = 0;
    for (std::size_t n : world.casts_per_call) {
        total += n;
        most = std::max(most, n);
    }
    std::printf("LDS-02: %zu rays over %zu casts (at most %zu in one step)\n", total, world.casts_per_call.size(), most);
    check(total == 360 && most == 2, "360 rays spread over the 200 steps of one rotation");

    const auto path = write_config("instant", R"({"type":"lidar","update_rate":10,"scan_mode":"instant",
        "scan":{"horizontal":{"samples":360,"min_angle":-3.14159265358979,"max_angle":3.12413936106985}},
        "range":{"min":0.1,"max":10}})");
    ScriptedWorld w2;
    auto b = make(path, &w2);
    std::uint64_t k2 = 0;
    const long at = next_scan(*b, k2, scan);
    check(at == 0 && w2.casts_per_call.size() == 1 && w2.casts_per_call[0] == 360 && scan.time_increment == 0.0,
          "an instant scan casts every ray at once at the start");
}

void error_model()
{
    // Everything 2 m away: LDS-02 gives 3 % of the distance, sigma 0.06 m.
    ScriptedWorld world;
    world.distance = [](const sa::Ray&) { return 2.0; };
    auto a = make(config("lds-02.json"), &world);
    if (!a) return check(false, "LDS-02");
    std::vector<double> v;
    std::uint64_t k = 0;
    sa::LidarScan scan{};
    for (int n = 0; n < 60; ++n) {
        next_scan(*a, k, scan);
        for (std::uint32_t i = 0; i < scan.horizontal_count; ++i) v.push_back(scan.ranges[i]);
    }
    double m = 0, s = 0;
    for (double x : v) m += x;
    m /= v.size();
    for (double x : v) s += (x - m) * (x - m);
    s = std::sqrt(s / (v.size() - 1));
    std::printf("LDS-02 at 2 m over %zu rays: mean %.4f sd %.4f (expected 2, 0.06)\n", v.size(), m, s);
    check(std::fabs(m - 2.0) < 0.003 && std::fabs(s / 0.06 - 1) < 0.03, "3 % of the distance");
}

void rows_3d()
{
    // 16 rows from -15 to +15 degrees; a floor 1 m below: only downward rows hit it.
    ScriptedWorld world;
    world.distance = [](const sa::Ray& r) { return r.direction.z < -1e-9 ? 1.0 / -r.direction.z : -1.0; };
    auto a = make(config("3d-16ch.json"), &world);
    if (!a) return check(false, "3D LiDAR");
    std::uint64_t k = 0;
    sa::LidarScan scan{};
    next_scan(*a, k, scan);
    check(scan.vertical_count == 16 && scan.horizontal_count == 1800, "16 x 1800");
    std::uint32_t lowest_hits = 0, highest_hits = 0;
    for (std::uint32_t h = 0; h < 1800; ++h) {
        lowest_hits += std::isfinite(scan.ranges[h]);
        highest_hits += std::isfinite(scan.ranges[15 * 1800 + h]);
    }
    check(lowest_hits == 1800 && highest_hits == 0, "row 0 is the lowest (sees the floor), row 15 the highest");
    bool on_floor = true, rings = true;
    for (std::uint32_t i = 0; i < scan.point_count; ++i) {
        on_floor &= std::fabs(scan.points[i].z + 1.0) < 0.2;  // noise 3 cm along the ray
        rings &= scan.points[i].ring < 8;
    }
    check(on_floor && rings && scan.point_count == 8 * 1800, "points on the floor, from the 8 downward rings");
}

void seeds_and_reset()
{
    ScriptedWorld world;
    world.distance = [](const sa::Ray&) { return 3.0; };
    auto a = make(config("lds-01.json"), &world, 1);
    auto b = make(config("lds-01.json"), &world, 1);
    auto c = make(config("lds-01.json"), &world, 2);
    auto first = [](sa::ILidarSensorAdapter& x) {
        std::uint64_t k = 0;
        sa::LidarScan s{};
        next_scan(x, k, s);
        return std::vector<float>(s.ranges, s.ranges + s.horizontal_count);
    };
    const auto va = first(*a), vb = first(*b), vc = first(*c);
    check(va == vb && va != vc, "same seed same scan; another seed another scan");
    std::string e;
    a->reset(e);
    check(first(*a) == va, "reset repeats the scan");
    sa::LidarAdapterSetup s{};
    s.config_path = config("lds-01.json");
    check(!ms::create_lidar_sensor_adapter(s, e) && !e.empty(), "the ray caster is required");
}

}  // namespace

int main()
{
    wall_scan_2d();
    rotation_timing();
    error_model();
    rows_3d();
    seeds_and_reset();
    if (failures == 0) std::printf("lidar_adapter_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
