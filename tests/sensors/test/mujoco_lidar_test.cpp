// LiDAR on a MuJoCo world: a 10 m x 10 m room, a vehicle with a horizontal 2D LiDAR 1 m up.
// Expected ranges come from the room's geometry: at azimuth a from the centre, 5 / max(|cos a|, |sin a|).
// The vehicle then flies forward during a rotating scan: each column sees the room from where
// the vehicle was when the rotation reached it.
#include "hakoniwa/mujoco_sensors/lidar_sensor_adapter.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"

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
  <worldbody>
    <geom name="ground" type="plane" size="20 20 0.1"/>
    <geom name="north" type="box" pos="5.1 0 1.5" size="0.1 5.2 1.5"/>
    <geom name="south" type="box" pos="-5.1 0 1.5" size="0.1 5.2 1.5"/>
    <geom name="west" type="box" pos="0 5.1 1.5" size="5.2 0.1 1.5"/>
    <geom name="east" type="box" pos="0 -5.1 1.5" size="5.2 0.1 1.5"/>
    <body name="vehicle" pos="0 0 1">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <site name="lidar" pos="0 0 0"/>
    </body>
  </worldbody>
</mujoco>
)";

double room_range(double az, double x_offset)
{
    // From (x_offset, 0) to the walls at x = +-5 and y = +-5.
    const double c = std::cos(az), s = std::sin(az);
    double t = 1e9;
    if (c > 1e-12) t = std::min(t, (5.0 - x_offset) / c);
    if (c < -1e-12) t = std::min(t, (-5.0 - x_offset) / c);
    if (s > 1e-12) t = std::min(t, 5.0 / s);
    if (s < -1e-12) t = std::min(t, -5.0 / s);
    return t;
}

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
    ms::MujocoRayCaster caster;
    std::string e;
    if (!caster.initialize(m, d, "lidar", "vehicle", e)) {
        std::fprintf(stderr, "caster: %s\n", e.c_str());
        return EXIT_FAILURE;
    }
    // 360 columns at 10 Hz, no noise.
    const char* path = "mujoco-lidar-test.json";
    std::ofstream(path) << R"({"type":"lidar","update_rate":10,"scan_mode":"rotating",
        "scan":{"horizontal":{"samples":360,"min_angle":-3.14159265358979,"max_angle":3.12413936106985}},
        "range":{"min":0.1,"max":20}})";
    sa::LidarAdapterSetup s{};
    s.config_path = path;
    s.ray_caster = &caster;
    s.physics_step_ns = 1000000;
    s.has_random_seed = true;
    auto a = ms::create_lidar_sensor_adapter(s, e);
    if (!a) {
        std::fprintf(stderr, "create: %s\n", e.c_str());
        return EXIT_FAILURE;
    }

    // At rest in the centre.
    mj_forward(m, d);
    sa::LidarScan scan{};
    for (std::uint64_t k = 0; k < 100; ++k) {
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        a->step(t, scan, e);
    }
    double worst = 0;
    for (std::uint32_t i = 0; i < 360; ++i)
        worst = std::max(worst, std::fabs(scan.ranges[i] - room_range(scan.angle_min + i * scan.angle_increment, 0.0)));
    std::printf("at rest: largest error against the room %.3g m\n", worst);
    check(scan.updated && worst < 1e-5, "the room's walls from the centre");

    // Flying north at 10 m/s from x = 0 during the next scan (1 m in 100 ms).
    std::string e2;
    a->reset(e2);
    for (std::uint64_t k = 0; k < 100; ++k) {
        d->qpos[0] = 10.0 * k * 1e-3;
        mj_forward(m, d);
        sa::SensorPoseTruth t{};
        t.sim_time_ns = k * 1000000;
        a->step(t, scan, e);
    }
    worst = 0;
    double stale_back = 0, stale_front = 0;
    for (std::uint32_t i = 0; i < 360; ++i) {
        // Column i is cast at step floor(i * 100 / 360), where the vehicle was at x = 10 m/s * that time.
        const double x = 10.0 * std::floor(i * 100.0 / 360.0) * 1e-3;
        const double az = scan.angle_min + i * scan.angle_increment;
        worst = std::max(worst, std::fabs(scan.ranges[i] - room_range(az, x)));
        if (i == 0) stale_back = scan.ranges[i];
        if (i == 180) stale_front = scan.ranges[i];
    }
    std::printf("flying at 10 m/s: largest error against the room seen from each column's time %.3g m;"
                " back wall %.3f m (cast at x = 0), front wall %.3f m (cast at x = 0.5)\n",
                worst, stale_back, stale_front);
    check(worst < 1e-5, "each column sees the room from where the vehicle was at that time");
    check(std::fabs(stale_back - 5.0) < 1e-5 && std::fabs(stale_front - 4.5) < 1e-5, "motion distortion over one rotation");

    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deleteSpec(spec);
    if (failures == 0) std::printf("mujoco_lidar_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
