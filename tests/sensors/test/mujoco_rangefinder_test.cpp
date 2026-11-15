// Rangefinder on a MuJoCo world: a vehicle with a downward rangefinder above flat ground and a
// box. The vehicle's own geoms surround the sensor and must not be hit. Expected ranges are
// computed by hand from the geometry.
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"
#include "hakoniwa/mujoco_sensors/rangefinder_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// Ground at z = 0; a 1 m tall box whose top is at z = 1 under x in [4, 6]. The vehicle has a body
// box that encloses the sensor site; the site's x axis points down (pitch +90 degrees).
const char* kWorld = R"(
<mujoco>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 0.1"/>
    <geom name="block" type="box" pos="5 0 0.5" size="1 1 0.5"/>
    <body name="vehicle" pos="0 0 2">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <body name="arm">
        <geom name="leg" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.01" mass="0.1"/>
      </body>
      <site name="rangefinder" pos="0 0 0" euler="0 90 0"/>
    </body>
  </worldbody>
</mujoco>
)";

constexpr double kPi = 3.14159265358979323846;

double measure(sa::IRangefinderSensorAdapter& a)
{
    std::string e;
    a.reset(e);
    sa::SensorPoseTruth t{};
    sa::RangeSample r{};
    if (!a.step(t, r, e)) std::fprintf(stderr, "  step: %s\n", e.c_str());
    return r.range;
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
    check(caster.initialize(m, d, "rangefinder", "vehicle", e), "ray caster on the site, excluding the vehicle");
    ms::MujocoRayCaster bad;
    check(!bad.initialize(m, d, "no_such_site", "vehicle", e), "an unknown site is an error");

    sa::RangefinderAdapterSetup s{};
    s.config_path = std::string(HAKO_SENSORS_CONFIG_DIR) + "/rangefinder/ideal.json";
    s.ray_caster = &caster;
    s.physics_step_ns = 1000000;
    s.has_random_seed = true;
    auto a = ms::create_rangefinder_sensor_adapter(s, e);
    if (!a) {
        std::fprintf(stderr, "create: %s\n", e.c_str());
        return EXIT_FAILURE;
    }

    // Hovering at 2 m: the ground is 2 m below (the frame and the leg around the sensor are skipped).
    mj_forward(m, d);
    const double level = measure(*a);
    std::printf("level at 2 m: %.9f m\n", level);
    check(std::fabs(level - 2.0) < 1e-9, "level: the ground 2 m below");

    // Pitched by 20 degrees (about y): the beam leaves the vertical by 20 degrees.
    const double tilt = 20.0 * kPi / 180.0;
    d->qpos[3] = std::cos(tilt / 2);
    d->qpos[5] = std::sin(tilt / 2);
    mj_forward(m, d);
    const double tilted = measure(*a);
    std::printf("pitched 20 deg: %.9f m (expected 2 / cos 20 = %.9f)\n", tilted, 2.0 / std::cos(tilt));
    check(std::fabs(tilted - 2.0 / std::cos(tilt)) < 1e-9, "tilted: the slant range to the ground");

    // Level above the box (x = 5): its top is at 1 m, so the range is 1 m.
    d->qpos[0] = 5.0;
    d->qpos[3] = 1.0;
    d->qpos[5] = 0.0;
    mj_forward(m, d);
    const double over_box = measure(*a);
    std::printf("over the box: %.9f m\n", over_box);
    check(std::fabs(over_box - 1.0) < 1e-9, "over the box: its top 1 m below");

    // 15 m up: beyond the 10 m maximum, +inf (REP 117).
    d->qpos[0] = 0.0;
    d->qpos[2] = 15.0;
    mj_forward(m, d);
    const double high = measure(*a);
    check(std::isinf(high) && high > 0, "beyond the maximum: +inf");

    // Without the exclusion, the vehicle's own frame is hit first.
    ms::MujocoRayCaster self;
    self.initialize(m, d, "rangefinder", "", e);
    sa::Ray down{};
    sa::RayHit hit{};
    self.cast(&down, 1, 100.0, &hit, e);
    check(hit.hit && hit.object_id == mj_name2id(m, mjOBJ_GEOM, "frame"), "without exclusion the own frame is hit");

    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deleteSpec(spec);
    if (failures == 0) std::printf("mujoco_rangefinder_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
