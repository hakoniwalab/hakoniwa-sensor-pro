// Depth camera on a MuJoCo world. A camera 2 m above flat ground, pitched 30 degrees down, sees
// the ground in its lower rows. For each pixel the expected planar depth is computed by hand:
// the pixel's ray in the world, the ground hit t = h / -dz, planar depth = t * (ray . forward).
// A box in front shows as a nearer region.
#include "hakoniwa/mujoco_sensors/depth_camera_sensor_adapter.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

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

// Site pitched 15 degrees down: MuJoCo euler about y by +15 degrees turns x towards -z.
const char* kWorld = R"(
<mujoco>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 0.1"/>
    <body name="vehicle" pos="0 0 2">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <site name="camera" pos="0 0 0" euler="0 15 0"/>
    </body>
  </worldbody>
</mujoco>
)";

const char* kBox = R"(
<mujoco>
  <worldbody>
    <geom name="ground" type="plane" size="50 50 0.1"/>
    <geom name="box" type="box" pos="3 0 1" size="0.5 0.5 1"/>
    <body name="vehicle" pos="0 0 1">
      <freejoint/>
      <geom name="frame" type="box" size="0.2 0.2 0.05" mass="1"/>
      <site name="camera" pos="0 0 0"/>
    </body>
  </worldbody>
</mujoco>
)";

struct World {
    mjSpec* spec{nullptr};
    mjModel* m{nullptr};
    mjData* d{nullptr};
    explicit World(const char* xml)
    {
        char err[1000] = "";
        spec = mj_parseXMLString(xml, nullptr, err, sizeof(err));
        m = spec ? mj_compile(spec, nullptr) : nullptr;
        if (!m) std::fprintf(stderr, "MuJoCo: %s\n", err);
        else {
            d = mj_makeData(m);
            mj_forward(m, d);
        }
    }
    ~World()
    {
        if (d) mj_deleteData(d);
        if (m) mj_deleteModel(m);
        if (spec) mj_deleteSpec(spec);
    }
};

sa::DepthImage capture(World& w, const std::string& json, ms::MujocoRayCaster& caster)
{
    std::string e;
    caster.initialize(w.m, w.d, "camera", "vehicle", e);
    const std::string path = "mujoco-depth-test.json";
    std::FILE* f = std::fopen(path.c_str(), "w");
    std::fputs(json.c_str(), f);
    std::fclose(f);
    sa::DepthCameraAdapterSetup s{};
    s.config_path = path;
    s.ray_caster = &caster;
    s.physics_step_ns = 1000000;
    s.has_random_seed = true;
    static std::unique_ptr<sa::IDepthCameraSensorAdapter> keep;
    keep = ms::create_depth_camera_sensor_adapter(s, e);
    sa::DepthImage img{};
    if (!keep) {
        std::fprintf(stderr, "create: %s\n", e.c_str());
        return img;
    }
    keep->step(sa::SensorPoseTruth{}, img, e);
    return img;
}

}  // namespace

int main()
{
    const std::string camera = R"({"type":"depth_camera","update_rate":25,"image":{"width":64,"height":48},
        "horizontal_fov":1.2,"depth_type":"planar","range":{"min":0.1,"max":30}})";
    {
        World w(kWorld);
        if (!w.m) return EXIT_FAILURE;
        ms::MujocoRayCaster caster;
        const auto img = capture(w, camera, caster);
        if (!img.data) return EXIT_FAILURE;
        const double f = img.info.k[0], cy = img.info.k[5];
        const double pitch = 15.0 * kPi / 180.0;
        // Camera axes in the world: forward (cos p, 0, -sin p), left (0, 1, 0), up (sin p, 0, cos p).
        double worst = 0;
        int ground = 0, sky = 0, wrong_sky = 0;
        for (std::uint32_t v = 0; v < 48; ++v)
            for (std::uint32_t u = 0; u < 64; ++u) {
                const double yn = (v - cy) / f;
                // Sensor ray (1, -xn, -yn) in the world; only its vertical part decides the ground hit.
                const double dz = -std::sin(pitch) * 1 + std::cos(pitch) * -yn;
                const float got = img.data[v * 64 + u];
                if (dz < -1e-9) {
                    const double t = 2.0 / -dz;  // along the unnormalised ray (forward component 1)
                    const double expect = t;     // planar depth = forward component * t
                    if (std::isfinite(got) && expect <= 30.0) {
                        ++ground;
                        worst = std::max(worst, std::fabs(got - expect));
                    } else if (expect <= 30.0) {
                        ++wrong_sky;
                    }
                } else {
                    sky += std::isinf(got) && got > 0;
                }
            }
        std::printf("pitched 15 deg at 2 m: %d ground pixels, largest error %.3g m; %d sky pixels\n", ground, worst, sky);
        check(ground > 0 && sky > 0 && wrong_sky == 0, "ground in the lower rows, sky (+inf) above the horizon");
        check(worst < 1e-4, "planar depth of every ground pixel matches the geometry");
    }
    {
        // A box 2.5 m ahead (front face at x = 2.5), 1 m wide: the centre is 2.5 m, the edges see beyond.
        World w(kBox);
        if (!w.m) return EXIT_FAILURE;
        ms::MujocoRayCaster caster;
        const auto img = capture(w, camera, caster);
        if (!img.data) return EXIT_FAILURE;
        const double f = img.info.k[0], cx = img.info.k[2];
        int on_face = 0;
        double worst = 0;
        for (std::uint32_t u = 0; u < 64; ++u) {
            const double xn = (u - cx) / f;
            const float got = img.data[23 * 64 + u];  // the row just above the centre (horizontal rays)
            if (std::fabs(2.5 * xn) < 0.49) {
                ++on_face;
                worst = std::max(worst, std::fabs(got - 2.5));
            }
        }
        std::printf("box face: %d pixels on the face, largest error %.3g m\n", on_face, worst);
        check(on_face > 0 && worst < 1e-4, "the box face at 2.5 m (planar depth is flat across it)");
    }
    if (failures == 0) std::printf("mujoco_depth_camera_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
