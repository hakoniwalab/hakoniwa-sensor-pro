// Mounting by body and pose (no site): the same results as a site placed at the same pose,
// while the body moves; the body's subtree is the vehicle and is never hit.
#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

// The site "ref" marks the mounting pose used below: 0.3 m forward, 0.1 m to the left and
// 0.05 m down of the vehicle origin, turned by MJCF euler "0 20 30". The mounting quaternion is
// read back from the compiled site, so the test compares the two ways of mounting, not a
// hand-made rotation.
const char* kWorld = R"(
<mujoco>
  <compiler angle="degree"/>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="50 50 0.1" rgba="0.5 0.5 0.5 1"/>
    <geom name="wall" type="box" pos="6 0 2" size="0.1 10 2" rgba="0 0 1 1"/>
    <geom name="post" type="box" pos="3 2 1" size="0.2 0.2 1" rgba="1 0 0 1"/>
    <body name="vehicle" pos="0 0 1.5">
      <freejoint/>
      <geom name="frame" type="box" size="0.4 0.4 0.1" mass="1"/>
      <body name="arm" pos="0.3 0.1 -0.05">
        <geom name="arm_geom" type="sphere" size="0.08" mass="0.1"/>
      </body>
      <site name="ref" pos="0.3 0.1 -0.05" euler="0 20 30"/>
    </body>
  </worldbody>
</mujoco>
)";

struct World {
    mjSpec* spec{nullptr};
    mjModel* m{nullptr};
    mjData* d{nullptr};
    World()
    {
        char err[1000] = "";
        spec = mj_parseXMLString(kWorld, nullptr, err, sizeof(err));
        m = spec ? mj_compile(spec, nullptr) : nullptr;
        if (m) {
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
    int site() const { return mj_name2id(m, mjOBJ_SITE, "ref"); }
    int body() const { return mj_name2id(m, mjOBJ_BODY, "vehicle"); }
};

// A fan of rays in the sensor axes.
std::vector<sa::Ray> fan()
{
    std::vector<sa::Ray> rays;
    for (int i = -4; i <= 4; ++i)
        for (int j = -2; j <= 2; ++j) {
            const double az = 0.2 * i, el = 0.15 * j;
            sa::Ray r{};
            r.direction = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
            rays.push_back(r);
        }
    return rays;
}

}  // namespace

TEST(MujocoMount, T_MJR_I1_070_BodyMountEqualsSite)
{
    // RIS-MJR-070
    World w;
    ASSERT_NE(w.m, nullptr);
    // The mounting pose: the site's pose in the vehicle frame, as MuJoCo compiled it.
    const double position[3] = {w.m->site_pos[3 * w.site()], w.m->site_pos[3 * w.site() + 1], w.m->site_pos[3 * w.site() + 2]};
    const double orientation[4] = {w.m->site_quat[4 * w.site()], w.m->site_quat[4 * w.site() + 1],
                                   w.m->site_quat[4 * w.site() + 2], w.m->site_quat[4 * w.site() + 3]};
    ms::MujocoRayCaster by_site, by_body;
    std::string e;
    ASSERT_TRUE(by_site.initialize(w.m, w.d, "ref", "vehicle", e)) << e;
    ASSERT_TRUE(by_body.initialize(w.m, w.d, w.body(), position, orientation, e)) << e;
    const auto rays = fan();
    // Three vehicle poses: at rest, moved and turned, moved again.
    const double poses[3][7] = {{0, 0, 1.5, 1, 0, 0, 0},
                                {0.5, -0.3, 1.2, std::cos(0.2), 0, 0, std::sin(0.2)},
                                {-1.0, 0.4, 2.0, std::cos(0.1), std::sin(0.1), 0, 0}};
    int hits = 0;
    for (const auto& pose : poses) {
        for (int i = 0; i < 7; ++i) w.d->qpos[i] = pose[i];
        mj_forward(w.m, w.d);
        std::vector<sa::RayHit> a(rays.size()), b(rays.size());
        ASSERT_TRUE(by_site.cast(rays.data(), rays.size(), 50.0, a.data(), e));
        ASSERT_TRUE(by_body.cast(rays.data(), rays.size(), 50.0, b.data(), e));
        for (std::size_t i = 0; i < rays.size(); ++i) {
            EXPECT_EQ(a[i].hit, b[i].hit);
            EXPECT_EQ(a[i].object_id, b[i].object_id);
            if (a[i].hit && b[i].hit) {
                EXPECT_NEAR(a[i].distance, b[i].distance, 1e-9);
                ++hits;
            }
        }
    }
    EXPECT_GT(hits, 50);
}

TEST(MujocoMount, T_MJR_I1_070_BodySubtreeIsTheVehicle)
{
    // RIS-MJR-070: the arm (a child body) sits on the mounting point and must not be hit.
    World w;
    ASSERT_NE(w.m, nullptr);
    const double position[3] = {0.3, 0.1, -0.05};
    const double identity[4] = {1, 0, 0, 0};
    ms::MujocoRayCaster caster;
    std::string e;
    ASSERT_TRUE(caster.initialize(w.m, w.d, w.body(), position, identity, e)) << e;
    sa::Ray forward{};
    sa::RayHit hit{};
    ASSERT_TRUE(caster.cast(&forward, 1, 50.0, &hit, e));
    // From x = 0.3 to the wall's face at x = 5.9.
    ASSERT_TRUE(hit.hit);
    EXPECT_EQ(hit.object_id, mj_name2id(w.m, mjOBJ_GEOM, "wall"));
    EXPECT_NEAR(hit.distance, 5.6, 1e-9);
    ms::MujocoRayCaster bad;
    EXPECT_FALSE(bad.initialize(w.m, w.d, 0, position, identity, e));  // the world body is not a vehicle
    EXPECT_FALSE(bad.initialize(w.m, w.d, w.m->nbody, position, identity, e));
}
