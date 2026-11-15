#include <gtest/gtest.h>

#include "hakoniwa/mujoco_sensors/mujoco_ray_caster.hpp"

#include <array>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

class World {
public:
    explicit World(const char* xml)
    {
        char message[1000] = "";
        specification = mj_parseXMLString(xml, nullptr, message, sizeof(message));
        EXPECT_NE(specification, nullptr) << message;
        if (!specification) return;
        model = mj_compile(specification, nullptr);
        EXPECT_NE(model, nullptr) << message;
        if (!model) return;
        data = mj_makeData(model);
        EXPECT_NE(data, nullptr);
        if (data) mj_forward(model, data);
    }

    ~World()
    {
        if (data) mj_deleteData(data);
        if (model) mj_deleteModel(model);
        if (specification) mj_deleteSpec(specification);
    }

    mjSpec* specification{nullptr};
    mjModel* model{nullptr};
    mjData* data{nullptr};
};

sa::Ray ray(double x, double y, double z)
{
    sa::Ray value{};
    value.direction = {x, y, z};
    return value;
}

void expect_hit(ms::MujocoRayCaster& caster, const sa::Ray& value, double max_distance,
                int object_id, double distance)
{
    sa::RayHit hit{};
    std::string error;
    ASSERT_TRUE(caster.cast(&value, 1, max_distance, &hit, error)) << error;
    ASSERT_TRUE(hit.hit);
    EXPECT_EQ(hit.object_id, object_id);
    EXPECT_NEAR(hit.distance, distance, 1e-9);
}

template <typename T>
std::vector<T> copy_values(const T* values, std::size_t count)
{
    return std::vector<T>(values, values + count);
}

template <typename T>
void expect_bit_identical(const std::vector<T>& before, const T* after)
{
    ASSERT_NE(after, nullptr);
    EXPECT_EQ(std::memcmp(before.data(), after, before.size() * sizeof(T)), 0);
}

TEST(MujocoRayCaster, T_MJR_I1_010_TransformsSensorAxesFromSite)
{
    // RIS-MJR-010
    const char* xml = R"(
<mujoco>
  <worldbody>
    <site name="plain" pos="0 0 1"/>
    <geom name="plain_x" type="box" pos="3 0 1" size="0.5 0.5 0.5"/>
    <geom name="plain_y" type="box" pos="0 4 1" size="0.5 0.5 0.5"/>
    <geom name="plain_z" type="box" pos="0 0 6" size="0.5 0.5 0.5"/>

    <site name="yaw" pos="20 0 1" euler="0 0 90"/>
    <geom name="yaw_x" type="box" pos="20 3 1" size="0.5 0.5 0.5"/>
    <geom name="yaw_y" type="box" pos="16 0 1" size="0.5 0.5 0.5"/>
    <geom name="yaw_z" type="box" pos="20 0 6" size="0.5 0.5 0.5"/>

    <site name="pitch" pos="40 0 10" euler="0 90 0"/>
    <geom name="pitch_x" type="box" pos="40 0 7" size="0.5 0.5 0.5"/>
    <geom name="pitch_y" type="box" pos="40 4 10" size="0.5 0.5 0.5"/>
    <geom name="pitch_z" type="box" pos="45 0 10" size="0.5 0.5 0.5"/>
  </worldbody>
</mujoco>)";
    World world(xml);
    ASSERT_NE(world.data, nullptr);

    const std::array<sa::Ray, 3> rays = {ray(1, 0, 0), ray(0, 1, 0), ray(0, 0, 1)};
    const std::array<const char*, 3> sites = {"plain", "yaw", "pitch"};
    const std::array<std::array<const char*, 3>, 3> geoms = {{
        {{"plain_x", "plain_y", "plain_z"}},
        {{"yaw_x", "yaw_y", "yaw_z"}},
        {{"pitch_x", "pitch_y", "pitch_z"}},
    }};
    const std::array<double, 3> distances = {2.5, 3.5, 4.5};

    for (std::size_t site = 0; site < sites.size(); ++site) {
        ms::MujocoRayCaster caster;
        std::string error;
        ASSERT_TRUE(caster.initialize(world.model, world.data, sites[site], "", error)) << error;
        for (std::size_t axis = 0; axis < rays.size(); ++axis) {
            const int geom_id = mj_name2id(world.model, mjOBJ_GEOM, geoms[site][axis]);
            ASSERT_GE(geom_id, 0);
            expect_hit(caster, rays[axis], 20.0, geom_id, distances[axis]);
        }
    }
}

TEST(MujocoRayCaster, T_MJR_I1_020_ExcludesVehicleSubtree)
{
    // RIS-MJR-020
    const char* xml = R"(
<mujoco>
  <worldbody>
    <geom name="outside" type="box" pos="5 0 0" size="0.5 0.5 0.5"/>
    <body name="unrelated" pos="0 5 0">
      <geom name="unrelated_geom" type="box" size="0.5 0.5 0.5"/>
    </body>
    <body name="vehicle">
      <geom name="vehicle_shell" type="box" size="1 1 1"/>
      <site name="sensor"/>
      <body name="vehicle_child" pos="2.5 0 0">
        <geom name="child_geom" type="box" size="0.5 0.5 0.5"/>
      </body>
    </body>
  </worldbody>
</mujoco>)";
    World world(xml);
    ASSERT_NE(world.data, nullptr);
    const sa::Ray forward = ray(1, 0, 0);
    const int shell_id = mj_name2id(world.model, mjOBJ_GEOM, "vehicle_shell");
    const int outside_id = mj_name2id(world.model, mjOBJ_GEOM, "outside");
    ASSERT_GE(shell_id, 0);
    ASSERT_GE(outside_id, 0);

    std::string error;
    ms::MujocoRayCaster excluded;
    ASSERT_TRUE(excluded.initialize(world.model, world.data, "sensor", "vehicle", error)) << error;
    expect_hit(excluded, forward, 10.0, outside_id, 4.5);

    ms::MujocoRayCaster not_excluded;
    ASSERT_TRUE(not_excluded.initialize(world.model, world.data, "sensor", "", error)) << error;
    expect_hit(not_excluded, forward, 10.0, shell_id, 1.0);

    ms::MujocoRayCaster wrong_exclusion;
    ASSERT_TRUE(wrong_exclusion.initialize(world.model, world.data, "sensor", "unrelated", error)) << error;
    expect_hit(wrong_exclusion, forward, 10.0, shell_id, 1.0);
}

TEST(MujocoRayCaster, T_MJR_I1_030_ReturnsGeomIdAndSensorNormal)
{
    // RIS-MJR-030
    const char* xml = R"(
<mujoco>
  <worldbody>
    <site name="plain" pos="0 0 1"/>
    <geom name="plain_wall" type="box" pos="5 0 1" size="0.5 2 2"/>
    <site name="yaw" pos="20 0 1" euler="0 0 90"/>
    <geom name="yaw_wall" type="box" pos="20 5 1" size="2 0.5 2"/>
  </worldbody>
</mujoco>)";
    World world(xml);
    ASSERT_NE(world.data, nullptr);
    const sa::Ray forward = ray(1, 0, 0);

    for (const char* site : {"plain", "yaw"}) {
        const std::string geom_name = std::string(site) + "_wall";
        const int expected_id = mj_name2id(world.model, mjOBJ_GEOM, geom_name.c_str());
        ASSERT_GE(expected_id, 0);
        ms::MujocoRayCaster caster;
        std::string error;
        ASSERT_TRUE(caster.initialize(world.model, world.data, site, "", error)) << error;
        sa::RayHit hit{};
        ASSERT_TRUE(caster.cast(&forward, 1, 10.0, &hit, error)) << error;
        ASSERT_TRUE(hit.hit);
        EXPECT_EQ(hit.object_id, expected_id);
        EXPECT_NEAR(hit.normal.x, -1.0, 1e-12);
        EXPECT_NEAR(hit.normal.y, 0.0, 1e-12);
        EXPECT_NEAR(hit.normal.z, 0.0, 1e-12);
    }
}

TEST(MujocoRayCaster, T_MJR_I1_040_EnforcesMaximumDistance)
{
    // RIS-MJR-040
    const char* xml = R"(
<mujoco>
  <worldbody>
    <site name="sensor"/>
    <geom name="target" type="box" pos="5.5 0 0" size="0.5 0.5 0.5"/>
  </worldbody>
</mujoco>)";
    World world(xml);
    ASSERT_NE(world.data, nullptr);
    ms::MujocoRayCaster caster;
    std::string error;
    ASSERT_TRUE(caster.initialize(world.model, world.data, "sensor", "", error)) << error;

    const sa::Ray forward = ray(1, 0, 0);
    sa::RayHit hit{};
    ASSERT_TRUE(caster.cast(&forward, 1, 4.99, &hit, error)) << error;
    EXPECT_FALSE(hit.hit);
    ASSERT_TRUE(caster.cast(&forward, 1, 5.01, &hit, error)) << error;
    ASSERT_TRUE(hit.hit);
    EXPECT_NEAR(hit.distance, 5.0, 1e-9);

    const sa::Ray empty = ray(0, 1, 0);
    ASSERT_TRUE(caster.cast(&empty, 1, 100.0, &hit, error)) << error;
    EXPECT_FALSE(hit.hit);
}

TEST(MujocoRayCaster, T_MJR_I1_050_RejectsInvalidInitializationAndUse)
{
    // RIS-MJR-050
    const char* xml = R"(
<mujoco>
  <worldbody>
    <site name="sensor"/>
    <body name="vehicle"><geom type="sphere" size="0.1"/></body>
  </worldbody>
</mujoco>)";
    World world(xml);
    ASSERT_NE(world.data, nullptr);
    std::string error;

    ms::MujocoRayCaster unknown_site;
    EXPECT_FALSE(unknown_site.initialize(world.model, world.data, "missing", "vehicle", error));
    EXPECT_FALSE(error.empty());
    error.clear();
    ms::MujocoRayCaster unknown_body;
    EXPECT_FALSE(unknown_body.initialize(world.model, world.data, "sensor", "missing", error));
    EXPECT_FALSE(error.empty());
    error.clear();
    ms::MujocoRayCaster null_model;
    EXPECT_FALSE(null_model.initialize(nullptr, world.data, "sensor", "vehicle", error));
    EXPECT_FALSE(error.empty());
    error.clear();
    ms::MujocoRayCaster null_data;
    EXPECT_FALSE(null_data.initialize(world.model, nullptr, "sensor", "vehicle", error));
    EXPECT_FALSE(error.empty());

    ms::MujocoRayCaster uninitialized;
    const sa::Ray forward = ray(1, 0, 0);
    sa::RayHit hit{};
    error.clear();
    EXPECT_FALSE(uninitialized.cast(&forward, 1, 10.0, &hit, error));
    EXPECT_FALSE(error.empty());
}

TEST(MujocoRayCaster, T_MJR_I1_060_DoesNotModifyModelOrData)
{
    // RIS-MJR-060
    const char* xml = R"(
<mujoco>
  <worldbody>
    <geom name="wall" type="box" pos="5 0 1" size="0.5 5 5"/>
    <body name="vehicle" pos="0 0 1">
      <freejoint/>
      <geom name="shell" type="box" size="0.5 0.5 0.5" mass="1"/>
      <site name="sensor"/>
    </body>
  </worldbody>
</mujoco>)";
    World world(xml);
    ASSERT_NE(world.data, nullptr);
    ms::MujocoRayCaster caster;
    std::string error;
    ASSERT_TRUE(caster.initialize(world.model, world.data, "sensor", "vehicle", error)) << error;

    const auto qpos = copy_values(world.data->qpos, static_cast<std::size_t>(world.model->nq));
    const auto qvel = copy_values(world.data->qvel, static_cast<std::size_t>(world.model->nv));
    const auto xpos = copy_values(world.data->xpos, static_cast<std::size_t>(world.model->nbody) * 3);
    const auto site_xpos = copy_values(world.data->site_xpos, static_cast<std::size_t>(world.model->nsite) * 3);
    const auto site_xmat = copy_values(world.data->site_xmat, static_cast<std::size_t>(world.model->nsite) * 9);
    const auto geom_size = copy_values(world.model->geom_size, static_cast<std::size_t>(world.model->ngeom) * 3);
    const auto geom_pos = copy_values(world.model->geom_pos, static_cast<std::size_t>(world.model->ngeom) * 3);
    const auto geom_quat = copy_values(world.model->geom_quat, static_cast<std::size_t>(world.model->ngeom) * 4);

    const std::array<sa::Ray, 4> rays = {
        ray(1, 0, 0), ray(0, 1, 0), ray(0, 0, 1), ray(-1, 0, 0),
    };
    std::array<sa::RayHit, 4> hits{};
    for (int iteration = 0; iteration < 1000; ++iteration)
        ASSERT_TRUE(caster.cast(rays.data(), rays.size(), 20.0, hits.data(), error)) << error;

    expect_bit_identical(qpos, world.data->qpos);
    expect_bit_identical(qvel, world.data->qvel);
    expect_bit_identical(xpos, world.data->xpos);
    expect_bit_identical(site_xpos, world.data->site_xpos);
    expect_bit_identical(site_xmat, world.data->site_xmat);
    expect_bit_identical(geom_size, world.model->geom_size);
    expect_bit_identical(geom_pos, world.model->geom_pos);
    expect_bit_identical(geom_quat, world.model->geom_quat);
}

}  // namespace
