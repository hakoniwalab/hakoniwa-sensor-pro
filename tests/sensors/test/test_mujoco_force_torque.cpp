// The true wrench from MuJoCo (I1): a tool hanging from a welded sensor body, rotated
// sensor axes, an external push, and the adapter on top of it.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include <mujoco/mujoco.h>

#include "hakoniwa/mujoco_sensors/force_torque_sensor_adapter.hpp"
#include "hakoniwa/mujoco_sensors/mujoco_wrench_source.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

// Sensor body (0.1 kg) welded under a fixed arm at 1 m; a 2 kg tool 0.1 m forward and 0.2 m below.
// site_quat rotates the site axes relative to the sensor body.
std::string model_xml(const std::string& site_quat)
{
    return R"(<mujoco><option gravity="0 0 -9.81" timestep="0.001"/>
<worldbody>
 <site name="on_world"/>
 <body name="arm" pos="0 0 1">
  <body name="ft_body">
   <site name="ft" quat=")" + site_quat + R"("/>
   <geom type="sphere" size="0.01" mass="0.1" contype="0" conaffinity="0"/>
   <body name="tool" pos="0.1 0 -0.2">
    <geom type="box" size="0.05 0.05 0.05" mass="2" contype="0" conaffinity="0"/>
   </body>
  </body>
 </body>
</worldbody></mujoco>)";
}

struct Model {
    mjModel* m{nullptr};
    mjData* d{nullptr};
    explicit Model(const std::string& xml)
    {
        char error[1000] = "";
        mjSpec* spec = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
        EXPECT_TRUE(spec) << error;
        m = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        EXPECT_TRUE(m);
        d = mj_makeData(m);
    }
    ~Model()
    {
        mj_deleteData(d);
        mj_deleteModel(m);
    }
};

constexpr double kG = 9.81;
constexpr double kMass = 2.1;  // sensor body and tool: everything below the sensor is the tool side

}  // namespace

// T-FTS-I1-010 [FTS-MJW-020] [FTS-MJW-030] [FTS-MJW-040] Hanging tool at rest: weight along gravity,
// torque r x F about the site
TEST(MujocoForceTorque, HangingToolReadsItsWeight)
{
    Model model(model_xml("1 0 0 0"));
    ms::MujocoWrenchSource source;
    std::string error;
    ASSERT_TRUE(source.initialize(model.m, "ft", error)) << error;
    mj_step(model.m, model.d);
    sa::ForceTorqueTruth t;
    ASSERT_TRUE(source.read(model.m, model.d, 1'000'000, t, error)) << error;
    EXPECT_NEAR(t.wrench.force.x, 0.0, 1e-9);
    EXPECT_NEAR(t.wrench.force.y, 0.0, 1e-9);
    EXPECT_NEAR(t.wrench.force.z, -kMass * kG, 1e-6);
    // Only the tool is off the site origin: r = (0.1, 0, -0.2), F = (0, 0, -2 g); r x F = (0, 0.2 g, 0).
    EXPECT_NEAR(t.wrench.torque.x, 0.0, 1e-9);
    EXPECT_NEAR(t.wrench.torque.y, 0.1 * 2.0 * kG, 1e-6);
    EXPECT_NEAR(t.wrench.torque.z, 0.0, 1e-9);
    EXPECT_EQ(t.sim_time_ns, 1'000'000u);
    EXPECT_DOUBLE_EQ(t.gravity_world.z, -kG);
}

// T-FTS-I1-020 [FTS-MJW-040] [FTS-MJW-050] Rotated site: the same wrench in the site axes
TEST(MujocoForceTorque, RotatedSiteAxes)
{
    // 90 degrees about x: site y = world z, site z = world -y.
    const double h = std::sqrt(0.5);
    Model model(model_xml("0.70710678118654757 0.70710678118654757 0 0"));
    ms::MujocoWrenchSource source;
    std::string error;
    ASSERT_TRUE(source.initialize(model.m, "ft", error)) << error;
    mj_step(model.m, model.d);
    sa::ForceTorqueTruth t;
    ASSERT_TRUE(source.read(model.m, model.d, 0, t, error)) << error;
    // World force (0, 0, -m g) in site axes: x = 0, y = -m g, z = 0.
    EXPECT_NEAR(t.wrench.force.x, 0.0, 1e-9);
    EXPECT_NEAR(t.wrench.force.y, -kMass * kG, 1e-6);
    EXPECT_NEAR(t.wrench.force.z, 0.0, 1e-9);
    // World-to-sensor rotation is the inverse of the site rotation.
    EXPECT_NEAR(t.orientation_world_to_sensor.w, h, 1e-9);
    EXPECT_NEAR(t.orientation_world_to_sensor.x, -h, 1e-9);
}

// T-FTS-I1-030 [FTS-MJW-020] An external push on the tool adds to the reading (the robot holds both)
TEST(MujocoForceTorque, ExternalPushAddsToReading)
{
    Model model(model_xml("1 0 0 0"));
    ms::MujocoWrenchSource source;
    std::string error;
    ASSERT_TRUE(source.initialize(model.m, "ft", error)) << error;
    const int tool = mj_name2id(model.m, mjOBJ_BODY, "tool");
    model.d->xfrc_applied[6 * tool + 0] = 5.0;  // 5 N along +x at the tool's centre of mass
    mj_step(model.m, model.d);
    sa::ForceTorqueTruth t;
    ASSERT_TRUE(source.read(model.m, model.d, 0, t, error)) << error;
    EXPECT_NEAR(t.wrench.force.x, 5.0, 1e-6);
    EXPECT_NEAR(t.wrench.force.z, -kMass * kG, 1e-6);
    // Torque of the push about the site: r x (5, 0, 0) with r = (0.1, 0, -0.2) -> (0, -1.0, 0); plus weight's 0.2 g.
    EXPECT_NEAR(t.wrench.torque.y, 0.1 * 2.0 * kG - 0.2 * 5.0, 1e-6);
}

// T-FTS-I1-040 [FTS-MJW-010] Missing site and a site on the world body are errors
TEST(MujocoForceTorque, SiteErrors)
{
    Model model(model_xml("1 0 0 0"));
    ms::MujocoWrenchSource source;
    std::string error;
    EXPECT_FALSE(source.initialize(model.m, "nope", error));
    EXPECT_FALSE(source.initialize(model.m, "on_world", error));
}

// T-FTS-I1-050 [FTS-MJW-060] The adapter on MuJoCo: an ideal sensor tared on the hanging tool reads the push
TEST(MujocoForceTorque, AdapterOnMujocoTareThenPush)
{
    Model model(model_xml("1 0 0 0"));
    ms::MujocoWrenchSource source;
    std::string error;
    ASSERT_TRUE(source.initialize(model.m, "ft", error)) << error;
    const std::string path = ::testing::TempDir() + "/ft_mujoco.json";
    std::ofstream(path) << R"({"type":"force_torque","update_rate":1000,"range":{"fxy":100,"fz":100,"txy":10,"tz":10}})";
    sa::ForceTorqueAdapterSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = 1'000'000;
    auto adapter = ms::create_force_torque_sensor_adapter(setup, error);
    ASSERT_TRUE(adapter) << error;
    const int tool = mj_name2id(model.m, mjOBJ_BODY, "tool");
    sa::ForceTorqueSample s;
    for (std::uint64_t k = 0; k < 300; ++k) {
        if (k == 200) {
            ASSERT_TRUE(adapter->tare(error)) << error;
            model.d->xfrc_applied[6 * tool + 2] = -3.0;  // push down 3 N
        }
        mj_step(model.m, model.d);
        sa::ForceTorqueTruth t;
        ASSERT_TRUE(source.read(model.m, model.d, k * 1'000'000, t, error)) << error;
        ASSERT_TRUE(adapter->step(t, s, error)) << error;
    }
    EXPECT_NEAR(s.wrench.force.z, -3.0, 1e-6);
    EXPECT_NEAR(s.wrench.force.x, 0.0, 1e-6);
}
