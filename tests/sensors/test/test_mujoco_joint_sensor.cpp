// The joint truth from MuJoCo (I1): a link held still against gravity by a position actuator.
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <mujoco/mujoco.h>

#include "hakoniwa/mujoco_sensors/mujoco_joint_source.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

// A 1 kg point-like link 0.5 m along x from a hinge about y, held at 0 by a stiff position actuator;
// a free sliding cart gives a non-hinge joint for the error case.
const char* kXml = R"(<mujoco><option gravity="0 0 -9.81" timestep="0.001"/>
<worldbody><body name="base" pos="0 0 1">
  <body name="link"><joint name="hinge" type="hinge" axis="0 1 0" damping="0.5" armature="0.01"/>
    <geom type="sphere" pos="0.5 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
  </body>
  <body name="cart" pos="0 0.3 0"><joint name="slide" type="slide" axis="1 0 0"/>
    <geom type="sphere" size="0.02" mass="1" contype="0" conaffinity="0"/>
  </body></body></worldbody>
<actuator><position joint="hinge" kp="2000" kv="50"/></actuator></mujoco>)";

struct Model {
    mjModel* m{nullptr};
    mjData* d{nullptr};
    Model()
    {
        char error[1000] = "";
        mjSpec* spec = mj_parseXMLString(kXml, nullptr, error, sizeof(error));
        EXPECT_TRUE(spec) << error;
        m = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        d = mj_makeData(m);
    }
    ~Model()
    {
        mj_deleteData(d);
        mj_deleteModel(m);
    }
};

}  // namespace

// T-JNT-I1-010 [JNT-MJS-020] [JNT-MJS-030] Held still: actuator and transmitted torque both balance gravity
TEST(MujocoJointSensor, HeldAgainstGravity)
{
    Model model;
    ms::MujocoJointSource source;
    std::string error;
    ASSERT_TRUE(source.initialize(model.m, "hinge", error)) << error;
    for (int k = 0; k < 3000; ++k) mj_step(model.m, model.d);
    sa::JointSensorTruth t;
    ASSERT_TRUE(source.read(model.m, model.d, 42, t, error)) << error;
    EXPECT_EQ(t.sim_time_ns, 42u);
    EXPECT_NEAR(t.velocity, 0.0, 1e-6);
    // Gravity on the link about +y at the hinge: r x F = (0.5,0,0) x (0,0,-9.81) = (0, 4.905, 0).
    // The actuator holds it with -4.905 N m about +y (plus a tiny position error).
    EXPECT_NEAR(t.actuator_torque, -0.5 * 9.81, 0.01);
    EXPECT_NEAR(t.transmitted_torque, t.actuator_torque, 1e-6);
    EXPECT_NEAR(t.position, model.d->qpos[model.m->jnt_qposadr[mj_name2id(model.m, mjOBJ_JOINT, "hinge")]], 0.0);
}

// T-JNT-I1-020 [JNT-MJS-010] Missing joint and a non-hinge joint are errors
TEST(MujocoJointSensor, JointErrors)
{
    Model model;
    ms::MujocoJointSource source;
    std::string error;
    EXPECT_FALSE(source.initialize(model.m, "nope", error));
    EXPECT_FALSE(source.initialize(model.m, "slide", error));
}
