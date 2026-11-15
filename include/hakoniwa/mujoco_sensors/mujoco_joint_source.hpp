#pragma once

#include <cstdint>
#include <string>

#include <mujoco/mujoco.h>

#include "hakoniwa/sensor_adapter/joint_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief The truth of one hinge joint in a MuJoCo model (docs/spec/joint 3).
 *
 * position = qpos, velocity = qvel, actuator_torque = qfrc_actuator of the joint's DOF,
 * transmitted_torque = the torque the parent side exerts on the child side about the
 * joint axis: cfrc_int of the joint's body moved to the joint anchor and projected on the axis.
 */
class MujocoJointSource {
public:
    /// Finds the joint. Returns false and sets error when it is missing or not a hinge.
    bool initialize(const mjModel* model, const std::string& joint_name, std::string& error);

    /// Same, for the joint that moves a body (the external sensor plugin's way): the body must
    /// have exactly one joint, a hinge.
    bool initialize(const mjModel* model, int body_id, std::string& error);

    /// Call after mj_step. Recomputes mj_rnePostConstraint and fills the truth.
    bool read(const mjModel* model, mjData* data, std::uint64_t sim_time_ns,
              sensor_adapter::JointSensorTruth& truth, std::string& error) const;

private:
    int joint_id_{-1};
    int body_id_{-1};
    int root_id_{-1};
    int qpos_adr_{-1};
    int dof_adr_{-1};
};

}  // namespace hakoniwa::mujoco_sensors
