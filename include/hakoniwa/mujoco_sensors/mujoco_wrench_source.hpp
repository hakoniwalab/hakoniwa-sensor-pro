#pragma once

#include <cstdint>
#include <string>

#include <mujoco/mujoco.h>

#include "hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief The true wrench of a force-torque sensor in a MuJoCo model (docs/spec/force_torque 3).
 *
 * The sensor is an MJCF site. Its body (the "sensor body", usually welded between the
 * wrist and the tool) and everything below it are the tool side. The wrench is what the
 * tool side exerts on the robot side, in the site axes, torque about the site origin:
 * a tool hanging at rest gives its weight, pointing along gravity.
 */
class MujocoWrenchSource {
public:
    /// Finds the site. Returns false and sets error when it is missing or on the world body.
    bool initialize(const mjModel* model, const std::string& site_name, std::string& error);

    /// Same, for a sensor given as a body and a mounting pose relative to that body's frame
    /// (the external sensor plugin's way): the body is the sensor body, the mount point is the
    /// sensor origin and the mount orientation gives the sensor axes.
    bool initialize(const mjModel* model, int body_id, const double mount_position[3],
                    const double mount_orientation[4], std::string& error);

    /// Call after mj_step. Recomputes mj_rnePostConstraint and fills the truth.
    bool read(const mjModel* model, mjData* data, std::uint64_t sim_time_ns,
              sensor_adapter::ForceTorqueTruth& truth, std::string& error) const;

private:
    int site_id_{-1};
    int body_id_{-1};
    mjtNum mount_position_[3]{0, 0, 0};
    mjtNum mount_rotation_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
    int root_id_{-1};
};

}  // namespace hakoniwa::mujoco_sensors
