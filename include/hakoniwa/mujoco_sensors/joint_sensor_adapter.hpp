#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/joint_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief Joint sensor adapter (encoder and joint torque): the error model of docs/spec/joint.
 *
 * setup.config_path is a JSON joint sensor profile: type "joint", update_rate, delay,
 * encoder {side, gear_ratio, counts_per_rev | bits, kind, wrap, offset, accuracy, backlash,
 * stiffness, noise_counts}, velocity {method, window, lsb}, torque {source, range,
 * scale_error, bias, noise, resolution, friction {coulomb, viscous}}. Angles are rad or
 * "<n>deg" / "<n>arcmin" / "<n>arcsec"; the velocity lsb is rad/s or "<n>rpm".
 * The adapter does not depend on MuJoCo. Returns nullptr and sets error on failure.
 */
std::unique_ptr<sensor_adapter::IJointSensorAdapter> create_joint_sensor_adapter(
    const sensor_adapter::JointSensorAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::mujoco_sensors
