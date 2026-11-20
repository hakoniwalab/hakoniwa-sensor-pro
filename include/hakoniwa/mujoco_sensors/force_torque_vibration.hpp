#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/force_torque_vibration.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief Force-torque vibration of a moving arm (hakoniwa-sensor-adapter docs/spec/force-torque-vibration.md, FTV-*).
 *
 * setup.config_path is a JSON file with "type": "force_torque_vibration" and the optional keys f_ref_hz, joints
 * [{name, gear_ratio, deadband_hz, orders [{order, exponent, resonance, amplitude}]}], modes [{frequency_hz,
 * damping, at_rest, per_activity, per_motion, motion_ramp}] and broadband {exponent, density_per_activity}. A joint, a mode and broadband
 * may also have run_log_sigma and run_group (run-to-run variation, FTV-350..380); a joint may have load_gain
 * {positive, negative, lag_s} (load direction, FTV-170..185; the host then passes the joint loads). Per-axis values are objects of
 * axis groups (fxy, fz, txy, tz) and axes (fx, fy, tx, ty), in N and N m. The model does not depend on MuJoCo.
 * Returns nullptr and sets error on failure.
 */
std::unique_ptr<sensor_adapter::IForceTorqueVibration> create_force_torque_vibration(
    const sensor_adapter::ForceTorqueVibrationSetup& setup, std::string& error);

}  // namespace hakoniwa::mujoco_sensors
