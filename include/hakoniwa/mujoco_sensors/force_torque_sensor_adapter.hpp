#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief 6-axis force-torque adapter: the error model of docs/spec/force_torque.
 *
 * setup.config_path is a JSON force-torque profile (N, N m, s, Hz):
 *   type "force_torque", update_rate, range {fxy, fz, txy, tz} (required);
 *   overload, delay, lowpass_cutoff_hz, nonlinearity, scale_error, crosstalk, bias,
 *   drift_per_hour, noise {definition, rate_hz | bandwidth_hz, fxy, fz, txy, tz},
 *   resolution, tare_window (optional). Axis-group values are numbers or "<n>%" of the range.
 * The adapter does not depend on MuJoCo: the host passes the true wrench.
 * Returns nullptr and sets error on failure.
 */
std::unique_ptr<sensor_adapter::IForceTorqueSensorAdapter> create_force_torque_sensor_adapter(
    const sensor_adapter::ForceTorqueAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::mujoco_sensors
