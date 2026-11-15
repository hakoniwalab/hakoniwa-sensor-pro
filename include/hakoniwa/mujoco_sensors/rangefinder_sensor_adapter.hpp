#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/rangefinder_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief Rangefinder adapter (laser, infrared or ultrasound) on the host's ray caster.
 *
 * setup.config_path is a JSON rangefinder profile (metres, radians):
 *   type "rangefinder", update_rate [Hz], radiation_type ("laser", "infrared", "ultrasound"),
 *   range {min, max, resolution}, cone {horizontal, vertical, rays} (full angles; 1 ray = a beam),
 *   accuracy [{range {min, max}, stddev | percent}], no_return ("inf" or "limit").
 * A cone casts its rays on a grid and reports the nearest hit projected on the sensor x axis.
 * setup.ray_caster is required. Returns nullptr and sets error on failure.
 */
std::unique_ptr<sensor_adapter::IRangefinderSensorAdapter> create_rangefinder_sensor_adapter(
    const sensor_adapter::RangefinderAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::mujoco_sensors
