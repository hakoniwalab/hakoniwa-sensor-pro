#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/lidar_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief LiDAR adapter (2D or 3D) on the host's ray caster.
 *
 * setup.config_path is a JSON LiDAR profile (metres, radians):
 *   type "lidar", update_rate [Hz] (scans per second), scan_mode ("rotating" or "instant"),
 *   scan {horizontal {samples, min_angle, max_angle}, vertical {samples, min_angle, max_angle}},
 *   range {min, max, resolution}, accuracy [{range {min, max}, stddev | percent}],
 *   no_return ("inf" or "limit").
 * Samples are spread evenly from min_angle to max_angle inclusive (one sample: min_angle).
 * A rotating scan casts each column of rays when the rotation reaches it, so the scan is spread
 * over the steps of one period; an instant scan casts every ray at the start of the period.
 * setup.ray_caster is required. Returns nullptr and sets error on failure.
 */
std::unique_ptr<sensor_adapter::ILidarSensorAdapter> create_lidar_sensor_adapter(
    const sensor_adapter::LidarAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::mujoco_sensors
