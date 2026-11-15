#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/camera_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief Depth camera adapter: every pixel measured with the host's ray caster or renderer.
 *
 * setup.config_path is a JSON depth camera profile (metres, radians):
 *   type "depth_camera", update_rate [Hz], image {width, height}, horizontal_fov [rad],
 *   depth_type ("planar": distance from the camera plane, or "perspective": along the ray),
 *   range {min, max, resolution}, accuracy [{range {min, max}, stddev | percent}],
 *   no_return ("inf" or "limit"), method ("ray": one ray per pixel, exact, needs
 *   setup.ray_caster; "render": the renderer's depth buffer, fast at high resolution, needs
 *   setup.renderer), clip {near, far} [m] for "render" (default: around the range).
 * A pinhole camera with square pixels: fx = fy = (width / 2) / tan(horizontal_fov / 2),
 * cx = (width - 1) / 2, cy = (height - 1) / 2 (pixel centres at integer coordinates, as
 * sensor_msgs/CameraInfo). The range and the error model apply to the reported depth.
 * Returns nullptr and sets error on failure.
 */
std::unique_ptr<sensor_adapter::IDepthCameraSensorAdapter> create_depth_camera_sensor_adapter(
    const sensor_adapter::DepthCameraAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::mujoco_sensors
