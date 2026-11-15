#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/camera_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief RGB camera rig adapter on the host's image renderer (up to eight cameras).
 *
 * setup.config_path is a JSON RGB camera profile (metres, radians):
 *   type "rgb_camera", update_rate [Hz], image {width, height}, horizontal_fov [rad],
 *   clip {near, far} [m], noise {mean, stddev} (per channel, in units of full scale 0..1),
 *   cameras [{name, position_m, orientation_deg}] (omitted for one legacy unnamed camera).
 * The pinhole model and CameraInfo are those of the depth camera. Noise is added to each
 * channel of each pixel, then the value is rounded and clipped to 0..255.
 * setup.renderer is required. Returns nullptr and sets error on failure.
 */
std::unique_ptr<sensor_adapter::IRgbCameraSensorAdapter> create_rgb_camera_sensor_adapter(
    const sensor_adapter::RgbCameraAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::mujoco_sensors
