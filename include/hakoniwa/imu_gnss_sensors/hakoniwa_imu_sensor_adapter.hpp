#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"

namespace hakoniwa::drone_pro::sensors {

/**
 * @brief Creates an IMU adapter backed by the Drone PRO sensor models (sensor model v2).
 *
 * setup.config_path is a drone_config JSON file; its components.sensors.imu,
 * acc and gyro give the models. The IMU mount must be zero (the host applies
 * the mounting pose). The interface frames (sensor FLU) are converted to the
 * models' FRD at the boundary.
 */
std::unique_ptr<hakoniwa::sensor_adapter::IImuSensorAdapter> create_hakoniwa_imu_sensor_adapter(
    const hakoniwa::sensor_adapter::ImuAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::drone_pro::sensors
