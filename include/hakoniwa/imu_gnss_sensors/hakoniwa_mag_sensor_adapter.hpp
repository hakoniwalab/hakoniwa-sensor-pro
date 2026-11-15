#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/mag_sensor_adapter.hpp"

namespace hakoniwa::drone_pro::sensors {

/**
 * @brief Creates a mag adapter backed by the Drone PRO GeomagnetModel (sensor model v2).
 *
 * setup.config_path is a drone_config JSON file; see sensors/README.md for the
 * parts it reads. The mount must be zero (the host applies the mounting pose).
 */
std::unique_ptr<hakoniwa::sensor_adapter::IMagSensorAdapter> create_hakoniwa_mag_sensor_adapter(
    const hakoniwa::sensor_adapter::MagAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::drone_pro::sensors
