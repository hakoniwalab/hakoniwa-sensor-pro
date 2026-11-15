#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/gps_sensor_adapter.hpp"

namespace hakoniwa::drone_pro::sensors {

/**
 * @brief Creates a gps adapter backed by the Drone PRO GpsModel (sensor model v2).
 *
 * setup.config_path is a drone_config JSON file; see sensors/README.md for the
 * parts it reads. The mount must be zero (the host applies the mounting pose).
 */
std::unique_ptr<hakoniwa::sensor_adapter::IGpsSensorAdapter> create_hakoniwa_gps_sensor_adapter(
    const hakoniwa::sensor_adapter::GpsAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::drone_pro::sensors
