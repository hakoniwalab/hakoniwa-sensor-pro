// The "hakoniwa" implementation descriptor: the entry point a host finds by name,
// and the sensor types it provides.
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

namespace sa = hakoniwa::sensor_adapter;

HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(hakoniwa);

TEST(ImplementationDescriptor, EntryPointNamesTheImplementation)
{
    const sa::SensorAdapterImplementation* implementation = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(hakoniwa)();
    ASSERT_NE(implementation, nullptr);
    EXPECT_EQ(implementation->version, sa::kSensorAdapterImplementationVersion);
    ASSERT_NE(implementation->name, nullptr);
    EXPECT_STREQ(implementation->name, "hakoniwa");
}

TEST(ImplementationDescriptor, ProvidesEveryAdapterOfThisLibraryAndNothingElse)
{
    const auto* implementation = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(hakoniwa)();
    ASSERT_NE(implementation, nullptr);
#if defined(HAKO_SENSORS_WITH_DRONE_PRO) && HAKO_SENSORS_WITH_DRONE_PRO
    // With the Drone PRO models: IMU, magnetometer, barometer and GPS as well.
    const std::vector<std::string> expected{"imu", "mag", "baro", "gps", "rangefinder", "lidar",
                                            "depth_camera", "rgb_camera", "force_torque", "joint"};
#else
    const std::vector<std::string> expected{"rangefinder", "lidar", "depth_camera", "rgb_camera", "force_torque", "joint"};
#endif
    EXPECT_EQ(sa::provided_sensor_types(*implementation), expected);
}

TEST(ImplementationDescriptor, CreatesTheJointAdapterAndReportsFailures)
{
    const auto* implementation = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(hakoniwa)();
    ASSERT_NE(implementation, nullptr);
    sa::JointSensorAdapterSetup setup;
    setup.config_path = std::string(HAKO_SENSORS_CONFIG_DIR) + "/joint/feetech_sts3215.json";
    setup.physics_step_ns = 2000000;
    std::string error;
    EXPECT_NE(implementation->create_joint(setup, error), nullptr) << error;

    setup.config_path = std::string(HAKO_SENSORS_CONFIG_DIR) + "/joint/does-not-exist.json";
    error.clear();
    EXPECT_EQ(implementation->create_joint(setup, error), nullptr);
    EXPECT_FALSE(error.empty());
}
