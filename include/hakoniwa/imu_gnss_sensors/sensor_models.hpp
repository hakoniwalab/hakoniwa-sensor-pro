#pragma once
#include "lowpass_config.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>
#include <memory>
#include <nlohmann/json_fwd.hpp>

namespace hako::aircraft::impl
{
struct Vector3
{
    double x{}, y{}, z{};
};
struct Matrix3
{
    std::array<std::array<double, 3>, 3> v{};
};
struct OrientationDeg
{
    double roll{}, pitch{}, yaw{};
};
// [SEN-COM-040] [SEN-COM-050] [SEN-COM-080] Build the configured mount rotation, with identity defaults.
Matrix3 mounting_rotation(const OrientationDeg &);
// [SEN-COM-090] [SEN-COM-100] Transform vectors through the shared mount rotation.
Vector3 body_to_sensor(const Matrix3 &, const Vector3 &);
Vector3 sensor_to_body(const Matrix3 &, const Vector3 &);
struct RigidBodyPointState
{
    Vector3 position_world, velocity_world, acceleration_body;
};
// [SEN-COM-030] [SEN-COM-110] Derive a mounted point state from rigid-body origin motion.
RigidBodyPointState rigid_body_point_state(const Vector3 &, const Vector3 &, const Vector3 &, const Matrix3 &,
                                           const Vector3 &, const Vector3 &, const Vector3 &);

// [SEN-COM-240] [SEN-COM-250] Implement stationary first-order Gauss-Markov drift.
class GaussMarkov
{
  public:
    GaussMarkov(double sigma, double correlation_time_sec, double step_sec, std::uint32_t seed);
    double value() const
    {
        return value_;
    }
    double step();
    void reset();

  private:
    double sigma_, phi_, drive_sigma_;
    std::uint32_t seed_;
    std::mt19937 engine_;
    double value_{};
};
// [SEN-COM-190] [SEN-COM-212] [SEN-COM-213] Apply resolution and range limits.
double round_to_resolution(double value, double resolution);
double clamp_to_range(double value, double minimum, double maximum);
// [SEN-COM-220] Derive independent deterministic random streams for each sensor characteristic.
std::uint32_t derive_sensor_seed(std::uint32_t, const std::string &, int, const std::string &,
                                 const std::string &characteristic_name);
std::uint32_t derive_sensor_seed(std::uint32_t, const std::string &, int, const std::string &);
// [SEN-COM-290] Convert mount position from the body origin to the center of mass.
Vector3 autopilot_position(const Vector3 &, const Vector3 &);
struct Px4BoardRotation
{
    int rotation{};
    OrientationDeg offset{};
};
// [SEN-COM-310] Generate PX4 board rotation parameters and reject unrepresentable mounts.
std::optional<Px4BoardRotation> make_px4_board_rotation(const OrientationDeg &);
Matrix3 px4_rotation_matrix(int rotation);
struct ArduPilotBoardRotation
{
    int orientation{};
    bool custom_enabled{};
    OrientationDeg custom{};
};
// [SEN-COM-320] Select a standard ArduPilot orientation or fall back to Custom 1.
ArduPilotBoardRotation make_ardupilot_board_rotation(const OrientationDeg &);
Matrix3 ardupilot_rotation_matrix(int orientation);
// [SEN-COM-370] Convert stationary drift sigma to SDFormat dynamic bias deviation.
double sdformat_dynamic_bias_stddev(double stationary_sigma, double correlation_time_sec);

// [SEN-GYR-040] [SEN-GYR-050] [SEN-GYR-100] Configure the gyro measurement pipeline.
// [SEN-GYR-130] [SEN-GYR-160] Configure sensitivity, alignment, and zero-rate errors.
// [SEN-GYR-060] [SEN-GYR-070] [SEN-GYR-080] Configure supported gyro drift modes.
// [SEN-GYR-140] [SEN-GYR-150] [SEN-GYR-170] Default and validate sensitivity and zero-rate offset.
// [SEN-GYR-190] [SEN-GYR-200] [SEN-GYR-210] Configure resolution rounding before clipping.
struct GyroModelConfig
{
    std::optional<double> update_period_sec;
    // [SEN-COM-460] components.sensors.gyro.filter: the optional IMU low-pass (none by default).
    std::optional<LowpassConfig> filter;
    Vector3 mount_position_m{};
    OrientationDeg mount_orientation_deg{};
    Vector3 noise_stddev{};
    Matrix3 sensitivity{};
    enum class ZeroRateMode
    {
        None,
        Fixed,
        Random
    } zero_rate_mode{ZeroRateMode::None};
    Vector3 fixed_zero_rate_deg_s{};
    double random_zero_rate_stddev_deg_s{};
    enum class DriftMode
    {
        None,
        ArdupilotSitl,
        RandomWalk,
        GaussMarkov
    } drift_mode{DriftMode::None};
    double drift_speed_deg_s_per_min{}, drift_time_min{};
    double random_walk_deg_s_sqrt_sec{}, gauss_markov_stddev_deg_s{}, correlation_time_sec{};
    std::optional<std::uint32_t> drift_seed;
    double resolution_deg_s{}, full_scale_deg_s{};
    std::vector<std::string> notice_settings;
};

GyroModelConfig parse_gyro_model_config(const nlohmann::json &root);
std::vector<std::string> sensor_model_v2_notice_lines(const GyroModelConfig &config,
                                                      const std::string &aircraft_name, int aircraft_index);
void announce_sensor_model_v2(const GyroModelConfig &config, const std::string &aircraft_name, int aircraft_index);

// [SEN-GYR-010] [SEN-GYR-020] [SEN-GYR-030] Model sensor-axis angular velocity using mount orientation only.
// [SEN-GYR-090] [SEN-GYR-110] [SEN-GYR-120] Apply drift and clipping on individual sensor axes.
// [SEN-GYR-180] [SEN-GYR-220] Keep fixed offsets stable and expose values in sensor axes.
// [SEN-COM-130] [SEN-COM-140] [SEN-COM-150] Preserve the common pipeline order and held samples.
// [SEN-COM-160] [SEN-COM-175] [SEN-COM-180] Apply delay, sampling, and independent noise.
// [SEN-COM-230] [SEN-COM-260] Reset state while keeping fixed offset separate from drift.
class GyroModel
{
  public:
    GyroModel(const GyroModelConfig &, double step_sec, int sample_count, double delay_msec,
              std::optional<std::uint32_t> simulation_seed, const std::string &aircraft_name, int aircraft_index);
    ~GyroModel();
    GyroModel(GyroModel &&) noexcept;
    GyroModel &operator=(GyroModel &&) noexcept;
    GyroModel(const GyroModel &) = delete;
    Vector3 step(const Vector3 &body_rate_rad_s);
    void reset();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// [SEN-ACC-040] [SEN-ACC-050] [SEN-ACC-060] Configure acceleration noise, drift, and range.
// [SEN-ACC-110] [SEN-ACC-120] Configure scale, cross-axis, and zero-g errors.
// [SEN-ACC-070] [SEN-ACC-080] [SEN-ACC-085] Configure noise and Gauss-Markov drift.
// [SEN-ACC-090] [SEN-ACC-095] Validate the shared accelerometer and gyro update period.
// [SEN-ACC-100] [SEN-ACC-130] [SEN-ACC-140] Convert gravity-based fixed and random offsets.
// [SEN-ACC-150] [SEN-ACC-170] [SEN-ACC-190] Validate offset exclusivity and optional resolution.
// [SEN-ACC-200] [SEN-ACC-230] Configure optional full-scale clipping.
struct AccelerationModelConfig
{
    std::optional<double> update_period_sec;
    // [SEN-COM-460] components.sensors.acc.filter: the optional IMU low-pass (none by default).
    std::optional<LowpassConfig> filter;
    Vector3 mount_position_m{};
    OrientationDeg mount_orientation_deg{};
    Vector3 noise_stddev{};
    Matrix3 sensitivity{};
    enum class ZeroGMode { None, Fixed, Random } zero_g_mode{ZeroGMode::None};
    Vector3 fixed_zero_g_mg{};
    double random_zero_g_stddev_mg{};
    double drift_stddev_mg{}, correlation_time_sec{};
    double resolution_mps2{}, full_scale_g{};
    std::vector<std::string> notice_settings;
};
// [SEN-COM-060] [SEN-COM-070] Share one IMU mount and reject sensor-local IMU mounts.
// [SEN-COM-176] [SEN-COM-185] [SEN-COM-186] Validate sampling and convert noise density to sigma.
// [SEN-COM-187] [SEN-COM-188] Reject conflicting noise forms and density without a period.
// [SEN-COM-210] [SEN-COM-211] Validate update periods and sensor-specific noise forms.
// [SEN-COM-214] [SEN-COM-215] Validate offset exclusivity and all configured value types.
// [SEN-COM-216] [SEN-COM-218] Warn for unknown fields and validate drift configuration.
// [SEN-COM-219] Validate mount shape, numeric values, and sensor placement rules.
AccelerationModelConfig parse_acceleration_model_config(const nlohmann::json &root);
// [SEN-ACC-010] [SEN-ACC-020] [SEN-ACC-025] Model mounted-point specific force in sensor axes.
// [SEN-ACC-030] [SEN-ACC-070] Use rigid-body rotational acceleration before sensor errors.
// [SEN-ACC-160] [SEN-ACC-180] Keep offsets fixed and round midpoint values away from zero.
// [SEN-ACC-210] [SEN-ACC-220] [SEN-ACC-250] Clamp each axis without reporting saturation.
class AccelerationModel
{
  public:
    AccelerationModel(const AccelerationModelConfig &, double, int, double, std::optional<std::uint32_t>,
                      const std::string &, int);
    ~AccelerationModel();
    AccelerationModel(AccelerationModel &&) noexcept;
    AccelerationModel &operator=(AccelerationModel &&) noexcept;
    Vector3 step(const Vector3 &origin_specific_force_body, const Vector3 &angular_rate_body,
                 const Vector3 &angular_acceleration_body);
    void reset();
    // [SEN-ACC-303] The zero-g offset in use (m/s^2, sensor axes): fixedMg, or the value drawn at the reset for
    // randomStdMg. It is the autopilot's calibration offset (CAL_ACC0_*OFF, INS_ACCOFFS_*).
    Vector3 zero_g_offset_mps2() const;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// [SEN-MAG-040] [SEN-MAG-050] [SEN-MAG-060] Configure magnetic noise, drift, resolution, and range.
// [SEN-MAG-035] Require all magnetic-field components in the simulation location.
// [SEN-MAG-090] [SEN-MAG-100] [SEN-MAG-110] Configure the ordered error pipeline and drift validation.
// [SEN-MAG-120] [SEN-MAG-130] [SEN-MAG-140] Configure the symmetric soft-iron matrix.
// [SEN-MAG-150] [SEN-MAG-160] [SEN-MAG-165] Configure hard iron and reject singular error matrices.
// [SEN-MAG-170] [SEN-MAG-190] [SEN-MAG-200] Configure optional resolution and full-scale clipping.
struct GeomagnetModelConfig
{
    std::optional<double> update_period_sec;
    OrientationDeg mount_orientation_deg{};
    Vector3 noise_stddev{};
    Matrix3 soft_iron{};
    Vector3 hard_iron_nt{};
    double drift_stddev_nt{}, correlation_time_sec{};
    double resolution_nt{}, full_scale_gauss{};
    std::vector<std::string> notice_settings;
};
GeomagnetModelConfig parse_geomagnet_model_config(const nlohmann::json &root);
// [SEN-MAG-010] [SEN-MAG-020] [SEN-MAG-030] Rotate the magnetic field without using mount position.
// [SEN-MAG-070] [SEN-MAG-080] Apply hard-iron and soft-iron errors in sensor axes.
// [SEN-MAG-180] [SEN-MAG-210] [SEN-MAG-220] Round and clamp each axis without reporting saturation.
// [SEN-MAG-230] Leave magnetic clipping disabled when no range is configured.
class GeomagnetModel
{
  public:
    GeomagnetModel(const GeomagnetModelConfig &, double, int, double, std::optional<std::uint32_t>,
                   const std::string &, int);
    ~GeomagnetModel();
    GeomagnetModel(GeomagnetModel &&) noexcept;
    GeomagnetModel &operator=(GeomagnetModel &&) noexcept;
    Vector3 step(const Vector3 &magnetic_field_body_nt);
    void reset();
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct AtmosphereValue
{
    double pressure_pa{}, temperature_c{};
};
// True when the sea-level pressure (Pa) and temperature (degC) can be used: standard_atmosphere()
// throws std::invalid_argument for the others.
bool valid_atmosphere(double sea_level_pressure_pa, double sea_level_temperature_c);
AtmosphereValue standard_atmosphere(double height_m, double sea_level_pressure_pa = 101325.0,
                                    double sea_level_temperature_c = 15.0);
// [SEN-BAR-010] [SEN-BAR-020] Compute atmosphere values and pressure altitude.
// [SEN-BAR-060] [SEN-BAR-070] [SEN-BAR-080] Use the default atmosphere and its two layers.
// [SEN-BAR-085] [SEN-BAR-090] [SEN-BAR-100] Clamp altitude and validate atmosphere constants.
// [SEN-BAR-105] [SEN-BAR-110] Retain valid weather and invert pressure by atmosphere layer.
double pressure_altitude(double pressure_pa, double sea_level_pressure_pa = 101325.0,
                         double sea_level_temperature_c = 15.0);

struct BarometricModelConfig
{
    std::optional<double> update_period_sec;
    Vector3 mount_position_m{};
    double noise_stddev_m{};
    enum class ZeroOffsetMode { None, Fixed, Random } zero_offset_mode{ZeroOffsetMode::None};
    double fixed_offset_m{}, random_offset_stddev_m{};
    double drift_stddev_m{}, correlation_time_sec{};
    double resolution_pa{};
    std::optional<double> minimum_pressure_pa, maximum_pressure_pa;
    std::vector<std::string> notice_settings;
};
// [SEN-BAR-120] [SEN-BAR-130] Use mount position for height and ignore mount orientation.
// [SEN-BAR-140] [SEN-BAR-150] [SEN-BAR-160] Configure the ordered height-error pipeline.
// [SEN-BAR-170] [SEN-BAR-180] [SEN-BAR-184] Configure scalar resolution, range, and noise.
// [SEN-BAR-186] Accept only Gauss-Markov barometer drift.
// [SEN-BAR-220] [SEN-BAR-230] [SEN-BAR-240] Configure exclusive fixed or random height offsets.
BarometricModelConfig parse_barometric_model_config(const nlohmann::json &root);
struct BarometricValue
{
    double pressure_pa{}, pressure_altitude_m{}, temperature_c{};
};
// [SEN-BAR-030] [SEN-BAR-040] [SEN-BAR-050] Model barometer position, delay, sampling, and noise.
// [SEN-BAR-060] [SEN-BAR-070] [SEN-BAR-080] Apply offset, drift, resolution, and valid pressure handling.
// [SEN-BAR-190] Derive altitude from pressure after applying height errors, resolution, and range.
// [SEN-BAR-200] Derive temperature from the sensor's true height before pressure height errors.
// [SEN-BAR-210] Apply delay to height while using current sea-level weather.
// [SEN-BAR-250] Keep the selected zero offset fixed during flight.
class BarometricModel
{
  public:
    BarometricModel(const BarometricModelConfig &, double, int, double, std::optional<std::uint32_t>,
                    const std::string &, int);
    ~BarometricModel();
    BarometricModel(BarometricModel &&) noexcept;
    BarometricModel &operator=(BarometricModel &&) noexcept;
    BarometricValue step(double height_m, double sea_level_pressure_pa, double sea_level_temperature_c);
    void reset();
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct GpsModelConfig
{
    std::optional<double> update_period_sec;
    Vector3 mount_position_m{};
    Vector3 position_noise_stddev_m{};
    Vector3 velocity_noise_stddev_mps{};
    Vector3 drift_stddev_m{};
    double correlation_time_sec{};
    double horizontal_accuracy_m{0.5}, vertical_accuracy_m{0.75}, speed_accuracy_mps{0.5};
    int satellites_visible{10};
    std::vector<std::string> notice_settings;
};
// [SEN-GPS-180] [SEN-GPS-190] Configure independent horizontal, vertical, and velocity noise.
// [SEN-GPS-200] [SEN-GPS-210] Configure and validate horizontal and vertical drift.
// [SEN-GPS-220] [SEN-GPS-230] Configure declared position and speed accuracy defaults.
// [SEN-GPS-240] Validate the configured visible-satellite count.
GpsModelConfig parse_gps_model_config(const nlohmann::json &root);
struct GpsModelValue
{
    double latitude_deg{}, longitude_deg{}, altitude_m{};
    Vector3 velocity_ned_mps{};
    double ground_speed_mps{}, course_deg{-1.0};
    double horizontal_accuracy_m{}, vertical_accuracy_m{}, speed_accuracy_mps{};
    int satellites_visible{};
    bool course_valid{};
};
// [SEN-GPS-010] [SEN-GPS-020] [SEN-GPS-030] Model mounted GPS position and velocity in NED.
// [SEN-GPS-040] [SEN-GPS-050] [SEN-GPS-060] Apply delay, sampling, position noise, and velocity noise.
// [SEN-GPS-070] [SEN-GPS-080] Report course, speed, and configured accuracy estimates.
// [SEN-GPS-090] [SEN-GPS-095] Keep the three-dimensional fix valid when course is unavailable.
// [SEN-GPS-100] [SEN-GPS-130] [SEN-GPS-140] Use antenna position and velocity but not mount orientation.
// [SEN-GPS-110] [SEN-GPS-120] Convert local NED through WGS-84 curvature without a geoid model.
// [SEN-GPS-150] [SEN-GPS-155] Apply errors in NED before converting coordinates.
// [SEN-GPS-160] [SEN-GPS-170] Derive course from sampled noisy velocity and hold samples.
class GpsModel
{
  public:
    GpsModel(const GpsModelConfig &, double, int, double, std::optional<std::uint32_t>,
             const std::string &, int, double, double, double);
    ~GpsModel();
    GpsModel(GpsModel &&) noexcept;
    GpsModel &operator=(GpsModel &&) noexcept;
    GpsModelValue step(const Vector3 &, const Vector3 &, const Vector3 &, const Matrix3 &);
    // [SEN-GPS-270] The body origin's true position and velocity in the GPS coordinates, without errors.
    GpsModelValue reference_state(const Vector3 &, const Vector3 &) const;
    void reset();
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

std::vector<std::string> sensor_model_v2_notice_lines(const GyroModelConfig &, const AccelerationModelConfig &,
                                                      const GeomagnetModelConfig &, const BarometricModelConfig &,
                                                      const GpsModelConfig &, const std::string &, int);
std::vector<std::string> sensor_model_v2_notice_lines(const GyroModelConfig &, const AccelerationModelConfig &,
                                                      const GeomagnetModelConfig &, const BarometricModelConfig &,
                                                      const std::string &, int);
void announce_sensor_model_v2(const GyroModelConfig &, const AccelerationModelConfig &, const GeomagnetModelConfig &,
                              const BarometricModelConfig &, const GpsModelConfig &, const std::string &, int);
void announce_sensor_model_v2(const GyroModelConfig &, const AccelerationModelConfig &, const GeomagnetModelConfig &,
                              const BarometricModelConfig &, const std::string &, int);
} // namespace hako::aircraft::impl
