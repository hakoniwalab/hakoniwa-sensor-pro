// The Hakoniwa IMU adapter against the Drone PRO sensor models it wraps: the same
// config, seed and inputs must give bit-identical values (only the FLU/FRD sign
// changes at the boundary), with the interface's timing, flags and calibration.
#include "hakoniwa/imu_gnss_sensors/hakoniwa_imu_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>

#include "hakoniwa/imu_gnss_sensors/sensor_models.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace model = hako::aircraft::impl;
using hakoniwa::drone_pro::sensors::create_hakoniwa_imu_sensor_adapter;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

std::string config(const char* file)
{
    return std::string(HAKO_SENSORS_CONFIG_DIR) + "/" + file;
}

sa::ImuAdapterSetup setup_for(const char* file)
{
    sa::ImuAdapterSetup setup{};
    setup.config_path = config(file);
    setup.vehicle_name = "Drone";
    setup.vehicle_index = 0;
    setup.physics_step_ns = 1000000;
    setup.has_random_seed = true;
    setup.random_seed = 9201;
    return setup;
}

nlohmann::json load(const char* file)
{
    std::ifstream in(config(file));
    return nlohmann::json::parse(in);
}

// A rich motion: rates and forces that change every step (sensor FLU).
sa::ImuTruth truth_at(int k)
{
    sa::ImuTruth t{};
    t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
    const double s = 0.001 * k;
    t.angular_velocity = {0.5 * std::sin(7.0 * s), -0.3 * std::cos(3.0 * s), 0.2 + 0.1 * std::sin(40.0 * s)};
    t.specific_force = {0.8 * std::sin(2.0 * s), -0.4 * std::cos(5.0 * s), 9.80665 + 0.5 * std::sin(93.0 * s)};
    return t;
}

bool same_bits(double a, double b)
{
    return a == b || (std::isnan(a) && std::isnan(b));
}

void bit_identical_to_the_models()
{
    std::string error;
    auto imu = create_hakoniwa_imu_sensor_adapter(setup_for("imu-full.json"), error);
    check(imu != nullptr, "adapter is created from the full config");
    if (!imu) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    const nlohmann::json root = load("imu-full.json");
    model::GyroModel gyro(model::parse_gyro_model_config(root), 0.001, 1, 2.0, 9201u, "Drone", 0);
    model::AccelerationModel accel(model::parse_acceleration_model_config(root), 0.001, 1, 1.0, 9201u, "Drone", 0);

    bool identical = true;
    bool timing = true;
    for (int k = 0; k < 3000; ++k) {
        const sa::ImuTruth t = truth_at(k);
        sa::ImuSample out{};
        check(imu->step(t, out, error), "step");
        // The models in FRD.
        const model::Vector3 g = gyro.step({t.angular_velocity.x, -t.angular_velocity.y, -t.angular_velocity.z});
        const model::Vector3 a =
            accel.step({t.specific_force.x, -t.specific_force.y, -t.specific_force.z}, {}, {});
        identical &= same_bits(out.gyro.value.x, g.x) && same_bits(out.gyro.value.y, -g.y) &&
                     same_bits(out.gyro.value.z, -g.z) && same_bits(out.accel.value.x, a.x) &&
                     same_bits(out.accel.value.y, -a.y) && same_bits(out.accel.value.z, -a.z);
        // 4 ms period from time 0.
        timing &= out.gyro.updated == (k % 4 == 0) && out.accel.updated == (k % 4 == 0) &&
                  out.gyro.sample_time_ns == static_cast<std::uint64_t>(k - k % 4) * 1000000;
    }
    check(identical, "adapter output is bit-identical to the Drone PRO models (FLU <-> FRD)");
    check(timing, "measured on the 4 ms boundaries from time 0");

    sa::ImuCalibration cal{};
    check(imu->calibration(cal), "calibration is available");
    const model::Vector3 z = accel.zero_g_offset_mps2();
    check(cal.accel_zero_g_offset.x == z.x && cal.accel_zero_g_offset.y == -z.y && cal.accel_zero_g_offset.z == -z.z,
          "calibration zero-g offset is the model's, in FLU");
    check(std::fabs(cal.accel_scale.x - 1.002) < 1e-12 && std::fabs(cal.accel_scale.z - 0.999) < 1e-12,
          "calibration scale is the model's sensitivity diagonal");
    std::vector<double> log;
    imu->log_values(log);
    check(log.size() == imu->log_columns().size() && log[1] == cal.accel_zero_g_offset.y, "log values follow columns");

    // reset gives the same sequence again
    std::vector<sa::ImuSample> first;
    check(imu->reset(error), "reset");
    for (int k = 0; k < 200; ++k) {
        sa::ImuSample out{};
        imu->step(truth_at(k), out, error);
        first.push_back(out);
    }
    check(imu->reset(error), "reset again");
    bool repeat = true;
    for (int k = 0; k < 200; ++k) {
        sa::ImuSample out{};
        imu->step(truth_at(k), out, error);
        repeat &= same_bits(out.gyro.value.x, first[k].gyro.value.x) &&
                  same_bits(out.accel.value.z, first[k].accel.value.z);
    }
    check(repeat, "reset repeats the same values");
}

void saturation_is_flagged()
{
    std::string error;
    auto imu = create_hakoniwa_imu_sensor_adapter(setup_for("imu-small-range.json"), error);
    check(imu != nullptr, "adapter is created from the small-range config");
    if (!imu) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    sa::ImuTruth t{};
    t.angular_velocity = {0.0, 0.0, 6.0};      // 344 deg/s > 250
    t.specific_force = {0.0, 0.0, 9.80665};    // 1 g < 2 g
    sa::ImuSample out{};
    check(imu->step(t, out, error), "step");
    check((out.gyro.flags & sa::kSensorFlagSaturated) != 0, "gyro beyond full scale is flagged");
    check(std::fabs(out.gyro.value.z - 250.0 * 3.14159265358979323846 / 180.0) < 1e-12, "gyro clipped at 250 deg/s");
    check(out.accel.flags == 0, "accel within range is not flagged");
}

void mount_is_rejected()
{
    std::string error;
    auto imu = create_hakoniwa_imu_sensor_adapter(setup_for("imu-mounted.json"), error);
    check(imu == nullptr && error.find("mount") != std::string::npos, "a non-zero IMU mount is rejected");
}

void time_jump_is_an_error()
{
    std::string error;
    auto imu = create_hakoniwa_imu_sensor_adapter(setup_for("imu-full.json"), error);
    sa::ImuTruth t = truth_at(0);
    t.sim_time_ns = 3;
    sa::ImuSample out{};
    check(imu && !imu->step(t, out, error) && !error.empty(), "a time jump is an error");
}

// A period shorter than the physics step can not be measured: 4 ms gyro and accelerometer
// with a 5 ms step are refused; with a 1 ms step they are accepted (the config's filter
// needs that rate).
void period_shorter_than_step_is_refused()
{
    std::string error;
    auto setup = setup_for("imu-full.json");
    setup.physics_step_ns = 5000000;
    auto imu = create_hakoniwa_imu_sensor_adapter(setup, error);
    check(imu == nullptr && error.find("shorter than the physics step") != std::string::npos,
          "a 4 ms period with a 5 ms physics step is refused");
    error.clear();
    setup.physics_step_ns = 1000000;
    const bool accepted = create_hakoniwa_imu_sensor_adapter(setup, error) != nullptr;
    check(accepted, "a 4 ms period with a 1 ms physics step is accepted");
    if (!accepted) std::fprintf(stderr, "  %s\n", error.c_str());
}

// [PER-010] [PER-020] [PER-110] [PER-120] [PER-130] [PER-150] imu-full: gyro and accelerometer
// 4 ms (updatePeriodMsec 4) with a 1 ms step. The gyro is changed to 2 ms after step 3 (steps 0..2
// done): it measures on the multiples of 2 from time 0, so the next ones are steps 4, 6, 8; step 3
// is not one. The accelerometer stays on 4 ms: steps 4, 8.
void periods_are_read_and_changed()
{
    std::string error;
    auto imu = create_hakoniwa_imu_sensor_adapter(setup_for("imu-full.json"), error);
    check(imu != nullptr, "period: created");
    if (!imu) return;
    std::uint64_t period = 0;
    check(imu->update_period_ns(period) && period == 4000000, "period: 4 ms for both parts");
    sa::ImuSample out{};
    for (int k = 0; k < 3; ++k) imu->step(truth_at(k), out, error);
    check(imu->set_part_update_period_ns(sa::ImuPart::gyro, 2000000, error), "period: the gyro to 2 ms");
    check(imu->part_update_period_ns(sa::ImuPart::gyro, period) && period == 2000000, "period: gyro reads 2 ms");
    check(imu->part_update_period_ns(sa::ImuPart::accel, period) && period == 4000000, "period: accel still 4 ms");
    check(!imu->update_period_ns(period), "period: two periods are read per part");
    const bool gyro_expected[] = {false, true, false, true, false, true, false};   // steps 3..9
    const bool accel_expected[] = {false, true, false, false, false, true, false};  // steps 3..9
    bool gyro_ok = true, accel_ok = true;
    for (int k = 3; k < 10; ++k) {
        check(imu->step(truth_at(k), out, error), "period: step");
        gyro_ok = gyro_ok && out.gyro.updated == gyro_expected[k - 3];
        accel_ok = accel_ok && out.accel.updated == accel_expected[k - 3];
    }
    check(gyro_ok, "period: the gyro measures on the multiples of 2 ms from time 0");
    check(accel_ok, "period: the accelerometer keeps its 4 ms");
    error.clear();
    check(!imu->set_part_update_period_ns(sa::ImuPart::gyro, 1500000, error) && !error.empty(),
          "period: 1.5 ms (not whole steps) is refused");
    error.clear();
    check(!imu->set_update_period_ns(0, error) && !error.empty(), "period: zero is refused");
    error.clear();
    check(!imu->set_update_period_ns(500000, error) && !error.empty(), "period: shorter than the step is refused");
    check(imu->part_update_period_ns(sa::ImuPart::gyro, period) && period == 2000000, "period: unchanged after refusals");
    check(imu->reset(error) && imu->update_period_ns(period) && period == 4000000, "period: reset returns 4 ms");
}

}  // namespace

int main()
{
    bit_identical_to_the_models();
    saturation_is_flagged();
    mount_is_rejected();
    time_jump_is_an_error();
    period_shorter_than_step_is_refused();
    periods_are_read_and_changed();
    if (failures == 0) {
        std::printf("test_hakoniwa_imu_sensor_adapter: OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
