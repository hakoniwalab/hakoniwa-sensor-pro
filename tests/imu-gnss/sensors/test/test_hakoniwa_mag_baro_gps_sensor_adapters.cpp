// The Hakoniwa magnetometer, barometer and GPS adapters against the Drone PRO
// sensor models they wrap: the same config, seed and inputs must give
// bit-identical values (only the FLU/FRD and NWU/NED signs change at the boundary).
#include "hakoniwa/imu_gnss_sensors/hakoniwa_baro_sensor_adapter.hpp"
#include "hakoniwa/imu_gnss_sensors/hakoniwa_gps_sensor_adapter.hpp"
#include "hakoniwa/imu_gnss_sensors/hakoniwa_mag_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <nlohmann/json.hpp>

#include "hakoniwa/imu_gnss_sensors/sensor_models.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace model = hako::aircraft::impl;
namespace hs = hakoniwa::drone_pro::sensors;

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

template <typename Setup>
Setup setup_for(const char* file)
{
    Setup setup{};
    setup.config_path = config(file);
    setup.vehicle_name = "Drone";
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

constexpr int kSteps = 5000;

void magnetometer()
{
    std::string error;
    auto mag = hs::create_hakoniwa_mag_sensor_adapter(setup_for<sa::MagAdapterSetup>("mag-baro-gps-full.json"), error);
    check(mag != nullptr, "mag adapter is created");
    if (!mag) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    model::GeomagnetModel ref(model::parse_geomagnet_model_config(load("mag-baro-gps-full.json")), 0.001, 1, 3.0,
                              9201u, "Drone", 0);
    bool identical = true, timing = true;
    for (int k = 0; k < kSteps; ++k) {
        sa::MagTruth t{};
        t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
        t.magnetic_field = {18000.0 + 500.0 * std::sin(0.003 * k), -4800.0 * std::cos(0.002 * k), -49500.0};
        sa::MagSample out{};
        check(mag->step(t, out, error), "mag step");
        const model::Vector3 r = ref.step({t.magnetic_field.x, -t.magnetic_field.y, -t.magnetic_field.z});
        identical &= out.value.x == r.x && out.value.y == -r.y && out.value.z == -r.z;
        timing &= out.updated == (k % 20 == 0);
    }
    check(identical, "mag is bit-identical to GeomagnetModel");
    check(timing, "mag measures every 20 ms from time 0");
}

void barometer()
{
    std::string error;
    auto baro =
        hs::create_hakoniwa_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("mag-baro-gps-full.json"), error);
    check(baro != nullptr, "baro adapter is created");
    if (!baro) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    model::BarometricModel ref(model::parse_barometric_model_config(load("mag-baro-gps-full.json")), 0.001, 1,
                               60.0, 9201u, "Drone", 0);
    bool identical = true, timing = true;
    for (int k = 0; k < kSteps; ++k) {
        sa::BaroTruth t{};
        t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
        t.altitude_m = 121.321 + 30.0 * std::sin(0.001 * k);
        t.sea_level_pressure_pa = 101325.0 - 200.0 * std::sin(0.0005 * k);
        t.sea_level_temperature_c = 15.0 + 2.0 * std::cos(0.0007 * k);
        sa::BaroSample out{};
        check(baro->step(t, out, error), "baro step");
        const model::BarometricValue r =
            ref.step(t.altitude_m, t.sea_level_pressure_pa, t.sea_level_temperature_c);
        identical &= out.pressure_pa == r.pressure_pa && out.temperature_c == r.temperature_c &&
                     out.pressure_altitude_m == r.pressure_altitude_m;
        timing &= out.updated == (k % 20 == 0);
    }
    check(identical, "baro is bit-identical to BarometricModel (pressure, temperature, pressure altitude)");
    check(timing, "baro measures every 20 ms from time 0");
}

void gps()
{
    std::string error;
    auto setup = setup_for<sa::GpsAdapterSetup>("mag-baro-gps-full.json");
    setup.origin = {47.641468, -122.140165, 121.321};
    auto gps = hs::create_hakoniwa_gps_sensor_adapter(setup, error);
    check(gps != nullptr, "gps adapter is created");
    if (!gps) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    model::GpsModel ref(model::parse_gps_model_config(load("mag-baro-gps-full.json")), 0.001, 1, 100.0, 9201u,
                        "Drone", 0, 47.641468, -122.140165, 121.321);
    model::Matrix3 identity{};
    identity.v = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    bool identical = true, timing = true;
    for (int k = 0; k < kSteps; ++k) {
        sa::GpsTruth t{};
        t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
        const double s = 0.001 * k;
        t.position_nwu = {5.0 * s, -3.0 * s, 10.0 + std::sin(s)};
        t.velocity_nwu = {5.0, -3.0, std::cos(s)};
        sa::GpsSample out{};
        check(gps->step(t, out, error), "gps step");
        const model::GpsModelValue r = ref.step({t.position_nwu.x, -t.position_nwu.y, -t.position_nwu.z},
                                                {t.velocity_nwu.x, -t.velocity_nwu.y, -t.velocity_nwu.z}, {},
                                                identity);
        identical &= out.position.latitude_deg == r.latitude_deg && out.position.longitude_deg == r.longitude_deg &&
                     out.position.altitude_m == r.altitude_m && out.velocity_nwu.x == r.velocity_ned_mps.x &&
                     out.velocity_nwu.y == -r.velocity_ned_mps.y && out.velocity_nwu.z == -r.velocity_ned_mps.z &&
                     out.ground_speed_mps == r.ground_speed_mps && out.course_valid == r.course_valid &&
                     (!r.course_valid || out.course_deg == r.course_deg) &&
                     out.horizontal_accuracy_m == r.horizontal_accuracy_m &&
                     out.satellites_visible == static_cast<std::uint32_t>(r.satellites_visible) &&
                     out.fix_type == 3;
        timing &= out.updated == (k % 100 == 0);
    }
    check(identical, "gps is bit-identical to GpsModel (position, velocity, course, receiver status)");
    check(timing, "gps measures every 100 ms from time 0");
}

void mounts_are_rejected()
{
    std::string error;
    check(!hs::create_hakoniwa_mag_sensor_adapter(setup_for<sa::MagAdapterSetup>("mag-baro-gps-mounted.json"), error),
          "a mag with an IMU mount orientation is rejected");
    check(!hs::create_hakoniwa_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("mag-baro-gps-mounted.json"),
                                                   error),
          "a baro with a mount position is rejected");
    check(!hs::create_hakoniwa_gps_sensor_adapter(setup_for<sa::GpsAdapterSetup>("mag-baro-gps-mounted.json"), error),
          "a gps with an antenna position is rejected");
}

// A period shorter than the physics step is refused: mag and baro 20 ms, GPS 100 ms; a
// period equal to the step is accepted.
void period_shorter_than_step_is_refused()
{
    std::string error;
    auto mag = setup_for<sa::MagAdapterSetup>("mag-baro-gps-full.json");
    mag.physics_step_ns = 25000000;
    check(!hs::create_hakoniwa_mag_sensor_adapter(mag, error) &&
              error.find("mag.updatePeriodMsec") != std::string::npos,
          "a 20 ms magnetometer with a 25 ms physics step is refused");
    error.clear();
    mag.physics_step_ns = 20000000;
    const bool equal = hs::create_hakoniwa_mag_sensor_adapter(mag, error) != nullptr;
    check(equal, "a 20 ms magnetometer with a 20 ms physics step is accepted");
    if (!equal) std::fprintf(stderr, "  %s\n", error.c_str());
    error.clear();
    auto baro = setup_for<sa::BaroAdapterSetup>("mag-baro-gps-full.json");
    baro.physics_step_ns = 25000000;
    check(!hs::create_hakoniwa_baro_sensor_adapter(baro, error) &&
              error.find("baro.updatePeriodMsec") != std::string::npos,
          "a 20 ms barometer with a 25 ms physics step is refused");
    error.clear();
    auto gps = setup_for<sa::GpsAdapterSetup>("mag-baro-gps-full.json");
    gps.physics_step_ns = 125000000;
    check(!hs::create_hakoniwa_gps_sensor_adapter(gps, error) &&
              error.find("gps.updatePeriodMsec") != std::string::npos,
          "a 100 ms GPS with a 125 ms physics step is refused");
}

// [PER-010] [PER-110] [PER-150] mag-baro-gps-full: the barometer at 20 ms with a 1 ms step. It is
// changed to 10 ms after step 3 (steps 0..2 done): the next measurements are at steps 10 and 20,
// and none before step 10 (with 20 ms the first one after step 0 would have been step 20).
void baro_period_is_changed()
{
    std::string error;
    auto baro = hs::create_hakoniwa_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("mag-baro-gps-full.json"), error);
    check(baro != nullptr, "baro period: created");
    if (!baro) return;
    std::uint64_t period = 0;
    check(baro->update_period_ns(period) && period == 20000000, "baro period: 20 ms");
    sa::BaroSample out{};
    sa::BaroTruth truth{};
    truth.altitude_m = 100.0;
    for (int k = 0; k < 3; ++k) {
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000U;
        baro->step(truth, out, error);
    }
    check(baro->set_update_period_ns(10000000, error), "baro period: to 10 ms");
    check(baro->update_period_ns(period) && period == 10000000, "baro period: reads 10 ms");
    std::vector<int> measured;
    for (int k = 3; k <= 20; ++k) {
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000U;
        check(baro->step(truth, out, error), "baro period: step");
        if (out.updated) measured.push_back(k);
    }
    check(measured == std::vector<int>({10, 20}), "baro period: measured at steps 10 and 20");
    check(baro->reset(error) && baro->update_period_ns(period) && period == 20000000, "baro period: reset returns 20 ms");
}

void reset_repeats()
{
    std::string error;
    auto baro =
        hs::create_hakoniwa_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("mag-baro-gps-full.json"), error);
    double first[300];
    for (int pass = 0; pass < 2; ++pass) {
        bool same = true;
        for (int k = 0; k < 300; ++k) {
            sa::BaroTruth t{};
            t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
            t.altitude_m = 100.0;
            sa::BaroSample out{};
            baro->step(t, out, error);
            if (pass == 0) first[k] = out.pressure_pa;
            else same &= out.pressure_pa == first[k];
        }
        if (pass == 1) check(same, "baro reset repeats the same values");
        baro->reset(error);
    }
}

}  // namespace

int main()
{
    magnetometer();
    barometer();
    gps();
    mounts_are_rejected();
    reset_repeats();
    period_shorter_than_step_is_refused();
    baro_period_is_changed();
    if (failures == 0) {
        std::printf("test_hakoniwa_mag_baro_gps_sensor_adapters: OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
