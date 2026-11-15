// Joint sensor adapter without MuJoCo (U2, U3): every stage against independently computed
// expectations, seeds, reset and the configuration.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "hakoniwa/mujoco_sensors/joint_sensor_adapter.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace ms = hakoniwa::mujoco_sensors;

namespace {

constexpr std::uint64_t kStep = 1'000'000;  // 1 ms
constexpr double kPi = 3.14159265358979323846;

std::string write_config(const std::string& name, const std::string& body)
{
    const std::string path = ::testing::TempDir() + "/joint_" + name + ".json";
    std::ofstream(path) << body;
    return path;
}

// update_rate 1000 Hz (one sample per step) and the given extra keys.
std::string config(const std::string& name, const std::string& extra, double rate = 1000)
{
    return write_config(name, "{\"type\":\"joint\",\"update_rate\":" + std::to_string(rate) +
                                  (extra.empty() ? "" : "," + extra) + "}");
}

std::unique_ptr<sa::IJointSensorAdapter> make(const std::string& path, std::uint64_t seed = 7, const char* name = "j1")
{
    sa::JointSensorAdapterSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStep;
    setup.vehicle_name = "arm";
    setup.sensor_name = name;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    auto adapter = ms::create_joint_sensor_adapter(setup, error);
    EXPECT_TRUE(adapter) << error;
    return adapter;
}

bool rejected(const std::string& path)
{
    sa::JointSensorAdapterSetup setup;
    setup.config_path = path;
    setup.physics_step_ns = kStep;
    std::string error;
    return ms::create_joint_sensor_adapter(setup, error) == nullptr;
}

using TruthFn = std::function<sa::JointSensorTruth(std::uint64_t)>;

std::vector<sa::JointSensorSample> run(sa::IJointSensorAdapter& a, std::uint64_t n, const TruthFn& truth)
{
    std::vector<sa::JointSensorSample> out;
    std::string error;
    for (std::uint64_t k = 0; k < n; ++k) {
        auto t = truth(k);
        t.sim_time_ns = k * kStep;
        sa::JointSensorSample s;
        EXPECT_TRUE(a.step(t, s, error)) << error;
        out.push_back(s);
    }
    return out;
}

sa::JointSensorTruth at(double q, double v = 0, double act = 0, double tr = 0)
{
    sa::JointSensorTruth t;
    t.position = q;
    t.velocity = v;
    t.actuator_torque = act;
    t.transmitted_torque = tr;
    return t;
}

double stddev(const std::vector<double>& v)
{
    double m = 0;
    for (double x : v) m += x;
    m /= static_cast<double>(v.size());
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

}  // namespace

// T-JNT-U2-010 [JNT-ADP-030] Ideal: truth on the period boundaries, no effort
TEST(JointSensor, IdealPassesTruth)
{
    auto a = make(config("ideal", "", 100));
    const auto s = run(*a, 25, [](std::uint64_t k) { return at(0.001 * k, 0.5); });
    for (std::uint64_t k = 0; k < 25; ++k) {
        EXPECT_EQ(s[k].updated, k % 10 == 0);
        EXPECT_DOUBLE_EQ(s[k].position, 0.001 * static_cast<double>(k - k % 10));
        EXPECT_DOUBLE_EQ(s[k].velocity, 0.5);
        EXPECT_FALSE(s[k].has_effort);
    }
}

// T-JNT-U2-020 [JNT-ADP-270] [JNT-ADP-290] Joint-side 12-bit counts, halves away from zero
TEST(JointSensor, JointSideCounts)
{
    auto a = make(config("counts", "\"encoder\":{\"bits\":12}"));
    const double lsb = 2 * kPi / 4096;
    const std::vector<std::pair<double, std::int64_t>> cases = {
        {0.0, 0}, {0.49 * lsb, 0}, {0.5 * lsb, 1}, {-0.5 * lsb, -1}, {1.0, 652}, {-1.0, -652}};
    std::string error;
    for (const auto& [q, count] : cases) {
        a->reset(error);
        const auto s = run(*a, 1, [q = q](std::uint64_t) { return at(q); });
        EXPECT_EQ(s[0].encoder_count, count) << q;
        EXPECT_DOUBLE_EQ(s[0].position, static_cast<double>(count) * lsb) << q;
    }
}

// T-JNT-U2-030 [JNT-ADP-240] Motor side: the joint resolution is gear times finer
TEST(JointSensor, MotorSideResolution)
{
    auto a = make(config("motor", "\"encoder\":{\"side\":\"motor\",\"gear_ratio\":100,\"bits\":12}"));
    const auto s = run(*a, 1, [](std::uint64_t) { return at(0.1); });
    const double lsb = 2 * kPi / (4096.0 * 100.0);
    EXPECT_EQ(s[0].encoder_count, std::llround(0.1 / lsb));
    EXPECT_NEAR(s[0].position, 0.1, lsb / 2);
}

// T-JNT-U2-040 [JNT-ADP-220] Backlash: the play operator (no reading change inside the band)
TEST(JointSensor, BacklashPlayOperator)
{
    auto a = make(config("backlash", "\"encoder\":{\"side\":\"motor\",\"bits\":36,\"backlash\":0.01}"));
    // q: 0 -> 0.02 (up), then down to 0.015, then down to 0.005.
    const std::vector<double> q = {0.0, 0.02, 0.015, 0.005};
    const auto s = run(*a, q.size(), [&](std::uint64_t k) { return at(q[k]); });
    const double tol = 1e-8;
    EXPECT_NEAR(s[0].position, 0.0, tol);    // first value: p = q
    EXPECT_NEAR(s[1].position, 0.015, tol);  // pushed up: p = q - b/2
    EXPECT_NEAR(s[2].position, 0.015, tol);  // reversed inside the band: unchanged
    EXPECT_NEAR(s[3].position, 0.010, tol);  // pushed down: p = q + b/2
}

// T-JNT-U2-050 [JNT-ADP-210] Stiffness: theta = q + tau_tr / K (motor side)
TEST(JointSensor, StiffnessDeflection)
{
    auto a = make(config("stiff", "\"encoder\":{\"side\":\"motor\",\"bits\":36,\"stiffness\":1000}"));
    const auto s = run(*a, 1, [](std::uint64_t) { return at(0.3, 0, 2.0, 2.0); });
    EXPECT_NEAR(s[0].position, 0.3 + 2.0 / 1000.0, 1e-8);
}

// T-JNT-U2-060 [JNT-ADP-230] [JNT-ADP-250] Offset within the bound; incremental starts at zero
TEST(JointSensor, OffsetAndIncremental)
{
    auto a = make(config("offset", "\"encoder\":{\"bits\":36,\"offset\":\"1deg\"}"));
    const auto s = run(*a, 3, [](std::uint64_t) { return at(0.5); });
    EXPECT_LE(std::fabs(s[0].position - 0.5), kPi / 180 + 1e-8);
    EXPECT_GT(std::fabs(s[0].position - 0.5), 1e-9);
    EXPECT_DOUBLE_EQ(s[1].position, s[0].position);
    auto b = make(config("incr", "\"encoder\":{\"bits\":36,\"kind\":\"incremental\",\"offset\":\"1deg\"}"));
    const auto r = run(*b, 2, [](std::uint64_t k) { return at(0.5 + 0.1 * k); });
    EXPECT_NEAR(r[0].position, 0.0, 1e-8);
    EXPECT_NEAR(r[1].position, 0.1, 1e-8);
}

// T-JNT-U2-070 [JNT-ADP-240] Cyclic error bounded by the accuracy, periodic in the shaft angle
TEST(JointSensor, CyclicErrorBound)
{
    auto a = make(config("cyclic", "\"encoder\":{\"bits\":36,\"accuracy\":\"0.1deg\"}"));
    const double bound = 0.1 * kPi / 180;
    std::string error;
    double max_err = 0;
    double e1 = 0, e2 = 0;
    for (int i = 0; i < 200; ++i) {
        const double q = -kPi + 2 * kPi * i / 200.0;
        a->reset(error);
        const auto s = run(*a, 1, [q](std::uint64_t) { return at(q); });
        max_err = std::max(max_err, std::fabs(s[0].position - q));
        if (i == 50) e1 = s[0].position - q;
    }
    a->reset(error);
    const double q2 = -kPi + 2 * kPi * 50 / 200.0 + 2 * kPi;  // one turn later
    e2 = run(*a, 1, [q2](std::uint64_t) { return at(q2); })[0].position - q2;
    EXPECT_LE(max_err, bound + 1e-8);
    EXPECT_GT(max_err, 0.0);
    EXPECT_NEAR(e1, e2, 1e-8);
}

// T-JNT-U2-080 [JNT-ADP-280] Single-turn wrap to 0..C-1
TEST(JointSensor, SingleTurnWrap)
{
    auto a = make(config("wrap", "\"encoder\":{\"counts_per_rev\":4096,\"wrap\":\"single_turn\"}"));
    const auto s = run(*a, 2, [](std::uint64_t k) { return at(k == 0 ? -0.5 * 2 * kPi / 4096 * 3 : 2 * kPi + 0.001); });
    EXPECT_EQ(s[0].encoder_count, 4096 - 2);  // -1.5 counts rounds to -2, wraps to 4094
    EXPECT_EQ(s[1].encoder_count, std::llround(0.001 * 4096 / (2 * kPi)));
}

// T-JNT-U2-090 [JNT-ADP-310] Velocity from counts over a window
TEST(JointSensor, VelocityFromDifference)
{
    auto a = make(config("vdiff", "\"encoder\":{\"bits\":12},\"velocity\":{\"method\":\"difference\",\"window\":2}", 100));
    // 0.05 rad/s: 10 ms per sample.
    const auto s = run(*a, 41, [](std::uint64_t k) { return at(0.05 * static_cast<double>(k) * 1e-3, 0.05); });
    const double lsb = 2 * kPi / 4096;
    std::vector<std::int64_t> counts;
    for (const auto& x : s)
        if (x.updated) counts.push_back(x.encoder_count);
    std::vector<double> v;
    for (const auto& x : s)
        if (x.updated) v.push_back(x.velocity);
    EXPECT_DOUBLE_EQ(v[0], 0.0);
    EXPECT_NEAR(v[1], static_cast<double>(counts[1] - counts[0]) * lsb / 0.01, 1e-12);
    for (std::size_t k = 2; k < v.size(); ++k)
        EXPECT_NEAR(v[k], static_cast<double>(counts[k] - counts[k - 2]) * lsb / 0.02, 1e-12) << k;
}

// T-JNT-U2-100 [JNT-ADP-320] Velocity lsb in rpm
TEST(JointSensor, VelocityLsbRpm)
{
    auto a = make(config("vlsb", "\"velocity\":{\"lsb\":\"0.229rpm\"}"));
    const double lsb = 0.229 * 2 * kPi / 60;
    const auto s = run(*a, 1, [](std::uint64_t) { return at(0, 1.0); });
    EXPECT_DOUBLE_EQ(s[0].velocity, lsb * std::round(1.0 / lsb));
}

// T-JNT-U2-110 [JNT-ADP-410..450] Torque: sensor and current with friction, errors, clamp, resolution
TEST(JointSensor, TorqueSourcesAndErrors)
{
    auto sensor = make(config("tsensor", "\"torque\":{\"source\":\"sensor\",\"range\":5}"));
    auto s = run(*sensor, 1, [](std::uint64_t) { return at(0, 0, 1.0, 1.7); });
    EXPECT_TRUE(s[0].has_effort);
    EXPECT_DOUBLE_EQ(s[0].effort, 1.7);
    auto current = make(config("tcurrent", "\"torque\":{\"source\":\"current\",\"range\":5,\"friction\":{\"coulomb\":0.1,\"viscous\":0.05}}"));
    s = run(*current, 1, [](std::uint64_t) { return at(0, 2.0, 1.0, 1.7); });
    EXPECT_DOUBLE_EQ(s[0].effort, 1.0 + 0.1 + 0.05 * 2.0);
    auto clamp = make(config("tclamp", "\"torque\":{\"source\":\"sensor\",\"range\":5,\"resolution\":0.25}"));
    s = run(*clamp, 2, [](std::uint64_t k) { return at(0, 0, 0, k == 0 ? 7.0 : 1.13); });
    EXPECT_DOUBLE_EQ(s[0].effort, 5.0);
    EXPECT_EQ(s[0].flags, static_cast<std::uint32_t>(sa::kSensorFlagSaturated));
    EXPECT_DOUBLE_EQ(s[1].effort, 1.25);
    auto unit = make(config("tunit", "\"torque\":{\"source\":\"sensor\",\"range\":10,\"scale_error\":\"1%\",\"bias\":\"2%\"}"));
    s = run(*unit, 2, [](std::uint64_t k) { return at(0, 0, 0, k == 0 ? 0.0 : 5.0); });
    const double b = s[0].effort;
    EXPECT_LE(std::fabs(b), 0.2);
    EXPECT_LE(std::fabs(s[1].effort - b - 5.0), 0.05 + 1e-12);
    auto noisy = make(config("tnoise", "\"torque\":{\"source\":\"sensor\",\"range\":10,\"noise\":0.02}"));
    s = run(*noisy, 20000, [](std::uint64_t) { return at(0); });
    std::vector<double> e;
    for (const auto& x : s) e.push_back(x.effort);
    EXPECT_NEAR(stddev(e), 0.02, 0.0006);
}

// T-JNT-U2-120 [JNT-ADP-260] Encoder noise in counts
TEST(JointSensor, EncoderNoiseCounts)
{
    auto a = make(config("enoise", "\"encoder\":{\"bits\":24,\"noise_counts\":5}"));
    const auto s = run(*a, 20000, [](std::uint64_t) { return at(0.3); });
    std::vector<double> c;
    for (const auto& x : s) c.push_back(static_cast<double>(x.encoder_count));
    EXPECT_NEAR(stddev(c), std::sqrt(25.0 + 1.0 / 12.0), 0.15);
}

// T-JNT-U2-130 [JNT-ADP-040] [JNT-ADP-050] Seeds, reset replays, independent streams
TEST(JointSensor, SeedsAndReset)
{
    const std::string path = config("seed", "\"encoder\":{\"bits\":24,\"noise_counts\":3,\"offset\":\"1deg\"},\"torque\":{\"source\":\"sensor\",\"range\":10,\"noise\":0.1}");
    auto truth = [](std::uint64_t) { return at(0.2, 0, 0, 1.0); };
    auto a = make(path, 5), b = make(path, 5), c = make(path, 6), other = make(path, 5, "j2");
    const auto sa_ = run(*a, 50, truth), sb = run(*b, 50, truth), sc = run(*c, 50, truth), so = run(*other, 50, truth);
    for (std::size_t k = 0; k < 50; ++k) {
        EXPECT_EQ(sa_[k].encoder_count, sb[k].encoder_count);
        EXPECT_DOUBLE_EQ(sa_[k].effort, sb[k].effort);
    }
    EXPECT_NE(sa_[0].encoder_count, sc[0].encoder_count);
    EXPECT_NE(sa_[0].encoder_count, so[0].encoder_count);  // another joint name: another stream
    std::string error;
    ASSERT_TRUE(a->reset(error));
    const auto again = run(*a, 50, truth);
    for (std::size_t k = 0; k < 50; ++k) EXPECT_EQ(again[k].encoder_count, sa_[k].encoder_count);
}

// T-JNT-U3-010 [JNT-ADP-510] [JNT-ADP-540] Configuration errors
TEST(JointSensor, ConfigErrors)
{
    EXPECT_TRUE(rejected(config("e1", "\"encodr\":{}")));
    EXPECT_TRUE(rejected(config("e2", "\"encoder\":{\"bits\":12,\"counts_per_rev\":4096}")));
    EXPECT_TRUE(rejected(config("e3", "\"encoder\":{\"bits\":12,\"backlash\":0.01}")));
    EXPECT_TRUE(rejected(config("e4", "\"velocity\":{\"method\":\"difference\"}")));
    EXPECT_TRUE(rejected(config("e5", "\"encoder\":{\"bits\":12,\"offset\":\"1rad?\"}")));
    EXPECT_TRUE(rejected(config("e6", "\"torque\":{\"source\":\"sensor\"}")));
    EXPECT_TRUE(rejected(config("e7", "\"torque\":{\"source\":\"guess\",\"range\":1}")));
    EXPECT_TRUE(rejected(config("e8", "\"encoder\":{\"side\":\"motor\",\"gear_ratio\":0.5,\"bits\":12}")));
}

// T-JNT-U3-020 [JNT-ADP-550] [JNT-ADP-610] Capabilities, log columns, example configs
TEST(JointSensor, CapabilitiesAndExamples)
{
    for (const char* file : {"feetech_sts3215.json", "dynamixel_xm430_w350.json"}) {
        auto a = make(std::string(HAKO_SENSORS_CONFIG_DIR) + "/joint/" + file);
        ASSERT_TRUE(a) << file;
        const auto caps = a->capabilities();
        EXPECT_TRUE(caps.encoder_counts && caps.velocity_from_difference && caps.velocity_lsb) << file;
        EXPECT_EQ(a->log_columns().size(), 5u);
    }
    auto d = make(std::string(HAKO_SENSORS_CONFIG_DIR) + "/joint/dynamixel_xm430_w350.json");
    EXPECT_TRUE(d->capabilities().torque_from_current);
}
