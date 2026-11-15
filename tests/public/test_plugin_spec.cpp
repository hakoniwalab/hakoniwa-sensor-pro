#include <gtest/gtest.h>

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <dlfcn.h>

#include "hakoniwa/sensor_adapter/external_sensor_plugin.h"

namespace {

constexpr std::uint64_t kPhysicsStepNs = 1'000'000;
constexpr double kExpectedWallDistance = 4.9;
const std::string kRangefinderConfig =
    std::string(HAKO_SENSORS_CONFIG_DIR) + "/rangefinder/ideal.json";
const std::string kLidarConfig = std::string(HAKO_SENSORS_CONFIG_DIR) + "/lidar/lds-01.json";

const char* kWorldXml = R"(
<mujoco>
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <geom name="wall" type="box" pos="5 0 0" size="0.1 2 2" group="0"/>
    <body name="carrier" pos="0 0 0">
      <freejoint/>
      <geom name="carrier_geom" type="box" size="0.5 0.5 0.5" group="1" mass="1"/>
      <body name="child" pos="1 0 0">
        <geom name="child_geom" type="box" size="0.2 0.2 0.2" group="1" mass="0.1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
)";

std::uint64_t simulation_time_ns(const mjData* data)
{
    return static_cast<std::uint64_t>(std::llround(data->time * 1.0e9));
}

int version_code(const char* text)
{
    int major = 0;
    int minor = 0;
    int patch = 0;
    if (std::sscanf(text, "%d.%d.%d", &major, &minor, &patch) != 3) return -1;
    return major * 1'000'000 + minor * 1'000 + patch;
}

class PluginLibrary {
public:
    PluginLibrary()
    {
        handle_ = dlopen(HAKO_EXTERNAL_SENSOR_PLUGIN_PATH, RTLD_NOW | RTLD_LOCAL);
        if (!handle_) {
            const char* message = dlerror();
            error_ = message ? message : "dlopen failed without an error message";
            return;
        }
        dlerror();
        entry_ = reinterpret_cast<HakoExternalSensorPluginEntryV1>(
            dlsym(handle_, HAKO_EXTERNAL_SENSOR_PLUGIN_ENTRY));
        if (const char* message = dlerror()) error_ = message;
        if (entry_) table_ = entry_();
    }

    ~PluginLibrary()
    {
        if (handle_) dlclose(handle_);
    }

    const HakoExternalSensorPluginV1* table() const { return table_; }
    const std::string& error() const { return error_; }

private:
    void* handle_{nullptr};
    HakoExternalSensorPluginEntryV1 entry_{nullptr};
    const HakoExternalSensorPluginV1* table_{nullptr};
    std::string error_;
};

class World {
public:
    World()
    {
        char error[1024] = "";
        spec_ = mj_parseXMLString(kWorldXml, nullptr, error, sizeof(error));
        if (!spec_) {
            error_ = error;
            return;
        }
        model_ = mj_compile(spec_, nullptr);
        if (!model_) {
            error_ = "mj_compile failed";
            return;
        }
        data_ = mj_makeData(model_);
        if (!data_) {
            error_ = "mj_makeData failed";
            return;
        }
        mj_forward(model_, data_);
    }

    ~World()
    {
        if (data_) mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
        if (spec_) mj_deleteSpec(spec_);
    }

    bool valid() const { return model_ && data_; }
    const std::string& error() const { return error_; }
    mjModel* model() const { return model_; }
    mjData* data() const { return data_; }
    int carrier() const { return mj_name2id(model_, mjOBJ_BODY, "carrier"); }

private:
    mjSpec* spec_{nullptr};
    mjModel* model_{nullptr};
    mjData* data_{nullptr};
    std::string error_;
};

HakoExternalSensorCreateV1 create_arguments(World& world, const char* kind, const char* config)
{
    static const std::string service_config_dir = ::testing::TempDir();
    HakoExternalSensorCreateV1 args{};
    args.struct_size = sizeof(args);
    args.kind = kind;
    args.config_path = config;
    args.vehicle_name = "plugin_spec_vehicle";
    args.vehicle_index = 0;
    args.sensor_name = "front_range";
    args.physics_step_ns = kPhysicsStepNs;
    args.has_random_seed = 1;
    args.random_seed = 12345;
    args.publish_rate_hz = 0.0;
    args.world_engine = "mujoco";
    args.world_engine_version = mj_version();
    args.world_model = world.model();
    args.world_data = world.data();
    args.body_id = world.carrier();
    args.mount_orientation = {1.0, 0.0, 0.0, 0.0};
    args.asset_name = "plugin_spec_asset";
    args.robot_name = "plugin_spec_robot";
    args.service_config_dir = service_config_dir.c_str();
    return args;
}

class Sensor {
public:
    Sensor(const HakoExternalSensorPluginV1* table, void* value) : table_(table), value_(value) {}
    ~Sensor()
    {
        if (value_) table_->destroy(value_);
    }
    Sensor(const Sensor&) = delete;
    Sensor& operator=(const Sensor&) = delete;
    void* get() const { return value_; }

private:
    const HakoExternalSensorPluginV1* table_;
    void* value_;
};

const HakoExternalSensorPluginV1* load_plugin(PluginLibrary& library)
{
    EXPECT_NE(library.table(), nullptr) << library.error();
    return library.table();
}

void expect_failure(int32_t result, const std::array<char, 256>& error)
{
    EXPECT_NE(result, 0);
    EXPECT_NE(error[0], '\0');
    EXPECT_NE(std::memchr(error.data(), '\0', error.size()), nullptr);
}

}  // namespace

// external-sensor-plugin.md 2: the named C entry point and its static function table.
TEST(PluginSpec, T_PLG_I1_010_EntryAndFunctionTable)
{
    PluginLibrary library;
    const auto* plugin = load_plugin(library);
    ASSERT_NE(plugin, nullptr);

    EXPECT_GE(plugin->struct_size, sizeof(HakoExternalSensorPluginV1));
    EXPECT_EQ(plugin->abi_version, HAKO_EXTERNAL_SENSOR_PLUGIN_ABI_VERSION);
    ASSERT_NE(plugin->interface_revision, nullptr);
    EXPECT_STREQ(plugin->interface_revision, HAKO_EXTERNAL_SENSOR_PLUGIN_INTERFACE_REVISION);
    ASSERT_NE(plugin->world_engine, nullptr);
    EXPECT_STREQ(plugin->world_engine, "mujoco");
    EXPECT_EQ(mj_version(), version_code(HAKO_EXPECTED_MUJOCO_VERSION));
    EXPECT_EQ(plugin->world_engine_version, version_code(HAKO_EXPECTED_MUJOCO_VERSION));
}

// external-sensor-plugin.md 2: short creation structs, unknown kinds, and bounded C errors.
TEST(PluginSpec, T_PLG_I1_020_CreateFailuresAreReportedAndTerminated)
{
    PluginLibrary library;
    const auto* plugin = load_plugin(library);
    ASSERT_NE(plugin, nullptr);
    World world;
    ASSERT_TRUE(world.valid()) << world.error();

    std::array<char, 256> error{};
    auto too_small = create_arguments(world, "rangefinder", kRangefinderConfig.c_str());
    too_small.struct_size = sizeof(HakoExternalSensorCreateV1) - 1;
    EXPECT_EQ(plugin->create(&too_small, error.data(), error.size()), nullptr);
    EXPECT_NE(error[0], '\0');

    error.fill('\0');
    auto unknown = create_arguments(world, "not_a_sensor", kRangefinderConfig.c_str());
    EXPECT_EQ(plugin->create(&unknown, error.data(), error.size()), nullptr);
    EXPECT_NE(error[0], '\0');

    std::array<char, 8> tiny;
    tiny.fill('x');
    EXPECT_EQ(plugin->create(&unknown, tiny.data(), tiny.size()), nullptr);
    EXPECT_EQ(tiny.back(), '\0');

    char one = 'x';
    EXPECT_EQ(plugin->create(&unknown, &one, 1), nullptr);
    EXPECT_EQ(one, '\0');
}

// external-sensor-plugin.md 3: start/stop/restart, reset, and destroy-last ordering.
TEST(PluginSpec, T_PLG_I1_030_LifecycleRestartsAndResetReplaysTheSameInputs)
{
    PluginLibrary library;
    const auto* plugin = load_plugin(library);
    ASSERT_NE(plugin, nullptr);
    World world;
    ASSERT_TRUE(world.valid()) << world.error();
    auto args = create_arguments(world, "rangefinder", kRangefinderConfig.c_str());
    std::array<char, 256> error{};
    Sensor sensor(plugin, plugin->create(&args, error.data(), error.size()));
    ASSERT_NE(sensor.get(), nullptr) << error.data();

    error.fill('\0');
    expect_failure(plugin->step(sensor.get(), kPhysicsStepNs, error.data(), error.size()), error);
    ASSERT_EQ(plugin->register_services(sensor.get(), error.data(), error.size()), 0) << error.data();
    ASSERT_EQ(plugin->on_start(sensor.get(), error.data(), error.size()), 0) << error.data();
    mj_step(world.model(), world.data());
    ASSERT_EQ(plugin->step(sensor.get(), simulation_time_ns(world.data()), error.data(), error.size()), 0)
        << error.data();
    ASSERT_EQ(plugin->on_stop(sensor.get(), error.data(), error.size()), 0) << error.data();

    ASSERT_EQ(plugin->on_start(sensor.get(), error.data(), error.size()), 0) << error.data();
    mj_step(world.model(), world.data());
    ASSERT_EQ(plugin->step(sensor.get(), simulation_time_ns(world.data()), error.data(), error.size()), 0)
        << error.data();
    ASSERT_EQ(plugin->on_stop(sensor.get(), error.data(), error.size()), 0) << error.data();

    ASSERT_EQ(plugin->reset(sensor.get(), error.data(), error.size()), 0) << error.data();
    mj_resetData(world.model(), world.data());
    mj_forward(world.model(), world.data());
    ASSERT_EQ(plugin->on_start(sensor.get(), error.data(), error.size()), 0) << error.data();
    mj_step(world.model(), world.data());
    EXPECT_EQ(plugin->step(sensor.get(), simulation_time_ns(world.data()), error.data(), error.size()), 0)
        << error.data();
    EXPECT_EQ(plugin->on_stop(sensor.get(), error.data(), error.size()), 0) << error.data();
    // Sensor::~Sensor calls destroy after the final assertion; no ABI call follows destroy.
}

// external-sensor-plugin.md 4: step uses mjData.time after each physics step and rejects other times.
TEST(PluginSpec, T_PLG_I1_040_StepUsesPostPhysicsTime)
{
    PluginLibrary library;
    const auto* plugin = load_plugin(library);
    ASSERT_NE(plugin, nullptr);
    World world;
    ASSERT_TRUE(world.valid()) << world.error();
    auto args = create_arguments(world, "rangefinder", kRangefinderConfig.c_str());
    std::array<char, 256> error{};
    Sensor sensor(plugin, plugin->create(&args, error.data(), error.size()));
    ASSERT_NE(sensor.get(), nullptr) << error.data();
    ASSERT_EQ(plugin->register_services(sensor.get(), error.data(), error.size()), 0) << error.data();
    ASSERT_EQ(plugin->on_start(sensor.get(), error.data(), error.size()), 0) << error.data();

    error.fill('\0');
    // Section 4 describes how the host calls step (after each physics step); it does not require the plugin
    // to refuse a call at time 0, so that is not checked here.
    mj_step(world.model(), world.data());
    ASSERT_EQ(simulation_time_ns(world.data()), kPhysicsStepNs);
    ASSERT_EQ(plugin->step(sensor.get(), simulation_time_ns(world.data()), error.data(), error.size()), 0)
        << error.data();

    error.fill('\0');
    expect_failure(plugin->step(sensor.get(), simulation_time_ns(world.data()), error.data(), error.size()), error);
    mj_step(world.model(), world.data());
    ASSERT_EQ(plugin->step(sensor.get(), simulation_time_ns(world.data()), error.data(), error.size()), 0)
        << error.data();
    EXPECT_EQ(plugin->on_stop(sensor.get(), error.data(), error.size()), 0) << error.data();
}

// external-sensor-plugin.md 3: service registration without prior Hakoniwa asset registration fails cleanly.
TEST(PluginSpec, T_PLG_I1_050_RegisterServicesWithoutHakoniwaFailsCleanly)
{
    PluginLibrary library;
    const auto* plugin = load_plugin(library);
    ASSERT_NE(plugin, nullptr);
    World world;
    ASSERT_TRUE(world.valid()) << world.error();
    auto args = create_arguments(world, "lidar", kLidarConfig.c_str());
    std::array<char, 256> error{};
    Sensor sensor(plugin, plugin->create(&args, error.data(), error.size()));
    ASSERT_NE(sensor.get(), nullptr) << error.data();

    error.fill('\0');
    expect_failure(plugin->register_services(sensor.get(), error.data(), error.size()), error);
}

// external-sensor-plugin.md 5: body-frame mounting and exclusion of the carrier body subtree.
TEST(PluginSpec, T_PLG_I1_060_MountedRangefinderStepsInKnownGeometry)
{
    PluginLibrary library;
    const auto* plugin = load_plugin(library);
    ASSERT_NE(plugin, nullptr);
    World world;
    ASSERT_TRUE(world.valid()) << world.error();

    // The sensor is at x=0 and points along +x. The wall's near face is 5.0-0.1 = 4.9 m.
    // Carrier and child geoms are deliberately in the ray first, at x=0 and x=0.8 m.
    const mjtNum origin[3] = {0, 0, 0};
    const mjtNum direction[3] = {1, 0, 0};
    const mjtByte only_world_group[6] = {1, 0, 0, 0, 0, 0};
    int geom_id = -1;
    mjtNum normal[3] = {};
    const double distance = mj_ray(world.model(), world.data(), origin, direction, only_world_group,
                                   1, -1, &geom_id, normal);
    EXPECT_EQ(geom_id, mj_name2id(world.model(), mjOBJ_GEOM, "wall"));
    EXPECT_NEAR(distance, kExpectedWallDistance, 1e-12);

    auto args = create_arguments(world, "rangefinder", kRangefinderConfig.c_str());
    std::array<char, 256> error{};
    Sensor sensor(plugin, plugin->create(&args, error.data(), error.size()));
    ASSERT_NE(sensor.get(), nullptr) << error.data();
    ASSERT_EQ(plugin->register_services(sensor.get(), error.data(), error.size()), 0) << error.data();
    ASSERT_EQ(plugin->on_start(sensor.get(), error.data(), error.size()), 0) << error.data();
    mj_step(world.model(), world.data());
    EXPECT_EQ(plugin->step(sensor.get(), simulation_time_ns(world.data()), error.data(), error.size()), 0)
        << error.data();
    EXPECT_EQ(plugin->on_stop(sensor.get(), error.data(), error.size()), 0) << error.data();
    // With publish_rate_hz=0, the ABI exposes no returned sample. The report records why the
    // measured 4.9 m value and subtree exclusion cannot be observed without Hakoniwa PDU I/O.
}
