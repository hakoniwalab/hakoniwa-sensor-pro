# hakoniwa-sensor-pro

Hakoniwa Lab's sensor implementation for simulation, distributed as binaries, with the tests that check it against its specification.

## What this is

Robots and vehicles in a simulation need sensor readings that behave like real ones: noisy, biased, drifting, delayed, quantised, saturating. This implementation produces them. It is designed so that each error effect a sensor datasheet describes can be set from the datasheet values, one by one, and turned off one by one.

It is one implementation of the public sensor interface [hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter) (MIT). A host that uses the interface, such as [hakoniwa-robot-runtime](https://github.com/hakoniwalab/hakoniwa-robot-runtime), selects it by its name, `hakoniwa`, in place of the true-value reference implementation that comes with the interface. The host gives each sensor the true value from its physics engine every step; the implementation returns the measured value.

## Specification

**The external specification is in hakoniwa-sensor-adapter, not here.** It defines, with numbered requirements, what every implementation of the interface must do: what each sensor measures, its error model, its configuration and its outputs ([docs/spec](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/README.md)). This repository holds the implementation's binaries and the tests that check them against that specification.

## Supported sensors

| Sensor | Interface (hakoniwa-sensor-adapter) | Specification | Error effects |
|---|---|---|---|
| Joint: encoder, joint torque | `joint_sensor_adapter.hpp` | [joint.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/joint.md) (`JNT-ADP-*`) | resolution (counts), offset, cyclic error, backlash, stiffness, noise, delay, velocity from differences, torque from a sensor or from motor current |
| Force-torque (6 axes) | `force_torque_sensor_adapter.hpp` | [force-torque.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/force-torque.md) (`FTS-ADP-*`) | delay, low-pass filter, nonlinearity, scale error and crosstalk, bias, drift, noise, resolution, range and overload, tare |
| IMU: gyroscope, accelerometer | `imu_sensor_adapter.hpp` | [gyro.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/gyro.md), [accelerometer.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/accelerometer.md) (`SEN-GYR-*`, `SEN-ACC-*`) | scale error and cross-axis sensitivity, filter, delay, offset, noise, drift (three models), resolution, saturation |
| Magnetometer | `mag_sensor_adapter.hpp` | [magnetometer.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/magnetometer.md) (`SEN-MAG-*`) | hard and soft iron, noise, drift, resolution, range |
| Barometer | `baro_sensor_adapter.hpp` | [barometer.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/barometer.md) (`SEN-BAR-*`) | standard atmosphere, offset, noise, drift, delay, resolution, range |
| GNSS | `gps_sensor_adapter.hpp` | [gps.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/gps.md) (`SEN-GPS-*`) | position and velocity noise, slowly varying error, delay, reported accuracy |
| 2D and 3D LiDAR | `lidar_sensor_adapter.hpp` | [lidar.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/lidar.md) (`RIS-LID-*`) | rotating or instant scan, accuracy per range band, resolution, range, no return |
| Rangefinder (beam or cone) | `rangefinder_sensor_adapter.hpp` | [rangefinder.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/rangefinder.md) (`RIS-RNG-*`) | accuracy, resolution, range, no return |
| Depth camera (rays or rendering) | `camera_sensor_adapter.hpp` | [depth-camera.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/depth-camera.md) (`RIS-DEP-*`) | accuracy, resolution, range, no return |
| RGB camera (single and stereo) | `camera_sensor_adapter.hpp` | [rgb-camera.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/rgb-camera.md) (`RIS-RGB-*`) | per-channel noise |

The common rules (sampling period, random numbers and reset, configuration) are in the specification's [README](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/README.md) (`RIS-COM-*`) and [drone-common.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/spec/drone-common.md) (`SEN-COM-*`). Every error effect is optional: a sensor with none configured returns the true value on its sampling period.

## Getting the binaries

Each [release](https://github.com/hakoniwalab/hakoniwa-sensor-pro/releases) has one archive per platform, `hakoniwa-sensor-pro-<version>-<platform>.zip`, with its SHA-256 in `.zip.sha256`:

| In the archive | Contents |
|---|---|
| `lib/` | `libhakoniwa_mujoco_sensors_adapters.a` (joint, force-torque, LiDAR, rangefinder, depth and RGB cameras; the `hakoniwa` implementation entry), `libhakoniwa_imu_gnss_sensor_adapters.a` and `libhakoniwa_imu_gnss_sensor_models.a` (IMU, magnetometer, barometer, GNSS), the MuJoCo parts (`libhakoniwa_mujoco_sensors_mujoco.a`, the ray caster; `libhakoniwa_mujoco_sensors_renderer.a`, the image renderer) and the external sensor plugin (`hakoniwa_mujoco_sensors_plugin.so`) |
| `include/` | the public headers of the implementation |
| `MANIFEST.json` | the source revisions, the toolchain and the SHA-256 of every file |
| `LICENSE`, `LICENSE.md`, `LICENSE-ja.md`, `THIRD_PARTY_NOTICES.md` | the license of the binaries and the third-party notices |

### Supported environments

| Platform | Archive | Built and tested on | Compiler |
|---|---|---|---|
| macOS, Apple silicon | `macos-arm64` | macOS 27.2 | Apple clang 21 |
| Ubuntu, x86-64 (also under WSL2 on Windows) | `linux-x64` | Ubuntu 24.04 | GCC 13 |

The libraries are C++17. A host also needs, for the MuJoCo parts, the MuJoCo version below, and for the image renderer (RGB and rendered depth cameras) OpenGL and GLFW 3.3 or later. Native Windows is not supported yet.

Built with the MuJoCo version in [MUJOCO_VERSION.txt](MUJOCO_VERSION.txt) (the implementation's own, synchronised with the tests). The MuJoCo parts (`libhakoniwa_mujoco_sensors_mujoco.a`, `libhakoniwa_mujoco_sensors_renderer.a`, the plugin) must be used with that version; the other libraries do not use MuJoCo. Each archive's `MANIFEST.json` names the MuJoCo library it was built with.

The plugin carries no library search path: a host that loads it supplies the Hakoniwa core libraries (`libassets`), MuJoCo and GLFW. A host links the libraries and registers the implementation through the interface's implementation entry (`hakoniwa-sensor-adapter`'s `sensor_adapter_implementation.hpp`). For hakoniwa-robot-runtime, see its sensor documentation.

## License

- **The binaries** (release archives): free for evaluation (including technical evaluation and proof of concept inside companies), personal use, and academic research and education. Building them into a product or service, commercial operation, work for third parties and redistribution need a commercial license from Hakoniwa Lab LLC (tmori@hakoniwa-lab.net). See [LICENSE.md](LICENSE.md) (English) and [LICENSE-ja.md](LICENSE-ja.md) (Japanese, which prevails).
- **The test sources, tools and documents** in this repository: MIT, see [LICENSE-tests](LICENSE-tests).
- **Third-party components** in the binaries: see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Tests

The tests check the binaries against the specification. There are two kinds, both built against the released binaries and public headers only:

- the implementation's own tests, copied unchanged ([config/synced-files.json](config/synced-files.json)) from the revisions the binaries are built from ([config/sources.json](config/sources.json)): `tests/sensors/`, `tests/imu-gnss/`;
- tests written in this repository from the public specification: `tests/public/` (requirements no copied test covered, and the external sensor plugin through its C boundary) and `tests/binary/` (checks of the binary set itself).

| Stage | What it checks |
|---|---|
| Sensor model | Each sensor's error model behind the interface, with the true values given by the test: hand-computed examples, statistics of the noise and drift, boundaries of rounding and ranges, seeds and reset, configuration errors |
| On a MuJoCo model | The sensors on a small MuJoCo world written in the test: rays against walls and floors, forces and joint torques of a hanging tool, rendered images against rays |
| External plugin | The shipped plugin loaded through its C boundary only: entry and version, errors, lifecycle (create, start, step, stop, restart, reset, destroy) and step times. The boundary has no call that returns a measurement, so the measured values are checked by the stages above, not here |

The IMU, magnetometer, barometer and GNSS tests use the model classes directly, through the public headers in `include/hakoniwa/imu_gnss_sensors/` (declarations only).

How to run them: [docs/running-tests.md](docs/running-tests.md). In short, with the release archive extracted beside this repository:

```bash
python3 tools/run-tests.py --dist ../hakoniwa-sensor-pro-0.1.0-macos-arm64 --mujoco-root ../mujoco-$(cat MUJOCO_VERSION.txt)
```

### Test status

The result of the last run on each platform, by sensor and stage (written by `tools/run-tests.py`):

<!-- test-status:begin -->

### linux-x64

- Run: 2026-11-15 11:05 UTC — 218 of 218 tests passed
- Binary set built 2026-11-15T11:04:49Z from implementation `6319a852`, IMU and GNSS models `2732fc53`, interface (hakoniwa-sensor-adapter) `433de4fe`
- Toolchain: c++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0
- Binary set: MANIFEST.json sha256 `d15321cc43df073136764e22d6b26b04b4ebdbab793d77c4a01343561c809cad` (every file checked against it before the run)

| Sensor | Sensor model | On a MuJoCo model | External plugin |
|---|---|---|---|
| Joint | 15 / 15 | 2 / 2 | — |
| Force-torque | 15 / 15 | 5 / 5 | — |
| IMU | 30 / 30 | — | — |
| Magnetometer | 6 / 6 | — | — |
| Barometer | 17 / 17 | — | — |
| GNSS | 16 / 16 | — | — |
| 2D/3D LiDAR | 12 / 12 | 2 / 2 | — |
| Rangefinder | 8 / 8 | 2 / 2 | — |
| Depth camera | 9 / 9 | 4 / 4 | — |
| RGB camera | 10 / 10 | 1 / 1 | — |
| Common | 50 / 50 | 17 / 17 | — |

Passed / run. Common: the implementation descriptor, the MuJoCo ray caster and image renderer, and mounting.

### macos-arm64

- Run: 2026-11-15 11:19 UTC — 225 of 225 tests passed
- Binary set built 2026-11-15T11:18:48Z from implementation `6319a852`, IMU and GNSS models `2732fc53`, interface (hakoniwa-sensor-adapter) `433de4fe`
- Toolchain: Apple clang version 21.0.0 (clang-2100.4.1.1)
- Binary set: MANIFEST.json sha256 `229c6d59699b1da04a24ade719c7dc1b240a10aad558741af801fba1dba796cd` (every file checked against it before the run)

| Sensor | Sensor model | On a MuJoCo model | External plugin |
|---|---|---|---|
| Joint | 18 / 18 | 2 / 2 | — |
| Force-torque | 20 / 20 | 5 / 5 | — |
| IMU | 34 / 34 | — | — |
| Magnetometer | 7 / 7 | — | — |
| Barometer | 17 / 17 | — | — |
| GNSS | 16 / 16 | — | — |
| 2D/3D LiDAR | 12 / 12 | 2 / 2 | — |
| Rangefinder | 8 / 8 | 2 / 2 | — |
| Depth camera | 9 / 9 | 4 / 4 | — |
| RGB camera | 10 / 10 | 1 / 1 | — |
| Common | 39 / 39 | 16 / 16 | 6 / 6 |

Passed / run. Common: the implementation descriptor, the MuJoCo ray caster and image renderer, and mounting.

<!-- test-status:end -->

### Requirement traceability

Every requirement of the specification, how it is checked, and the result on each platform. A test names the requirements it checks in its source (`[FTS-ADP-120]`); the IMU, magnetometer, barometer and GNSS tests carry a test ID in their name (`T_GYR_U2_020_Yaw90`) that the [test design](https://github.com/toppers/hakoniwa-drone-core/tree/main/docs/architecture/aircraft/sensor) links to the requirements. Requirements met by the host (for example the mounting pose, or the connection to an autopilot) are not the implementation's and are listed apart. The full table, requirement by requirement: [docs/traceability.md](docs/traceability.md).

<!-- traceability:begin -->

| Specification | Requirements | Tested | Passed | Not tested here: reason |
|---|---:|---:|---:|---|
| [Common (rangefinder, LiDAR, depth and RGB cameras)](docs/traceability.md#common-rangefinder-lidar-depth-and-rgb-cameras) | 22 | 9 | 9 | Hakoniwa Lab internal test 13 |
| [Rangefinder](docs/traceability.md#rangefinder) | 7 | 7 | 7 | — |
| [LiDAR](docs/traceability.md#lidar) | 14 | 14 | 14 | — |
| [Depth camera](docs/traceability.md#depth-camera) | 8 | 8 | 8 | — |
| [RGB camera](docs/traceability.md#rgb-camera) | 9 | 9 | 9 | — |
| [Force-torque sensor](docs/traceability.md#force-torque-sensor) | 33 | 33 | 33 | — |
| [Joint sensor](docs/traceability.md#joint-sensor) | 27 | 27 | 27 | — |
| [IMU, magnetometer, barometer and GNSS: common](docs/traceability.md#imu-magnetometer-barometer-and-gnss:-common) | 74 | 29 | 29 | retired 1, host requirement 29, checked by each sensor's tests 8, Hakoniwa Lab internal test 6, code review 1 |
| [Gyroscope](docs/traceability.md#gyroscope) | 25 | 20 | 20 | host requirement 4, code review 1 |
| [Accelerometer](docs/traceability.md#accelerometer) | 41 | 25 | 25 | host requirement 15, code review 1 |
| [Magnetometer](docs/traceability.md#magnetometer) | 44 | 23 | 23 | host requirement 20, code review 1 |
| [Barometer](docs/traceability.md#barometer) | 41 | 28 | 28 | host requirement 13 |
| [GPS](docs/traceability.md#gps) | 42 | 29 | 29 | host requirement 13 |

Passed: tested requirements whose tests all passed on every platform. The reason for each requirement not tested here is in [docs/traceability.md](docs/traceability.md).

<!-- traceability:end -->

## What these results do not show

- They are tests of the sensor models. How the sensors behave mounted on a robot in a running simulation is shown by the hosts; for a robot arm, see the [comparison with Gazebo](https://github.com/hakoniwalab/hakoniwa-robot-runtime/blob/main/ros2/docs/gazebo-comparison/README.md) (SO-101 with ROS 2, ros2_control and MoveIt; an overhead 3D LiDAR and a wrist force-torque sensor of this implementation, without noise).
- The effect of sensor errors on a controller's behaviour (for example on a robot arm's control) has not been evaluated.
- Characteristics a datasheet lists but the specification does not model (temperature, vibration inside the sensor, and the others each specification lists under "Not modelled") are not reproduced.

## For maintainers

| Tool | What it does |
|---|---|
| `tools/sync.py` | Copies the tests, fixtures and headers listed in [config/sync.json](config/sync.json) from the private sources at the revisions in [config/sources.json](config/sources.json). `--check` reports files that differ |
| `tools/build-binaries.py` | Builds the binary set into `dist/<platform>/` from checkouts at those revisions |
| `tools/package.py` | Makes the release archive `dist/hakoniwa-sensor-pro-<version>-<platform>.zip` and its `.sha256` |
| `tools/run-tests.py --update-repository` | Builds and runs the tests against a binary set and writes the results into `results/`, [docs/traceability.md](docs/traceability.md) and this README |

To move to new revisions: change [config/sources.json](config/sources.json), then run `sync.py`, `build-binaries.py`, `run-tests.py --update-repository` and `package.py`.
