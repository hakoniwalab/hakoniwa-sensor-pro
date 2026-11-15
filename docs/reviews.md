# Code review records

Requirements checked by reading the implementation's code rather than by a test. The implementation's source is not published; each record says what was checked, at which revision, and the result. The review items are those of the test design in [hakoniwa-drone-core](https://github.com/toppers/hakoniwa-drone-core/blob/221afa40809c9edb476f88c71383a43c134d4b9d/docs/architecture/aircraft/sensor/test-design.md) (R-02, R-03).

## R-02 Units and axes — SEN-COM-010

- Revision: IMU and GNSS models `5efb0f1a`
- Date: 2026-11-15
- Checked: the axes and units of the models and of the adapters that put them behind the public interface.

| Item | Found | Result |
|---|---|---|
| Model axes | The models compute in world NED and body/sensor FRD | as specified |
| Interface axes | The public interface is world NWU and sensor FLU. The adapters convert at the boundary only, in one header (y and z change sign both ways), for every value in and out | as specified |
| Units | Lengths m, angular velocity rad/s, acceleration m/s², pressure Pa, magnetic field as specified per sensor; configuration angles in degrees are converted when the configuration is read | as specified |
| Conversion in one place | The degree-to-radian conversion is written in more than one place (a helper, a local constant, and a few inline expressions) with the same factor | finding: correct values, not centralised |

Result: **meets SEN-COM-010.** Finding for maintenance: centralise the degree-to-radian conversion (no effect on the output).

## R-03 Things the sensors must not do — SEN-GYR-120, SEN-ACC-220, SEN-MAG-220

"Do not report saturation upward."

- Revision: IMU and GNSS models `5efb0f1a`; interface `433de4fe`
- Date: 2026-11-15
- Checked: whether the gyroscope, accelerometer and magnetometer models report that a value was clipped at the full scale.

| Item | Found | Result |
|---|---|---|
| Models | The models clip each axis to the full scale and return only the value; they keep no flag, count or log entry of the clipping | as specified |
| Adapters | The adapters set the interface's `kSensorFlagSaturated` on a sample whose axis sits at the full-scale limit, as the interface design requires of every implementation ([imu-adapter.md](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/docs/design/imu-adapter.md), 3.8) | as the interface requires |

Result: **the models meet the requirements.** Finding for the specification: "upward" in these requirements means the autopilot (the host decides what it passes on); the adapter's flag is the interface's own feature and is not passed to an autopilot by Hakoniwa's hosts. The requirement rows say `Adapter`; they should say `Host` or state that the interface's flag is not covered.
