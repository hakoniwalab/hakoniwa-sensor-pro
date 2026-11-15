#pragma once

// Settings of the optional IMU low-pass filter ([SEN-COM-460] .. [SEN-COM-470]); the filter itself is internal
// to the implementation (src/aircraft/impl/sensors/imu_lowpass.hpp).

namespace hako::aircraft::impl {

struct LowpassConfig {
    enum class Type { Chebyshev2, Butterworth };
    Type type{Type::Chebyshev2};
    int order{0};
    double stopband_hz{0.0};              // chebyshev2
    double stopband_attenuation_db{0.0};  // chebyshev2
    double cutoff_hz{0.0};                // butterworth (-3 dB)
};

}  // namespace hako::aircraft::impl
