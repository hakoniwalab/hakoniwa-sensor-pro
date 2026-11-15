#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace hako::aircraft::impl {

/*
 * Sensor transport delay (sensor delay support).
 *
 * A real sensor reports the vehicle as it was some time ago: a GNSS receiver's solution is
 * 100-200 ms old when it arrives, a barometer's filtered altitude lags too. The flight
 * controller's estimator knows this lag (ArduPilot GPS_DELAY_MS / EK3_HGT_DELAY, PX4
 * SENS_GPS0_DELAY / EKF2_BARO_DELAY) and fuses each measurement at its own time.
 *
 * The delay line holds the sensor's input (the vehicle state) for each physics step and returns
 * the one from delay_steps steps ago. The physics step is fixed, so the delay is a whole number of
 * steps (components.sensors.<name>.delayMsec rounded to the step); no interpolation. Until the
 * history is full it returns the oldest sample, as if the vehicle had been at rest before start.
 * With 0 steps it returns the input unchanged.
 */
template <typename T>
class SensorDelayLine {
public:
    static std::size_t steps_for(double delay_msec, double step_sec)
    {
        if (!std::isfinite(delay_msec) || delay_msec < 0.0) {
            throw std::invalid_argument("sensor delayMsec must be a finite value >= 0");
        }
        if (!(step_sec > 0.0)) {
            throw std::invalid_argument("sensor delay needs a positive simulation step");
        }
        return static_cast<std::size_t>(std::llround(delay_msec * 1.0e-3 / step_sec));
    }

    void set_delay_steps(std::size_t steps)
    {
        delay_steps_ = steps;
        history_.clear();
    }

    std::size_t delay_steps() const { return delay_steps_; }

    // Forget the history (simulation reset); the next input starts it again.
    void clear() { history_.clear(); }

    // Push this step's input and return the one to use now.
    const T& apply(const T& input)
    {
        if (delay_steps_ == 0) {
            last_ = input;
            return last_;
        }
        history_.push_back(input);
        if (history_.size() > delay_steps_ + 1) {
            history_.pop_front();
        }
        return history_.front();
    }

private:
    std::size_t delay_steps_{0};
    std::deque<T> history_{};
    T last_{};
};

}  // namespace hako::aircraft::impl
