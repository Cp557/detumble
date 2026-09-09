#include "detumble/gyroscope.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace detumble {
namespace {

constexpr double schedule_tolerance_s = 1.0e-12;

void validate_time(const double elapsed_time_s) {
    if (!std::isfinite(elapsed_time_s) || elapsed_time_s < 0.0) {
        throw std::invalid_argument{
            "Gyroscope sample time must be finite and nonnegative"
        };
    }
}

}  // namespace

GyroscopeMeasurement ideal_gyroscope_measurement(
    const double sample_time_s,
    const Eigen::Vector3d& true_angular_velocity_body_rad_s
) {
    validate_time(sample_time_s);
    if (!true_angular_velocity_body_rad_s.allFinite()) {
        throw std::invalid_argument{
            "Gyroscope angular velocity must be finite"
        };
    }

    return {
        .sample_time_s = sample_time_s,
        .angular_velocity_body_rad_s = true_angular_velocity_body_rad_s
    };
}

IdealGyroscope::IdealGyroscope(GyroscopeConfig config)
    : config_{std::move(config)}, error_model_{config_.error} {
    if (!std::isfinite(config_.sample_rate_hz)
        || config_.sample_rate_hz <= 0.0) {
        throw std::invalid_argument{
            "Gyroscope sample rate must be finite and positive"
        };
    }
    reset();
}

void IdealGyroscope::reset() {
    error_model_.reset();
    next_sample_time_s_ = 0.0;
    last_update_time_s_ = 0.0;
    has_update_time_ = false;
    last_measurement_.reset();
}

bool IdealGyroscope::sample_due(const double elapsed_time_s) const {
    validate_time(elapsed_time_s);
    return elapsed_time_s + schedule_tolerance_s >= next_sample_time_s_;
}

std::optional<GyroscopeMeasurement> IdealGyroscope::sample_if_due(
    const double elapsed_time_s,
    const Eigen::Vector3d& true_angular_velocity_body_rad_s
) {
    validate_time(elapsed_time_s);
    if (has_update_time_
        && elapsed_time_s + schedule_tolerance_s < last_update_time_s_) {
        throw std::invalid_argument{
            "Gyroscope updates must use nondecreasing time"
        };
    }
    last_update_time_s_ = elapsed_time_s;
    has_update_time_ = true;

    if (!sample_due(elapsed_time_s)) {
        return std::nullopt;
    }

    last_measurement_ = ideal_gyroscope_measurement(
        elapsed_time_s,
        error_model_.apply(true_angular_velocity_body_rad_s)
    );
    const double sample_period_s = 1.0 / config_.sample_rate_hz;
    do {
        next_sample_time_s_ += sample_period_s;
    } while (next_sample_time_s_
             <= elapsed_time_s + schedule_tolerance_s);
    return last_measurement_;
}

const GyroscopeConfig& IdealGyroscope::config() const {
    return config_;
}

const std::optional<GyroscopeMeasurement>&
IdealGyroscope::last_measurement() const {
    return last_measurement_;
}

const Eigen::Vector3d& IdealGyroscope::bias_rad_s() const {
    return error_model_.bias();
}

}  // namespace detumble
