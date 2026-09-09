#include "detumble/magnetometer.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace detumble {
namespace {

constexpr double schedule_tolerance_s = 1.0e-12;

void validate_time(const double elapsed_time_s) {
    if (!std::isfinite(elapsed_time_s) || elapsed_time_s < 0.0) {
        throw std::invalid_argument{
            "Magnetometer time must be finite and nonnegative"
        };
    }
}

}  // namespace

IdealMagnetometer::IdealMagnetometer(MagnetometerConfig config)
    : config_{std::move(config)}, error_model_{config_.error} {
    if (!std::isfinite(config_.sample_rate_hz)
        || config_.sample_rate_hz <= 0.0) {
        throw std::invalid_argument{
            "Magnetometer sample rate must be finite and positive"
        };
    }

    reset();
}

void IdealMagnetometer::reset() {
    error_model_.reset();
    next_sample_time_s_ = 0.0;
    last_update_time_s_ = 0.0;
    has_update_time_ = false;
    last_measurement_.reset();
}

bool IdealMagnetometer::sample_due(const double elapsed_time_s) const {
    validate_time(elapsed_time_s);
    return elapsed_time_s + schedule_tolerance_s >= next_sample_time_s_;
}

std::optional<MagnetometerMeasurement> IdealMagnetometer::sample_if_due(
    const double elapsed_time_s,
    const Eigen::Vector3d& magnetic_field_body_T,
    const bool torquers_enabled
) {
    validate_time(elapsed_time_s);
    if (!magnetic_field_body_T.allFinite()) {
        throw std::invalid_argument{
            "Magnetometer field must be finite"
        };
    }
    if (has_update_time_
        && elapsed_time_s + schedule_tolerance_s < last_update_time_s_) {
        throw std::invalid_argument{
            "Magnetometer updates must use nondecreasing time"
        };
    }
    last_update_time_s_ = elapsed_time_s;
    has_update_time_ = true;

    if (!sample_due(elapsed_time_s)) {
        return std::nullopt;
    }
    if (torquers_enabled) {
        throw std::logic_error{
            "Magnetorquers must be disabled before magnetometer sampling"
        };
    }

    last_measurement_ = MagnetometerMeasurement{
        .sample_time_s = elapsed_time_s,
        .magnetic_field_body_T = error_model_.apply(magnetic_field_body_T)
    };

    const double sample_period_s = 1.0 / config_.sample_rate_hz;
    do {
        next_sample_time_s_ += sample_period_s;
    } while (next_sample_time_s_
             <= elapsed_time_s + schedule_tolerance_s);

    return last_measurement_;
}

const MagnetometerConfig& IdealMagnetometer::config() const {
    return config_;
}

const std::optional<MagnetometerMeasurement>&
IdealMagnetometer::last_measurement() const {
    return last_measurement_;
}

const Eigen::Vector3d& IdealMagnetometer::bias_T() const {
    return error_model_.bias();
}

}  // namespace detumble
