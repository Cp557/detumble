#include "detumble/bdot.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace detumble {

BdotEstimator::BdotEstimator(BdotEstimatorConfig config)
    : config_{std::move(config)} {
    if (!std::isfinite(config_.filter_time_constant_s)
        || config_.filter_time_constant_s < 0.0) {
        throw std::invalid_argument{
            "B-dot filter time constant must be finite and nonnegative"
        };
    }
}

void BdotEstimator::reset() {
    previous_measurement_.reset();
    last_estimate_.reset();
}

std::optional<BdotEstimate> BdotEstimator::update(
    const MagnetometerMeasurement& measurement
) {
    if (!std::isfinite(measurement.sample_time_s)
        || measurement.sample_time_s < 0.0
        || !measurement.magnetic_field_body_T.allFinite()) {
        throw std::invalid_argument{
            "B-dot measurements must contain finite time and field values"
        };
    }

    if (!previous_measurement_.has_value()) {
        previous_measurement_ = measurement;
        return std::nullopt;
    }

    const double sample_interval_s =
        measurement.sample_time_s - previous_measurement_->sample_time_s;
    if (!std::isfinite(sample_interval_s) || sample_interval_s <= 0.0) {
        throw std::invalid_argument{
            "B-dot measurements must have strictly increasing sample times"
        };
    }

    const Eigen::Vector3d raw_bdot_body_T_s =
        (measurement.magnetic_field_body_T
         - previous_measurement_->magnetic_field_body_T)
        / sample_interval_s;
    const double previous_weight = config_.filter_time_constant_s == 0.0
        ? 0.0
        : std::exp(
              -sample_interval_s / config_.filter_time_constant_s
          );
    const Eigen::Vector3d filtered_bdot_body_T_s =
        last_estimate_.has_value()
        ? previous_weight * last_estimate_->filtered_bdot_body_T_s
            + (1.0 - previous_weight) * raw_bdot_body_T_s
        : raw_bdot_body_T_s;

    previous_measurement_ = measurement;
    last_estimate_ = BdotEstimate{
        .sample_time_s = measurement.sample_time_s,
        .raw_bdot_body_T_s = raw_bdot_body_T_s,
        .filtered_bdot_body_T_s = filtered_bdot_body_T_s
    };
    return last_estimate_;
}

const BdotEstimatorConfig& BdotEstimator::config() const {
    return config_;
}

const std::optional<BdotEstimate>& BdotEstimator::last_estimate() const {
    return last_estimate_;
}

Eigen::Vector3d bdot_dipole_command_body_A_m2(
    const BdotControllerConfig& controller_config,
    const MagnetorquerConfig& magnetorquer_config,
    const Eigen::Vector3d& filtered_bdot_body_T_s
) {
    if (!std::isfinite(controller_config.gain_A_m2_s_per_T)
        || controller_config.gain_A_m2_s_per_T < 0.0) {
        throw std::invalid_argument{
            "B-dot gain must be finite and nonnegative"
        };
    }
    if (!filtered_bdot_body_T_s.allFinite()) {
        throw std::invalid_argument{"Filtered B-dot must be finite"};
    }

    return saturate_magnetorquer_dipole_body_A_m2(
        magnetorquer_config,
        -controller_config.gain_A_m2_s_per_T
            * filtered_bdot_body_T_s
    );
}

}  // namespace detumble
