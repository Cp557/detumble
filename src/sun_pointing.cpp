#include "detumble/sun_pointing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "detumble/attitude.hpp"

namespace detumble {
namespace {

constexpr double minimum_direction_norm = 1.0e-12;
constexpr double axis_orthogonality_tolerance = 1.0e-6;

bool finite_nonnegative(const double value) {
    return std::isfinite(value) && value >= 0.0;
}

void validate_controller_config(const SunPointingControllerConfig& config) {
    if (!config.sun_axis_body.allFinite()
        || !config.roll_axis_body.allFinite()
        || !config.roll_reference_inertial.allFinite()
        || config.sun_axis_body.norm() < minimum_direction_norm
        || config.roll_axis_body.norm() < minimum_direction_norm
        || config.roll_reference_inertial.norm() < minimum_direction_norm
        || std::abs(
               config.sun_axis_body.normalized().dot(
                   config.roll_axis_body.normalized()
               )
           ) > axis_orthogonality_tolerance) {
        throw std::invalid_argument{
            "Sun-pointing body axes must be finite, nonzero, and orthogonal"
        };
    }
    if (!config.proportional_gain_body_Nm_per_rad.allFinite()
        || !config.derivative_gain_body_Nm_s_per_rad.allFinite()
        || !config.maximum_torque_body_Nm.allFinite()
        || !config.acquisition_maximum_torque_body_Nm.allFinite()
        || (config.proportional_gain_body_Nm_per_rad.array() < 0.0).any()
        || (config.derivative_gain_body_Nm_s_per_rad.array() < 0.0).any()
        || (config.maximum_torque_body_Nm.array() <= 0.0).any()
        || (config.acquisition_maximum_torque_body_Nm.array() <= 0.0).any()
        || (config.acquisition_maximum_torque_body_Nm.array()
            > config.maximum_torque_body_Nm.array()).any()
        || !finite_nonnegative(config.attitude_deadband_rad)
        || !finite_nonnegative(config.rate_deadband_rad_s)
        || !finite_nonnegative(config.acquisition_threshold_rad)
        || !finite_nonnegative(config.steady_pointing_threshold_rad)
        || config.steady_pointing_threshold_rad
            > config.acquisition_threshold_rad
        || !finite_nonnegative(config.hysteresis_rad)
        || !finite_nonnegative(config.dwell_time_s)) {
        throw std::invalid_argument{
            "Sun-pointing gains, limits, thresholds, and dwell must be valid"
        };
    }
}

Eigen::Matrix3d basis_from_primary_and_secondary(
    const Eigen::Vector3d& primary,
    const Eigen::Vector3d& secondary
) {
    const Eigen::Vector3d first = primary.normalized();
    const Eigen::Vector3d second_rejected = secondary
        - secondary.dot(first) * first;
    if (second_rejected.norm() < minimum_direction_norm) {
        throw std::invalid_argument{
            "Sun direction and roll reference must not be parallel"
        };
    }
    const Eigen::Vector3d second = second_rejected.normalized();
    const Eigen::Vector3d third = first.cross(second);

    Eigen::Matrix3d basis;
    basis.col(0) = first;
    basis.col(1) = second;
    basis.col(2) = third;
    return basis;
}

Eigen::Vector3d saturated_torque(
    const Eigen::Vector3d& requested,
    const Eigen::Vector3d& maximum
) {
    return requested.cwiseMax(-maximum).cwiseMin(maximum);
}

}  // namespace

Eigen::Quaterniond desired_sun_pointing_attitude(
    const SunPointingControllerConfig& config,
    const Eigen::Vector3d& sun_direction_inertial
) {
    validate_controller_config(config);
    if (!sun_direction_inertial.allFinite()
        || sun_direction_inertial.norm() < minimum_direction_norm) {
        throw std::invalid_argument{
            "Inertial Sun direction must be finite and nonzero"
        };
    }

    const Eigen::Matrix3d body_basis = basis_from_primary_and_secondary(
        config.sun_axis_body,
        config.roll_axis_body
    );
    const Eigen::Matrix3d inertial_basis = basis_from_primary_and_secondary(
        sun_direction_inertial,
        config.roll_reference_inertial
    );
    return normalized_attitude(
        Eigen::Quaterniond{inertial_basis * body_basis.transpose()}
    );
}

Eigen::Vector3d shortest_attitude_error_body_rad(
    const Eigen::Quaterniond& estimated_body_to_inertial,
    const Eigen::Quaterniond& desired_body_to_inertial
) {
    const Eigen::Quaterniond estimated =
        normalized_attitude(estimated_body_to_inertial);
    const Eigen::Quaterniond desired =
        normalized_attitude(desired_body_to_inertial);
    Eigen::Quaterniond error = estimated.conjugate() * desired;
    if (error.w() < 0.0) {
        error.coeffs() *= -1.0;
    }

    const double vector_norm = error.vec().norm();
    if (vector_norm < minimum_direction_norm) {
        return Eigen::Vector3d::Zero();
    }
    const double angle_rad = 2.0 * std::atan2(vector_norm, error.w());
    return (angle_rad / vector_norm) * error.vec();
}

SunPointingCommand sun_pointing_command(
    const SunPointingControllerConfig& config,
    const Eigen::Quaterniond& estimated_body_to_inertial,
    const Eigen::Vector3d& measured_angular_velocity_body_rad_s,
    const Eigen::Quaterniond& desired_body_to_inertial
) {
    validate_controller_config(config);
    if (!measured_angular_velocity_body_rad_s.allFinite()) {
        throw std::invalid_argument{
            "Sun-pointing measured angular velocity must be finite"
        };
    }

    SunPointingCommand command;
    command.desired_body_to_inertial =
        normalized_attitude(desired_body_to_inertial);
    command.attitude_error_body_rad = shortest_attitude_error_body_rad(
        estimated_body_to_inertial,
        desired_body_to_inertial
    );
    command.pointing_error_rad = command.attitude_error_body_rad.norm();
    command.in_deadband =
        command.pointing_error_rad <= config.attitude_deadband_rad
        && measured_angular_velocity_body_rad_s.norm()
            <= config.rate_deadband_rad_s;
    command.active = true;
    if (command.in_deadband) {
        return command;
    }

    command.requested_torque_body_Nm =
        config.proportional_gain_body_Nm_per_rad.cwiseProduct(
            command.attitude_error_body_rad
        )
        - config.derivative_gain_body_Nm_s_per_rad.cwiseProduct(
            measured_angular_velocity_body_rad_s
        );
    command.limited_torque_body_Nm = saturated_torque(
        command.requested_torque_body_Nm,
        config.maximum_torque_body_Nm
    );
    command.saturated = !command.limited_torque_body_Nm.isApprox(
        command.requested_torque_body_Nm,
        1.0e-15
    );
    return command;
}

SunPointingTracker::SunPointingTracker(SunPointingControllerConfig config)
    : config_{config} {
    validate_controller_config(config_);
}

void SunPointingTracker::reset() {
    state_ = {};
    has_sample_time_ = false;
    acquisition_timer_active_ = false;
    steady_timer_active_ = false;
    acquisition_start_time_s_ = 0.0;
    steady_start_time_s_ = 0.0;
}

void SunPointingTracker::update(
    const double sample_time_s,
    const double pointing_error_rad
) {
    if (!finite_nonnegative(sample_time_s)
        || !finite_nonnegative(pointing_error_rad)
        || (has_sample_time_ && sample_time_s < state_.sample_time_s)) {
        throw std::invalid_argument{
            "Sun-pointing tracker inputs must be finite and chronological"
        };
    }
    state_.sample_time_s = sample_time_s;
    has_sample_time_ = true;

    const auto update_threshold = [sample_time_s, pointing_error_rad](
        const double threshold_rad,
        const double hysteresis_rad,
        const double dwell_time_s,
        bool& timer_active,
        double& start_time_s,
        double& elapsed_s,
        bool& achieved
    ) {
        if (pointing_error_rad <= threshold_rad) {
            if (!timer_active) {
                timer_active = true;
                start_time_s = sample_time_s;
            }
            elapsed_s = sample_time_s - start_time_s;
            achieved = elapsed_s >= dwell_time_s;
        } else if (pointing_error_rad >= threshold_rad + hysteresis_rad) {
            timer_active = false;
            elapsed_s = 0.0;
            achieved = false;
        }
    };

    update_threshold(
        config_.acquisition_threshold_rad,
        config_.hysteresis_rad,
        config_.dwell_time_s,
        acquisition_timer_active_,
        acquisition_start_time_s_,
        state_.acquisition_elapsed_s,
        state_.target_acquired
    );
    update_threshold(
        config_.steady_pointing_threshold_rad,
        config_.hysteresis_rad,
        config_.dwell_time_s,
        steady_timer_active_,
        steady_start_time_s_,
        state_.steady_pointing_elapsed_s,
        state_.steady_pointing
    );
}

const SunPointingControllerConfig& SunPointingTracker::config() const {
    return config_;
}

const SunPointingTrackerState& SunPointingTracker::state() const {
    return state_;
}

}  // namespace detumble
