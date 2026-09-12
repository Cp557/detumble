#include "detumble/flight_software.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace detumble {
namespace {

constexpr double timestamp_tolerance_s = 1.0e-12;

}  // namespace

std::string_view to_string(const FlightMode mode) {
    switch (mode) {
    case FlightMode::boot:
        return "BOOT";
    case FlightMode::detumble:
        return "DETUMBLE";
    case FlightMode::sun_acquire:
        return "SUN_ACQUIRE";
    case FlightMode::sun_point:
        return "SUN_POINT";
    case FlightMode::safe:
        return "SAFE";
    }
    return "UNKNOWN";
}

DetumbleFlightSoftware::DetumbleFlightSoftware(
    DetumbleFlightSoftwareConfig config,
    MagnetorquerConfig magnetorquer_config
)
    : config_{std::move(config)},
      magnetorquer_config_{std::move(magnetorquer_config)},
      estimator_{config_.estimator} {
    if (!std::isfinite(config_.completion_threshold_rad_s)
        || config_.completion_threshold_rad_s < 0.0
        || !std::isfinite(config_.completion_hysteresis_rad_s)
        || config_.completion_hysteresis_rad_s < 0.0
        || !std::isfinite(config_.completion_dwell_time_s)
        || config_.completion_dwell_time_s < 0.0) {
        throw std::invalid_argument{
            "Detumble thresholds and dwell time must be finite and nonnegative"
        };
    }
    static_cast<void>(bdot_dipole_command_body_A_m2(
        config_.controller,
        magnetorquer_config_,
        Eigen::Vector3d::Zero()
    ));
    reset();
}

void DetumbleFlightSoftware::reset() {
    estimator_.reset();
    state_ = {};
    below_threshold_start_s_.reset();
}

void DetumbleFlightSoftware::update(
    const MagnetometerMeasurement& magnetometer,
    const GyroscopeMeasurement& gyroscope
) {
    if (!std::isfinite(gyroscope.sample_time_s)
        || gyroscope.sample_time_s < 0.0
        || !gyroscope.angular_velocity_body_rad_s.allFinite()) {
        throw std::invalid_argument{
            "Gyroscope measurements must contain finite time and rate values"
        };
    }
    if (std::abs(magnetometer.sample_time_s - gyroscope.sample_time_s)
        > timestamp_tolerance_s) {
        throw std::invalid_argument{
            "Magnetometer and gyroscope samples must share a timestamp"
        };
    }

    const std::optional<BdotEstimate> estimate =
        estimator_.update(magnetometer);
    state_.measured_angular_velocity_body_rad_s =
        gyroscope.angular_velocity_body_rad_s;
    update_completion(gyroscope);
    if (!estimate.has_value()) {
        state_.commanded_dipole_body_A_m2.setZero();
        return;
    }

    state_.mode = FlightMode::detumble;
    state_.raw_bdot_body_T_s = estimate->raw_bdot_body_T_s;
    state_.filtered_bdot_body_T_s = estimate->filtered_bdot_body_T_s;
    state_.commanded_dipole_body_A_m2 = state_.detumble_complete
        ? Eigen::Vector3d::Zero()
        : bdot_dipole_command_body_A_m2(
              config_.controller,
              magnetorquer_config_,
              state_.filtered_bdot_body_T_s
          );
}

const DetumbleFlightSoftwareConfig& DetumbleFlightSoftware::config() const {
    return config_;
}

const MagnetorquerConfig& DetumbleFlightSoftware::magnetorquer_config() const {
    return magnetorquer_config_;
}

const DetumbleFlightSoftwareState& DetumbleFlightSoftware::state() const {
    return state_;
}

void DetumbleFlightSoftware::update_completion(
    const GyroscopeMeasurement& gyroscope
) {
    if (state_.detumble_complete) {
        return;
    }

    const double angular_speed_rad_s =
        gyroscope.angular_velocity_body_rad_s.norm();
    if (angular_speed_rad_s <= config_.completion_threshold_rad_s) {
        if (!below_threshold_start_s_.has_value()) {
            below_threshold_start_s_ = gyroscope.sample_time_s;
        }
    } else if (
        angular_speed_rad_s
        >= config_.completion_threshold_rad_s
            + config_.completion_hysteresis_rad_s
    ) {
        below_threshold_start_s_.reset();
    }

    state_.below_threshold_elapsed_s = below_threshold_start_s_.has_value()
        ? gyroscope.sample_time_s - *below_threshold_start_s_
        : 0.0;
    if (state_.below_threshold_elapsed_s
        >= config_.completion_dwell_time_s) {
        state_.detumble_complete = true;
    }
}

std::string_view to_string(const ModeTransitionReason reason) {
    switch (reason) {
    case ModeTransitionReason::sensors_ready:
        return "sensors ready";
    case ModeTransitionReason::detumble_complete:
        return "detumble complete";
    case ModeTransitionReason::sun_unavailable:
        return "Sun unavailable";
    case ModeTransitionReason::attitude_invalid:
        return "attitude estimate invalid";
    case ModeTransitionReason::sun_acquired:
        return "Sun acquired";
    case ModeTransitionReason::pointing_lost:
        return "Sun pointing lost";
    case ModeTransitionReason::navigation_recovered:
        return "Sun and attitude estimate recovered";
    }
    return "unknown";
}

AutonomousFlightSoftware::AutonomousFlightSoftware(
    AutonomousFlightSoftwareConfig config,
    MagnetorquerConfig magnetorquer_config
)
    : config_{std::move(config)},
      detumble_{config_.detumble, std::move(magnetorquer_config)},
      attitude_estimator_{config_.attitude_estimator},
      sun_pointing_tracker_{config_.sun_pointing} {
    const double dwell_times[] = {
        config_.boot_to_detumble_dwell_time_s,
        config_.post_detumble_dwell_time_s,
        config_.acquire_to_point_dwell_time_s,
        config_.pointing_loss_dwell_time_s,
        config_.safe_entry_dwell_time_s,
        config_.safe_recovery_dwell_time_s
    };
    for (const double dwell_time_s : dwell_times) {
        if (!std::isfinite(dwell_time_s) || dwell_time_s < 0.0) {
            throw std::invalid_argument{
                "Mission mode dwell times must be finite and nonnegative"
            };
        }
    }
    reset();
}

void AutonomousFlightSoftware::reset() {
    detumble_.reset();
    attitude_estimator_.reset();
    sun_pointing_tracker_.reset();
    state_ = {};
    events_.clear();
    pending_mode_.reset();
    pending_reason_.reset();
    pending_start_time_s_ = 0.0;
    mode_start_time_s_ = 0.0;
    has_sample_time_ = false;
}

void AutonomousFlightSoftware::update(
    const AutonomousFlightSoftwareInput& input
) {
    const double sample_time_s = input.gyroscope.sample_time_s;
    if (!std::isfinite(sample_time_s) || sample_time_s < 0.0
        || (has_sample_time_ && sample_time_s < state_.sample_time_s)
        || !input.gyroscope.angular_velocity_body_rad_s.allFinite()
        || !input.magnetic_field_inertial_T.allFinite()
        || !input.sun_direction_inertial.allFinite()
        || input.sun_direction_inertial.norm() <= 1.0e-12) {
        throw std::invalid_argument{
            "Autonomous flight-software inputs must be finite and chronological"
        };
    }
    if (input.magnetometer.has_value()
        && std::abs(input.magnetometer->sample_time_s - sample_time_s)
            > timestamp_tolerance_s) {
        throw std::invalid_argument{
            "Mission magnetometer and gyroscope timestamps must match"
        };
    }
    if (input.sun_sensor.has_value()
        && std::abs(input.sun_sensor->sample_time_s - sample_time_s)
            > timestamp_tolerance_s) {
        throw std::invalid_argument{
            "Mission Sun-sensor and gyroscope timestamps must match"
        };
    }

    state_.sample_time_s = sample_time_s;
    state_.mode_elapsed_s = sample_time_s - mode_start_time_s_;
    has_sample_time_ = true;

    if (input.magnetometer.has_value()) {
        detumble_.update(*input.magnetometer, input.gyroscope);
    }

    AttitudeEstimatorInput estimator_input{
        .sample_time_s = sample_time_s,
        .angular_velocity_body_rad_s =
            input.gyroscope.angular_velocity_body_rad_s,
        .gyroscope_valid = true
    };
    if (input.magnetometer.has_value()) {
        estimator_input.magnetic_field = AttitudeVectorObservation{
            .measured_body = input.magnetometer->magnetic_field_body_T,
            .reference_inertial = input.magnetic_field_inertial_T,
            .valid = true
        };
    }
    if (input.sun_sensor.has_value()) {
        estimator_input.sun_direction = AttitudeVectorObservation{
            .measured_body = input.sun_sensor->sun_direction_body.value_or(
                Eigen::Vector3d::Zero()
            ),
            .reference_inertial = input.sun_direction_inertial,
            .valid = input.sun_sensor->sun_direction_body.has_value()
        };
    }
    attitude_estimator_.update(estimator_input);

    state_.sun_available = !input.in_eclipse
        && input.sun_sensor.has_value()
        && input.sun_sensor->sun_direction_body.has_value();
    state_.attitude_valid = attitude_estimator_.estimate().valid;
    state_.sun_pointing = {};
    if ((state_.mode == FlightMode::sun_acquire
         || state_.mode == FlightMode::sun_point)
        && state_.sun_available && state_.attitude_valid) {
        update_pointing(input);
    }

    evaluate_mode(input);
    if ((state_.mode == FlightMode::sun_acquire
         || state_.mode == FlightMode::sun_point)
        && !state_.sun_pointing.active
        && state_.sun_available && state_.attitude_valid) {
        update_pointing(input);
    }
    state_.mode_elapsed_s = sample_time_s - mode_start_time_s_;
    update_actuator_command();
}

const AutonomousFlightSoftwareConfig&
AutonomousFlightSoftware::config() const {
    return config_;
}

const AutonomousFlightSoftwareState&
AutonomousFlightSoftware::state() const {
    return state_;
}

const DetumbleFlightSoftware& AutonomousFlightSoftware::detumble() const {
    return detumble_;
}

const AttitudeEstimator&
AutonomousFlightSoftware::attitude_estimator() const {
    return attitude_estimator_;
}

const SunPointingTracker&
AutonomousFlightSoftware::sun_pointing_tracker() const {
    return sun_pointing_tracker_;
}

const std::vector<ModeTransitionEvent>&
AutonomousFlightSoftware::events() const {
    return events_;
}

void AutonomousFlightSoftware::evaluate_mode(
    const AutonomousFlightSoftwareInput& input
) {
    switch (state_.mode) {
    case FlightMode::boot:
        if (detumble_.state().mode == FlightMode::detumble) {
            request_transition(
                FlightMode::detumble,
                ModeTransitionReason::sensors_ready,
                config_.boot_to_detumble_dwell_time_s
            );
        } else {
            clear_pending_transition();
        }
        break;
    case FlightMode::detumble:
        if (!detumble_.state().detumble_complete) {
            clear_pending_transition();
        } else if (input.in_eclipse || !state_.sun_available) {
            request_transition(
                FlightMode::safe,
                ModeTransitionReason::sun_unavailable,
                config_.post_detumble_dwell_time_s
            );
        } else if (!state_.attitude_valid) {
            request_transition(
                FlightMode::safe,
                ModeTransitionReason::attitude_invalid,
                config_.post_detumble_dwell_time_s
            );
        } else {
            request_transition(
                FlightMode::sun_acquire,
                ModeTransitionReason::detumble_complete,
                config_.post_detumble_dwell_time_s
            );
        }
        break;
    case FlightMode::sun_acquire:
    case FlightMode::sun_point:
        if (input.in_eclipse || !state_.sun_available) {
            request_transition(
                FlightMode::safe,
                ModeTransitionReason::sun_unavailable,
                config_.safe_entry_dwell_time_s
            );
        } else if (!state_.attitude_valid) {
            request_transition(
                FlightMode::safe,
                ModeTransitionReason::attitude_invalid,
                config_.safe_entry_dwell_time_s
            );
        } else if (state_.mode == FlightMode::sun_acquire
                   && sun_pointing_tracker_.state().target_acquired) {
            request_transition(
                FlightMode::sun_point,
                ModeTransitionReason::sun_acquired,
                config_.acquire_to_point_dwell_time_s
            );
        } else if (
            state_.mode == FlightMode::sun_point
            && state_.sun_pointing.pointing_error_rad
                >= config_.sun_pointing.acquisition_threshold_rad
                    + config_.sun_pointing.hysteresis_rad
        ) {
            request_transition(
                FlightMode::sun_acquire,
                ModeTransitionReason::pointing_lost,
                config_.pointing_loss_dwell_time_s
            );
        } else {
            clear_pending_transition();
        }
        break;
    case FlightMode::safe:
        if (state_.sun_available && state_.attitude_valid) {
            request_transition(
                FlightMode::sun_acquire,
                ModeTransitionReason::navigation_recovered,
                config_.safe_recovery_dwell_time_s
            );
        } else {
            clear_pending_transition();
        }
        break;
    }
}

void AutonomousFlightSoftware::update_pointing(
    const AutonomousFlightSoftwareInput& input
) {
    SunPointingControllerConfig controller_config = config_.sun_pointing;
    if (state_.mode == FlightMode::sun_acquire) {
        controller_config.maximum_torque_body_Nm =
            config_.sun_pointing.acquisition_maximum_torque_body_Nm;
    }
    const Eigen::Quaterniond desired_attitude =
        desired_sun_pointing_attitude(
            controller_config,
            input.sun_direction_inertial
        );
    state_.sun_pointing = sun_pointing_command(
        controller_config,
        attitude_estimator_.estimate().body_to_inertial,
        input.gyroscope.angular_velocity_body_rad_s,
        desired_attitude
    );
    sun_pointing_tracker_.update(
        input.gyroscope.sample_time_s,
        state_.sun_pointing.pointing_error_rad
    );
}

void AutonomousFlightSoftware::request_transition(
    const FlightMode new_mode,
    const ModeTransitionReason reason,
    const double dwell_time_s
) {
    if (!pending_mode_.has_value() || *pending_mode_ != new_mode
        || !pending_reason_.has_value() || *pending_reason_ != reason) {
        pending_mode_ = new_mode;
        pending_reason_ = reason;
        pending_start_time_s_ = state_.sample_time_s;
    }
    state_.pending_transition_elapsed_s =
        state_.sample_time_s - pending_start_time_s_;
    if (state_.pending_transition_elapsed_s >= dwell_time_s) {
        transition_to(new_mode, reason);
    }
}

void AutonomousFlightSoftware::clear_pending_transition() {
    pending_mode_.reset();
    pending_reason_.reset();
    pending_start_time_s_ = state_.sample_time_s;
    state_.pending_transition_elapsed_s = 0.0;
}

void AutonomousFlightSoftware::transition_to(
    const FlightMode new_mode,
    const ModeTransitionReason reason
) {
    if (new_mode == state_.mode) {
        clear_pending_transition();
        return;
    }
    const FlightMode old_mode = state_.mode;
    events_.push_back(ModeTransitionEvent{
        .sample_time_s = state_.sample_time_s,
        .old_mode = old_mode,
        .new_mode = new_mode,
        .reason = reason
    });
    state_.mode = new_mode;
    mode_start_time_s_ = state_.sample_time_s;
    if (new_mode == FlightMode::safe
        || new_mode == FlightMode::sun_acquire) {
        sun_pointing_tracker_.reset();
        state_.sun_pointing = {};
    }
    clear_pending_transition();
}

void AutonomousFlightSoftware::update_actuator_command() {
    state_.actuator_command = ActuatorCommand{
        .sample_time_s = state_.sample_time_s
    };
    if (state_.mode == FlightMode::detumble) {
        state_.actuator_command.magnetorquer_dipole_body_A_m2 =
            detumble_.state().commanded_dipole_body_A_m2;
        state_.actuator_command.magnetorquers_enabled =
            !detumble_.state().detumble_complete;
    } else if (
        (state_.mode == FlightMode::sun_acquire
         || state_.mode == FlightMode::sun_point)
        && state_.sun_pointing.active
    ) {
        state_.actuator_command.reaction_wheel_torque_body_Nm =
            state_.sun_pointing.limited_torque_body_Nm;
        state_.actuator_command.reaction_wheels_enabled = true;
    }
}

}  // namespace detumble
