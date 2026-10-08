#include "detumble/flight_software.hpp"
#include <cmath>
#include <stdexcept>
namespace detumble {
std::string_view to_string(const FlightMode mode) {
    switch (mode) {
    case FlightMode::boot:
        return "BOOT";
    case FlightMode::detumble:
        return "DETUMBLE";
    case FlightMode::low_rate:
        return "LOW_RATE";
    case FlightMode::safe:
        return "SAFE";
    }
    return "UNKNOWN";
}
std::string_view to_string(const DetumbleController controller) {
    switch (controller) {
    case DetumbleController::bdot:
        return "bdot";
    case DetumbleController::estimated_rate:
        return "estimated-rate";
    }
    throw std::invalid_argument{"Invalid controller"};
}
DetumbleController parse_controller(const std::string_view name) {
    if (name == "bdot")
        return DetumbleController::bdot;
    if (name == "estimated-rate")
        return DetumbleController::estimated_rate;
    throw std::invalid_argument{"Controller must be bdot or estimated-rate"};
}
DetumbleFlightSoftware::DetumbleFlightSoftware(DetumbleFlightSoftwareConfig config,
                                               MagnetorquerConfig magnetorquer,
                                               RigidBodyProperties body,
                                               VectorSensorErrorConfig sensor)
    : config_{config}, magnetorquer_{std::move(magnetorquer)},
      measurement_saturation_limit_{sensor.saturation_limit},
      bdot_estimator_{config_.bdot_estimator},
      attitude_estimator_{config_.attitude_estimator, std::move(body),
                          std::move(sensor)} {
    const double values[] = {
        config_.completion_threshold_rad_s, config_.completion_hysteresis_rad_s,
        config_.completion_dwell_time_s, config_.handoff_dwell_time_s,
        config_.measurement_timeout_s};
    for (double value : values)
        if (!std::isfinite(value) || value < 0.0)
            throw std::invalid_argument{
                "Flight software settings must be finite and nonnegative"};
    if (config_.measurement_timeout_s == 0.0)
        throw std::invalid_argument{"Measurement timeout must be positive"};
    static_cast<void>(to_string(config_.controller_selection));
    static_cast<void>(bdot_dipole_command_body_A_m2(config_.controller, magnetorquer_,
                                                    Eigen::Vector3d::Zero()));
}
void DetumbleFlightSoftware::reset() {
    state_ = {};
    events_.clear();
    bdot_estimator_.reset();
    attitude_estimator_.reset();
    confident_since_s_.reset();
    below_threshold_since_s_.reset();
    last_sample_s_.reset();
    last_update_s_ = -1.0;
}
void DetumbleFlightSoftware::transition(const FlightMode mode, const double time_s) {
    if (mode != state_.mode) {
        events_.push_back({time_s, state_.mode, mode});
        state_.mode = mode;
    }
}
void DetumbleFlightSoftware::predict(const double time_s,
                                     const Eigen::Vector3d &reference,
                                     const Eigen::Vector3d &dipole, const double dt) {
    attitude_estimator_.predict(time_s, reference, dipole, dt);
}
void DetumbleFlightSoftware::update(
    const double time_s, const std::optional<MagnetometerMeasurement> &measurement,
    const Eigen::Vector3d &reference) {
    if (!std::isfinite(time_s) || time_s < 0.0 || time_s < last_update_s_)
        throw std::invalid_argument{"Flight software time must be chronological"};
    last_update_s_ = time_s;
    const bool invalid =
        measurement &&
        (!measurement->magnetic_field_body_T.allFinite() ||
         measurement->magnetic_field_body_T.norm() < 1e-6 ||
         (measurement->magnetic_field_body_T.cwiseAbs().array() >=
          measurement_saturation_limit_.array())
             .any() ||
         !std::isfinite(measurement->sample_time_s) ||
         std::abs(measurement->sample_time_s - time_s) > 1e-8 ||
         (last_sample_s_ && measurement->sample_time_s <= *last_sample_s_));
    const bool stale =
        !measurement &&
        ((last_sample_s_ && time_s - *last_sample_s_ > config_.measurement_timeout_s) ||
         (!last_sample_s_ && time_s > config_.measurement_timeout_s));
    if (invalid || stale) {
        state_.commanded_dipole_body_A_m2.setZero();
        state_.estimated_rate_control_active = false;
        state_.low_rate_confirmed = false;
        confident_since_s_.reset();
        below_threshold_since_s_.reset();
        last_sample_s_.reset();
        bdot_estimator_.reset();
        attitude_estimator_.reset();
        transition(FlightMode::safe, time_s);
        return;
    }
    if (!measurement)
        return;
    last_sample_s_ = time_s;
    const auto bdot = bdot_estimator_.update(*measurement);
    attitude_estimator_.update(*measurement, reference);
    if (!bdot) {
        state_.commanded_dipole_body_A_m2.setZero();
        return;
    }
    state_.filtered_bdot_body_T_s = bdot->filtered_bdot_body_T_s;
    const auto &estimate = attitude_estimator_.estimate();
    if (estimate.confident) {
        if (!confident_since_s_)
            confident_since_s_ = time_s;
    } else {
        confident_since_s_.reset();
        below_threshold_since_s_.reset();
        state_.low_rate_confirmed = false;
    }
    const double upper_rate = estimate.attitude.angular_velocity_body_rad_s.norm() +
                              3.0 * estimate.rate_standard_deviation_rad_s;
    if (estimate.confident && upper_rate <= config_.completion_threshold_rad_s) {
        if (!below_threshold_since_s_)
            below_threshold_since_s_ = time_s;
        if (time_s - *below_threshold_since_s_ >= config_.completion_dwell_time_s)
            state_.low_rate_confirmed = true;
    } else {
        below_threshold_since_s_.reset();
        if (upper_rate >
            config_.completion_threshold_rad_s + config_.completion_hysteresis_rad_s)
            state_.low_rate_confirmed = false;
    }
    state_.estimated_rate_control_active =
        config_.controller_selection == DetumbleController::estimated_rate &&
        confident_since_s_ &&
        time_s - *confident_since_s_ >= config_.handoff_dwell_time_s;
    state_.commanded_dipole_body_A_m2 =
        state_.estimated_rate_control_active
            ? saturate_magnetorquer_dipole_body_A_m2(
                  magnetorquer_,
                  config_.controller.gain_A_m2_s_per_T *
                      estimate.attitude.angular_velocity_body_rad_s.cross(
                          measurement->magnetic_field_body_T))
            : bdot_dipole_command_body_A_m2(config_.controller, magnetorquer_,
                                            bdot->filtered_bdot_body_T_s);
    transition(state_.low_rate_confirmed ? FlightMode::low_rate : FlightMode::detumble,
               time_s);
}
} // namespace detumble
