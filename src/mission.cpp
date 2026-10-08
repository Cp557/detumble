#include "detumble/mission.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace detumble {
namespace {
SimulationConfig simulation_config(const MissionRunConfig &config) {
    if (!std::isfinite(config.duration_s) || config.duration_s < 0.0 ||
        !std::isfinite(config.initial_rate_rad_s) || config.initial_rate_rad_s < 0.0)
        throw std::invalid_argument{
            "Mission duration and initial rate must be finite and nonnegative"};
    return {config.seed, config.time_step_s, config.initial_rate_rad_s,
            config.initial_rate_rad_s};
}
SensorSuiteConfig sensors(const MissionRunConfig &config) {
    return config.sensors.value_or(realistic_sensor_suite_config(config.seed));
}
} // namespace
std::string_view to_string(const MissionFailureReason reason) {
    switch (reason) {
    case MissionFailureReason::none:
        return "none";
    case MissionFailureReason::detumble_timeout:
        return "detumble timeout";
    case MissionFailureReason::insufficient_verification:
        return "insufficient verification";
    case MissionFailureReason::residual_rate_exceeded:
        return "residual rate exceeded";
    }
    return "unknown";
}
DetumbleMission::DetumbleMission(MissionRunConfig config)
    : config_{std::move(config)}, simulation_{simulation_config(config_)},
      environment_{config_.environment},
      navigation_{config_.navigation_environment.value_or(config_.environment)},
      magnetic_control_{sensors(config_).magnetometer},
      flight_software_{config_.flight_software,
                       {},
                       simulation_.body(),
                       sensors(config_).magnetometer.error} {
    const double values[] = {config_.success.maximum_rate_rad_s,
                             config_.success.dwell_time_s,
                             config_.success.final_window_s,
                             config_.success.maximum_final_window_rate_rad_s};
    for (double value : values)
        if (!std::isfinite(value) || value < 0.0)
            throw std::invalid_argument{
                "Mission success settings must be finite and nonnegative"};
    if (config_.success.final_window_s <= 0.0 || config_.success.dwell_time_s <= 0.0)
        throw std::invalid_argument{"Mission verification windows must be positive"};
    reset();
}
void DetumbleMission::reset() {
    simulation_.reset();
    magnetic_control_.reset();
    flight_software_.reset();
    metrics_ = {};
    metrics_.seed = config_.seed;
    below_threshold_since_s_.reset();
    maximum_rates_.clear();
    squared_rate_error_sum_ = 0.0;
    preceding_mode_ = FlightMode::boot;
    telemetry_ = {};
    telemetry_.truth = simulation_.state();
    telemetry_.environment =
        environment_.sample(simulation_.state().body_to_inertial, 0.0);
    telemetry_.rotational_energy_J = simulation_.metrics().rotational_kinetic_energy_j;
}
void DetumbleMission::step() {
    const double time = simulation_.elapsed_time_s();
    const double dt = config_.time_step_s;
    const auto &environment = telemetry_.environment;
    const Eigen::Vector3d reference = navigation_.reference_field_inertial_T(time);
    auto magnetic =
        magnetic_control_.update(time, environment.magnetic_field_body_T,
                                 flight_software_.state().commanded_dipole_body_A_m2);
    flight_software_.update(time, magnetic.magnetometer_measurement, reference);
    if (flight_software_.state().mode == FlightMode::safe) {
        magnetic.applied_dipole_body_A_m2.setZero();
        magnetic.applied_torque_body_Nm.setZero();
    }
    flight_software_.predict(time, reference, magnetic.applied_dipole_body_A_m2, dt);
    simulation_.step(magnetic.applied_torque_body_Nm);
    telemetry_ = {simulation_.elapsed_time_s(),
                  simulation_.state(),
                  environment_.sample(simulation_.state().body_to_inertial,
                                      simulation_.elapsed_time_s()),
                  magnetic,
                  magnetic_control_.magnetometer().last_measurement(),
                  flight_software_.estimate(),
                  flight_software_.state(),
                  simulation_.metrics().rotational_kinetic_energy_j,
                  false};
    const double speed = simulation_.state().angular_velocity_body_rad_s.norm();
    if (speed <= config_.success.maximum_rate_rad_s) {
        if (!below_threshold_since_s_)
            below_threshold_since_s_ = telemetry_.time_s;
        if (!metrics_.detumble_time_s &&
            telemetry_.time_s - *below_threshold_since_s_ >=
                config_.success.dwell_time_s)
            metrics_.detumble_time_s = telemetry_.time_s;
    } else
        below_threshold_since_s_.reset();
    telemetry_.goal_achieved = metrics_.detumble_time_s.has_value();
    while (!maximum_rates_.empty() && maximum_rates_.back().second <= speed)
        maximum_rates_.pop_back();
    maximum_rates_.emplace_back(telemetry_.time_s, speed);
    while (maximum_rates_.front().first <
           telemetry_.time_s - config_.success.final_window_s)
        maximum_rates_.pop_front();
    metrics_.integrated_dipole_squared_A2_m4_s +=
        magnetic.applied_dipole_body_A_m2.squaredNorm() * dt;
    if ((magnetic.commanded_dipole_body_A_m2 - magnetic.limited_dipole_body_A_m2)
                .norm() > 1e-12 ||
        magnetic.limited_dipole_body_A_m2.cwiseAbs().maxCoeff() >= 0.2 - 1e-12)
        ++metrics_.saturation_steps;
    if (telemetry_.flight.estimated_rate_control_active) {
        ++metrics_.estimated_rate_control_steps;
        if (!metrics_.controller_handoff_time_s)
            metrics_.controller_handoff_time_s = telemetry_.time_s;
    }
    if (telemetry_.estimate.confident && !metrics_.estimator_convergence_time_s)
        metrics_.estimator_convergence_time_s = telemetry_.time_s;
    if (telemetry_.flight.mode == FlightMode::low_rate &&
        preceding_mode_ != FlightMode::low_rate &&
        speed > config_.success.maximum_rate_rad_s)
        ++metrics_.false_low_rate_events;
    preceding_mode_ = telemetry_.flight.mode;
    if (magnetic.magnetometer_measurement && telemetry_.estimate.valid) {
        squared_rate_error_sum_ +=
            (telemetry_.estimate.attitude.angular_velocity_body_rad_s -
             simulation_.state().angular_velocity_body_rad_s)
                .squaredNorm();
        ++metrics_.estimate_samples;
    }
}
MissionMetrics DetumbleMission::metrics() const {
    auto result = metrics_;
    result.final_rate_rad_s = telemetry_.truth.angular_velocity_body_rad_s.norm();
    result.final_window_maximum_rate_rad_s = maximum_rates_.empty()
                                                 ? result.final_rate_rad_s
                                                 : maximum_rates_.front().second;
    result.final_rate_estimation_error_rad_s =
        telemetry_.estimate.valid
            ? (telemetry_.estimate.attitude.angular_velocity_body_rad_s -
               telemetry_.truth.angular_velocity_body_rad_s)
                  .norm()
            : std::numeric_limits<double>::quiet_NaN();
    result.final_attitude_estimation_error_rad =
        telemetry_.estimate.valid
            ? attitude_error_angle_rad(telemetry_.estimate.attitude.body_to_inertial,
                                       telemetry_.truth.body_to_inertial)
            : std::numeric_limits<double>::quiet_NaN();
    result.rms_rate_estimation_error_rad_s =
        result.estimate_samples > 0
            ? std::sqrt(squared_rate_error_sum_ /
                        static_cast<double>(result.estimate_samples))
            : std::numeric_limits<double>::quiet_NaN();
    result.failure_reason = !result.detumble_time_s
                                ? MissionFailureReason::detumble_timeout
                            : telemetry_.time_s < config_.success.final_window_s
                                ? MissionFailureReason::insufficient_verification
                            : result.final_window_maximum_rate_rad_s >
                                    config_.success.maximum_final_window_rate_rad_s
                                ? MissionFailureReason::residual_rate_exceeded
                                : MissionFailureReason::none;
    result.success = result.failure_reason == MissionFailureReason::none;
    return result;
}
MissionMetrics run_native_mission(const MissionRunConfig &config,
                                  const MissionTelemetryCallback &callback) {
    DetumbleMission mission{config};
    while (mission.telemetry().time_s + 0.5 * config.time_step_s < config.duration_s) {
        mission.step();
        if (callback)
            callback(mission.telemetry());
    }
    return mission.metrics();
}
} // namespace detumble
