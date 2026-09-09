#include "detumble/mission.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "detumble/frames.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/reaction_wheel.hpp"
#include "detumble/simulation.hpp"

namespace detumble {
namespace {

double pointing_error_rad(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& sun_direction_inertial,
    const Eigen::Vector3d& sun_axis_body
) {
    const Eigen::Vector3d pointing_axis_inertial = rotate_body_to_inertial(
        body_to_inertial,
        sun_axis_body
    ).normalized();
    return std::acos(std::clamp(
        pointing_axis_inertial.dot(sun_direction_inertial.normalized()),
        -1.0,
        1.0
    ));
}

MissionFailureReason determine_failure(
    const MissionMetrics& metrics,
    const MissionSuccessCriteria& criteria
) {
    if (!metrics.detumble_time_s.has_value()) {
        return MissionFailureReason::detumble_timeout;
    }
    if (!metrics.acquisition_time_s.has_value()) {
        return MissionFailureReason::sun_acquisition_timeout;
    }
    if (criteria.require_steady_pointing
        && !metrics.steady_pointing_time_s.has_value()) {
        return MissionFailureReason::pointing_not_steady;
    }
    if (metrics.final_rate_rad_s > criteria.maximum_final_rate_rad_s) {
        return MissionFailureReason::final_rate_exceeded;
    }
    if (metrics.rms_true_pointing_error_rad
        > criteria.maximum_rms_pointing_error_rad) {
        return MissionFailureReason::rms_pointing_error_exceeded;
    }
    return MissionFailureReason::none;
}

}  // namespace

std::string_view to_string(const MissionFailureReason reason) {
    switch (reason) {
    case MissionFailureReason::none:
        return "none";
    case MissionFailureReason::detumble_timeout:
        return "detumble timeout";
    case MissionFailureReason::sun_acquisition_timeout:
        return "Sun acquisition timeout";
    case MissionFailureReason::pointing_not_steady:
        return "pointing not steady";
    case MissionFailureReason::final_rate_exceeded:
        return "final rate exceeded";
    case MissionFailureReason::rms_pointing_error_exceeded:
        return "RMS pointing error exceeded";
    }
    return "unknown";
}

MissionMetrics run_native_mission(
    const MissionRunConfig& config,
    const MissionTelemetryCallback& telemetry_callback
) {
    if (!std::isfinite(config.duration_s) || config.duration_s < 0.0
        || !std::isfinite(config.time_step_s) || config.time_step_s <= 0.0
        || !std::isfinite(config.initial_rate_rad_s)
        || config.initial_rate_rad_s < 0.0
        || !std::isfinite(config.success.maximum_final_rate_rad_s)
        || config.success.maximum_final_rate_rad_s < 0.0
        || !std::isfinite(
            config.success.maximum_rms_pointing_error_rad
        )
        || config.success.maximum_rms_pointing_error_rad < 0.0) {
        throw std::invalid_argument{"Mission timing and rate must be valid"};
    }

    const SensorSuiteConfig sensors = config.sensors.value_or(
        realistic_sensor_suite_config(config.seed)
    );
    Simulation simulation{SimulationConfig{
        .seed = config.seed,
        .time_step_s = config.time_step_s,
        .minimum_angular_speed_rad_s = config.initial_rate_rad_s,
        .maximum_angular_speed_rad_s = config.initial_rate_rad_s
    }};
    MagneticControlCycle magnetic_control{sensors.magnetometer};
    IdealGyroscope gyroscope{sensors.gyroscope};
    IdealCoarseSunSensorArray sun_sensors{sensors.sun_sensor};
    AutonomousFlightSoftware flight_software{config.flight_software};
    ReactionWheelCluster reaction_wheels;
    MissionMetrics metrics;
    metrics.seed = config.seed;
    double pointing_error_squared_sum{};

    while (simulation.elapsed_time_s() + 0.5 * config.time_step_s
           < config.duration_s) {
        const EnvironmentState environment = sample_environment(
            config.environment,
            simulation.state().body_to_inertial,
            simulation.elapsed_time_s()
        );
        const auto gyroscope_measurement = gyroscope.sample_if_due(
            simulation.elapsed_time_s(),
            simulation.state().angular_velocity_body_rad_s
        );
        const auto sun_sensor_measurement = sun_sensors.sample_if_due(
            simulation.elapsed_time_s(),
            environment.sun_direction_body,
            environment.in_eclipse
        );
        const MagneticControlOutput magnetic_output = magnetic_control.update(
            simulation.elapsed_time_s(),
            environment.magnetic_field_body_T,
            flight_software.state().actuator_command
                .magnetorquer_dipole_body_A_m2
        );
        if (gyroscope_measurement.has_value()) {
            flight_software.update(AutonomousFlightSoftwareInput{
                .gyroscope = *gyroscope_measurement,
                .magnetometer = magnetic_output.magnetometer_measurement,
                .sun_sensor = sun_sensor_measurement,
                .magnetic_field_inertial_T =
                    environment.magnetic_field_inertial_T,
                .sun_direction_inertial = environment.sun_direction_inertial,
                .in_eclipse = environment.in_eclipse
            });
        }
        reaction_wheels.update(
            ReactionWheelCommand{
                .sample_time_s = simulation.elapsed_time_s(),
                .requested_spacecraft_torque_body_Nm =
                    flight_software.state().actuator_command
                        .reaction_wheel_torque_body_Nm,
                .enabled = flight_software.state().actuator_command
                    .reaction_wheels_enabled
            },
            config.time_step_s
        );
        simulation.step(
            magnetic_output.applied_torque_body_Nm
            + reaction_wheels.telemetry().applied_spacecraft_torque_body_Nm
        );

        const double time_s = simulation.elapsed_time_s();
        const auto& flight_state = flight_software.state();
        const auto& tracking = flight_software.sun_pointing_tracker().state();
        const auto& estimate = flight_software.attitude_estimator().estimate();
        const EnvironmentState updated_environment = sample_environment(
            config.environment,
            simulation.state().body_to_inertial,
            time_s
        );
        const double true_pointing_error = pointing_error_rad(
            simulation.state().body_to_inertial,
            updated_environment.sun_direction_inertial,
            config.flight_software.sun_pointing.sun_axis_body
        );
        const double attitude_error = estimate.valid
            ? attitude_error_angle_rad(
                  estimate.body_to_inertial,
                  simulation.state().body_to_inertial
              )
            : std::numbers::pi;

        if (!metrics.detumble_time_s.has_value()
            && flight_software.detumble().state().detumble_complete) {
            metrics.detumble_time_s = time_s;
        }
        if (!metrics.acquisition_time_s.has_value()
            && tracking.target_acquired) {
            metrics.acquisition_time_s = time_s;
        }
        if (!metrics.steady_pointing_time_s.has_value()
            && tracking.steady_pointing) {
            metrics.steady_pointing_time_s = time_s;
        }
        for (const double wheel_speed :
             reaction_wheels.telemetry().wheel_speed_rad_s) {
            metrics.maximum_wheel_speed_rad_s = std::max(
                metrics.maximum_wheel_speed_rad_s,
                std::abs(wheel_speed)
            );
        }
        if (std::ranges::any_of(
                reaction_wheels.telemetry().speed_saturated,
                [](const bool saturated) { return saturated; }
            )) {
            ++metrics.wheel_saturation_steps;
        }
        if (flight_state.mode == FlightMode::sun_point
            && !updated_environment.in_eclipse) {
            pointing_error_squared_sum +=
                true_pointing_error * true_pointing_error;
            metrics.maximum_true_pointing_error_rad = std::max(
                metrics.maximum_true_pointing_error_rad,
                true_pointing_error
            );
            ++metrics.pointing_sample_count;
        }
        if (telemetry_callback) {
            telemetry_callback(MissionTelemetrySample{
                .time_s = time_s,
                .mode = flight_state.mode,
                .body_rate_rad_s = simulation.state()
                    .angular_velocity_body_rad_s.norm(),
                .true_pointing_error_rad = true_pointing_error,
                .estimated_pointing_error_rad =
                    flight_state.sun_pointing.pointing_error_rad,
                .attitude_estimation_error_rad = attitude_error,
                .in_eclipse = updated_environment.in_eclipse,
                .detumble_complete = flight_software.detumble()
                    .state().detumble_complete,
                .target_acquired = tracking.target_acquired,
                .steady_pointing = tracking.steady_pointing
            });
        }
    }

    const EnvironmentState final_environment = sample_environment(
        config.environment,
        simulation.state().body_to_inertial,
        simulation.elapsed_time_s()
    );
    const auto& estimate = flight_software.attitude_estimator().estimate();
    metrics.final_mode = flight_software.state().mode;
    metrics.final_rate_rad_s =
        simulation.state().angular_velocity_body_rad_s.norm();
    metrics.final_true_pointing_error_rad = pointing_error_rad(
        simulation.state().body_to_inertial,
        final_environment.sun_direction_inertial,
        config.flight_software.sun_pointing.sun_axis_body
    );
    metrics.final_estimated_pointing_error_rad =
        flight_software.state().sun_pointing.pointing_error_rad;
    metrics.final_attitude_estimation_error_rad = estimate.valid
        ? attitude_error_angle_rad(
              estimate.body_to_inertial,
              simulation.state().body_to_inertial
          )
        : std::numbers::pi;
    metrics.rms_true_pointing_error_rad = metrics.pointing_sample_count == 0
        ? 0.0
        : std::sqrt(
              pointing_error_squared_sum
              / static_cast<double>(metrics.pointing_sample_count)
          );
    metrics.failure_reason = determine_failure(
        metrics,
        config.success
    );
    metrics.success = metrics.failure_reason == MissionFailureReason::none;
    return metrics;
}

}  // namespace detumble
