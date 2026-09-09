#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/reaction_wheel.hpp"
#include "detumble/simulation.hpp"

TEST_CASE("seeded mission autonomously detumbles and points at the Sun") {
    constexpr double duration_s = 6'000.0;
    constexpr double time_step_s = 0.01;
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;

    detumble::Simulation simulation{
        detumble::SimulationConfig{
            .seed = 42,
            .time_step_s = time_step_s,
            .minimum_angular_speed_rad_s = 10.0 * degrees_to_radians,
            .maximum_angular_speed_rad_s = 10.0 * degrees_to_radians
        }
    };
    const detumble::EnvironmentConfig environment_config;
    detumble::MagneticControlCycle magnetic_control;
    detumble::IdealGyroscope gyroscope;
    detumble::IdealCoarseSunSensorArray sun_sensors;
    detumble::AutonomousFlightSoftware flight_software;
    detumble::ReactionWheelCluster reaction_wheels;
    detumble::MagneticControlOutput magnetic_output;
    double maximum_wheel_speed_rad_s = 0.0;
    bool command_outside_active_mode{};

    while (simulation.elapsed_time_s() + 0.5 * time_step_s < duration_s) {
        const detumble::EnvironmentState environment =
            detumble::sample_environment(
                environment_config,
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
        magnetic_output = magnetic_control.update(
            simulation.elapsed_time_s(),
            environment.magnetic_field_body_T,
            flight_software.state().actuator_command
                .magnetorquer_dipole_body_A_m2
        );
        if (gyroscope_measurement.has_value()) {
            flight_software.update(
                detumble::AutonomousFlightSoftwareInput{
                    .gyroscope = *gyroscope_measurement,
                    .magnetometer =
                        magnetic_output.magnetometer_measurement,
                    .sun_sensor = sun_sensor_measurement,
                    .magnetic_field_inertial_T =
                        environment.magnetic_field_inertial_T,
                    .sun_direction_inertial =
                        environment.sun_direction_inertial,
                    .in_eclipse = environment.in_eclipse
                }
            );
        }

        const detumble::ActuatorCommand& actuator_command =
            flight_software.state().actuator_command;
        if (flight_software.state().mode == detumble::FlightMode::boot
            || flight_software.state().mode == detumble::FlightMode::safe) {
            command_outside_active_mode = command_outside_active_mode
                || actuator_command.magnetorquers_enabled
                || actuator_command.reaction_wheels_enabled;
        }
        reaction_wheels.update(
            detumble::ReactionWheelCommand{
                .sample_time_s = simulation.elapsed_time_s(),
                .requested_spacecraft_torque_body_Nm =
                    actuator_command.reaction_wheel_torque_body_Nm,
                .enabled = actuator_command.reaction_wheels_enabled
            },
            time_step_s
        );
        simulation.step(
            magnetic_output.applied_torque_body_Nm
            + reaction_wheels.telemetry()
                  .applied_spacecraft_torque_body_Nm
        );
        for (const double wheel_speed_rad_s :
             reaction_wheels.telemetry().wheel_speed_rad_s) {
            maximum_wheel_speed_rad_s = std::max(
                maximum_wheel_speed_rad_s,
                std::abs(wheel_speed_rad_s)
            );
        }
    }

    REQUIRE(flight_software.state().mode == detumble::FlightMode::sun_point);
    REQUIRE_FALSE(command_outside_active_mode);
    REQUIRE(flight_software.detumble().state().detumble_complete);
    REQUIRE(flight_software.attitude_estimator().estimate().valid);
    REQUIRE(flight_software.sun_pointing_tracker().state().steady_pointing);
    REQUIRE(
        flight_software.state().sun_pointing.pointing_error_rad
        < 0.25 * degrees_to_radians
    );
    REQUIRE(
        simulation.state().angular_velocity_body_rad_s.norm()
        < 0.02 * degrees_to_radians
    );
    REQUIRE(
        maximum_wheel_speed_rad_s
        < reaction_wheels.config().wheels.front().maximum_speed_rad_s
    );
    REQUIRE(reaction_wheels.telemetry().allocation_error_body_Nm.norm()
            < 1.0e-12);

    const auto& events = flight_software.events();
    REQUIRE(events.size() == 4);
    REQUIRE(events[0].new_mode == detumble::FlightMode::detumble);
    REQUIRE(events[1].new_mode == detumble::FlightMode::safe);
    REQUIRE(events[2].new_mode == detumble::FlightMode::sun_acquire);
    REQUIRE(events[3].new_mode == detumble::FlightMode::sun_point);
    REQUIRE(events[0].sample_time_s < events[1].sample_time_s);
    REQUIRE(events[1].sample_time_s < events[2].sample_time_s);
    REQUIRE(events[2].sample_time_s < events[3].sample_time_s);
}
