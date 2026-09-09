#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/reaction_wheel.hpp"
#include "detumble/sensor_suite.hpp"
#include "detumble/simulation.hpp"
#include "detumble/sun_pointing.hpp"
#include "detumble/version.hpp"

namespace {

struct CliOptions {
    double duration_s{6'000.0};
    double time_step_s{0.01};
    std::uint64_t seed{42};
    double initial_rate_deg_s{10.0};
    std::string sensor_profile{"realistic"};
    std::optional<std::string> output_path;
    bool show_help{};
};

void print_usage(std::ostream& output) {
    output
        << "Usage: detumble [options]\n\n"
        << "Options:\n"
        << "  --duration SECONDS          Simulation duration (default: 6000)\n"
        << "  --time-step SECONDS         Fixed timestep (default: 0.01)\n"
        << "  --seed INTEGER              Deterministic seed (default: 42)\n"
        << "  --initial-rate-deg-s RATE   Initial angular speed (default: 10)\n"
        << "  --sensor-profile PROFILE    realistic or ideal (default: realistic)\n"
        << "  --output PATH               Write full-rate telemetry CSV\n"
        << "  --help                      Show this message\n";
}

double parse_double(
    const std::string_view option,
    const std::string& value
) {
    std::size_t parsed_characters = 0;
    const double parsed = std::stod(value, &parsed_characters);
    if (parsed_characters != value.size() || !std::isfinite(parsed)) {
        throw std::invalid_argument{
            std::string{option} + " requires a finite number"
        };
    }
    return parsed;
}

std::uint64_t parse_seed(const std::string& value) {
    if (value.empty() || value.front() == '-') {
        throw std::invalid_argument{"--seed requires a nonnegative integer"};
    }
    std::size_t parsed_characters = 0;
    const unsigned long long parsed = std::stoull(value, &parsed_characters);
    if (parsed_characters != value.size()) {
        throw std::invalid_argument{"--seed requires a nonnegative integer"};
    }
    return static_cast<std::uint64_t>(parsed);
}

CliOptions parse_options(const int argument_count, char** arguments) {
    CliOptions options;
    for (int index = 1; index < argument_count; ++index) {
        const std::string_view option{arguments[index]};
        if (option == "--help") {
            options.show_help = true;
            continue;
        }
        if (index + 1 >= argument_count) {
            throw std::invalid_argument{
                std::string{option} + " requires a value"
            };
        }

        const std::string value{arguments[++index]};
        if (option == "--duration") {
            options.duration_s = parse_double(option, value);
        } else if (option == "--time-step") {
            options.time_step_s = parse_double(option, value);
        } else if (option == "--seed") {
            options.seed = parse_seed(value);
        } else if (option == "--initial-rate-deg-s") {
            options.initial_rate_deg_s = parse_double(option, value);
        } else if (option == "--sensor-profile") {
            options.sensor_profile = value;
        } else if (option == "--output") {
            options.output_path = value;
        } else {
            throw std::invalid_argument{"Unknown option: " + std::string{option}};
        }
    }

    if (options.duration_s < 0.0) {
        throw std::invalid_argument{"--duration must be nonnegative"};
    }
    if (options.time_step_s <= 0.0) {
        throw std::invalid_argument{"--time-step must be positive"};
    }
    if (options.initial_rate_deg_s < 0.0) {
        throw std::invalid_argument{
            "--initial-rate-deg-s must be nonnegative"
        };
    }
    if (options.sensor_profile != "realistic"
        && options.sensor_profile != "ideal") {
        throw std::invalid_argument{
            "--sensor-profile must be realistic or ideal"
        };
    }
    return options;
}

detumble::SensorSuiteConfig sensor_config(const CliOptions& options) {
    return options.sensor_profile == "realistic"
        ? detumble::realistic_sensor_suite_config(options.seed)
        : detumble::ideal_sensor_suite_config(options.seed);
}

void write_csv_header(
    std::ostream& output,
    const detumble::ReactionWheelClusterConfig& wheel_config
) {
    output
        << "time_s,q_w,q_x,q_y,q_z,"
        << "estimated_q_w,estimated_q_x,estimated_q_y,estimated_q_z,"
        << "attitude_estimate_sample_time_s,attitude_estimate_valid,"
        << "attitude_error_deg,attitude_correction_status,"
        << "attitude_correction_count,"
        << "sun_pointing_active,sun_pointing_error_deg,"
        << "sun_pointing_requested_torque_x_body_Nm,"
        << "sun_pointing_requested_torque_y_body_Nm,"
        << "sun_pointing_requested_torque_z_body_Nm,"
        << "sun_pointing_limited_torque_x_body_Nm,"
        << "sun_pointing_limited_torque_y_body_Nm,"
        << "sun_pointing_limited_torque_z_body_Nm,"
        << "sun_target_acquired,sun_steady_pointing,"
        << "wheel_applied_torque_x_body_Nm,"
        << "wheel_applied_torque_y_body_Nm,"
        << "wheel_applied_torque_z_body_Nm,"
        << "wheel_allocation_error_x_body_Nm,"
        << "wheel_allocation_error_y_body_Nm,"
        << "wheel_allocation_error_z_body_Nm,"
        << "wheel_allocation_saturated,";
    for (std::size_t index = 0; index < wheel_config.wheels.size(); ++index) {
        output << "wheel_" << index << "_speed_rad_s,"
               << "wheel_" << index << "_commanded_motor_torque_Nm,"
               << "wheel_" << index << "_applied_motor_torque_Nm,"
               << "wheel_" << index << "_torque_saturated,"
               << "wheel_" << index << "_speed_saturated,"
               << "wheel_" << index << "_failed,";
    }
    output
        << "omega_x_body_rad_s,omega_y_body_rad_s,omega_z_body_rad_s,"
        << "rotational_energy_J,"
        << "B_x_body_T,B_y_body_T,B_z_body_T,"
        << "sun_x_inertial,sun_y_inertial,sun_z_inertial,"
        << "sun_x_body,sun_y_body,sun_z_body,in_eclipse,"
        << "sun_sensor_sample_time_s,sun_sensor_valid,"
        << "css_pos_x,css_neg_x,css_pos_y,css_neg_y,"
        << "css_pos_z,css_neg_z,"
        << "dipole_command_x_body_A_m2,dipole_command_y_body_A_m2,"
        << "dipole_command_z_body_A_m2,"
        << "dipole_applied_x_body_A_m2,dipole_applied_y_body_A_m2,"
        << "dipole_applied_z_body_A_m2,"
        << "torque_x_body_Nm,torque_y_body_Nm,torque_z_body_Nm,"
        << "flight_mode,detumble_complete\n";
}

void write_csv_row(
    std::ostream& output,
    const detumble::Simulation& simulation,
    const detumble::EnvironmentState& environment,
    const detumble::MagneticControlOutput& magnetic_output,
    const detumble::AutonomousFlightSoftware& flight_software,
    const detumble::ReactionWheelTelemetry& wheel_telemetry,
    const std::optional<detumble::CoarseSunSensorMeasurement>&
        sun_sensor_measurement
) {
    const detumble::AutonomousFlightSoftwareState& flight_state =
        flight_software.state();
    const detumble::AttitudeEstimate& attitude_estimate =
        flight_software.attitude_estimator().estimate();
    const detumble::SunPointingCommand& sun_pointing =
        flight_state.sun_pointing;
    const detumble::SunPointingTrackerState& sun_tracking =
        flight_software.sun_pointing_tracker().state();
    const detumble::AttitudeState& state = simulation.state();
    const Eigen::Vector3d& rate = state.angular_velocity_body_rad_s;
    const Eigen::Vector3d& field = environment.magnetic_field_body_T;
    const Eigen::Vector3d& command =
        magnetic_output.commanded_dipole_body_A_m2;
    const Eigen::Vector3d& applied =
        magnetic_output.applied_dipole_body_A_m2;
    const Eigen::Vector3d& torque = magnetic_output.applied_torque_body_Nm;
    const bool sun_sensor_valid = sun_sensor_measurement.has_value()
        && sun_sensor_measurement->sun_direction_body.has_value();
    const std::array<double, 6> sun_sensor_illumination =
        sun_sensor_measurement.has_value()
        ? sun_sensor_measurement->illumination
        : std::array<double, 6>{};
    constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
    const double attitude_error_deg = attitude_estimate.valid
        ? radians_to_degrees * detumble::attitude_error_angle_rad(
              attitude_estimate.body_to_inertial,
              state.body_to_inertial
          )
        : -1.0;

    output << simulation.elapsed_time_s() << ','
           << state.body_to_inertial.w() << ','
           << state.body_to_inertial.x() << ','
           << state.body_to_inertial.y() << ','
           << state.body_to_inertial.z() << ','
           << attitude_estimate.body_to_inertial.w() << ','
           << attitude_estimate.body_to_inertial.x() << ','
           << attitude_estimate.body_to_inertial.y() << ','
           << attitude_estimate.body_to_inertial.z() << ','
           << attitude_estimate.sample_time_s << ','
           << (attitude_estimate.valid ? 1 : 0) << ','
           << attitude_error_deg << ','
           << detumble::to_string(attitude_estimate.correction_status) << ','
           << attitude_estimate.accepted_correction_count << ','
           << (sun_pointing.active ? 1 : 0) << ','
           << sun_pointing.pointing_error_rad * radians_to_degrees << ','
           << sun_pointing.requested_torque_body_Nm.x() << ','
           << sun_pointing.requested_torque_body_Nm.y() << ','
           << sun_pointing.requested_torque_body_Nm.z() << ','
           << sun_pointing.limited_torque_body_Nm.x() << ','
           << sun_pointing.limited_torque_body_Nm.y() << ','
           << sun_pointing.limited_torque_body_Nm.z() << ','
           << (sun_tracking.target_acquired ? 1 : 0) << ','
           << (sun_tracking.steady_pointing ? 1 : 0) << ','
           << wheel_telemetry.applied_spacecraft_torque_body_Nm.x() << ','
           << wheel_telemetry.applied_spacecraft_torque_body_Nm.y() << ','
           << wheel_telemetry.applied_spacecraft_torque_body_Nm.z() << ','
           << wheel_telemetry.allocation_error_body_Nm.x() << ','
           << wheel_telemetry.allocation_error_body_Nm.y() << ','
           << wheel_telemetry.allocation_error_body_Nm.z() << ','
           << (wheel_telemetry.allocation_saturated ? 1 : 0) << ',';
    for (std::size_t index = 0;
         index < wheel_telemetry.wheel_speed_rad_s.size();
         ++index) {
        output << wheel_telemetry.wheel_speed_rad_s[index] << ','
               << wheel_telemetry.commanded_motor_torque_Nm[index] << ','
               << wheel_telemetry.applied_motor_torque_Nm[index] << ','
               << (wheel_telemetry.torque_saturated[index] ? 1 : 0) << ','
               << (wheel_telemetry.speed_saturated[index] ? 1 : 0) << ','
               << (wheel_telemetry.failed[index] ? 1 : 0) << ',';
    }
    output << rate.x() << ',' << rate.y() << ',' << rate.z() << ','
           << simulation.metrics().rotational_kinetic_energy_j << ','
           << field.x() << ',' << field.y() << ',' << field.z() << ','
           << environment.sun_direction_inertial.x() << ','
           << environment.sun_direction_inertial.y() << ','
           << environment.sun_direction_inertial.z() << ','
           << environment.sun_direction_body.x() << ','
           << environment.sun_direction_body.y() << ','
           << environment.sun_direction_body.z() << ','
           << (environment.in_eclipse ? 1 : 0) << ','
           << (sun_sensor_measurement.has_value()
                   ? sun_sensor_measurement->sample_time_s
                   : -1.0)
           << ','
           << (sun_sensor_valid ? 1 : 0) << ','
           << sun_sensor_illumination[0] << ','
           << sun_sensor_illumination[1] << ','
           << sun_sensor_illumination[2] << ','
           << sun_sensor_illumination[3] << ','
           << sun_sensor_illumination[4] << ','
           << sun_sensor_illumination[5] << ','
           << command.x() << ',' << command.y() << ',' << command.z() << ','
           << applied.x() << ',' << applied.y() << ',' << applied.z() << ','
           << torque.x() << ',' << torque.y() << ',' << torque.z() << ','
           << detumble::to_string(flight_state.mode) << ','
           << (flight_software.detumble().state().detumble_complete ? 1 : 0)
           << '\n';
}

int run_native(const CliOptions& options) {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
    const double initial_rate_rad_s =
        options.initial_rate_deg_s * degrees_to_radians;
    detumble::Simulation simulation{
        detumble::SimulationConfig{
            .seed = options.seed,
            .time_step_s = options.time_step_s,
            .minimum_angular_speed_rad_s = initial_rate_rad_s,
            .maximum_angular_speed_rad_s = initial_rate_rad_s
        }
    };
    const detumble::EnvironmentConfig environment_config;
    const detumble::SensorSuiteConfig sensors = sensor_config(options);
    detumble::MagneticControlCycle magnetic_control{sensors.magnetometer};
    detumble::IdealGyroscope gyroscope{sensors.gyroscope};
    detumble::IdealCoarseSunSensorArray sun_sensors{sensors.sun_sensor};
    detumble::AutonomousFlightSoftware flight_software;
    detumble::ReactionWheelCluster reaction_wheels;
    double maximum_wheel_speed_rad_s = 0.0;
    const double initial_energy_j =
        simulation.metrics().rotational_kinetic_energy_j;

    std::ofstream csv;
    if (options.output_path.has_value()) {
        const std::filesystem::path output_path{*options.output_path};
        if (output_path.has_parent_path()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        csv.open(output_path);
        if (!csv) {
            throw std::runtime_error{
                "Could not open telemetry output: " + *options.output_path
            };
        }
        csv << std::setprecision(17);
        write_csv_header(csv, reaction_wheels.config());
    }

    detumble::MagneticControlOutput magnetic_output;
    while (simulation.elapsed_time_s() + 0.5 * options.time_step_s
           < options.duration_s) {
        const detumble::EnvironmentState environment =
            detumble::sample_environment(
                environment_config,
                simulation.state().body_to_inertial,
                simulation.elapsed_time_s()
            );
        const std::optional<detumble::GyroscopeMeasurement>
            gyroscope_measurement = gyroscope.sample_if_due(
                simulation.elapsed_time_s(),
                simulation.state().angular_velocity_body_rad_s
            );
        const std::optional<detumble::CoarseSunSensorMeasurement>
            sun_sensor_measurement = sun_sensors.sample_if_due(
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
        reaction_wheels.update(
            detumble::ReactionWheelCommand{
                .sample_time_s = simulation.elapsed_time_s(),
                .requested_spacecraft_torque_body_Nm =
                    flight_software.state().actuator_command
                        .reaction_wheel_torque_body_Nm,
                .enabled = flight_software.state().actuator_command
                    .reaction_wheels_enabled
            },
            simulation.config().time_step_s
        );
        for (const double wheel_speed_rad_s :
             reaction_wheels.telemetry().wheel_speed_rad_s) {
            maximum_wheel_speed_rad_s = std::max(
                maximum_wheel_speed_rad_s,
                std::abs(wheel_speed_rad_s)
            );
        }
        simulation.step(
            magnetic_output.applied_torque_body_Nm
            + reaction_wheels.telemetry()
                  .applied_spacecraft_torque_body_Nm
        );

        if (csv) {
            write_csv_row(
                csv,
                simulation,
                detumble::sample_environment(
                    environment_config,
                    simulation.state().body_to_inertial,
                    simulation.elapsed_time_s()
                ),
                magnetic_output,
                flight_software,
                reaction_wheels.telemetry(),
                sun_sensors.last_measurement()
            );
        }
    }

    const detumble::RotationalMetrics final_metrics = simulation.metrics();
    std::cout << std::setprecision(10)
              << "Detumble " << detumble::version() << '\n'
              << "Scenario: autonomous detumble and reaction-wheel Sun pointing\n"
              << "Seed: " << options.seed << '\n'
              << "Sensors: " << options.sensor_profile << '\n'
              << "Time: " << simulation.elapsed_time_s() << " s\n"
              << "Initial angular speed: " << options.initial_rate_deg_s
              << " deg/s\n"
              << "Final angular speed: "
              << simulation.state().angular_velocity_body_rad_s.norm()
                     * radians_to_degrees
              << " deg/s\n"
              << "Initial rotational energy: " << initial_energy_j << " J\n"
              << "Final rotational energy: "
              << final_metrics.rotational_kinetic_energy_j << " J\n"
              << "Flight mode: "
              << detumble::to_string(flight_software.state().mode) << '\n'
              << "Detumble complete: "
              << (flight_software.detumble().state().detumble_complete
                      ? "yes"
                      : "no")
              << '\n'
              << "Attitude estimate: "
              << (flight_software.attitude_estimator().estimate().valid
                      ? "valid"
                      : "invalid")
              << '\n';
    if (flight_software.attitude_estimator().estimate().valid) {
        std::cout << "Final attitude error: "
                  << radians_to_degrees
                         * detumble::attitude_error_angle_rad(
                             flight_software.attitude_estimator()
                                 .estimate().body_to_inertial,
                             simulation.state().body_to_inertial
                         )
                  << " deg\n";
    }
    if (flight_software.state().sun_pointing.active) {
        std::cout << "Final Sun-pointing error: "
                  << flight_software.state().sun_pointing.pointing_error_rad
                         * radians_to_degrees
                  << " deg\n"
                  << "Steady Sun pointing: "
                  << (flight_software.sun_pointing_tracker()
                              .state().steady_pointing
                          ? "yes"
                          : "no")
                  << '\n'
                  << "Maximum wheel speed: "
                  << maximum_wheel_speed_rad_s * 60.0
                         / (2.0 * std::numbers::pi)
                  << " rpm\n"
                  << "Wheel allocation error: "
                  << reaction_wheels.telemetry()
                         .allocation_error_body_Nm.norm()
                  << " N m\n";
    }
    std::cout << "Mode transitions: " << flight_software.events().size()
              << '\n';
    for (const detumble::ModeTransitionEvent& event :
         flight_software.events()) {
        std::cout << "  " << event.sample_time_s << " s: "
                  << detumble::to_string(event.old_mode) << " -> "
                  << detumble::to_string(event.new_mode) << " ("
                  << detumble::to_string(event.reason) << ")\n";
    }
    if (options.output_path.has_value()) {
        std::cout << "Telemetry: " << *options.output_path << '\n';
    }
    return 0;
}

}  // namespace

int main(const int argument_count, char** arguments) {
    try {
        const CliOptions options = parse_options(argument_count, arguments);
        if (options.show_help) {
            print_usage(std::cout);
            return 0;
        }
        return run_native(options);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n\n";
        print_usage(std::cerr);
        return 2;
    }
}
