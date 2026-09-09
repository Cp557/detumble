#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <imgui.h>
#include <implot.h>
#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include "detumble/frames.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/orbit.hpp"
#include "detumble/reaction_wheel.hpp"
#include "detumble/sensor_suite.hpp"
#include "detumble/simulation.hpp"
#include "detumble/spacecraft_visual.hpp"
#include "detumble/sun_pointing.hpp"
#include "detumble/version.hpp"

namespace {

constexpr std::size_t telemetry_capacity = 400;
constexpr std::size_t replay_capacity = 60'001;
constexpr double telemetry_sample_period_s = 0.1;
constexpr double telemetry_schedule_tolerance_s = 1.0e-12;
constexpr double plot_window_s = 30.0;
constexpr double meters_to_kilometers = 1.0e-3;
constexpr double tesla_to_microtesla = 1.0e6;
constexpr double earth_display_radius = 2.0;
constexpr double meters_to_spacecraft_display_units = 10.0;
constexpr double vector_overlay_smoothing_time_s = 0.18;
constexpr double vector_overlay_visibility_fraction = 0.02;
constexpr double vector_overlay_minimum_length_fraction = 0.25;
constexpr Color angular_velocity_color{255, 125, 35, 255};
constexpr Color magnetic_dipole_color{235, 75, 220, 255};
constexpr Color control_torque_color{45, 220, 225, 255};
constexpr Color sun_direction_color{255, 225, 95, 255};

ImVec2 available_plot_size() {
    return {
        -1.0F,
        std::max(180.0F, ImGui::GetContentRegionAvail().y)
    };
}

Vector3 to_raylib(const Eigen::Vector3d& vector) {
    return {
        static_cast<float>(vector.x()),
        static_cast<float>(vector.y()),
        static_cast<float>(vector.z())
    };
}

struct SmoothedVectorOverlay {
    void reset() {
        displayed_body.setZero();
        initialized = false;
    }

    void update(
        const Eigen::Vector3d& target_body,
        const double frame_time_s
    ) {
        if (!initialized) {
            displayed_body = target_body;
            initialized = true;
            return;
        }
        const double bounded_frame_time_s = std::clamp(
            frame_time_s,
            0.0,
            0.1
        );
        const double blend = 1.0 - std::exp(
            -bounded_frame_time_s / vector_overlay_smoothing_time_s
        );
        displayed_body += blend * (target_body - displayed_body);
    }

    Eigen::Vector3d displayed_body{Eigen::Vector3d::Zero()};
    bool initialized{};
};

double scaled_vector_overlay_length(
    const Eigen::Vector3d& vector,
    const double reference_magnitude,
    const double maximum_length
) {
    if (!std::isfinite(reference_magnitude) || reference_magnitude <= 0.0
        || !std::isfinite(maximum_length) || maximum_length <= 0.0) {
        return 0.0;
    }
    const double magnitude_fraction = std::clamp(
        vector.norm() / reference_magnitude,
        0.0,
        1.0
    );
    if (magnitude_fraction < vector_overlay_visibility_fraction) {
        return 0.0;
    }
    const double length_fraction = std::max(
        magnitude_fraction,
        vector_overlay_minimum_length_fraction
    );
    return maximum_length * length_fraction;
}

struct TelemetryHistory {
    std::vector<double> time_s;
    std::vector<double> true_body_rate_x_deg_s;
    std::vector<double> true_body_rate_y_deg_s;
    std::vector<double> true_body_rate_z_deg_s;
    std::vector<double> measured_body_rate_x_deg_s;
    std::vector<double> measured_body_rate_y_deg_s;
    std::vector<double> measured_body_rate_z_deg_s;
    std::vector<double> position_x_inertial_km;
    std::vector<double> position_y_inertial_km;
    std::vector<double> position_z_inertial_km;
    std::vector<double> magnetic_field_x_body_uT;
    std::vector<double> magnetic_field_y_body_uT;
    std::vector<double> magnetic_field_z_body_uT;
    std::vector<double> measured_magnetic_field_x_body_uT;
    std::vector<double> measured_magnetic_field_y_body_uT;
    std::vector<double> measured_magnetic_field_z_body_uT;
    std::array<std::vector<double>, 6> sun_sensor_illumination;
    std::vector<double> sun_sensor_valid_value;
    std::vector<double> eclipse_value;
    std::vector<double> commanded_dipole_x_body_A_m2;
    std::vector<double> commanded_dipole_y_body_A_m2;
    std::vector<double> commanded_dipole_z_body_A_m2;
    std::vector<double> limited_dipole_x_body_A_m2;
    std::vector<double> limited_dipole_y_body_A_m2;
    std::vector<double> limited_dipole_z_body_A_m2;
    std::vector<double> rotational_energy_j;
    std::vector<double> flight_mode_value;
    std::vector<double> detumble_complete_value;
    std::vector<double> quaternion_norm_error;
    std::vector<double> attitude_error_deg;
    std::vector<double> sun_pointing_error_deg;
    std::vector<double> sun_pointing_torque_x_mNm;
    std::vector<double> sun_pointing_torque_y_mNm;
    std::vector<double> sun_pointing_torque_z_mNm;
    std::array<std::vector<double>, 4> wheel_speed_rpm;
    std::array<std::vector<double>, 4> wheel_motor_torque_mNm;
    std::vector<double> wheel_allocation_error_mNm;

    void clear() {
        time_s.clear();
        true_body_rate_x_deg_s.clear();
        true_body_rate_y_deg_s.clear();
        true_body_rate_z_deg_s.clear();
        measured_body_rate_x_deg_s.clear();
        measured_body_rate_y_deg_s.clear();
        measured_body_rate_z_deg_s.clear();
        position_x_inertial_km.clear();
        position_y_inertial_km.clear();
        position_z_inertial_km.clear();
        magnetic_field_x_body_uT.clear();
        magnetic_field_y_body_uT.clear();
        magnetic_field_z_body_uT.clear();
        measured_magnetic_field_x_body_uT.clear();
        measured_magnetic_field_y_body_uT.clear();
        measured_magnetic_field_z_body_uT.clear();
        for (auto& illumination : sun_sensor_illumination) {
            illumination.clear();
        }
        sun_sensor_valid_value.clear();
        eclipse_value.clear();
        commanded_dipole_x_body_A_m2.clear();
        commanded_dipole_y_body_A_m2.clear();
        commanded_dipole_z_body_A_m2.clear();
        limited_dipole_x_body_A_m2.clear();
        limited_dipole_y_body_A_m2.clear();
        limited_dipole_z_body_A_m2.clear();
        rotational_energy_j.clear();
        flight_mode_value.clear();
        detumble_complete_value.clear();
        quaternion_norm_error.clear();
        attitude_error_deg.clear();
        sun_pointing_error_deg.clear();
        sun_pointing_torque_x_mNm.clear();
        sun_pointing_torque_y_mNm.clear();
        sun_pointing_torque_z_mNm.clear();
        for (auto& speed : wheel_speed_rpm) {
            speed.clear();
        }
        for (auto& torque : wheel_motor_torque_mNm) {
            torque.clear();
        }
        wheel_allocation_error_mNm.clear();
    }

    void append(
        const detumble::Simulation& simulation,
        const detumble::EnvironmentState& environment,
        const detumble::MagneticControlOutput& magnetic_control,
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
        const double sample_time_s = simulation.elapsed_time_s();
        if (!time_s.empty()
            && sample_time_s + telemetry_schedule_tolerance_s
                < time_s.back() + telemetry_sample_period_s) {
            return;
        }

        if (time_s.size() == telemetry_capacity) {
            constexpr std::size_t discard_count = telemetry_capacity / 4;
            erase_first(time_s, discard_count);
            erase_first(true_body_rate_x_deg_s, discard_count);
            erase_first(true_body_rate_y_deg_s, discard_count);
            erase_first(true_body_rate_z_deg_s, discard_count);
            erase_first(measured_body_rate_x_deg_s, discard_count);
            erase_first(measured_body_rate_y_deg_s, discard_count);
            erase_first(measured_body_rate_z_deg_s, discard_count);
            erase_first(position_x_inertial_km, discard_count);
            erase_first(position_y_inertial_km, discard_count);
            erase_first(position_z_inertial_km, discard_count);
            erase_first(magnetic_field_x_body_uT, discard_count);
            erase_first(magnetic_field_y_body_uT, discard_count);
            erase_first(magnetic_field_z_body_uT, discard_count);
            erase_first(measured_magnetic_field_x_body_uT, discard_count);
            erase_first(measured_magnetic_field_y_body_uT, discard_count);
            erase_first(measured_magnetic_field_z_body_uT, discard_count);
            for (auto& illumination : sun_sensor_illumination) {
                erase_first(illumination, discard_count);
            }
            erase_first(sun_sensor_valid_value, discard_count);
            erase_first(eclipse_value, discard_count);
            erase_first(commanded_dipole_x_body_A_m2, discard_count);
            erase_first(commanded_dipole_y_body_A_m2, discard_count);
            erase_first(commanded_dipole_z_body_A_m2, discard_count);
            erase_first(limited_dipole_x_body_A_m2, discard_count);
            erase_first(limited_dipole_y_body_A_m2, discard_count);
            erase_first(limited_dipole_z_body_A_m2, discard_count);
            erase_first(rotational_energy_j, discard_count);
            erase_first(flight_mode_value, discard_count);
            erase_first(detumble_complete_value, discard_count);
            erase_first(quaternion_norm_error, discard_count);
            erase_first(attitude_error_deg, discard_count);
            erase_first(sun_pointing_error_deg, discard_count);
            erase_first(sun_pointing_torque_x_mNm, discard_count);
            erase_first(sun_pointing_torque_y_mNm, discard_count);
            erase_first(sun_pointing_torque_z_mNm, discard_count);
            for (auto& speed : wheel_speed_rpm) {
                erase_first(speed, discard_count);
            }
            for (auto& torque : wheel_motor_torque_mNm) {
                erase_first(torque, discard_count);
            }
            erase_first(wheel_allocation_error_mNm, discard_count);
        }

        const Eigen::Vector3d& rate =
            simulation.state().angular_velocity_body_rad_s;
        const Eigen::Vector3d position_km =
            meters_to_kilometers * environment.orbit.position_inertial_m;
        const Eigen::Vector3d magnetic_field_body_uT =
            tesla_to_microtesla * environment.magnetic_field_body_T;
        constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
        const Eigen::Vector3d measured_rate = flight_software.detumble()
            .state().measured_angular_velocity_body_rad_s;
        time_s.push_back(sample_time_s);
        true_body_rate_x_deg_s.push_back(radians_to_degrees * rate.x());
        true_body_rate_y_deg_s.push_back(radians_to_degrees * rate.y());
        true_body_rate_z_deg_s.push_back(radians_to_degrees * rate.z());
        measured_body_rate_x_deg_s.push_back(
            radians_to_degrees * measured_rate.x()
        );
        measured_body_rate_y_deg_s.push_back(
            radians_to_degrees * measured_rate.y()
        );
        measured_body_rate_z_deg_s.push_back(
            radians_to_degrees * measured_rate.z()
        );
        position_x_inertial_km.push_back(position_km.x());
        position_y_inertial_km.push_back(position_km.y());
        position_z_inertial_km.push_back(position_km.z());
        magnetic_field_x_body_uT.push_back(magnetic_field_body_uT.x());
        magnetic_field_y_body_uT.push_back(magnetic_field_body_uT.y());
        magnetic_field_z_body_uT.push_back(magnetic_field_body_uT.z());
        Eigen::Vector3d measured_field_body_uT = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN()
        );
        if (magnetic_control.magnetometer_measurement.has_value()) {
            measured_field_body_uT = tesla_to_microtesla
                * magnetic_control.magnetometer_measurement
                      ->magnetic_field_body_T;
        }
        measured_magnetic_field_x_body_uT.push_back(
            measured_field_body_uT.x()
        );
        measured_magnetic_field_y_body_uT.push_back(
            measured_field_body_uT.y()
        );
        measured_magnetic_field_z_body_uT.push_back(
            measured_field_body_uT.z()
        );
        for (std::size_t index = 0;
             index < sun_sensor_illumination.size();
             ++index) {
            sun_sensor_illumination[index].push_back(
                sun_sensor_measurement.has_value()
                ? sun_sensor_measurement->illumination[index]
                : 0.0
            );
        }
        sun_sensor_valid_value.push_back(
            sun_sensor_measurement.has_value()
                && sun_sensor_measurement->sun_direction_body.has_value()
            ? 1.0
            : 0.0
        );
        eclipse_value.push_back(environment.in_eclipse ? 1.0 : 0.0);
        commanded_dipole_x_body_A_m2.push_back(
            magnetic_control.commanded_dipole_body_A_m2.x()
        );
        commanded_dipole_y_body_A_m2.push_back(
            magnetic_control.commanded_dipole_body_A_m2.y()
        );
        commanded_dipole_z_body_A_m2.push_back(
            magnetic_control.commanded_dipole_body_A_m2.z()
        );
        limited_dipole_x_body_A_m2.push_back(
            magnetic_control.limited_dipole_body_A_m2.x()
        );
        limited_dipole_y_body_A_m2.push_back(
            magnetic_control.limited_dipole_body_A_m2.y()
        );
        limited_dipole_z_body_A_m2.push_back(
            magnetic_control.limited_dipole_body_A_m2.z()
        );
        rotational_energy_j.push_back(
            simulation.metrics().rotational_kinetic_energy_j
        );
        flight_mode_value.push_back(static_cast<double>(flight_state.mode));
        detumble_complete_value.push_back(
            flight_software.detumble().state().detumble_complete ? 1.0 : 0.0
        );
        quaternion_norm_error.push_back(
            simulation.metrics().quaternion_norm - 1.0
        );
        attitude_error_deg.push_back(
            attitude_estimate.valid
            ? radians_to_degrees * detumble::attitude_error_angle_rad(
                  attitude_estimate.body_to_inertial,
                  simulation.state().body_to_inertial
              )
            : std::numeric_limits<double>::quiet_NaN()
        );
        sun_pointing_error_deg.push_back(
            sun_pointing.active
            ? sun_pointing.pointing_error_rad * radians_to_degrees
            : std::numeric_limits<double>::quiet_NaN()
        );
        constexpr double newton_meters_to_millinewton_meters = 1.0e3;
        const Eigen::Vector3d sun_pointing_torque_mNm =
            newton_meters_to_millinewton_meters
            * wheel_telemetry.applied_spacecraft_torque_body_Nm;
        sun_pointing_torque_x_mNm.push_back(sun_pointing_torque_mNm.x());
        sun_pointing_torque_y_mNm.push_back(sun_pointing_torque_mNm.y());
        sun_pointing_torque_z_mNm.push_back(sun_pointing_torque_mNm.z());
        constexpr double radians_per_second_to_rpm =
            60.0 / (2.0 * std::numbers::pi);
        for (std::size_t index = 0; index < wheel_speed_rpm.size(); ++index) {
            const bool present =
                index < wheel_telemetry.wheel_speed_rad_s.size();
            wheel_speed_rpm[index].push_back(
                present
                ? radians_per_second_to_rpm
                    * wheel_telemetry.wheel_speed_rad_s[index]
                : 0.0
            );
            wheel_motor_torque_mNm[index].push_back(
                present
                ? newton_meters_to_millinewton_meters
                    * wheel_telemetry.applied_motor_torque_Nm[index]
                : 0.0
            );
        }
        wheel_allocation_error_mNm.push_back(
            newton_meters_to_millinewton_meters
            * wheel_telemetry.allocation_error_body_Nm.norm()
        );
    }

private:
    static void erase_first(
        std::vector<double>& values,
        const std::size_t count
    ) {
        values.erase(
            values.begin(),
            values.begin() + static_cast<std::ptrdiff_t>(count)
        );
    }
};

struct ViewerOptions {
    enum class SpacecraftRenderMode {
        solid,
        transparent,
        cutaway
    };

    bool orbit_overview{};
    bool show_body_axes{};
    bool show_magnetic_field{};
    bool show_angular_velocity{true};
    bool show_magnetic_dipole{};
    bool show_control_torque{};
    bool show_sun_direction{true};
    bool show_orbit_path{true};
    SpacecraftRenderMode spacecraft_render_mode{SpacecraftRenderMode::solid};
};

struct UiVisibility {
    bool show_telemetry{};
    bool show_diagnostics{};
};

struct ViewerScenario {
    const char* name;
    const char* description;
    std::uint64_t seed;
    double initial_rate_deg_s;
    float suggested_speed;
};

constexpr std::array<ViewerScenario, 3> viewer_scenarios{{
    {
        "Nominal mission",
        "10 deg/s tumble with realistic seeded sensors",
        42,
        10.0,
        50.0F
    },
    {
        "Gentle tumble",
        "5 deg/s starting rate for a shorter detumble",
        7,
        5.0,
        25.0F
    },
    {
        "Fast tumble",
        "15 deg/s upper design case",
        2025,
        15.0,
        100.0F
    }
}};

struct RecordedFrame {
    double time_s{};
    Eigen::Quaterniond body_to_inertial{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_velocity_body_rad_s{Eigen::Vector3d::Zero()};
    detumble::EnvironmentState environment;
    Eigen::Vector3d limited_dipole_body_A_m2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d applied_control_torque_body_Nm{Eigen::Vector3d::Zero()};
    detumble::FlightMode mode{detumble::FlightMode::boot};
    double sun_pointing_error_deg{std::numeric_limits<double>::quiet_NaN()};
    bool detumble_complete{};
    bool sun_available{};
    bool attitude_valid{};
    bool magnetorquers_enabled{};
    bool reaction_wheels_enabled{};
    std::optional<detumble::CoarseSunSensorMeasurement> sun_measurement;
};

struct ReplayBuffer {
    void clear() {
        frames.clear();
    }

    [[nodiscard]] static RecordedFrame capture(
        const detumble::Simulation& simulation,
        const detumble::EnvironmentState& environment,
        const detumble::MagneticControlOutput& magnetic_output,
        const detumble::AutonomousFlightSoftware& flight_software,
        const detumble::ReactionWheelTelemetry& wheel_telemetry,
        const std::optional<detumble::CoarseSunSensorMeasurement>&
            sun_measurement
    ) {
        const detumble::AutonomousFlightSoftwareState& flight_state =
            flight_software.state();
        return RecordedFrame{
            .time_s = simulation.elapsed_time_s(),
            .body_to_inertial = simulation.state().body_to_inertial,
            .angular_velocity_body_rad_s =
                simulation.state().angular_velocity_body_rad_s,
            .environment = environment,
            .limited_dipole_body_A_m2 =
                magnetic_output.limited_dipole_body_A_m2,
            .applied_control_torque_body_Nm =
                magnetic_output.applied_torque_body_Nm
                + wheel_telemetry.applied_spacecraft_torque_body_Nm,
            .mode = flight_state.mode,
            .sun_pointing_error_deg = flight_state.sun_pointing.active
                ? flight_state.sun_pointing.pointing_error_rad
                    * 180.0 / std::numbers::pi
                : std::numeric_limits<double>::quiet_NaN(),
            .detumble_complete =
                flight_software.detumble().state().detumble_complete,
            .sun_available = flight_state.sun_available,
            .attitude_valid = flight_state.attitude_valid,
            .magnetorquers_enabled =
                flight_state.actuator_command.magnetorquers_enabled,
            .reaction_wheels_enabled =
                flight_state.actuator_command.reaction_wheels_enabled,
            .sun_measurement = sun_measurement
        };
    }

    void append(
        const detumble::Simulation& simulation,
        const detumble::EnvironmentState& environment,
        const detumble::MagneticControlOutput& magnetic_output,
        const detumble::AutonomousFlightSoftware& flight_software,
        const detumble::ReactionWheelTelemetry& wheel_telemetry,
        const std::optional<detumble::CoarseSunSensorMeasurement>&
            sun_measurement
    ) {
        const double sample_time_s = simulation.elapsed_time_s();
        if (!frames.empty()
            && sample_time_s + telemetry_schedule_tolerance_s
                < frames.back().time_s + telemetry_sample_period_s) {
            return;
        }
        if (frames.size() == replay_capacity) {
            frames.erase(frames.begin(), frames.begin() + replay_capacity / 10);
        }
        frames.push_back(capture(
            simulation,
            environment,
            magnetic_output,
            flight_software,
            wheel_telemetry,
            sun_measurement
        ));
    }

    std::vector<RecordedFrame> frames;
};

struct ReplayState {
    void start(const ReplayBuffer& replay) {
        active = !replay.frames.empty();
        playing = replay.frames.size() > 1;
        frame_index = 0;
        accumulator_s = 0.0;
    }

    void stop() {
        active = false;
        playing = false;
        accumulator_s = 0.0;
    }

    void update(const ReplayBuffer& replay, const double wall_time_s) {
        if (!active || !playing || replay.frames.size() < 2) {
            return;
        }
        accumulator_s += wall_time_s * static_cast<double>(speed);
        while (frame_index + 1 < replay.frames.size()) {
            const double interval_s = replay.frames[frame_index + 1].time_s
                - replay.frames[frame_index].time_s;
            if (accumulator_s + telemetry_schedule_tolerance_s < interval_s) {
                break;
            }
            accumulator_s = std::max(0.0, accumulator_s - interval_s);
            ++frame_index;
        }
        if (frame_index + 1 == replay.frames.size()) {
            playing = false;
            accumulator_s = 0.0;
        }
    }

    [[nodiscard]] const RecordedFrame& frame(
        const ReplayBuffer& replay
    ) const {
        return replay.frames.at(frame_index);
    }

    bool active{};
    bool playing{};
    std::size_t frame_index{};
    float speed{1.0F};
    double accumulator_s{};
};

struct ViewerPerformance {
    void update(
        const double frame_time_s,
        const std::uint64_t simulated_steps,
        const double simulation_wall_time_s,
        const double fixed_step_s
    ) {
        elapsed_wall_s += frame_time_s;
        frame_count += 1;
        step_count += simulated_steps;
        simulation_wall_s += simulation_wall_time_s;
        if (elapsed_wall_s >= 0.5) {
            frames_per_second = static_cast<double>(frame_count)
                / elapsed_wall_s;
            steps_per_second = static_cast<double>(step_count)
                / elapsed_wall_s;
            real_time_factor = simulation_wall_s > 0.0
                ? static_cast<double>(step_count) * fixed_step_s
                    / simulation_wall_s
                : 0.0;
            elapsed_wall_s = 0.0;
            frame_count = 0;
            step_count = 0;
            simulation_wall_s = 0.0;
        }
    }

    double elapsed_wall_s{};
    std::uint64_t frame_count{};
    std::uint64_t step_count{};
    double simulation_wall_s{};
    double frames_per_second{};
    double steps_per_second{};
    double real_time_factor{};
};

detumble::EnvironmentState current_environment(
    const detumble::Simulation& simulation,
    const detumble::EnvironmentConfig& config
) {
    return detumble::sample_environment(
        config,
        simulation.state().body_to_inertial,
        simulation.elapsed_time_s()
    );
}

detumble::MagneticControlOutput advance_simulation(
    detumble::Simulation& simulation,
    const detumble::EnvironmentConfig& environment_config,
    detumble::MagneticControlCycle& magnetic_control,
    detumble::IdealGyroscope& gyroscope,
    detumble::IdealCoarseSunSensorArray& sun_sensors,
    detumble::AutonomousFlightSoftware& flight_software,
    detumble::ReactionWheelCluster& reaction_wheels,
    const Eigen::Vector3d& manual_dipole_body_A_m2,
    const bool automatic_bdot
) {
    const detumble::EnvironmentState environment = current_environment(
        simulation,
        environment_config
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
    const Eigen::Vector3d& commanded_dipole_body_A_m2 = automatic_bdot
        ? flight_software.state().actuator_command
              .magnetorquer_dipole_body_A_m2
        : manual_dipole_body_A_m2;
    detumble::MagneticControlOutput output = magnetic_control.update(
        simulation.elapsed_time_s(),
        environment.magnetic_field_body_T,
        commanded_dipole_body_A_m2
    );
    if (gyroscope_measurement.has_value()) {
        flight_software.update(detumble::AutonomousFlightSoftwareInput{
            .gyroscope = *gyroscope_measurement,
            .magnetometer = output.magnetometer_measurement,
            .sun_sensor = sun_sensor_measurement,
            .magnetic_field_inertial_T =
                environment.magnetic_field_inertial_T,
            .sun_direction_inertial = environment.sun_direction_inertial,
            .in_eclipse = environment.in_eclipse
        });
    }
    const Eigen::Vector3d& wheel_request = flight_software.state()
        .actuator_command.reaction_wheel_torque_body_Nm;
    const bool wheels_enabled = flight_software.state()
        .actuator_command.reaction_wheels_enabled;
    reaction_wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = simulation.elapsed_time_s(),
            .requested_spacecraft_torque_body_Nm = wheel_request,
            .enabled = wheels_enabled
        },
        simulation.config().time_step_s
    );
    simulation.step(
        output.applied_torque_body_Nm
        + reaction_wheels.telemetry().applied_spacecraft_torque_body_Nm
    );
    return output;
}

struct OrbitCamera {
    float azimuth_rad{0.75F};
    float elevation_rad{0.35F};
    float radius{7.0F};

    [[nodiscard]] Camera3D camera() const {
        const float horizontal_radius = radius * std::cos(elevation_rad);
        return {
            .position = {
                horizontal_radius * std::cos(azimuth_rad),
                radius * std::sin(elevation_rad),
                horizontal_radius * std::sin(azimuth_rad)
            },
            .target = {0.0F, 0.0F, 0.0F},
            .up = {0.0F, 1.0F, 0.0F},
            .fovy = 45.0F,
            .projection = CAMERA_PERSPECTIVE
        };
    }

    void close_up() {
        azimuth_rad = 0.75F;
        elevation_rad = 0.35F;
        radius = 6.0F;
    }

    void cutaway() {
        azimuth_rad = 0.65F;
        elevation_rad = 0.20F;
        radius = 5.5F;
    }

    void orbit_overview() {
        azimuth_rad = 0.75F;
        elevation_rad = 0.45F;
        radius = 7.0F;
    }

    void update() {
        if (ImGui::GetIO().WantCaptureMouse) {
            return;
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            const Vector2 delta = GetMouseDelta();
            azimuth_rad -= delta.x * 0.008F;
            elevation_rad = std::clamp(
                elevation_rad + delta.y * 0.008F,
                -1.35F,
                1.35F
            );
        }

        const float wheel = GetMouseWheelMove();
        radius = std::clamp(
            radius * std::exp(-wheel * 0.12F),
            4.5F,
            14.0F
        );
    }
};

struct SpacecraftModel {
    Model model{};
    float uniform_display_scale{1.0F};
    bool loaded_custom_glb{};
};

SpacecraftModel load_spacecraft_model() {
    const std::filesystem::path asset_path =
        std::filesystem::path{GetApplicationDirectory()}
        / "assets/models/detumble_3u.glb";

    SpacecraftModel spacecraft;
    spacecraft.model = LoadModel(asset_path.string().c_str());
    spacecraft.loaded_custom_glb = IsModelValid(spacecraft.model);

    if (!spacecraft.loaded_custom_glb) {
        TraceLog(
            LOG_WARNING,
            "Could not load %s; using a generated cube",
            asset_path.string().c_str()
        );
        spacecraft.model = LoadModelFromMesh(GenMeshCube(1.0F, 1.0F, 1.0F));
    }

    const BoundingBox bounds = GetModelBoundingBox(spacecraft.model);
    const Vector3 dimensions = Vector3Subtract(bounds.max, bounds.min);
    if (spacecraft.loaded_custom_glb) {
        spacecraft.uniform_display_scale =
            static_cast<float>(meters_to_spacecraft_display_units);
        TraceLog(
            LOG_INFO,
            "MODEL: Detumble 3U bounds %.4f x %.4f x %.4f m",
            dimensions.x,
            dimensions.y,
            dimensions.z
        );
        if (dimensions.z <= std::max(dimensions.x, dimensions.y)) {
            TraceLog(
                LOG_WARNING,
                "MODEL: expected body +Z to be the custom model's long axis"
            );
        }
    } else {
        const Vector3 center = Vector3Scale(
            Vector3Add(bounds.min, bounds.max),
            0.5F
        );
        spacecraft.model.transform = MatrixTranslate(
            -center.x,
            -center.y,
            -center.z
        );
    }
    return spacecraft;
}

void draw_spacecraft(
    const SpacecraftModel& spacecraft,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale,
    const ViewerOptions::SpacecraftRenderMode render_mode
) {
    if (render_mode == ViewerOptions::SpacecraftRenderMode::cutaway) {
        return;
    }

    const Quaternion rotation{
        static_cast<float>(body_to_inertial.x()),
        static_cast<float>(body_to_inertial.y()),
        static_cast<float>(body_to_inertial.z()),
        static_cast<float>(body_to_inertial.w())
    };
    Vector3 rotation_axis{};
    float rotation_angle_rad{};
    QuaternionToAxisAngle(rotation, &rotation_axis, &rotation_angle_rad);
    const float rotation_angle_deg = rotation_angle_rad * 180.0F / PI;

    const float model_scale = static_cast<float>(
        spacecraft.uniform_display_scale * visual_scale
    );
    const Vector3 draw_scale = spacecraft.loaded_custom_glb
        ? Vector3{model_scale, model_scale, model_scale}
        : Vector3{model_scale, model_scale, 3.4F * model_scale};
    const Color tint =
        render_mode == ViewerOptions::SpacecraftRenderMode::transparent
        ? Color{255, 255, 255, 80}
        : WHITE;
    DrawModelEx(
        spacecraft.model,
        to_raylib(origin_inertial),
        rotation_axis,
        rotation_angle_deg,
        draw_scale,
        tint
    );
}

void draw_body_outline(
    const detumble::SpacecraftVisualConfig& visual_config,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    const Eigen::Vector3d half_dimensions =
        0.5 * meters_to_spacecraft_display_units
        * visual_config.body_dimensions_m;
    const std::array<Eigen::Vector3d, 8> corners_body{
        Eigen::Vector3d{
            -half_dimensions.x(),
            -half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            -half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            -half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            -half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            half_dimensions.y(),
            half_dimensions.z()
        }
    };
    constexpr std::array<std::array<std::size_t, 2>, 12> edges{
        std::array<std::size_t, 2>{0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    std::array<Vector3, 8> corners_inertial{};
    for (std::size_t index = 0; index < corners_body.size(); ++index) {
        corners_inertial[index] = to_raylib(
            origin_inertial
            + visual_scale * detumble::rotate_body_to_inertial(
                  body_to_inertial,
                  corners_body[index]
              )
        );
    }
    for (const auto& edge : edges) {
        DrawLine3D(
            corners_inertial[edge[0]],
            corners_inertial[edge[1]],
            Color{110, 200, 255, 255}
        );
    }
}

Eigen::Vector3d component_center_inertial(
    const Eigen::Vector3d& center_body_m,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    return origin_inertial
        + visual_scale * meters_to_spacecraft_display_units
            * detumble::rotate_body_to_inertial(
                body_to_inertial,
                center_body_m
            );
}

void draw_configured_rod(
    const detumble::RodComponent& rod,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale,
    const Color color
) {
    const Eigen::Vector3d center = component_center_inertial(
        rod.center_body_m,
        body_to_inertial,
        origin_inertial,
        visual_scale
    );
    const Eigen::Vector3d axis = detumble::rotate_body_to_inertial(
        body_to_inertial,
        rod.axis_body
    );
    const double half_length = 0.5 * visual_scale
        * meters_to_spacecraft_display_units * rod.length_m;
    const float radius = static_cast<float>(
        visual_scale * meters_to_spacecraft_display_units * rod.radius_m
    );
    DrawCylinderEx(
        to_raylib(center - half_length * axis),
        to_raylib(center + half_length * axis),
        radius,
        radius,
        16,
        color
    );
}

void draw_configured_wheel(
    const detumble::ReactionWheelComponent& wheel,
    const std::size_t wheel_index,
    const detumble::ReactionWheelTelemetry& telemetry,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    const Eigen::Vector3d center = component_center_inertial(
        wheel.center_body_m,
        body_to_inertial,
        origin_inertial,
        visual_scale
    );
    const Eigen::Vector3d axis = detumble::rotate_body_to_inertial(
        body_to_inertial,
        wheel.spin_axis_body
    );
    const double half_thickness = 0.5 * visual_scale
        * meters_to_spacecraft_display_units * wheel.thickness_m;
    const float radius = static_cast<float>(
        visual_scale * meters_to_spacecraft_display_units * wheel.radius_m
    );
    const bool has_telemetry =
        wheel_index < telemetry.wheel_angle_rad.size();
    const bool failed = has_telemetry && telemetry.failed[wheel_index];
    const bool saturated = has_telemetry
        && (telemetry.torque_saturated[wheel_index]
            || telemetry.speed_saturated[wheel_index]);
    const Color wheel_color = failed
        ? Color{235, 60, 60, 255}
        : saturated
            ? Color{250, 210, 55, 255}
            : Color{230, 135, 35, 255};
    DrawCylinderEx(
        to_raylib(center - half_thickness * axis),
        to_raylib(center + half_thickness * axis),
        radius,
        radius,
        24,
        wheel_color
    );

    Eigen::Vector3d radial = axis.cross(Eigen::Vector3d::UnitZ());
    if (radial.squaredNorm() < 1.0e-12) {
        radial = axis.cross(Eigen::Vector3d::UnitY());
    }
    radial.normalize();
    const double wheel_angle_rad = has_telemetry
        ? telemetry.wheel_angle_rad[wheel_index]
        : 0.0;
    radial = Eigen::AngleAxisd{wheel_angle_rad, axis} * radial;
    const Eigen::Vector3d marker_center = center + half_thickness * axis;
    DrawLine3D(
        to_raylib(marker_center),
        to_raylib(marker_center + 0.8 * static_cast<double>(radius) * radial),
        failed ? Color{255, 220, 220, 255} : Color{40, 30, 20, 255}
    );
}

void draw_configured_surface_sensor(
    const detumble::SurfaceSensorComponent& sensor,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale,
    const Color color
) {
    const Eigen::Vector3d center = component_center_inertial(
        sensor.center_body_m,
        body_to_inertial,
        origin_inertial,
        visual_scale
    );
    const Eigen::Vector3d normal = detumble::rotate_body_to_inertial(
        body_to_inertial,
        sensor.outward_normal_body
    );
    const double half_thickness = 0.00075 * visual_scale
        * meters_to_spacecraft_display_units;
    const float half_width = static_cast<float>(
        0.5 * visual_scale * meters_to_spacecraft_display_units
        * sensor.side_length_m
    );
    DrawCylinderEx(
        to_raylib(center - half_thickness * normal),
        to_raylib(center + half_thickness * normal),
        half_width,
        half_width,
        4,
        color
    );
}

void draw_configured_components(
    const detumble::SpacecraftVisualConfig& visual_config,
    const detumble::ReactionWheelTelemetry& wheel_telemetry,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    constexpr std::array<Color, 3> torquer_colors{
        Color{235, 55, 55, 255},
        Color{55, 215, 75, 255},
        Color{55, 105, 245, 255}
    };
    for (std::size_t index = 0;
         index < visual_config.magnetorquers.size();
         ++index) {
        draw_configured_rod(
            visual_config.magnetorquers[index],
            body_to_inertial,
            origin_inertial,
            visual_scale,
            torquer_colors[index]
        );
    }

    for (std::size_t index = 0;
         index < visual_config.reaction_wheels.size();
         ++index) {
        draw_configured_wheel(
            visual_config.reaction_wheels[index],
            index,
            wheel_telemetry,
            body_to_inertial,
            origin_inertial,
            visual_scale
        );
    }

    DrawSphere(
        to_raylib(component_center_inertial(
            visual_config.magnetometer.center_body_m,
            body_to_inertial,
            origin_inertial,
            visual_scale
        )),
        static_cast<float>(0.07 * visual_scale),
        Color{245, 80, 200, 255}
    );
}

void draw_sun_sensor_status(
    const detumble::SpacecraftVisualConfig& visual_config,
    const std::optional<detumble::CoarseSunSensorMeasurement>& measurement,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    for (std::size_t index = 0;
         index < visual_config.coarse_sun_sensors.size();
         ++index) {
        Color color{35, 115, 130, 255};
        if (measurement.has_value() && measurement->in_eclipse) {
            color = Color{100, 105, 115, 255};
        } else if (measurement.has_value()
                   && measurement->sensor_visible[index]) {
            color = sun_direction_color;
        }
        detumble::SurfaceSensorComponent status_sensor =
            visual_config.coarse_sun_sensors[index];
        status_sensor.center_body_m +=
            0.0015 * status_sensor.outward_normal_body;
        draw_configured_surface_sensor(
            status_sensor,
            body_to_inertial,
            origin_inertial,
            visual_scale,
            color
        );
    }
}

void draw_axis(
    const Eigen::Vector3d& origin_inertial,
    const Eigen::Vector3d& direction_inertial,
    const double axis_length,
    const Color color
) {
    const Vector3 origin = to_raylib(origin_inertial);
    const Vector3 end = to_raylib(
        origin_inertial + axis_length * direction_inertial
    );
    const Vector3 arrow_base = to_raylib(
        origin_inertial + 0.88 * axis_length * direction_inertial
    );

    DrawLine3D(origin, end, color);
    DrawCylinderEx(
        arrow_base,
        end,
        static_cast<float>(0.032 * axis_length),
        0.0F,
        10,
        color
    );
}

void draw_axis_label(
    const Eigen::Vector3d& origin_inertial,
    const Eigen::Vector3d& direction_inertial,
    const double axis_length,
    const Color color,
    const char* label,
    const Camera3D& camera
) {
    const Vector3 end = to_raylib(
        origin_inertial + axis_length * direction_inertial
    );
    const Vector2 label_position = GetWorldToScreen(end, camera);
    DrawText(
        label,
        static_cast<int>(label_position.x) + 5,
        static_cast<int>(label_position.y) - 5,
        18,
        color
    );
}

void draw_body_axes(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double axis_length
) {
    draw_axis(
        origin_inertial,
        detumble::rotate_body_to_inertial(
            body_to_inertial,
            Eigen::Vector3d::UnitX()
        ),
        axis_length,
        RED
    );
    draw_axis(
        origin_inertial,
        detumble::rotate_body_to_inertial(
            body_to_inertial,
            Eigen::Vector3d::UnitY()
        ),
        axis_length,
        GREEN
    );
    draw_axis(
        origin_inertial,
        detumble::rotate_body_to_inertial(
            body_to_inertial,
            Eigen::Vector3d::UnitZ()
        ),
        axis_length,
        BLUE
    );
}

void draw_body_axis_labels(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double axis_length,
    const Camera3D& camera
) {
    draw_axis_label(
        origin_inertial,
        detumble::rotate_body_to_inertial(
            body_to_inertial,
            Eigen::Vector3d::UnitX()
        ),
        axis_length,
        RED,
        "+X body",
        camera
    );
    draw_axis_label(
        origin_inertial,
        detumble::rotate_body_to_inertial(
            body_to_inertial,
            Eigen::Vector3d::UnitY()
        ),
        axis_length,
        GREEN,
        "+Y body",
        camera
    );
    draw_axis_label(
        origin_inertial,
        detumble::rotate_body_to_inertial(
            body_to_inertial,
            Eigen::Vector3d::UnitZ()
        ),
        axis_length,
        BLUE,
        "+Z body",
        camera
    );
}

double orbit_display_scale(const detumble::EnvironmentConfig& config) {
    return earth_display_radius / config.orbit.earth_radius_m;
}

Eigen::Vector3d orbit_position_display(
    const detumble::EnvironmentConfig& config,
    const detumble::EnvironmentState& environment
) {
    return orbit_display_scale(config)
        * environment.orbit.position_inertial_m;
}

void draw_orbit_path(const detumble::EnvironmentConfig& config) {
    constexpr int segment_count = 180;
    const double period_s = detumble::circular_orbit_period_s(config.orbit);
    const double display_scale = orbit_display_scale(config);

    Eigen::Vector3d previous = display_scale
        * detumble::circular_orbit_state(config.orbit, 0.0)
              .position_inertial_m;
    for (int segment = 1; segment <= segment_count; ++segment) {
        const double elapsed_time_s = period_s
            * static_cast<double>(segment)
            / static_cast<double>(segment_count);
        const Eigen::Vector3d current = display_scale
            * detumble::circular_orbit_state(config.orbit, elapsed_time_s)
                  .position_inertial_m;
        DrawLine3D(
            to_raylib(previous),
            to_raylib(current),
            Color{85, 140, 190, 210}
        );
        previous = current;
    }
}

void draw_earth() {
    DrawSphere(
        {},
        static_cast<float>(earth_display_radius),
        Color{24, 66, 120, 255}
    );
    DrawSphereWires(
        {},
        static_cast<float>(earth_display_radius * 1.002),
        18,
        36,
        Color{70, 125, 180, 180}
    );
}

void draw_earth_shadow(const Eigen::Vector3d& sun_direction_inertial) {
    const Eigen::Vector3d shadow_direction = -sun_direction_inertial.normalized();
    DrawCylinderEx(
        to_raylib(earth_display_radius * shadow_direction),
        to_raylib(6.0 * shadow_direction),
        static_cast<float>(earth_display_radius),
        static_cast<float>(earth_display_radius),
        36,
        Color{8, 10, 20, 95}
    );
}

void draw_sun_marker(const Eigen::Vector3d& sun_direction_inertial) {
    const Eigen::Vector3d direction = sun_direction_inertial.normalized();
    draw_axis(
        Eigen::Vector3d::Zero(),
        direction,
        4.5,
        sun_direction_color
    );
    DrawSphere(
        to_raylib(5.2 * direction),
        0.28F,
        sun_direction_color
    );
}

void draw_magnetic_field_vector(
    const Eigen::Vector3d& origin_inertial,
    const Eigen::Vector3d& magnetic_field_inertial_T,
    const double length
) {
    draw_axis(
        origin_inertial,
        magnetic_field_inertial_T.normalized(),
        length,
        Color{255, 195, 55, 255}
    );
}

void draw_magnetic_field_label(
    const Eigen::Vector3d& origin_inertial,
    const Eigen::Vector3d& magnetic_field_inertial_T,
    const double length,
    const Camera3D& camera
) {
    draw_axis_label(
        origin_inertial,
        magnetic_field_inertial_T.normalized(),
        length,
        Color{255, 195, 55, 255},
        "B field",
        camera
    );
}

void draw_vector_overlay(
    const Eigen::Vector3d& origin_inertial,
    const Eigen::Vector3d& vector_inertial,
    const double length,
    const Color color
) {
    if (vector_inertial.squaredNorm() <= 1.0e-30 || length <= 0.0) {
        return;
    }
    draw_axis(
        origin_inertial,
        vector_inertial.normalized(),
        length,
        color
    );
}

void draw_vector_overlay_label(
    const Eigen::Vector3d& origin_inertial,
    const Eigen::Vector3d& vector_inertial,
    const double length,
    const Color color,
    const char* label,
    const Camera3D& camera
) {
    if (vector_inertial.squaredNorm() <= 1.0e-30 || length <= 0.0) {
        return;
    }
    draw_axis_label(
        origin_inertial,
        vector_inertial.normalized(),
        length,
        color,
        label,
        camera
    );
}

void record_viewer_sample(
    const detumble::Simulation& simulation,
    const detumble::EnvironmentConfig& environment_config,
    const detumble::MagneticControlOutput& magnetic_output,
    const detumble::AutonomousFlightSoftware& flight_software,
    const detumble::ReactionWheelTelemetry& wheel_telemetry,
    const std::optional<detumble::CoarseSunSensorMeasurement>& sun_measurement,
    TelemetryHistory& telemetry,
    ReplayBuffer& replay
) {
    const detumble::EnvironmentState environment = current_environment(
        simulation,
        environment_config
    );
    telemetry.append(
        simulation,
        environment,
        magnetic_output,
        flight_software,
        wheel_telemetry,
        sun_measurement
    );
    replay.append(
        simulation,
        environment,
        magnetic_output,
        flight_software,
        wheel_telemetry,
        sun_measurement
    );
}

void reset_viewer_scenario(
    const ViewerScenario& scenario,
    const detumble::EnvironmentConfig& environment_config,
    detumble::Simulation& simulation,
    detumble::MagneticControlCycle& magnetic_control,
    detumble::IdealGyroscope& gyroscope,
    detumble::IdealCoarseSunSensorArray& sun_sensors,
    detumble::AutonomousFlightSoftware& flight_software,
    detumble::ReactionWheelCluster& reaction_wheels,
    detumble::MagneticControlOutput& magnetic_output,
    TelemetryHistory& telemetry,
    ReplayBuffer& replay,
    ReplayState& replay_state
) {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    detumble::SimulationConfig simulation_config;
    simulation_config.seed = scenario.seed;
    simulation_config.minimum_angular_speed_rad_s =
        scenario.initial_rate_deg_s * degrees_to_radians;
    simulation_config.maximum_angular_speed_rad_s =
        scenario.initial_rate_deg_s * degrees_to_radians;
    simulation = detumble::Simulation{simulation_config};

    const detumble::SensorSuiteConfig sensors =
        detumble::realistic_sensor_suite_config(scenario.seed);
    magnetic_control = detumble::MagneticControlCycle{sensors.magnetometer};
    gyroscope = detumble::IdealGyroscope{sensors.gyroscope};
    sun_sensors = detumble::IdealCoarseSunSensorArray{sensors.sun_sensor};
    flight_software = detumble::AutonomousFlightSoftware{};
    reaction_wheels = detumble::ReactionWheelCluster{};
    magnetic_output = {};
    telemetry.clear();
    replay.clear();
    replay_state.stop();
    record_viewer_sample(
        simulation,
        environment_config,
        magnetic_output,
        flight_software,
        reaction_wheels.telemetry(),
        sun_sensors.last_measurement(),
        telemetry,
        replay
    );
}

const char* actuator_status(const RecordedFrame& frame) {
    if (frame.magnetorquers_enabled) {
        return "magnetorquers active";
    }
    if (frame.reaction_wheels_enabled) {
        return "reaction wheels active";
    }
    return "actuators idle";
}

void draw_scenario_status_panel(
    const ViewerScenario& scenario,
    const RecordedFrame& frame,
    const bool replay_active,
    const bool diagnostics_visible
) {
    const float right_aligned_x = std::max(
        440.0F,
        static_cast<float>(GetScreenWidth()) - 376.0F
    );
    ImGui::SetNextWindowPos(
        {diagnostics_visible ? 440.0F : right_aligned_x, 16.0F},
        ImGuiCond_Always
    );
    ImGui::SetNextWindowSize({360.0F, 205.0F}, ImGuiCond_Always);
    if (ImGui::Begin("Scenario status")) {
        ImGui::TextColored(
            replay_active
                ? ImVec4{1.0F, 0.8F, 0.3F, 1.0F}
                : ImVec4{0.45F, 1.0F, 0.55F, 1.0F},
            "%s | native C++",
            replay_active ? "REPLAY" : "LIVE"
        );
        ImGui::Text("Scenario: %s", scenario.name);
        ImGui::Text(
            "Seed %llu | time %.1f s",
            static_cast<unsigned long long>(scenario.seed),
            frame.time_s
        );
        ImGui::Text("Mode: %s", detumble::to_string(frame.mode).data());
        ImGui::Text(
            "Body rate: %.3f deg/s",
            frame.angular_velocity_body_rad_s.norm()
                * 180.0 / std::numbers::pi
        );
        if (std::isfinite(frame.sun_pointing_error_deg)) {
            ImGui::Text(
                "Sun-pointing error: %.2f deg",
                frame.sun_pointing_error_deg
            );
        } else {
            ImGui::TextDisabled("Sun-pointing error: not active");
        }
        ImGui::Text(
            "Lighting: %s | %s",
            frame.environment.in_eclipse ? "ECLIPSE" : "SUNLIT",
            actuator_status(frame)
        );
        ImGui::Text(
            "Navigation: attitude %s, Sun %s",
            frame.attitude_valid ? "valid" : "pending",
            frame.sun_available ? "available" : "unavailable"
        );
    }
    ImGui::End();
}

void draw_help_window(bool& show_help) {
    if (!show_help) {
        return;
    }
    ImGui::SetNextWindowSize({430.0F, 300.0F}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Controls and legend", &show_help)) {
        ImGui::SeparatorText("Keyboard");
        ImGui::Text("Space   pause or resume live/replay");
        ImGui::Text("R       reset the selected scenario");
        ImGui::Text("C / X / O   close-up / cutaway / orbit view");
        ImGui::Text("T / D   telemetry / engineering diagnostics");
        ImGui::Text("H       show or hide this help");
        ImGui::SeparatorText("Mouse");
        ImGui::Text("Right-drag   orbit the camera");
        ImGui::Text("Wheel        zoom");
        ImGui::SeparatorText("Telemetry meaning");
        ImGui::BulletText("Truth: physics state and environment");
        ImGui::BulletText("Measured: seeded sensor output");
        ImGui::BulletText("Estimate: onboard attitude solution");
        ImGui::BulletText("Command: controller or actuator request");
        ImGui::TextDisabled(
            "Magenta/cyan actuator arrows are optional diagnostics."
        );
    }
    ImGui::End();
}

void draw_control_panel(
    detumble::Simulation& simulation,
    const detumble::EnvironmentConfig& environment_config,
    const detumble::EnvironmentState& environment,
    detumble::MagneticControlCycle& magnetic_control,
    detumble::IdealGyroscope& gyroscope,
    detumble::IdealCoarseSunSensorArray& sun_sensors,
    detumble::AutonomousFlightSoftware& flight_software,
    detumble::ReactionWheelCluster& reaction_wheels,
    const Eigen::Vector3d& manual_dipole_body_A_m2,
    const bool automatic_bdot,
    detumble::MagneticControlOutput& magnetic_output,
    TelemetryHistory& telemetry,
    ReplayBuffer& replay,
    ReplayState& replay_state,
    ViewerOptions& options,
    UiVisibility& ui_visibility,
    OrbitCamera& camera,
    bool& playing,
    float& playback_speed,
    double& time_accumulator_s,
    int& selected_scenario_index,
    int& requested_scenario_index,
    bool& show_help,
    const ViewerPerformance& performance,
    const bool loaded_glb
) {
    ImGui::SetNextWindowPos({16.0F, 16.0F}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(
        {390.0F, 0.0F},
        {440.0F, static_cast<float>(GetScreenHeight()) - 32.0F}
    );
    if (ImGui::Begin(
            "Controls",
            nullptr,
            ImGuiWindowFlags_AlwaysAutoResize
        )) {
        if (replay_state.active) {
            if (ImGui::Button(replay_state.playing ? "Pause replay" : "Play replay")) {
                replay_state.playing = !replay_state.playing;
            }
            ImGui::SameLine();
            if (ImGui::Button("Return to live")) {
                replay_state.stop();
            }
            int replay_frame = static_cast<int>(replay_state.frame_index);
            const int last_frame = static_cast<int>(replay.frames.size() - 1);
            if (ImGui::SliderInt("Recorded frame", &replay_frame, 0, last_frame)) {
                replay_state.frame_index = static_cast<std::size_t>(replay_frame);
                replay_state.playing = false;
                replay_state.accumulator_s = 0.0;
            }
            ImGui::SliderFloat(
                "Replay speed",
                &replay_state.speed,
                0.1F,
                100.0F,
                "%.1fx",
                ImGuiSliderFlags_Logarithmic
            );
        } else {
            if (ImGui::Button(playing ? "Pause" : "Resume")) {
                playing = !playing;
            }
            ImGui::SameLine();
            if (ImGui::Button("Single step")) {
                playing = false;
                magnetic_output = advance_simulation(
                    simulation,
                    environment_config,
                    magnetic_control,
                    gyroscope,
                    sun_sensors,
                    flight_software,
                    reaction_wheels,
                    manual_dipole_body_A_m2,
                    automatic_bdot
                );
                record_viewer_sample(
                    simulation,
                    environment_config,
                    magnetic_output,
                    flight_software,
                    reaction_wheels.telemetry(),
                    sun_sensors.last_measurement(),
                    telemetry,
                    replay
                );
            }
            ImGui::SameLine();
            if (ImGui::Button("Replay")) {
                playing = false;
                replay_state.start(replay);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            playing = false;
            time_accumulator_s = 0.0;
            simulation.reset();
            magnetic_control.reset();
            gyroscope.reset();
            sun_sensors.reset();
            flight_software.reset();
            reaction_wheels.reset();
            magnetic_output = {};
            telemetry.clear();
            replay.clear();
            replay_state.stop();
            record_viewer_sample(
                simulation,
                environment_config,
                magnetic_output,
                flight_software,
                reaction_wheels.telemetry(),
                sun_sensors.last_measurement(),
                telemetry,
                replay
            );
        }

        ImGui::TextColored(
            {0.55F, 1.0F, 0.6F, 1.0F},
            "Flight software: Native C++"
        );

        if (!replay_state.active) {
            ImGui::SliderFloat(
                "Simulation speed",
                &playback_speed,
                0.1F,
                100.0F,
                "%.1fx",
                ImGuiSliderFlags_Logarithmic
            );
        }

        if (ImGui::BeginCombo(
                "Scenario",
                viewer_scenarios[static_cast<std::size_t>(
                    selected_scenario_index
                )].name
            )) {
            for (std::size_t index = 0; index < viewer_scenarios.size(); ++index) {
                const bool selected = static_cast<int>(index)
                    == selected_scenario_index;
                if (ImGui::Selectable(viewer_scenarios[index].name, selected)) {
                    requested_scenario_index = static_cast<int>(index);
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::TextDisabled(
            "%s",
            viewer_scenarios[static_cast<std::size_t>(
                selected_scenario_index
            )].description
        );
        if (ImGui::Button("Help (H)")) {
            show_help = !show_help;
        }
        ImGui::SameLine();
        ImGui::Checkbox("Telemetry (T)", &ui_visibility.show_telemetry);
        ImGui::SameLine();
        ImGui::Checkbox("Diagnostics (D)", &ui_visibility.show_diagnostics);

        if (ImGui::CollapsingHeader("View and overlays")) {
            if (ImGui::Button("Close-up (C)")) {
                options.orbit_overview = false;
                options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::solid;
                camera.close_up();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cutaway (X)")) {
                options.orbit_overview = false;
                options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::cutaway;
                camera.cutaway();
            }
            ImGui::SameLine();
            if (ImGui::Button("Orbit (O)")) {
                options.orbit_overview = true;
                options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::solid;
                camera.orbit_overview();
            }
            ImGui::Checkbox("Orbit path", &options.show_orbit_path);

            ImGui::SeparatorText("Spacecraft rendering");
            if (ImGui::RadioButton(
                    "Solid",
                    options.spacecraft_render_mode
                        == ViewerOptions::SpacecraftRenderMode::solid
                )) {
                options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::solid;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton(
                    "Transparent",
                    options.spacecraft_render_mode
                        == ViewerOptions::SpacecraftRenderMode::transparent
                )) {
                options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::transparent;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton(
                    "Cutaway",
                    options.spacecraft_render_mode
                        == ViewerOptions::SpacecraftRenderMode::cutaway
                )) {
                options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::cutaway;
            }

            ImGui::SeparatorText("Vector overlays");
            ImGui::Checkbox("Body axes", &options.show_body_axes);
            ImGui::SameLine();
            ImGui::Checkbox(
                "Angular velocity",
                &options.show_angular_velocity
            );
            ImGui::Checkbox("B field", &options.show_magnetic_field);
            ImGui::SameLine();
            ImGui::Checkbox("Dipole", &options.show_magnetic_dipole);
            ImGui::SameLine();
            ImGui::Checkbox("Torque", &options.show_control_torque);
            ImGui::SameLine();
            ImGui::Checkbox("Sun", &options.show_sun_direction);
        }

        if (ImGui::CollapsingHeader("Detailed mission state")) {
            const Eigen::Vector3d rate_deg_s =
                simulation.state().angular_velocity_body_rad_s
                * (180.0 / std::numbers::pi);
            ImGui::Separator();
            ImGui::Text("Time: %.2f s", simulation.elapsed_time_s());
            ImGui::Text("Fixed step: %.3f s", simulation.config().time_step_s);
            ImGui::Text(
                "Seed: %llu",
                static_cast<unsigned long long>(simulation.config().seed)
            );
            ImGui::Text("Sensors: realistic seeded errors");
            ImGui::Text(
                "Body rate: [%.2f, %.2f, %.2f] deg/s",
                rate_deg_s.x(),
                rate_deg_s.y(),
                rate_deg_s.z()
            );
            ImGui::Text(
                "Quaternion norm: %.12f",
                simulation.metrics().quaternion_norm
            );
            const Eigen::Vector3d position_km =
                meters_to_kilometers * environment.orbit.position_inertial_m;
            const Eigen::Vector3d magnetic_field_body_uT =
                tesla_to_microtesla * environment.magnetic_field_body_T;
            if (ImGui::CollapsingHeader("Environment")) {
                ImGui::Text(
                    "Orbit: %.0f km, %.1f deg, %.2f min",
                    environment_config.orbit.altitude_m * meters_to_kilometers,
                    environment_config.orbit.inclination_rad
                        * 180.0 / std::numbers::pi,
                    detumble::circular_orbit_period_s(environment_config.orbit)
                        / 60.0
                );
                ImGui::Text(
                    "Position ECI: [%.0f, %.0f, %.0f] km",
                    position_km.x(),
                    position_km.y(),
                    position_km.z()
                );
                ImGui::Text(
                    "Magnetic field body: [%.1f, %.1f, %.1f] uT",
                    magnetic_field_body_uT.x(),
                    magnetic_field_body_uT.y(),
                    magnetic_field_body_uT.z()
                );
                ImGui::Text(
                    "Field magnitude: %.1f uT",
                    environment.magnetic_field_inertial_T.norm()
                        * tesla_to_microtesla
                );
                ImGui::Text(
                    "Sun direction body: [%.2f, %.2f, %.2f]",
                    environment.sun_direction_body.x(),
                    environment.sun_direction_body.y(),
                    environment.sun_direction_body.z()
                );
            }
            const auto& sun_measurement = sun_sensors.last_measurement();
            const bool valid_sun_measurement = sun_measurement.has_value()
                && sun_measurement->sun_direction_body.has_value();
            if (environment.in_eclipse) {
                ImGui::TextColored(
                    {1.0F, 0.55F, 0.35F, 1.0F},
                    "Lighting: ECLIPSE - Sun sensors invalid"
                );
            } else {
                ImGui::TextColored(
                    {1.0F, 0.9F, 0.4F, 1.0F},
                    "Lighting: SUNLIT - Sun measurement %s",
                    valid_sun_measurement ? "valid" : "pending"
                );
            }
            const detumble::AttitudeEstimate& estimate =
                flight_software.attitude_estimator().estimate();
            if (estimate.valid) {
                const double attitude_error_deg =
                    detumble::attitude_error_angle_rad(
                        estimate.body_to_inertial,
                        simulation.state().body_to_inertial
                    ) * 180.0 / std::numbers::pi;
                ImGui::TextColored(
                    {0.45F, 0.9F, 1.0F, 1.0F},
                    "Estimate: VALID - error %.3f deg",
                    attitude_error_deg
                );
            } else {
                ImGui::TextDisabled("Estimate: INVALID - awaiting TRIAD");
            }
            ImGui::Text(
                "Estimator correction: %s",
                detumble::to_string(estimate.correction_status).data()
            );
            const detumble::SunPointingCommand& sun_pointing =
                flight_software.state().sun_pointing;
            if (sun_pointing.active) {
                constexpr double radians_to_degrees =
                    180.0 / std::numbers::pi;
                ImGui::TextColored(
                    {0.45F, 1.0F, 0.55F, 1.0F},
                    "Sun control: %s - error %.2f deg",
                    flight_software.sun_pointing_tracker()
                            .state().steady_pointing
                        ? "POINTING"
                        : "ACQUIRING",
                    sun_pointing.pointing_error_rad * radians_to_degrees
                );
                ImGui::Text(
                    "Wheel request: [%.3f, %.3f, %.3f] mN m%s",
                    1.0e3 * sun_pointing.limited_torque_body_Nm.x(),
                    1.0e3 * sun_pointing.limited_torque_body_Nm.y(),
                    1.0e3 * sun_pointing.limited_torque_body_Nm.z(),
                    sun_pointing.saturated ? " - CTRL LIMITED" : ""
                );
            } else {
                ImGui::TextDisabled(
                    "Sun control: inactive in %s mode",
                    detumble::to_string(flight_software.state().mode).data()
                );
            }
            ImGui::Text(
                "Mission mode: %s  elapsed: %.1f s",
                detumble::to_string(flight_software.state().mode).data(),
                flight_software.state().mode_elapsed_s
            );
            if (ImGui::CollapsingHeader("Mode transition history")) {
                if (flight_software.events().empty()) {
                    ImGui::TextDisabled("No transitions yet");
                }
                const auto& events = flight_software.events();
                const std::size_t first = events.size() > 6
                    ? events.size() - 6 : 0;
                for (std::size_t index = first;
                     index < events.size();
                     ++index) {
                    const auto& event = events[index];
                    ImGui::Text(
                        "%.1f s  %s -> %s",
                        event.sample_time_s,
                        detumble::to_string(event.old_mode).data(),
                        detumble::to_string(event.new_mode).data()
                    );
                    ImGui::TextDisabled(
                        "  %s",
                        detumble::to_string(event.reason).data()
                    );
                }
            }
            ImGui::Text(
                "Model: %s",
                loaded_glb ? "custom Detumble 3U GLB" : "fallback cube"
            );
        }
        if (ImGui::CollapsingHeader("Performance")) {
            ImGui::Text("Viewer: %.1f frames/s", performance.frames_per_second);
            ImGui::Text(
                "Simulation: %.0f fixed steps/s",
                performance.steps_per_second
            );
            ImGui::Text(
                "Physics throughput: %.0fx real time",
                performance.real_time_factor
            );
            ImGui::TextDisabled(
                "Measured over rolling 0.5-second wall-time windows"
            );
        }
        ImGui::TextDisabled("Right-drag orbit | wheel zoom | H help");
    }
    ImGui::End();
}

void draw_magnetic_control_panel(
    Eigen::Vector3d& manual_dipole_body_A_m2,
    bool& automatic_bdot,
    const detumble::MagneticControlCycle& magnetic_control,
    const detumble::AutonomousFlightSoftware& flight_software,
    const detumble::EnvironmentState& environment
) {
    ImGui::SetNextWindowPos({994.0F, 16.0F}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({430.0F, 400.0F}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Magnetic actuators")) {
        ImGui::Checkbox("Automatic B-dot control", &automatic_bdot);
        constexpr double command_min_A_m2 = -0.3;
        constexpr double command_max_A_m2 = 0.3;
        ImGui::BeginDisabled(automatic_bdot);
        ImGui::SliderScalarN(
            "Manual dipole (A m^2)",
            ImGuiDataType_Double,
            manual_dipole_body_A_m2.data(),
            3,
            &command_min_A_m2,
            &command_max_A_m2,
            "%.3f"
        );
        ImGui::EndDisabled();

        const Eigen::Vector3d& limit =
            magnetic_control.magnetorquer_config()
                .maximum_dipole_body_A_m2;
        const Eigen::Vector3d& requested_dipole_body_A_m2 = automatic_bdot
            ? flight_software.state().actuator_command
                  .magnetorquer_dipole_body_A_m2
            : manual_dipole_body_A_m2;
        const Eigen::Vector3d limited_dipole_body_A_m2 =
            detumble::saturate_magnetorquer_dipole_body_A_m2(
                magnetic_control.magnetorquer_config(),
                requested_dipole_body_A_m2
            );
        const Eigen::Vector3d nominal_torque_uNm =
            detumble::magnetic_torque_body_Nm(
                limited_dipole_body_A_m2,
                environment.magnetic_field_body_T
            ) * 1.0e6;
        ImGui::Text(
            "Per-axis limit: [%.2f, %.2f, %.2f] A m^2",
            limit.x(),
            limit.y(),
            limit.z()
        );
        ImGui::Text(
            "Limited target: [%.3f, %.3f, %.3f] A m^2",
            limited_dipole_body_A_m2.x(),
            limited_dipole_body_A_m2.y(),
            limited_dipole_body_A_m2.z()
        );
        ImGui::Text(
            "Nominal torque: [%.3f, %.3f, %.3f] uN m",
            nominal_torque_uNm.x(),
            nominal_torque_uNm.y(),
            nominal_torque_uNm.z()
        );

        const detumble::DetumbleFlightSoftwareState& flight_state =
            flight_software.detumble().state();
        if (automatic_bdot && flight_state.detumble_complete) {
            ImGui::TextColored(
                {0.4F, 0.8F, 1.0F, 1.0F},
                "Actuator mode: DETUMBLE COMPLETE"
            );
        } else if (automatic_bdot) {
            ImGui::TextColored(
                {0.5F, 1.0F, 0.5F, 1.0F},
                "Actuator mode: AUTOMATIC B-DOT"
            );
        } else if (limited_dipole_body_A_m2.isZero()) {
            ImGui::Text("Actuator mode: IDLE - zero command");
        } else {
            ImGui::TextColored(
                {0.5F, 1.0F, 0.5F, 1.0F},
                "Actuator mode: MANUAL ACTUATION"
            );
        }

        constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
        const double measured_speed_deg_s =
            flight_state.measured_angular_velocity_body_rad_s.norm()
            * radians_to_degrees;
        const double threshold_deg_s =
            flight_software.detumble().config()
                .completion_threshold_rad_s
            * radians_to_degrees;
        ImGui::Text(
            "Flight mode: %s%s",
            detumble::to_string(flight_state.mode).data(),
            flight_state.detumble_complete ? " - COMPLETE" : ""
        );
        ImGui::Text(
            "Gyro speed: %.3f deg/s  threshold: %.3f",
            measured_speed_deg_s,
            threshold_deg_s
        );
        ImGui::Text(
            "Dwell: %.1f / %.1f s",
            flight_state.below_threshold_elapsed_s,
            flight_software.detumble().config().completion_dwell_time_s
        );
        const Eigen::Vector3d filtered_bdot_uT_s =
            flight_state.filtered_bdot_body_T_s * tesla_to_microtesla;
        ImGui::Text(
            "Filtered B-dot: [%.2f, %.2f, %.2f] uT/s",
            filtered_bdot_uT_s.x(),
            filtered_bdot_uT_s.y(),
            filtered_bdot_uT_s.z()
        );

        ImGui::Text(
            "Sampling blanking: automatic at %.1f Hz",
            magnetic_control.magnetometer().config().sample_rate_hz
        );
        const auto& measurement =
            magnetic_control.magnetometer().last_measurement();
        if (measurement.has_value()) {
            const Eigen::Vector3d measured_field_uT =
                measurement->magnetic_field_body_T * tesla_to_microtesla;
            ImGui::Text(
                "Last sample: %.2f s  [%.1f, %.1f, %.1f] uT",
                measurement->sample_time_s,
                measured_field_uT.x(),
                measured_field_uT.y(),
                measured_field_uT.z()
            );
        } else {
            ImGui::TextDisabled("No magnetometer sample yet");
        }
        ImGui::TextDisabled(
            "Coils turn off for one 0.01 s step per sample"
        );
        ImGui::TextDisabled("Commands beyond the per-axis limits saturate");
    }
    ImGui::End();
}

void draw_reaction_wheel_panel(
    detumble::ReactionWheelCluster& reaction_wheels
) {
    ImGui::SetNextWindowPos({994.0F, 430.0F}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({430.0F, 330.0F}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Reaction wheels")) {
        const detumble::ReactionWheelTelemetry& telemetry =
            reaction_wheels.telemetry();
        const Eigen::Vector3d requested_mNm =
            1.0e3 * telemetry.requested_spacecraft_torque_body_Nm;
        const Eigen::Vector3d applied_mNm =
            1.0e3 * telemetry.applied_spacecraft_torque_body_Nm;
        ImGui::Text(
            "Requested body torque: [%.3f, %.3f, %.3f] mN m",
            requested_mNm.x(),
            requested_mNm.y(),
            requested_mNm.z()
        );
        ImGui::Text(
            "Applied body torque:   [%.3f, %.3f, %.3f] mN m",
            applied_mNm.x(),
            applied_mNm.y(),
            applied_mNm.z()
        );
        ImGui::Text(
            "Allocation error: %.4f mN m%s",
            1.0e3 * telemetry.allocation_error_body_Nm.norm(),
            telemetry.allocation_saturated ? " - LIMITED" : ""
        );
        ImGui::SeparatorText("Wheel telemetry and failures");

        constexpr double radians_per_second_to_rpm =
            60.0 / (2.0 * std::numbers::pi);
        for (std::size_t index = 0;
             index < reaction_wheels.config().wheels.size();
             ++index) {
            ImGui::PushID(static_cast<int>(index));
            bool failed = telemetry.failed[index];
            if (ImGui::Checkbox("Fail", &failed)) {
                reaction_wheels.set_wheel_failed(index, failed);
            }
            ImGui::SameLine();
            const bool saturated = telemetry.torque_saturated[index]
                || telemetry.speed_saturated[index];
            const ImVec4 color = failed
                ? ImVec4{1.0F, 0.35F, 0.35F, 1.0F}
                : saturated
                    ? ImVec4{1.0F, 0.85F, 0.25F, 1.0F}
                    : ImVec4{0.85F, 0.85F, 0.85F, 1.0F};
            ImGui::TextColored(
                color,
                "%s  %7.1f rpm  %+.3f mN m",
                reaction_wheels.config().wheels[index].name.data(),
                radians_per_second_to_rpm
                    * telemetry.wheel_speed_rad_s[index],
                1.0e3 * telemetry.applied_motor_torque_Nm[index]
            );
            ImGui::PopID();
        }
        ImGui::TextDisabled(
            "Orange = nominal, gold = limited, red = failed"
        );
    }
    ImGui::End();
}

void draw_telemetry_plots(const TelemetryHistory& telemetry) {
    const float telemetry_width = std::max(
        420.0F,
        static_cast<float>(GetScreenWidth()) - 32.0F
    );
    const float telemetry_height = std::clamp(
        0.38F * static_cast<float>(GetScreenHeight()),
        260.0F,
        360.0F
    );
    ImGui::SetNextWindowPos(
        {
            16.0F,
            static_cast<float>(GetScreenHeight()) - telemetry_height - 16.0F
        },
        ImGuiCond_Always
    );
    ImGui::SetNextWindowSize(
        {telemetry_width, telemetry_height},
        ImGuiCond_Always
    );
    if (!ImGui::Begin("Telemetry history")) {
        ImGui::End();
        return;
    }

    const double newest_time_s = telemetry.time_s.back();
    const double plot_start_s = std::max(0.0, newest_time_s - plot_window_s);
    const double plot_end_s = std::max(plot_window_s, newest_time_s);
    const int point_count = static_cast<int>(telemetry.time_s.size());

    if (ImGui::BeginTabBar("Telemetry tabs")) {
        if (ImGui::BeginTabItem("Body rates")) {
            ImGui::TextDisabled(
                "Thick = truth state; thin = gyroscope measurement"
            );
            if (ImPlot::BeginPlot("Body rates", available_plot_size())) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "rate (deg/s)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupLegend(
                    ImPlotLocation_South,
                    ImPlotLegendFlags_Outside
                    | ImPlotLegendFlags_Horizontal
                );
                const std::array<const char*, 3> truth_labels{
                    "truth X", "truth Y", "truth Z"
                };
                const std::array<const char*, 3> measured_labels{
                    "gyro X", "gyro Y", "gyro Z"
                };
                const std::array<const double*, 3> truth_values{
                    telemetry.true_body_rate_x_deg_s.data(),
                    telemetry.true_body_rate_y_deg_s.data(),
                    telemetry.true_body_rate_z_deg_s.data()
                };
                const std::array<const double*, 3> measured_values{
                    telemetry.measured_body_rate_x_deg_s.data(),
                    telemetry.measured_body_rate_y_deg_s.data(),
                    telemetry.measured_body_rate_z_deg_s.data()
                };
                const std::array<ImVec4, 3> colors{
                    ImVec4{0.95F, 0.35F, 0.35F, 1.0F},
                    ImVec4{0.35F, 0.85F, 0.45F, 1.0F},
                    ImVec4{0.35F, 0.60F, 1.0F, 1.0F}
                };
                for (std::size_t axis = 0; axis < colors.size(); ++axis) {
                    ImPlot::PlotLine(
                        truth_labels[axis],
                        telemetry.time_s.data(),
                        truth_values[axis],
                        point_count,
                        ImPlotSpec{
                            ImPlotProp_LineColor,
                            colors[axis],
                            ImPlotProp_LineWeight,
                            2.5F
                        }
                    );
                    ImPlot::PlotLine(
                        measured_labels[axis],
                        telemetry.time_s.data(),
                        measured_values[axis],
                        point_count,
                        ImPlotSpec{
                            ImPlotProp_LineColor,
                            colors[axis],
                            ImPlotProp_LineWeight,
                            1.0F
                        }
                    );
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Magnetic field")) {
            ImGui::TextDisabled(
                "Thick = environment truth; thin = magnetometer measurement"
            );
            if (ImPlot::BeginPlot(
                    "Body-frame magnetic field",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "field (uT)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupLegend(
                    ImPlotLocation_South,
                    ImPlotLegendFlags_Outside
                    | ImPlotLegendFlags_Horizontal
                );
                const std::array<const char*, 3> truth_labels{
                    "truth Bx", "truth By", "truth Bz"
                };
                const std::array<const char*, 3> measured_labels{
                    "measured Bx", "measured By", "measured Bz"
                };
                const std::array<const double*, 3> truth_values{
                    telemetry.magnetic_field_x_body_uT.data(),
                    telemetry.magnetic_field_y_body_uT.data(),
                    telemetry.magnetic_field_z_body_uT.data()
                };
                const std::array<const double*, 3> measured_values{
                    telemetry.measured_magnetic_field_x_body_uT.data(),
                    telemetry.measured_magnetic_field_y_body_uT.data(),
                    telemetry.measured_magnetic_field_z_body_uT.data()
                };
                const std::array<ImVec4, 3> colors{
                    ImVec4{0.95F, 0.35F, 0.35F, 1.0F},
                    ImVec4{0.35F, 0.85F, 0.45F, 1.0F},
                    ImVec4{0.35F, 0.60F, 1.0F, 1.0F}
                };
                for (std::size_t axis = 0; axis < colors.size(); ++axis) {
                    ImPlot::PlotLine(
                        truth_labels[axis],
                        telemetry.time_s.data(),
                        truth_values[axis],
                        point_count,
                        ImPlotSpec{
                            ImPlotProp_LineColor,
                            colors[axis],
                            ImPlotProp_LineWeight,
                            2.5F
                        }
                    );
                    ImPlot::PlotLine(
                        measured_labels[axis],
                        telemetry.time_s.data(),
                        measured_values[axis],
                        point_count,
                        ImPlotSpec{
                            ImPlotProp_LineColor,
                            colors[axis],
                            ImPlotProp_LineWeight,
                            1.0F
                        }
                    );
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Sun sensors")) {
            if (ImPlot::BeginPlot(
                    "Coarse Sun sensor illumination",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "normalized response"
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_Y1,
                    -0.05,
                    1.05,
                    ImPlotCond_Always
                );
                constexpr std::array<const char*, 6> sensor_names{
                    "+X", "-X", "+Y", "-Y", "+Z", "-Z"
                };
                for (std::size_t index = 0;
                     index < sensor_names.size();
                     ++index) {
                    ImPlot::PlotLine(
                        sensor_names[index],
                        telemetry.time_s.data(),
                        telemetry.sun_sensor_illumination[index].data(),
                        point_count
                    );
                }
                ImPlot::PlotLine(
                    "valid Sun vector",
                    telemetry.time_s.data(),
                    telemetry.sun_sensor_valid_value.data(),
                    point_count
                );
                ImPlot::PlotLine(
                    "eclipse",
                    telemetry.time_s.data(),
                    telemetry.eclipse_value.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("ECI position")) {
            if (ImPlot::BeginPlot(
                    "Spacecraft position",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "position (km)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::PlotLine(
                    "x",
                    telemetry.time_s.data(),
                    telemetry.position_x_inertial_km.data(),
                    point_count
                );
                ImPlot::PlotLine(
                    "y",
                    telemetry.time_s.data(),
                    telemetry.position_y_inertial_km.data(),
                    point_count
                );
                ImPlot::PlotLine(
                    "z",
                    telemetry.time_s.data(),
                    telemetry.position_z_inertial_km.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Magnetorquers")) {
            if (ImPlot::BeginPlot(
                    "Requested and limited dipole",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "dipole (A m^2)"
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_Y1,
                    -0.32,
                    0.32,
                    ImPlotCond_Always
                );
                ImPlot::SetupLegend(
                    ImPlotLocation_South,
                    ImPlotLegendFlags_Outside
                    | ImPlotLegendFlags_Horizontal
                );
                const ImVec4 x_color{0.95F, 0.35F, 0.35F, 1.0F};
                const ImVec4 y_color{0.35F, 0.85F, 0.45F, 1.0F};
                const ImVec4 z_color{0.35F, 0.60F, 1.0F, 1.0F};
                const ImPlotSpec requested_x_style{
                    ImPlotProp_LineColor,
                    x_color,
                    ImPlotProp_LineWeight,
                    1.0F
                };
                const ImPlotSpec limited_x_style{
                    ImPlotProp_LineColor,
                    x_color,
                    ImPlotProp_LineWeight,
                    3.0F
                };
                const ImPlotSpec requested_y_style{
                    ImPlotProp_LineColor,
                    y_color,
                    ImPlotProp_LineWeight,
                    1.0F
                };
                const ImPlotSpec limited_y_style{
                    ImPlotProp_LineColor,
                    y_color,
                    ImPlotProp_LineWeight,
                    3.0F
                };
                const ImPlotSpec requested_z_style{
                    ImPlotProp_LineColor,
                    z_color,
                    ImPlotProp_LineWeight,
                    1.0F
                };
                const ImPlotSpec limited_z_style{
                    ImPlotProp_LineColor,
                    z_color,
                    ImPlotProp_LineWeight,
                    3.0F
                };
                ImPlot::PlotLine(
                    "requested X",
                    telemetry.time_s.data(),
                    telemetry.commanded_dipole_x_body_A_m2.data(),
                    point_count,
                    requested_x_style
                );
                ImPlot::PlotLine(
                    "limited X",
                    telemetry.time_s.data(),
                    telemetry.limited_dipole_x_body_A_m2.data(),
                    point_count,
                    limited_x_style
                );
                ImPlot::PlotLine(
                    "requested Y",
                    telemetry.time_s.data(),
                    telemetry.commanded_dipole_y_body_A_m2.data(),
                    point_count,
                    requested_y_style
                );
                ImPlot::PlotLine(
                    "limited Y",
                    telemetry.time_s.data(),
                    telemetry.limited_dipole_y_body_A_m2.data(),
                    point_count,
                    limited_y_style
                );
                ImPlot::PlotLine(
                    "requested Z",
                    telemetry.time_s.data(),
                    telemetry.commanded_dipole_z_body_A_m2.data(),
                    point_count,
                    requested_z_style
                );
                ImPlot::PlotLine(
                    "limited Z",
                    telemetry.time_s.data(),
                    telemetry.limited_dipole_z_body_A_m2.data(),
                    point_count,
                    limited_z_style
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Energy")) {
            if (ImPlot::BeginPlot(
                    "Rotational kinetic energy",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "energy (J)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupAxisFormat(ImAxis_Y1, "%.2e");
                ImPlot::PlotLine(
                    "rotational energy",
                    telemetry.time_s.data(),
                    telemetry.rotational_energy_j.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Flight mode")) {
            ImGui::TextDisabled(
                "0 BOOT, 1 DETUMBLE, 2 SUN_ACQUIRE, 3 SUN_POINT, 4 SAFE"
            );
            if (ImPlot::BeginPlot(
                    "Flight mode and completion",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "mode"
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_Y1,
                    -0.2,
                    4.2,
                    ImPlotCond_Always
                );
                ImPlot::PlotLine(
                    "mission mode",
                    telemetry.time_s.data(),
                    telemetry.flight_mode_value.data(),
                    point_count
                );
                ImPlot::PlotLine(
                    "complete",
                    telemetry.time_s.data(),
                    telemetry.detumble_complete_value.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Attitude estimate")) {
            ImGui::TextDisabled(
                "Truth is used here only to score the estimator"
            );
            if (ImPlot::BeginPlot(
                    "Estimated-versus-true attitude error",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "rotation error (deg)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_Y1,
                    0.0,
                    180.0,
                    ImPlotCond_Once
                );
                ImPlot::PlotLine(
                    "attitude error",
                    telemetry.time_s.data(),
                    telemetry.attitude_error_deg.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Sun pointing")) {
            const float plot_height = std::max(
                120.0F,
                0.46F * ImGui::GetContentRegionAvail().y
            );
            if (ImPlot::BeginPlot(
                    "Sun-pointing error",
                    {-1.0F, plot_height}
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "error (deg)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::PlotLine(
                    "pointing error",
                    telemetry.time_s.data(),
                    telemetry.sun_pointing_error_deg.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot(
                    "Reaction-wheel spacecraft torque",
                    {-1.0F, plot_height}
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "torque (mN m)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::PlotLine(
                    "tx",
                    telemetry.time_s.data(),
                    telemetry.sun_pointing_torque_x_mNm.data(),
                    point_count
                );
                ImPlot::PlotLine(
                    "ty",
                    telemetry.time_s.data(),
                    telemetry.sun_pointing_torque_y_mNm.data(),
                    point_count
                );
                ImPlot::PlotLine(
                    "tz",
                    telemetry.time_s.data(),
                    telemetry.sun_pointing_torque_z_mNm.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Reaction wheels")) {
            const float plot_height = std::max(
                120.0F,
                0.46F * ImGui::GetContentRegionAvail().y
            );
            if (ImPlot::BeginPlot(
                    "Wheel speeds",
                    {-1.0F, plot_height}
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "speed (rpm)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                constexpr std::array<const char*, 4> labels{
                    "RW +X", "RW +Y", "RW -X", "RW -Y"
                };
                for (std::size_t index = 0; index < labels.size(); ++index) {
                    ImPlot::PlotLine(
                        labels[index],
                        telemetry.time_s.data(),
                        telemetry.wheel_speed_rpm[index].data(),
                        point_count
                    );
                }
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot(
                    "Wheel motor torque and allocation error",
                    {-1.0F, plot_height}
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "torque (mN m)",
                    0,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                constexpr std::array<const char*, 4> labels{
                    "motor +X", "motor +Y", "motor -X", "motor -Y"
                };
                for (std::size_t index = 0; index < labels.size(); ++index) {
                    ImPlot::PlotLine(
                        labels[index],
                        telemetry.time_s.data(),
                        telemetry.wheel_motor_torque_mNm[index].data(),
                        point_count
                    );
                }
                ImPlot::PlotLine(
                    "allocation error norm",
                    telemetry.time_s.data(),
                    telemetry.wheel_allocation_error_mNm.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Quaternion norm")) {
            if (ImPlot::BeginPlot(
                    "Quaternion norm error",
                    available_plot_size()
                )) {
                ImPlot::SetupAxes(
                    "simulation time (s)",
                    "|q| - 1"
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    plot_start_s,
                    plot_end_s,
                    ImPlotCond_Always
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_Y1,
                    -1.0e-12,
                    1.0e-12,
                    ImPlotCond_Always
                );
                ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1e");
                ImPlot::PlotLine(
                    "norm error",
                    telemetry.time_s.data(),
                    telemetry.quaternion_norm_error.data(),
                    point_count
                );
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

}  // namespace

int main() {
    constexpr int initial_width = 1440;
    constexpr int initial_height = 900;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(
        initial_width,
        initial_height,
        "Detumble - CubeSat attitude viewer"
    );
    SetTargetFPS(120);
    rlImGuiSetup(true);
    ImGui::GetIO().IniFilename = nullptr;
    ImPlot::CreateContext();

    SpacecraftModel spacecraft = load_spacecraft_model();
    const detumble::SpacecraftVisualConfig spacecraft_visual_config =
        detumble::generic_3u_visual_config();
    detumble::Simulation simulation;
    const detumble::EnvironmentConfig environment_config;
    const detumble::SensorSuiteConfig sensors =
        detumble::realistic_sensor_suite_config(simulation.config().seed);
    detumble::MagneticControlCycle magnetic_control{sensors.magnetometer};
    detumble::IdealGyroscope gyroscope{sensors.gyroscope};
    detumble::IdealCoarseSunSensorArray sun_sensors{sensors.sun_sensor};
    detumble::AutonomousFlightSoftware flight_software;
    detumble::ReactionWheelCluster reaction_wheels;
    Eigen::Vector3d manual_dipole_body_A_m2 = Eigen::Vector3d::Zero();
    bool automatic_bdot = true;
    detumble::MagneticControlOutput magnetic_output;
    TelemetryHistory telemetry;
    ReplayBuffer replay;
    ReplayState replay_state;
    OrbitCamera orbit_camera;
    ViewerOptions viewer_options;
    UiVisibility ui_visibility;
    SmoothedVectorOverlay dipole_overlay;
    SmoothedVectorOverlay torque_overlay;
    std::uint64_t previous_overlay_step_count = simulation.step_count();
    std::size_t previous_replay_frame_index{};
    ViewerPerformance performance;

    bool playing = true;
    int selected_scenario_index = 0;
    int requested_scenario_index = -1;
    bool show_help = false;
    float playback_speed = viewer_scenarios.front().suggested_speed;
    double time_accumulator_s = 0.0;
    reset_viewer_scenario(
        viewer_scenarios.front(),
        environment_config,
        simulation,
        magnetic_control,
        gyroscope,
        sun_sensors,
        flight_software,
        reaction_wheels,
        magnetic_output,
        telemetry,
        replay,
        replay_state
    );

    while (!WindowShouldClose()) {
        if (requested_scenario_index >= 0) {
            selected_scenario_index = requested_scenario_index;
            requested_scenario_index = -1;
            const ViewerScenario& scenario = viewer_scenarios[
                static_cast<std::size_t>(selected_scenario_index)
            ];
            reset_viewer_scenario(
                scenario,
                environment_config,
                simulation,
                magnetic_control,
                gyroscope,
                sun_sensors,
                flight_software,
                reaction_wheels,
                magnetic_output,
                telemetry,
                replay,
                replay_state
            );
            playback_speed = scenario.suggested_speed;
            time_accumulator_s = 0.0;
            playing = true;
            dipole_overlay.reset();
            torque_overlay.reset();
        }

        if (!ImGui::GetIO().WantCaptureKeyboard) {
            if (IsKeyPressed(KEY_H)) {
                show_help = !show_help;
            }
            if (IsKeyPressed(KEY_T)) {
                ui_visibility.show_telemetry =
                    !ui_visibility.show_telemetry;
            }
            if (IsKeyPressed(KEY_D)) {
                ui_visibility.show_diagnostics =
                    !ui_visibility.show_diagnostics;
            }
            if (IsKeyPressed(KEY_SPACE)) {
                if (replay_state.active) {
                    replay_state.playing = !replay_state.playing;
                } else {
                    playing = !playing;
                }
            }
            if (IsKeyPressed(KEY_R)) {
                requested_scenario_index = selected_scenario_index;
            }
            if (IsKeyPressed(KEY_C)) {
                viewer_options.orbit_overview = false;
                viewer_options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::solid;
                orbit_camera.close_up();
            }
            if (IsKeyPressed(KEY_X)) {
                viewer_options.orbit_overview = false;
                viewer_options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::cutaway;
                orbit_camera.cutaway();
            }
            if (IsKeyPressed(KEY_O)) {
                viewer_options.orbit_overview = true;
                viewer_options.spacecraft_render_mode =
                    ViewerOptions::SpacecraftRenderMode::solid;
                orbit_camera.orbit_overview();
            }
        }

        orbit_camera.update();
        const double frame_time_s = static_cast<double>(GetFrameTime());
        replay_state.update(replay, frame_time_s);
        std::uint64_t simulated_steps = 0;
        const auto simulation_start = std::chrono::steady_clock::now();

        if (playing && !replay_state.active) {
            time_accumulator_s += frame_time_s
                * static_cast<double>(playback_speed);
            while (time_accumulator_s >= simulation.config().time_step_s) {
                magnetic_output = advance_simulation(
                    simulation,
                    environment_config,
                    magnetic_control,
                    gyroscope,
                    sun_sensors,
                    flight_software,
                    reaction_wheels,
                    manual_dipole_body_A_m2,
                    automatic_bdot
                );
                record_viewer_sample(
                    simulation,
                    environment_config,
                    magnetic_output,
                    flight_software,
                    reaction_wheels.telemetry(),
                    sun_sensors.last_measurement(),
                    telemetry,
                    replay
                );
                time_accumulator_s -= simulation.config().time_step_s;
                ++simulated_steps;
            }
        }
        const double simulation_wall_time_s =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - simulation_start
            ).count();
        performance.update(
            frame_time_s,
            simulated_steps,
            simulation_wall_time_s,
            simulation.config().time_step_s
        );

        const detumble::EnvironmentState live_environment =
            current_environment(simulation, environment_config);
        const RecordedFrame live_frame = ReplayBuffer::capture(
            simulation,
            live_environment,
            magnetic_output,
            flight_software,
            reaction_wheels.telemetry(),
            sun_sensors.last_measurement()
        );
        const RecordedFrame& display_frame = replay_state.active
            ? replay_state.frame(replay)
            : live_frame;
        const detumble::EnvironmentState& environment =
            display_frame.environment;
        if (simulation.step_count() < previous_overlay_step_count) {
            dipole_overlay.reset();
            torque_overlay.reset();
        }
        if (replay_state.active
            && replay_state.frame_index < previous_replay_frame_index) {
            dipole_overlay.reset();
            torque_overlay.reset();
        }
        previous_overlay_step_count = simulation.step_count();
        previous_replay_frame_index = replay_state.active
            ? replay_state.frame_index : 0;
        dipole_overlay.update(
            display_frame.limited_dipole_body_A_m2,
            frame_time_s
        );
        const Eigen::Vector3d& control_torque_body_Nm =
            display_frame.applied_control_torque_body_Nm;
        torque_overlay.update(control_torque_body_Nm, frame_time_s);
        const Eigen::Vector3d spacecraft_origin =
            viewer_options.orbit_overview
            ? orbit_position_display(environment_config, environment)
            : Eigen::Vector3d::Zero();
        const double spacecraft_visual_scale =
            viewer_options.orbit_overview ? 0.16 : 1.0;
        const double body_axis_length =
            viewer_options.orbit_overview ? 0.55 : 2.2;
        const double magnetic_field_vector_length =
            viewer_options.orbit_overview ? 0.75 : 2.4;
        const double angular_velocity_vector_length =
            viewer_options.orbit_overview ? 0.70 : 2.7;
        const double dipole_vector_length =
            viewer_options.orbit_overview ? 0.60 : 2.1;
        const double torque_vector_length =
            viewer_options.orbit_overview ? 0.50 : 1.8;
        const double sun_vector_length =
            viewer_options.orbit_overview ? 4.5 : 3.0;
        const Eigen::Quaterniond& body_to_inertial =
            display_frame.body_to_inertial;
        const Eigen::Vector3d angular_velocity_inertial =
            detumble::rotate_body_to_inertial(
                body_to_inertial,
                display_frame.angular_velocity_body_rad_s
            );
        const Eigen::Vector3d magnetic_dipole_inertial =
            detumble::rotate_body_to_inertial(
                body_to_inertial,
                dipole_overlay.displayed_body
            );
        const Eigen::Vector3d control_torque_inertial =
            detumble::rotate_body_to_inertial(
                body_to_inertial,
                torque_overlay.displayed_body
            );
        const double dipole_reference_A_m2 = magnetic_control
            .magnetorquer_config().maximum_dipole_body_A_m2.norm();
        const double magnetic_torque_reference_Nm =
            dipole_reference_A_m2 * environment.magnetic_field_body_T.norm();
        const bool using_reaction_wheels =
            display_frame.mode == detumble::FlightMode::sun_acquire
            || display_frame.mode == detumble::FlightMode::sun_point;
        const double torque_reference_Nm = using_reaction_wheels
            ? flight_software.config().sun_pointing
                  .maximum_torque_body_Nm.norm()
            : magnetic_torque_reference_Nm;
        const double displayed_dipole_length = scaled_vector_overlay_length(
            dipole_overlay.displayed_body,
            dipole_reference_A_m2,
            dipole_vector_length
        );
        const double displayed_torque_length = scaled_vector_overlay_length(
            torque_overlay.displayed_body,
            torque_reference_Nm,
            torque_vector_length
        );
        const Camera3D camera = orbit_camera.camera();
        BeginDrawing();
        ClearBackground(Color{10, 15, 24, 255});

        BeginMode3D(camera);
        if (viewer_options.orbit_overview) {
            draw_earth_shadow(environment.sun_direction_inertial);
            if (viewer_options.show_orbit_path) {
                draw_orbit_path(environment_config);
            }
            draw_earth();
            if (viewer_options.show_sun_direction) {
                draw_sun_marker(environment.sun_direction_inertial);
            }
        } else {
            DrawGrid(12, 1.0F);
        }
        draw_spacecraft(
            spacecraft,
            body_to_inertial,
            spacecraft_origin,
            spacecraft_visual_scale,
            viewer_options.spacecraft_render_mode
        );
        if (viewer_options.spacecraft_render_mode
                != ViewerOptions::SpacecraftRenderMode::solid
            || !spacecraft.loaded_custom_glb) {
            draw_body_outline(
                spacecraft_visual_config,
                body_to_inertial,
                spacecraft_origin,
                spacecraft_visual_scale
            );
        }
        draw_sun_sensor_status(
            spacecraft_visual_config,
            display_frame.sun_measurement,
            body_to_inertial,
            spacecraft_origin,
            spacecraft_visual_scale
        );
        if (viewer_options.spacecraft_render_mode
            != ViewerOptions::SpacecraftRenderMode::solid) {
            draw_configured_components(
                spacecraft_visual_config,
                reaction_wheels.telemetry(),
                body_to_inertial,
                spacecraft_origin,
                spacecraft_visual_scale
            );
        }
        if (viewer_options.show_body_axes) {
            draw_body_axes(
                body_to_inertial,
                spacecraft_origin,
                body_axis_length
            );
        }
        if (viewer_options.show_angular_velocity) {
            draw_vector_overlay(
                spacecraft_origin,
                angular_velocity_inertial,
                angular_velocity_vector_length,
                angular_velocity_color
            );
        }
        if (viewer_options.show_magnetic_field) {
            draw_magnetic_field_vector(
                spacecraft_origin,
                environment.magnetic_field_inertial_T,
                magnetic_field_vector_length
            );
        }
        if (viewer_options.show_magnetic_dipole) {
            draw_vector_overlay(
                spacecraft_origin,
                magnetic_dipole_inertial,
                displayed_dipole_length,
                magnetic_dipole_color
            );
        }
        if (viewer_options.show_control_torque) {
            draw_vector_overlay(
                spacecraft_origin,
                control_torque_inertial,
                displayed_torque_length,
                control_torque_color
            );
        }
        if (viewer_options.show_sun_direction
            && !viewer_options.orbit_overview) {
            draw_vector_overlay(
                spacecraft_origin,
                environment.sun_direction_inertial,
                sun_vector_length,
                sun_direction_color
            );
        }
        EndMode3D();

        if (viewer_options.show_body_axes) {
            draw_body_axis_labels(
                body_to_inertial,
                spacecraft_origin,
                body_axis_length,
                camera
            );
        }
        if (viewer_options.show_angular_velocity) {
            draw_vector_overlay_label(
                spacecraft_origin,
                angular_velocity_inertial,
                angular_velocity_vector_length,
                angular_velocity_color,
                "omega",
                camera
            );
        }
        if (viewer_options.show_magnetic_field) {
            draw_magnetic_field_label(
                spacecraft_origin,
                environment.magnetic_field_inertial_T,
                magnetic_field_vector_length,
                camera
            );
        }
        if (viewer_options.show_magnetic_dipole) {
            draw_vector_overlay_label(
                spacecraft_origin,
                magnetic_dipole_inertial,
                displayed_dipole_length,
                magnetic_dipole_color,
                "m command",
                camera
            );
        }
        if (viewer_options.show_control_torque) {
            draw_vector_overlay_label(
                spacecraft_origin,
                control_torque_inertial,
                displayed_torque_length,
                control_torque_color,
                "tau applied",
                camera
            );
        }
        if (viewer_options.show_sun_direction) {
            draw_vector_overlay_label(
                viewer_options.orbit_overview
                    ? Eigen::Vector3d::Zero()
                    : spacecraft_origin,
                environment.sun_direction_inertial,
                sun_vector_length,
                sun_direction_color,
                "Sun",
                camera
            );
        }

        DrawText(
            environment.in_eclipse ? "ECLIPSE" : "SUNLIT",
            GetScreenWidth() / 2 - 45,
            20,
            24,
            environment.in_eclipse
                ? Color{255, 120, 75, 255}
                : sun_direction_color
        );

        DrawText(
            TextFormat("Detumble %s", detumble::version().data()),
            GetScreenWidth() - 210,
            GetScreenHeight() - 30,
            18,
            GRAY
        );

        rlImGuiBegin();
        draw_control_panel(
            simulation,
            environment_config,
            environment,
            magnetic_control,
            gyroscope,
            sun_sensors,
            flight_software,
            reaction_wheels,
            manual_dipole_body_A_m2,
            automatic_bdot,
            magnetic_output,
            telemetry,
            replay,
            replay_state,
            viewer_options,
            ui_visibility,
            orbit_camera,
            playing,
            playback_speed,
            time_accumulator_s,
            selected_scenario_index,
            requested_scenario_index,
            show_help,
            performance,
            spacecraft.loaded_custom_glb
        );
        draw_scenario_status_panel(
            viewer_scenarios[static_cast<std::size_t>(
                selected_scenario_index
            )],
            display_frame,
            replay_state.active,
            ui_visibility.show_diagnostics
        );
        if (!replay_state.active && ui_visibility.show_diagnostics) {
            draw_magnetic_control_panel(
                manual_dipole_body_A_m2,
                automatic_bdot,
                magnetic_control,
                flight_software,
                environment
            );
            draw_reaction_wheel_panel(reaction_wheels);
        }
        if (ui_visibility.show_telemetry) {
            draw_telemetry_plots(telemetry);
        }
        draw_help_window(show_help);
        rlImGuiEnd();

        EndDrawing();
    }

    UnloadModel(spacecraft.model);
    ImPlot::DestroyContext();
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
