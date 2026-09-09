#pragma once

#include <numbers>
#include <optional>
#include <string_view>
#include <vector>

#include <Eigen/Core>

#include "detumble/attitude_estimator.hpp"
#include "detumble/bdot.hpp"
#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/sun_pointing.hpp"

namespace detumble {

enum class FlightMode {
    boot,
    detumble,
    sun_acquire,
    sun_point,
    safe
};

[[nodiscard]] std::string_view to_string(FlightMode mode);

struct DetumbleFlightSoftwareConfig {
    BdotEstimatorConfig estimator;
    BdotControllerConfig controller;
    double completion_threshold_rad_s{0.5 * std::numbers::pi / 180.0};
    double completion_hysteresis_rad_s{0.2 * std::numbers::pi / 180.0};
    double completion_dwell_time_s{30.0};
};

struct DetumbleFlightSoftwareState {
    FlightMode mode{FlightMode::boot};
    Eigen::Vector3d commanded_dipole_body_A_m2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d measured_angular_velocity_body_rad_s{
        Eigen::Vector3d::Zero()
    };
    Eigen::Vector3d raw_bdot_body_T_s{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_bdot_body_T_s{Eigen::Vector3d::Zero()};
    double below_threshold_elapsed_s{};
    bool detumble_complete{};
};

class DetumbleFlightSoftware {
public:
    explicit DetumbleFlightSoftware(
        DetumbleFlightSoftwareConfig config = {},
        MagnetorquerConfig magnetorquer_config = {}
    );

    void reset();
    void update(
        const MagnetometerMeasurement& magnetometer,
        const GyroscopeMeasurement& gyroscope
    );

    [[nodiscard]] const DetumbleFlightSoftwareConfig& config() const;
    [[nodiscard]] const MagnetorquerConfig& magnetorquer_config() const;
    [[nodiscard]] const DetumbleFlightSoftwareState& state() const;

private:
    void update_completion(const GyroscopeMeasurement& gyroscope);

    DetumbleFlightSoftwareConfig config_;
    MagnetorquerConfig magnetorquer_config_;
    BdotEstimator estimator_;
    DetumbleFlightSoftwareState state_;
    std::optional<double> below_threshold_start_s_;
};

enum class ModeTransitionReason {
    sensors_ready,
    detumble_complete,
    sun_unavailable,
    attitude_invalid,
    sun_acquired,
    pointing_lost,
    navigation_recovered
};

[[nodiscard]] std::string_view to_string(ModeTransitionReason reason);

struct ModeTransitionEvent {
    double sample_time_s{};
    FlightMode old_mode{FlightMode::boot};
    FlightMode new_mode{FlightMode::boot};
    ModeTransitionReason reason{ModeTransitionReason::sensors_ready};
};

struct ActuatorCommand {
    double sample_time_s{};
    Eigen::Vector3d magnetorquer_dipole_body_A_m2{
        Eigen::Vector3d::Zero()
    };
    Eigen::Vector3d reaction_wheel_torque_body_Nm{
        Eigen::Vector3d::Zero()
    };
    bool magnetorquers_enabled{};
    bool reaction_wheels_enabled{};
};

struct AutonomousFlightSoftwareInput {
    GyroscopeMeasurement gyroscope;
    std::optional<MagnetometerMeasurement> magnetometer;
    std::optional<CoarseSunSensorMeasurement> sun_sensor;
    Eigen::Vector3d magnetic_field_inertial_T{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sun_direction_inertial{Eigen::Vector3d::UnitX()};
    bool in_eclipse{};
};

struct AutonomousFlightSoftwareConfig {
    DetumbleFlightSoftwareConfig detumble;
    AttitudeEstimatorConfig attitude_estimator;
    SunPointingControllerConfig sun_pointing;
    double boot_to_detumble_dwell_time_s{0.1};
    double post_detumble_dwell_time_s{0.5};
    double acquire_to_point_dwell_time_s{0.5};
    double pointing_loss_dwell_time_s{2.0};
    double safe_entry_dwell_time_s{0.5};
    double safe_recovery_dwell_time_s{2.0};
};

struct AutonomousFlightSoftwareState {
    double sample_time_s{};
    double mode_elapsed_s{};
    double pending_transition_elapsed_s{};
    FlightMode mode{FlightMode::boot};
    bool sun_available{};
    bool attitude_valid{};
    ActuatorCommand actuator_command;
    SunPointingCommand sun_pointing;
};

class AutonomousFlightSoftware {
public:
    explicit AutonomousFlightSoftware(
        AutonomousFlightSoftwareConfig config = {},
        MagnetorquerConfig magnetorquer_config = {}
    );

    void reset();
    void update(const AutonomousFlightSoftwareInput& input);

    [[nodiscard]] const AutonomousFlightSoftwareConfig& config() const;
    [[nodiscard]] const AutonomousFlightSoftwareState& state() const;
    [[nodiscard]] const DetumbleFlightSoftware& detumble() const;
    [[nodiscard]] const AttitudeEstimator& attitude_estimator() const;
    [[nodiscard]] const SunPointingTracker& sun_pointing_tracker() const;
    [[nodiscard]] const std::vector<ModeTransitionEvent>& events() const;

private:
    void evaluate_mode(const AutonomousFlightSoftwareInput& input);
    void update_pointing(const AutonomousFlightSoftwareInput& input);
    void request_transition(
        FlightMode new_mode,
        ModeTransitionReason reason,
        double dwell_time_s
    );
    void clear_pending_transition();
    void transition_to(FlightMode new_mode, ModeTransitionReason reason);
    void update_actuator_command();

    AutonomousFlightSoftwareConfig config_;
    DetumbleFlightSoftware detumble_;
    AttitudeEstimator attitude_estimator_;
    SunPointingTracker sun_pointing_tracker_;
    AutonomousFlightSoftwareState state_;
    std::vector<ModeTransitionEvent> events_;
    std::optional<FlightMode> pending_mode_;
    std::optional<ModeTransitionReason> pending_reason_;
    double pending_start_time_s_{};
    double mode_start_time_s_{};
    bool has_sample_time_{};
};

}  // namespace detumble
