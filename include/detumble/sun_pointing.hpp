#pragma once

#include <numbers>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace detumble {

struct SunPointingControllerConfig {
    Eigen::Vector3d sun_axis_body{Eigen::Vector3d::UnitZ()};
    Eigen::Vector3d roll_axis_body{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d roll_reference_inertial{Eigen::Vector3d::UnitZ()};
    Eigen::Vector3d proportional_gain_body_Nm_per_rad{
        0.0015,
        0.0015,
        0.0015
    };
    Eigen::Vector3d derivative_gain_body_Nm_s_per_rad{
        0.016,
        0.016,
        0.0065
    };
    Eigen::Vector3d maximum_torque_body_Nm{
        0.001,
        0.001,
        0.001
    };
    double attitude_deadband_rad{0.1 * std::numbers::pi / 180.0};
    double rate_deadband_rad_s{0.01 * std::numbers::pi / 180.0};
    double acquisition_threshold_rad{10.0 * std::numbers::pi / 180.0};
    double steady_pointing_threshold_rad{
        2.0 * std::numbers::pi / 180.0
    };
    double hysteresis_rad{2.0 * std::numbers::pi / 180.0};
    double dwell_time_s{10.0};
};

struct SunPointingCommand {
    Eigen::Quaterniond desired_body_to_inertial{
        Eigen::Quaterniond::Identity()
    };
    Eigen::Vector3d attitude_error_body_rad{Eigen::Vector3d::Zero()};
    Eigen::Vector3d requested_torque_body_Nm{Eigen::Vector3d::Zero()};
    Eigen::Vector3d limited_torque_body_Nm{Eigen::Vector3d::Zero()};
    double pointing_error_rad{};
    bool in_deadband{};
    bool saturated{};
    bool active{};
};

struct SunPointingTrackerState {
    double sample_time_s{};
    double acquisition_elapsed_s{};
    double steady_pointing_elapsed_s{};
    bool target_acquired{};
    bool steady_pointing{};
};

[[nodiscard]] Eigen::Quaterniond desired_sun_pointing_attitude(
    const SunPointingControllerConfig& config,
    const Eigen::Vector3d& sun_direction_inertial
);

[[nodiscard]] Eigen::Vector3d shortest_attitude_error_body_rad(
    const Eigen::Quaterniond& estimated_body_to_inertial,
    const Eigen::Quaterniond& desired_body_to_inertial
);

[[nodiscard]] SunPointingCommand sun_pointing_command(
    const SunPointingControllerConfig& config,
    const Eigen::Quaterniond& estimated_body_to_inertial,
    const Eigen::Vector3d& measured_angular_velocity_body_rad_s,
    const Eigen::Quaterniond& desired_body_to_inertial
);

class SunPointingTracker {
public:
    explicit SunPointingTracker(SunPointingControllerConfig config = {});

    void reset();
    void update(double sample_time_s, double pointing_error_rad);

    [[nodiscard]] const SunPointingControllerConfig& config() const;
    [[nodiscard]] const SunPointingTrackerState& state() const;

private:
    SunPointingControllerConfig config_;
    SunPointingTrackerState state_;
    bool has_sample_time_{};
    bool acquisition_timer_active_{};
    bool steady_timer_active_{};
    double acquisition_start_time_s_{};
    double steady_start_time_s_{};
};

}  // namespace detumble
