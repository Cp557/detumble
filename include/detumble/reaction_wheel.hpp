#pragma once

#include <cstddef>
#include <numbers>
#include <string_view>
#include <vector>

#include <Eigen/Core>

namespace detumble {

struct ReactionWheelDefinition {
    std::string_view name;
    Eigen::Vector3d spin_axis_body{Eigen::Vector3d::UnitZ()};
    double rotor_inertia_kg_m2{1.0e-5};
    double maximum_motor_torque_Nm{1.0e-3};
    double maximum_speed_rad_s{200.0 * std::numbers::pi};
    double initial_speed_rad_s{};
};

struct ReactionWheelClusterConfig {
    std::vector<ReactionWheelDefinition> wheels;
};

struct ReactionWheelCommand {
    double sample_time_s{};
    Eigen::Vector3d requested_spacecraft_torque_body_Nm{
        Eigen::Vector3d::Zero()
    };
    bool enabled{};
};

struct ReactionWheelTelemetry {
    double sample_time_s{};
    Eigen::Vector3d requested_spacecraft_torque_body_Nm{
        Eigen::Vector3d::Zero()
    };
    Eigen::Vector3d applied_spacecraft_torque_body_Nm{
        Eigen::Vector3d::Zero()
    };
    Eigen::Vector3d allocation_error_body_Nm{Eigen::Vector3d::Zero()};
    Eigen::Vector3d stored_momentum_body_Nm_s{Eigen::Vector3d::Zero()};
    std::vector<double> commanded_motor_torque_Nm;
    std::vector<double> applied_motor_torque_Nm;
    std::vector<double> wheel_speed_rad_s;
    std::vector<double> wheel_angle_rad;
    std::vector<bool> torque_saturated;
    std::vector<bool> speed_saturated;
    std::vector<bool> failed;
    bool allocation_saturated{};
};

[[nodiscard]] ReactionWheelClusterConfig
generic_4_wheel_pyramid_config();

class ReactionWheelCluster {
public:
    explicit ReactionWheelCluster(
        ReactionWheelClusterConfig config =
            generic_4_wheel_pyramid_config()
    );

    void reset();
    void set_wheel_failed(std::size_t wheel_index, bool failed);
    void update(const ReactionWheelCommand& command, double time_step_s);

    [[nodiscard]] const ReactionWheelClusterConfig& config() const;
    [[nodiscard]] const ReactionWheelTelemetry& telemetry() const;

private:
    ReactionWheelClusterConfig config_;
    ReactionWheelTelemetry telemetry_;
    std::vector<bool> failed_;
    bool has_sample_time_{};
};

}  // namespace detumble
