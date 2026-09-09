#include "detumble/reaction_wheel.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>
#include <Eigen/QR>

namespace detumble {
namespace {

constexpr double minimum_axis_norm = 1.0e-12;
constexpr double comparison_tolerance = 1.0e-15;
constexpr double allocation_error_tolerance_Nm = 1.0e-12;

void validate_config(const ReactionWheelClusterConfig& config) {
    if (config.wheels.empty()) {
        throw std::invalid_argument{
            "Reaction-wheel cluster requires at least one wheel"
        };
    }
    for (const ReactionWheelDefinition& wheel : config.wheels) {
        if (wheel.name.empty()
            || !wheel.spin_axis_body.allFinite()
            || wheel.spin_axis_body.norm() < minimum_axis_norm
            || !std::isfinite(wheel.rotor_inertia_kg_m2)
            || wheel.rotor_inertia_kg_m2 <= 0.0
            || !std::isfinite(wheel.maximum_motor_torque_Nm)
            || wheel.maximum_motor_torque_Nm <= 0.0
            || !std::isfinite(wheel.maximum_speed_rad_s)
            || wheel.maximum_speed_rad_s <= 0.0
            || !std::isfinite(wheel.initial_speed_rad_s)
            || std::abs(wheel.initial_speed_rad_s)
                > wheel.maximum_speed_rad_s) {
            throw std::invalid_argument{
                "Reaction-wheel definitions must contain valid axes and limits"
            };
        }
    }
}

Eigen::MatrixXd active_axis_matrix(
    const ReactionWheelClusterConfig& config,
    const std::vector<bool>& failed,
    std::vector<std::size_t>& active_indices
) {
    active_indices.clear();
    for (std::size_t index = 0; index < config.wheels.size(); ++index) {
        if (!failed[index]) {
            active_indices.push_back(index);
        }
    }

    Eigen::MatrixXd axes{
        3,
        static_cast<Eigen::Index>(active_indices.size())
    };
    for (std::size_t column = 0; column < active_indices.size(); ++column) {
        axes.col(static_cast<Eigen::Index>(column)) =
            config.wheels[active_indices[column]].spin_axis_body.normalized();
    }
    return axes;
}

}  // namespace

ReactionWheelClusterConfig generic_4_wheel_pyramid_config() {
    constexpr double axis_component = 0.7071067811865476;
    constexpr double maximum_speed_rad_s =
        6'000.0 * 2.0 * std::numbers::pi / 60.0;
    return {
        .wheels = {
            ReactionWheelDefinition{
                .name = "ReactionWheel_PosX",
                .spin_axis_body = {axis_component, 0.0, axis_component},
                .maximum_speed_rad_s = maximum_speed_rad_s
            },
            ReactionWheelDefinition{
                .name = "ReactionWheel_PosY",
                .spin_axis_body = {0.0, axis_component, axis_component},
                .maximum_speed_rad_s = maximum_speed_rad_s
            },
            ReactionWheelDefinition{
                .name = "ReactionWheel_NegX",
                .spin_axis_body = {-axis_component, 0.0, axis_component},
                .maximum_speed_rad_s = maximum_speed_rad_s
            },
            ReactionWheelDefinition{
                .name = "ReactionWheel_NegY",
                .spin_axis_body = {0.0, -axis_component, axis_component},
                .maximum_speed_rad_s = maximum_speed_rad_s
            }
        }
    };
}

ReactionWheelCluster::ReactionWheelCluster(ReactionWheelClusterConfig config)
    : config_{std::move(config)} {
    validate_config(config_);
    for (ReactionWheelDefinition& wheel : config_.wheels) {
        wheel.spin_axis_body.normalize();
    }
    failed_.assign(config_.wheels.size(), false);
    reset();
}

void ReactionWheelCluster::reset() {
    telemetry_ = {};
    const std::size_t wheel_count = config_.wheels.size();
    telemetry_.commanded_motor_torque_Nm.assign(wheel_count, 0.0);
    telemetry_.applied_motor_torque_Nm.assign(wheel_count, 0.0);
    telemetry_.wheel_speed_rad_s.resize(wheel_count);
    telemetry_.wheel_angle_rad.assign(wheel_count, 0.0);
    telemetry_.torque_saturated.assign(wheel_count, false);
    telemetry_.speed_saturated.assign(wheel_count, false);
    telemetry_.failed = failed_;
    for (std::size_t index = 0; index < wheel_count; ++index) {
        telemetry_.wheel_speed_rad_s[index] =
            config_.wheels[index].initial_speed_rad_s;
    }
    has_sample_time_ = false;
}

void ReactionWheelCluster::set_wheel_failed(
    const std::size_t wheel_index,
    const bool failed
) {
    if (wheel_index >= failed_.size()) {
        throw std::out_of_range{"Reaction-wheel index is out of range"};
    }
    failed_[wheel_index] = failed;
    telemetry_.failed = failed_;
}

void ReactionWheelCluster::update(
    const ReactionWheelCommand& command,
    const double time_step_s
) {
    if (!std::isfinite(command.sample_time_s) || command.sample_time_s < 0.0
        || (has_sample_time_
            && command.sample_time_s < telemetry_.sample_time_s)
        || !command.requested_spacecraft_torque_body_Nm.allFinite()
        || !std::isfinite(time_step_s) || time_step_s <= 0.0) {
        throw std::invalid_argument{
            "Reaction-wheel commands require valid chronological inputs"
        };
    }

    const std::size_t wheel_count = config_.wheels.size();
    telemetry_.sample_time_s = command.sample_time_s;
    telemetry_.requested_spacecraft_torque_body_Nm = command.enabled
        ? command.requested_spacecraft_torque_body_Nm
        : Eigen::Vector3d::Zero();
    telemetry_.commanded_motor_torque_Nm.assign(wheel_count, 0.0);
    telemetry_.applied_motor_torque_Nm.assign(wheel_count, 0.0);
    telemetry_.torque_saturated.assign(wheel_count, false);
    telemetry_.speed_saturated.assign(wheel_count, false);
    telemetry_.failed = failed_;
    telemetry_.allocation_saturated = false;

    std::vector<std::size_t> active_indices;
    const Eigen::MatrixXd axes = active_axis_matrix(
        config_,
        failed_,
        active_indices
    );
    if (command.enabled && !active_indices.empty()) {
        const Eigen::VectorXd motor_torque =
            axes.completeOrthogonalDecomposition().solve(
                -telemetry_.requested_spacecraft_torque_body_Nm
            );
        for (std::size_t column = 0; column < active_indices.size(); ++column) {
            telemetry_.commanded_motor_torque_Nm[active_indices[column]] =
                motor_torque(static_cast<Eigen::Index>(column));
        }
    }

    for (std::size_t index = 0; index < wheel_count; ++index) {
        if (failed_[index]) {
            continue;
        }
        const ReactionWheelDefinition& wheel = config_.wheels[index];
        const double requested_motor_torque =
            telemetry_.commanded_motor_torque_Nm[index];
        double applied_motor_torque = std::clamp(
            requested_motor_torque,
            -wheel.maximum_motor_torque_Nm,
            wheel.maximum_motor_torque_Nm
        );
        telemetry_.torque_saturated[index] = std::abs(
            applied_motor_torque - requested_motor_torque
        ) > comparison_tolerance;

        const double old_speed = telemetry_.wheel_speed_rad_s[index];
        if (std::abs(old_speed) >= wheel.maximum_speed_rad_s
            && applied_motor_torque * old_speed > 0.0) {
            applied_motor_torque = 0.0;
            telemetry_.speed_saturated[index] = true;
        }

        const double unconstrained_speed = old_speed
            + applied_motor_torque / wheel.rotor_inertia_kg_m2 * time_step_s;
        const double new_speed = std::clamp(
            unconstrained_speed,
            -wheel.maximum_speed_rad_s,
            wheel.maximum_speed_rad_s
        );
        if (std::abs(new_speed - unconstrained_speed)
            > comparison_tolerance) {
            applied_motor_torque =
                (new_speed - old_speed) * wheel.rotor_inertia_kg_m2
                / time_step_s;
            telemetry_.speed_saturated[index] = true;
        }
        telemetry_.applied_motor_torque_Nm[index] = applied_motor_torque;
        telemetry_.wheel_speed_rad_s[index] = new_speed;
        telemetry_.wheel_angle_rad[index] = std::remainder(
            telemetry_.wheel_angle_rad[index]
                + 0.5 * (old_speed + new_speed) * time_step_s,
            2.0 * std::numbers::pi
        );
    }

    telemetry_.applied_spacecraft_torque_body_Nm.setZero();
    telemetry_.stored_momentum_body_Nm_s.setZero();
    for (std::size_t index = 0; index < wheel_count; ++index) {
        const ReactionWheelDefinition& wheel = config_.wheels[index];
        telemetry_.applied_spacecraft_torque_body_Nm -=
            wheel.spin_axis_body
            * telemetry_.applied_motor_torque_Nm[index];
        telemetry_.stored_momentum_body_Nm_s +=
            wheel.spin_axis_body * wheel.rotor_inertia_kg_m2
            * telemetry_.wheel_speed_rad_s[index];
        telemetry_.allocation_saturated =
            telemetry_.allocation_saturated
            || telemetry_.torque_saturated[index]
            || telemetry_.speed_saturated[index];
    }
    telemetry_.allocation_error_body_Nm =
        telemetry_.requested_spacecraft_torque_body_Nm
        - telemetry_.applied_spacecraft_torque_body_Nm;
    telemetry_.allocation_saturated = telemetry_.allocation_saturated
        || telemetry_.allocation_error_body_Nm.norm()
            > allocation_error_tolerance_Nm;
    has_sample_time_ = true;
}

const ReactionWheelClusterConfig& ReactionWheelCluster::config() const {
    return config_;
}

const ReactionWheelTelemetry& ReactionWheelCluster::telemetry() const {
    return telemetry_;
}

}  // namespace detumble
