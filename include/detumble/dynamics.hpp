#pragma once

#include <Eigen/Core>

#include "detumble/attitude.hpp"

namespace detumble {

struct RigidBodyProperties {
    double mass_kg{};
    Eigen::Vector3d dimensions_body_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d center_of_mass_body_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d principal_moments_body_kg_m2{Eigen::Vector3d::Zero()};
};

struct RotationalMetrics {
    Eigen::Vector3d angular_momentum_body_kg_m2_s{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_momentum_inertial_kg_m2_s{
        Eigen::Vector3d::Zero()
    };
    double rotational_kinetic_energy_j{};
    double quaternion_norm{};
};

[[nodiscard]] RigidBodyProperties generic_3u_cubesat();

[[nodiscard]] Eigen::Vector3d angular_acceleration_body_rad_s2(
    const RigidBodyProperties& body,
    const Eigen::Vector3d& angular_velocity_body_rad_s,
    const Eigen::Vector3d& applied_torque_body_Nm
);

[[nodiscard]] AttitudeState propagate_rigid_body_rk4(
    const RigidBodyProperties& body,
    const AttitudeState& state,
    const Eigen::Vector3d& applied_torque_body_Nm,
    double time_step_s
);

[[nodiscard]] RotationalMetrics rotational_metrics(
    const RigidBodyProperties& body,
    const AttitudeState& state
);

}  // namespace detumble
