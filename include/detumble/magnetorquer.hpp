#pragma once

#include <Eigen/Core>

namespace detumble {

struct MagnetorquerConfig {
    Eigen::Vector3d maximum_dipole_body_A_m2{
        Eigen::Vector3d::Constant(0.2)
    };
};

[[nodiscard]] Eigen::Vector3d saturate_magnetorquer_dipole_body_A_m2(
    const MagnetorquerConfig& config,
    const Eigen::Vector3d& commanded_dipole_body_A_m2
);

[[nodiscard]] Eigen::Vector3d magnetic_torque_body_Nm(
    const Eigen::Vector3d& applied_dipole_body_A_m2,
    const Eigen::Vector3d& magnetic_field_body_T
);

}  // namespace detumble
