#include "detumble/magnetorquer.hpp"

#include <stdexcept>

#include <Eigen/Geometry>

namespace detumble {

Eigen::Vector3d saturate_magnetorquer_dipole_body_A_m2(
    const MagnetorquerConfig& config,
    const Eigen::Vector3d& commanded_dipole_body_A_m2
) {
    if (!config.maximum_dipole_body_A_m2.allFinite()
        || (config.maximum_dipole_body_A_m2.array() < 0.0).any()) {
        throw std::invalid_argument{
            "Magnetorquer dipole limits must be finite and nonnegative"
        };
    }
    if (!commanded_dipole_body_A_m2.allFinite()) {
        throw std::invalid_argument{
            "Commanded magnetorquer dipole must be finite"
        };
    }

    return commanded_dipole_body_A_m2.cwiseMax(
        -config.maximum_dipole_body_A_m2
    ).cwiseMin(config.maximum_dipole_body_A_m2);
}

Eigen::Vector3d magnetic_torque_body_Nm(
    const Eigen::Vector3d& applied_dipole_body_A_m2,
    const Eigen::Vector3d& magnetic_field_body_T
) {
    if (!applied_dipole_body_A_m2.allFinite()) {
        throw std::invalid_argument{
            "Applied magnetorquer dipole must be finite"
        };
    }
    if (!magnetic_field_body_T.allFinite()) {
        throw std::invalid_argument{
            "Magnetic field must be finite"
        };
    }

    return applied_dipole_body_A_m2.cross(magnetic_field_body_T);
}

}  // namespace detumble
