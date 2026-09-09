#include "detumble/attitude.hpp"

#include <cmath>
#include <stdexcept>

namespace detumble {

Eigen::Quaterniond normalized_attitude(
    const Eigen::Quaterniond& body_to_inertial
) {
    constexpr double minimum_norm = 1.0e-12;

    const double norm = body_to_inertial.norm();
    if (!std::isfinite(norm) || norm < minimum_norm) {
        throw std::invalid_argument{
            "Attitude quaternion must have a finite, nonzero norm"
        };
    }

    return body_to_inertial.normalized();
}

Eigen::Quaterniond integrate_attitude(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& angular_velocity_body_rad_s,
    const double time_step_s
) {
    if (!std::isfinite(time_step_s) || time_step_s < 0.0) {
        throw std::invalid_argument{
            "Attitude integration timestep must be finite and nonnegative"
        };
    }
    if (!angular_velocity_body_rad_s.allFinite()) {
        throw std::invalid_argument{"Angular velocity must be finite"};
    }

    const Eigen::Quaterniond attitude = normalized_attitude(body_to_inertial);
    const Eigen::Vector3d rotation_vector =
        angular_velocity_body_rad_s * time_step_s;
    const double rotation_angle_rad = rotation_vector.norm();

    if (rotation_angle_rad == 0.0) {
        return attitude;
    }

    const Eigen::Vector3d rotation_axis =
        rotation_vector / rotation_angle_rad;
    const Eigen::Quaterniond incremental_rotation{
        Eigen::AngleAxisd{rotation_angle_rad, rotation_axis}
    };

    return normalized_attitude(attitude * incremental_rotation);
}

}  // namespace detumble
