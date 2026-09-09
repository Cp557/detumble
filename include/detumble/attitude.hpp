#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace detumble {

struct AttitudeState {
    Eigen::Quaterniond body_to_inertial{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_velocity_body_rad_s{Eigen::Vector3d::Zero()};
};

[[nodiscard]] Eigen::Quaterniond normalized_attitude(
    const Eigen::Quaterniond& body_to_inertial
);

[[nodiscard]] Eigen::Quaterniond integrate_attitude(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& angular_velocity_body_rad_s,
    double time_step_s
);

}  // namespace detumble
