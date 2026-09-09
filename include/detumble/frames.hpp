#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace detumble {

[[nodiscard]] Eigen::Vector3d rotate_body_to_inertial(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& vector_body
);

[[nodiscard]] Eigen::Vector3d rotate_inertial_to_body(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& vector_inertial
);

}  // namespace detumble
