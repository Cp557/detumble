#include "detumble/frames.hpp"

#include "detumble/attitude.hpp"

namespace detumble {

Eigen::Vector3d rotate_body_to_inertial(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& vector_body
) {
    return normalized_attitude(body_to_inertial) * vector_body;
}

Eigen::Vector3d rotate_inertial_to_body(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& vector_inertial
) {
    return normalized_attitude(body_to_inertial).conjugate()
        * vector_inertial;
}

}  // namespace detumble
