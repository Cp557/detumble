#include "detumble/environment.hpp"

#include <cmath>
#include <stdexcept>

#include "detumble/frames.hpp"

namespace detumble {

bool earth_occults_sun(
    const Eigen::Vector3d& spacecraft_position_inertial_m,
    const Eigen::Vector3d& sun_direction_inertial,
    const double earth_radius_m
) {
    if (!spacecraft_position_inertial_m.allFinite()
        || !sun_direction_inertial.allFinite()
        || sun_direction_inertial.isZero()
        || !std::isfinite(earth_radius_m)
        || earth_radius_m <= 0.0
        || spacecraft_position_inertial_m.norm() <= earth_radius_m) {
        throw std::invalid_argument{
            "Eclipse geometry requires a valid position, Sun direction, "
            "and Earth radius"
        };
    }

    const Eigen::Vector3d sun_direction = sun_direction_inertial.normalized();
    const double distance_to_closest_point_m =
        -spacecraft_position_inertial_m.dot(sun_direction);
    if (distance_to_closest_point_m <= 0.0) {
        return false;
    }

    const Eigen::Vector3d closest_point = spacecraft_position_inertial_m
        + distance_to_closest_point_m * sun_direction;
    return closest_point.norm() <= earth_radius_m;
}

EnvironmentState sample_environment(
    const EnvironmentConfig& config,
    const Eigen::Quaterniond& body_to_inertial,
    const double elapsed_time_s
) {
    const OrbitState orbit = circular_orbit_state(
        config.orbit,
        elapsed_time_s
    );
    const Eigen::Vector3d magnetic_field_inertial_T =
        tilted_dipole_field_inertial_T(
            config.magnetic_field,
            orbit.position_inertial_m,
            elapsed_time_s
        );
    if (!config.sun.direction_inertial.allFinite()
        || config.sun.direction_inertial.isZero()) {
        throw std::invalid_argument{
            "Inertial Sun direction must be finite and nonzero"
        };
    }
    const Eigen::Vector3d sun_direction_inertial =
        config.sun.direction_inertial.normalized();

    return {
        .orbit = orbit,
        .magnetic_field_inertial_T = magnetic_field_inertial_T,
        .magnetic_field_body_T = rotate_inertial_to_body(
            body_to_inertial,
            magnetic_field_inertial_T
        ),
        .sun_direction_inertial = sun_direction_inertial,
        .sun_direction_body = rotate_inertial_to_body(
            body_to_inertial,
            sun_direction_inertial
        ),
        .in_eclipse = earth_occults_sun(
            orbit.position_inertial_m,
            sun_direction_inertial,
            config.orbit.earth_radius_m
        )
    };
}

}  // namespace detumble
