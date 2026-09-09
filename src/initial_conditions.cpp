#include "detumble/initial_conditions.hpp"

#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>

namespace detumble {
namespace {

double uniform_unit_interval(std::mt19937_64& generator) {
    constexpr double inverse_two_to_53 = 1.0 / 9007199254740992.0;
    return static_cast<double>(generator() >> 11U) * inverse_two_to_53;
}

Eigen::Quaterniond random_unit_quaternion(std::mt19937_64& generator) {
    const double u1 = uniform_unit_interval(generator);
    const double u2 = uniform_unit_interval(generator);
    const double u3 = uniform_unit_interval(generator);
    const double first_radius = std::sqrt(1.0 - u1);
    const double second_radius = std::sqrt(u1);
    const double first_angle = 2.0 * std::numbers::pi * u2;
    const double second_angle = 2.0 * std::numbers::pi * u3;

    const double x = first_radius * std::sin(first_angle);
    const double y = first_radius * std::cos(first_angle);
    const double z = second_radius * std::sin(second_angle);
    const double w = second_radius * std::cos(second_angle);
    return Eigen::Quaterniond{w, x, y, z};
}

Eigen::Vector3d random_unit_vector(std::mt19937_64& generator) {
    const double z = 2.0 * uniform_unit_interval(generator) - 1.0;
    const double azimuth_rad =
        2.0 * std::numbers::pi * uniform_unit_interval(generator);
    const double radial = std::sqrt(1.0 - z * z);

    return {
        radial * std::cos(azimuth_rad),
        radial * std::sin(azimuth_rad),
        z
    };
}

}  // namespace

AttitudeState random_attitude_state(
    const std::uint64_t seed,
    const double minimum_angular_speed_rad_s,
    const double maximum_angular_speed_rad_s
) {
    if (!std::isfinite(minimum_angular_speed_rad_s)
        || !std::isfinite(maximum_angular_speed_rad_s)
        || minimum_angular_speed_rad_s < 0.0
        || maximum_angular_speed_rad_s < minimum_angular_speed_rad_s) {
        throw std::invalid_argument{
            "Angular-speed bounds must be finite, nonnegative, and ordered"
        };
    }

    std::mt19937_64 generator{seed};
    const double angular_speed_rad_s = minimum_angular_speed_rad_s
        + (maximum_angular_speed_rad_s - minimum_angular_speed_rad_s)
            * uniform_unit_interval(generator);

    return {
        .body_to_inertial = random_unit_quaternion(generator),
        .angular_velocity_body_rad_s =
            angular_speed_rad_s * random_unit_vector(generator)
    };
}

}  // namespace detumble
