#include "detumble/orbit.hpp"

#include <cmath>
#include <stdexcept>

namespace detumble {
namespace {

void validate_config(const CircularOrbitConfig& config) {
    if (!std::isfinite(config.earth_radius_m)
        || config.earth_radius_m <= 0.0) {
        throw std::invalid_argument{
            "Earth radius must be finite and positive"
        };
    }
    if (!std::isfinite(config.altitude_m) || config.altitude_m < 0.0) {
        throw std::invalid_argument{
            "Orbit altitude must be finite and nonnegative"
        };
    }
    if (!std::isfinite(config.gravitational_parameter_m3_s2)
        || config.gravitational_parameter_m3_s2 <= 0.0) {
        throw std::invalid_argument{
            "Gravitational parameter must be finite and positive"
        };
    }
    if (!std::isfinite(config.inclination_rad)
        || config.inclination_rad < 0.0
        || config.inclination_rad > std::numbers::pi) {
        throw std::invalid_argument{
            "Orbit inclination must be between zero and pi radians"
        };
    }
    if (!std::isfinite(config.initial_argument_of_latitude_rad)) {
        throw std::invalid_argument{
            "Initial argument of latitude must be finite"
        };
    }
}

}  // namespace

double circular_orbit_radius_m(const CircularOrbitConfig& config) {
    validate_config(config);
    return config.earth_radius_m + config.altitude_m;
}

double circular_orbit_mean_motion_rad_s(
    const CircularOrbitConfig& config
) {
    const double radius_m = circular_orbit_radius_m(config);
    return std::sqrt(
        config.gravitational_parameter_m3_s2
        / (radius_m * radius_m * radius_m)
    );
}

double circular_orbit_period_s(const CircularOrbitConfig& config) {
    return 2.0 * std::numbers::pi
        / circular_orbit_mean_motion_rad_s(config);
}

OrbitState circular_orbit_state(
    const CircularOrbitConfig& config,
    const double elapsed_time_s
) {
    validate_config(config);
    if (!std::isfinite(elapsed_time_s)) {
        throw std::invalid_argument{"Orbit time must be finite"};
    }

    const double radius_m = circular_orbit_radius_m(config);
    const double mean_motion_rad_s =
        circular_orbit_mean_motion_rad_s(config);
    const double argument_of_latitude_rad = std::remainder(
        config.initial_argument_of_latitude_rad
            + mean_motion_rad_s * elapsed_time_s,
        2.0 * std::numbers::pi
    );
    const double cosine_argument = std::cos(argument_of_latitude_rad);
    const double sine_argument = std::sin(argument_of_latitude_rad);
    const double cosine_inclination = std::cos(config.inclination_rad);
    const double sine_inclination = std::sin(config.inclination_rad);
    const double orbital_speed_m_s = mean_motion_rad_s * radius_m;

    return {
        .position_inertial_m = {
            radius_m * cosine_argument,
            radius_m * sine_argument * cosine_inclination,
            radius_m * sine_argument * sine_inclination
        },
        .velocity_inertial_m_s = {
            -orbital_speed_m_s * sine_argument,
            orbital_speed_m_s * cosine_argument * cosine_inclination,
            orbital_speed_m_s * cosine_argument * sine_inclination
        }
    };
}

}  // namespace detumble
