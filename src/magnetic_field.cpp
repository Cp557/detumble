#include "detumble/magnetic_field.hpp"

#include <cmath>
#include <stdexcept>

namespace detumble {
namespace {

void validate_config(const TiltedDipoleConfig& config) {
    if (!std::isfinite(config.reference_radius_m)
        || config.reference_radius_m <= 0.0) {
        throw std::invalid_argument{
            "Magnetic reference radius must be finite and positive"
        };
    }
    if (!std::isfinite(config.equatorial_surface_field_T)
        || config.equatorial_surface_field_T <= 0.0) {
        throw std::invalid_argument{
            "Equatorial magnetic field must be finite and positive"
        };
    }
    if (!std::isfinite(config.tilt_rad)
        || config.tilt_rad < 0.0
        || config.tilt_rad > std::numbers::pi) {
        throw std::invalid_argument{
            "Magnetic dipole tilt must be between zero and pi radians"
        };
    }
    if (!std::isfinite(config.earth_rotation_rate_rad_s)
        || !std::isfinite(config.initial_rotation_phase_rad)) {
        throw std::invalid_argument{
            "Earth rotation rate and phase must be finite"
        };
    }
}

}  // namespace

Eigen::Vector3d magnetic_dipole_axis_inertial(
    const TiltedDipoleConfig& config,
    const double elapsed_time_s
) {
    validate_config(config);
    if (!std::isfinite(elapsed_time_s)) {
        throw std::invalid_argument{"Magnetic-field time must be finite"};
    }

    const double rotation_phase_rad = std::remainder(
        config.initial_rotation_phase_rad
            + config.earth_rotation_rate_rad_s * elapsed_time_s,
        2.0 * std::numbers::pi
    );
    const double horizontal_component = std::sin(config.tilt_rad);

    return {
        horizontal_component * std::cos(rotation_phase_rad),
        horizontal_component * std::sin(rotation_phase_rad),
        -std::cos(config.tilt_rad)
    };
}

Eigen::Vector3d tilted_dipole_field_inertial_T(
    const TiltedDipoleConfig& config,
    const Eigen::Vector3d& position_inertial_m,
    const double elapsed_time_s
) {
    validate_config(config);
    if (!position_inertial_m.allFinite()) {
        throw std::invalid_argument{
            "Magnetic-field position must be finite"
        };
    }

    const double radius_m = position_inertial_m.norm();
    if (radius_m < config.reference_radius_m) {
        throw std::invalid_argument{
            "Magnetic-field position must be at or above Earth surface"
        };
    }

    const Eigen::Vector3d radial_direction =
        position_inertial_m / radius_m;
    const Eigen::Vector3d dipole_axis =
        magnetic_dipole_axis_inertial(config, elapsed_time_s);
    const double radial_scale = std::pow(
        config.reference_radius_m / radius_m,
        3.0
    );

    return config.equatorial_surface_field_T * radial_scale
        * (3.0 * dipole_axis.dot(radial_direction) * radial_direction
           - dipole_axis);
}

}  // namespace detumble
