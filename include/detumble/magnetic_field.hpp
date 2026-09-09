#pragma once

#include <numbers>

#include <Eigen/Core>

namespace detumble {

struct TiltedDipoleConfig {
    double reference_radius_m{6'371'000.0};
    double equatorial_surface_field_T{3.12e-5};
    double tilt_rad{11.0 * std::numbers::pi / 180.0};
    double earth_rotation_rate_rad_s{7.2921150e-5};
    double initial_rotation_phase_rad{};
};

[[nodiscard]] Eigen::Vector3d magnetic_dipole_axis_inertial(
    const TiltedDipoleConfig& config,
    double elapsed_time_s
);

[[nodiscard]] Eigen::Vector3d tilted_dipole_field_inertial_T(
    const TiltedDipoleConfig& config,
    const Eigen::Vector3d& position_inertial_m,
    double elapsed_time_s
);

}  // namespace detumble
