#pragma once

#include <numbers>

#include <Eigen/Core>

namespace detumble {

struct CircularOrbitConfig {
    double earth_radius_m{6'371'000.0};
    double altitude_m{500'000.0};
    double gravitational_parameter_m3_s2{3.986004418e14};
    double inclination_rad{51.6 * std::numbers::pi / 180.0};
    double initial_argument_of_latitude_rad{};
};

struct OrbitState {
    Eigen::Vector3d position_inertial_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_inertial_m_s{Eigen::Vector3d::Zero()};
};

[[nodiscard]] double circular_orbit_radius_m(
    const CircularOrbitConfig& config
);

[[nodiscard]] double circular_orbit_mean_motion_rad_s(
    const CircularOrbitConfig& config
);

[[nodiscard]] double circular_orbit_period_s(
    const CircularOrbitConfig& config
);

[[nodiscard]] OrbitState circular_orbit_state(
    const CircularOrbitConfig& config,
    double elapsed_time_s
);

}  // namespace detumble
