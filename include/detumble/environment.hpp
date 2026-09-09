#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "detumble/magnetic_field.hpp"
#include "detumble/orbit.hpp"

namespace detumble {

struct SunEnvironmentConfig {
    Eigen::Vector3d direction_inertial{Eigen::Vector3d::UnitX()};
};

struct EnvironmentConfig {
    CircularOrbitConfig orbit{};
    TiltedDipoleConfig magnetic_field{};
    SunEnvironmentConfig sun{};
};

struct EnvironmentState {
    OrbitState orbit{};
    Eigen::Vector3d magnetic_field_inertial_T{Eigen::Vector3d::Zero()};
    Eigen::Vector3d magnetic_field_body_T{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sun_direction_inertial{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d sun_direction_body{Eigen::Vector3d::UnitX()};
    bool in_eclipse{};
};

[[nodiscard]] bool earth_occults_sun(
    const Eigen::Vector3d& spacecraft_position_inertial_m,
    const Eigen::Vector3d& sun_direction_inertial,
    double earth_radius_m
);

[[nodiscard]] EnvironmentState sample_environment(
    const EnvironmentConfig& config,
    const Eigen::Quaterniond& body_to_inertial,
    double elapsed_time_s
);

}  // namespace detumble
