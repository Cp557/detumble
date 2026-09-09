#pragma once

#include <array>
#include <string_view>

#include <Eigen/Core>

namespace detumble {

struct RodComponent {
    std::string_view name;
    Eigen::Vector3d center_body_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis_body{Eigen::Vector3d::UnitX()};
    double length_m{};
    double radius_m{};
};

struct SurfaceSensorComponent {
    std::string_view name;
    Eigen::Vector3d center_body_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d outward_normal_body{Eigen::Vector3d::UnitX()};
    double side_length_m{};
};

struct ReactionWheelComponent {
    std::string_view name;
    Eigen::Vector3d center_body_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d spin_axis_body{Eigen::Vector3d::UnitZ()};
    double radius_m{};
    double thickness_m{};
};

struct SpacecraftVisualConfig {
    Eigen::Vector3d body_dimensions_m{0.10, 0.10, 0.34};
    std::array<RodComponent, 3> magnetorquers;
    std::array<SurfaceSensorComponent, 6> coarse_sun_sensors;
    SurfaceSensorComponent magnetometer;
    std::array<ReactionWheelComponent, 4> reaction_wheels;
};

[[nodiscard]] SpacecraftVisualConfig generic_3u_visual_config();

}  // namespace detumble
