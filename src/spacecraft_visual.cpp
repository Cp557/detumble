#include "detumble/spacecraft_visual.hpp"

namespace detumble {

SpacecraftVisualConfig generic_3u_visual_config() {
    return {.body_dimensions_m = {0.10, 0.10, 0.34},
            .magnetorquers = {RodComponent{.name = "Magnetorquer_X",
                                           .center_body_m = {0.0, 0.0, 0.065},
                                           .axis_body = Eigen::Vector3d::UnitX(),
                                           .length_m = 0.075,
                                           .radius_m = 0.004},
                              RodComponent{.name = "Magnetorquer_Y",
                                           .center_body_m = {0.0, 0.0, -0.015},
                                           .axis_body = Eigen::Vector3d::UnitY(),
                                           .length_m = 0.075,
                                           .radius_m = 0.004},
                              RodComponent{.name = "Magnetorquer_Z",
                                           .center_body_m = {0.025, -0.025, 0.0},
                                           .axis_body = Eigen::Vector3d::UnitZ(),
                                           .length_m = 0.24,
                                           .radius_m = 0.004}},
            .magnetometer = {.name = "Magnetometer",
                             .center_body_m = {0.058, 0.0, 0.125},
                             .outward_normal_body = Eigen::Vector3d::UnitX(),
                             .side_length_m = 0.012}};
}

} // namespace detumble
