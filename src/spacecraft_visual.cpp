#include "detumble/spacecraft_visual.hpp"

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/reaction_wheel.hpp"

namespace detumble {

SpacecraftVisualConfig generic_3u_visual_config() {
    const CoarseSunSensorArrayConfig sun_sensors =
        generic_3u_coarse_sun_sensor_config();
    const ReactionWheelClusterConfig reaction_wheels =
        generic_4_wheel_pyramid_config();
    return {
        .body_dimensions_m = {0.10, 0.10, 0.34},
        .magnetorquers = {
            RodComponent{
                .name = "Magnetorquer_X",
                .center_body_m = {0.0, 0.0, 0.065},
                .axis_body = Eigen::Vector3d::UnitX(),
                .length_m = 0.075,
                .radius_m = 0.004
            },
            RodComponent{
                .name = "Magnetorquer_Y",
                .center_body_m = {0.0, 0.0, -0.015},
                .axis_body = Eigen::Vector3d::UnitY(),
                .length_m = 0.075,
                .radius_m = 0.004
            },
            RodComponent{
                .name = "Magnetorquer_Z",
                .center_body_m = {0.025, -0.025, 0.0},
                .axis_body = Eigen::Vector3d::UnitZ(),
                .length_m = 0.24,
                .radius_m = 0.004
            }
        },
        .coarse_sun_sensors = {
            SurfaceSensorComponent{
                .name = sun_sensors.sensors[0].name,
                .center_body_m = {0.0515, 0.0, 0.0},
                .outward_normal_body =
                    sun_sensors.sensors[0].outward_normal_body,
                .side_length_m = 0.016
            },
            SurfaceSensorComponent{
                .name = sun_sensors.sensors[1].name,
                .center_body_m = {-0.0515, 0.0, 0.0},
                .outward_normal_body =
                    sun_sensors.sensors[1].outward_normal_body,
                .side_length_m = 0.016
            },
            SurfaceSensorComponent{
                .name = sun_sensors.sensors[2].name,
                .center_body_m = {0.0, 0.0515, 0.0},
                .outward_normal_body =
                    sun_sensors.sensors[2].outward_normal_body,
                .side_length_m = 0.016
            },
            SurfaceSensorComponent{
                .name = sun_sensors.sensors[3].name,
                .center_body_m = {0.0, -0.0515, 0.0},
                .outward_normal_body =
                    sun_sensors.sensors[3].outward_normal_body,
                .side_length_m = 0.016
            },
            SurfaceSensorComponent{
                .name = sun_sensors.sensors[4].name,
                .center_body_m = {0.0, 0.0, 0.1715},
                .outward_normal_body =
                    sun_sensors.sensors[4].outward_normal_body,
                .side_length_m = 0.016
            },
            SurfaceSensorComponent{
                .name = sun_sensors.sensors[5].name,
                .center_body_m = {0.0, 0.0, -0.1715},
                .outward_normal_body =
                    sun_sensors.sensors[5].outward_normal_body,
                .side_length_m = 0.016
            }
        },
        .magnetometer = {
            .name = "Magnetometer",
            .center_body_m = {0.058, 0.0, 0.125},
            .outward_normal_body = Eigen::Vector3d::UnitX(),
            .side_length_m = 0.012
        },
        .reaction_wheels = {
            ReactionWheelComponent{
                .name = reaction_wheels.wheels[0].name,
                .center_body_m = {0.022, 0.0, -0.065},
                .spin_axis_body = reaction_wheels.wheels[0].spin_axis_body,
                .radius_m = 0.018,
                .thickness_m = 0.009
            },
            ReactionWheelComponent{
                .name = reaction_wheels.wheels[1].name,
                .center_body_m = {0.0, 0.022, -0.025},
                .spin_axis_body = reaction_wheels.wheels[1].spin_axis_body,
                .radius_m = 0.018,
                .thickness_m = 0.009
            },
            ReactionWheelComponent{
                .name = reaction_wheels.wheels[2].name,
                .center_body_m = {-0.022, 0.0, 0.015},
                .spin_axis_body = reaction_wheels.wheels[2].spin_axis_body,
                .radius_m = 0.018,
                .thickness_m = 0.009
            },
            ReactionWheelComponent{
                .name = reaction_wheels.wheels[3].name,
                .center_body_m = {0.0, -0.022, 0.055},
                .spin_axis_body = reaction_wheels.wheels[3].spin_axis_body,
                .radius_m = 0.018,
                .thickness_m = 0.009
            }
        }
    };
}

}  // namespace detumble
