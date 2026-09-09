#include <array>
#include <numbers>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/dynamics.hpp"
#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/frames.hpp"
#include "detumble/reaction_wheel.hpp"
#include "detumble/spacecraft_visual.hpp"

TEST_CASE("visual configuration matches the generic 3U body") {
    const detumble::SpacecraftVisualConfig visual =
        detumble::generic_3u_visual_config();
    const detumble::RigidBodyProperties body = detumble::generic_3u_cubesat();

    REQUIRE(visual.body_dimensions_m == body.dimensions_body_m);
    REQUIRE(body.center_of_mass_body_m.isZero());
}

TEST_CASE("magnetorquer rods align with the three body axes") {
    const detumble::SpacecraftVisualConfig visual =
        detumble::generic_3u_visual_config();
    const std::array<Eigen::Vector3d, 3> expected_axes{
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ()
    };

    for (std::size_t index = 0; index < visual.magnetorquers.size(); ++index) {
        REQUIRE(visual.magnetorquers[index].axis_body == expected_axes[index]);
        REQUIRE(
            visual.magnetorquers[index].axis_body.norm()
            == Catch::Approx(1.0)
        );
    }
}

TEST_CASE("surface sensor normals point away from the body origin") {
    const detumble::SpacecraftVisualConfig visual =
        detumble::generic_3u_visual_config();
    const detumble::CoarseSunSensorArrayConfig sensors =
        detumble::generic_3u_coarse_sun_sensor_config();

    for (std::size_t index = 0;
         index < visual.coarse_sun_sensors.size();
         ++index) {
        const auto& sensor = visual.coarse_sun_sensors[index];
        REQUIRE(sensor.outward_normal_body.norm() == Catch::Approx(1.0));
        REQUIRE(sensor.center_body_m.dot(sensor.outward_normal_body) > 0.0);
        REQUIRE(sensor.name.compare(sensors.sensors[index].name) == 0);
        REQUIRE(
            sensor.outward_normal_body
            == sensors.sensors[index].outward_normal_body
        );
    }
    REQUIRE(
        visual.magnetometer.center_body_m.x()
        > 0.5 * visual.body_dimensions_m.x()
    );
}

TEST_CASE("reaction wheel axes form a normalized four-wheel pyramid") {
    const detumble::SpacecraftVisualConfig visual =
        detumble::generic_3u_visual_config();
    const detumble::ReactionWheelClusterConfig physics =
        detumble::generic_4_wheel_pyramid_config();

    for (std::size_t index = 0;
         index < visual.reaction_wheels.size();
         ++index) {
        const auto& wheel = visual.reaction_wheels[index];
        REQUIRE(wheel.spin_axis_body.norm() == Catch::Approx(1.0));
        REQUIRE(wheel.spin_axis_body.z() > 0.0);
        REQUIRE(wheel.name.compare(physics.wheels[index].name) == 0);
        REQUIRE(wheel.spin_axis_body == physics.wheels[index].spin_axis_body);
    }
    REQUIRE(
        visual.reaction_wheels[0].spin_axis_body.x()
        == -visual.reaction_wheels[2].spin_axis_body.x()
    );
    REQUIRE(
        visual.reaction_wheels[1].spin_axis_body.y()
        == -visual.reaction_wheels[3].spin_axis_body.y()
    );
}

TEST_CASE("configured component centers follow body-to-inertial rotation") {
    const detumble::SpacecraftVisualConfig visual =
        detumble::generic_3u_visual_config();
    const Eigen::Quaterniond quarter_turn{
        Eigen::AngleAxisd{0.5 * std::numbers::pi, Eigen::Vector3d::UnitZ()}
    };
    const Eigen::Vector3d center_body =
        visual.magnetorquers[0].center_body_m;

    const Eigen::Vector3d center_inertial =
        detumble::rotate_body_to_inertial(quarter_turn, center_body);

    REQUIRE(center_inertial.x() == Catch::Approx(-center_body.y()));
    REQUIRE(center_inertial.y() == Catch::Approx(center_body.x()));
    REQUIRE(center_inertial.z() == Catch::Approx(center_body.z()));
}
