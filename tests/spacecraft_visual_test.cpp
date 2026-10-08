#include <array>
#include <numbers>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/dynamics.hpp"
#include "detumble/frames.hpp"
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
        Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};

    for (std::size_t index = 0; index < visual.magnetorquers.size(); ++index) {
        REQUIRE(visual.magnetorquers[index].axis_body == expected_axes[index]);
        REQUIRE(visual.magnetorquers[index].axis_body.norm() == Catch::Approx(1.0));
    }
}

TEST_CASE("magnetometer is mounted outside the body") {
    const auto visual = detumble::generic_3u_visual_config();
    REQUIRE(visual.magnetometer.center_body_m.x() > 0.5 * visual.body_dimensions_m.x());
    REQUIRE(visual.magnetometer.outward_normal_body == Eigen::Vector3d::UnitX());
}

TEST_CASE("configured component centers follow body-to-inertial rotation") {
    const detumble::SpacecraftVisualConfig visual =
        detumble::generic_3u_visual_config();
    const Eigen::Quaterniond quarter_turn{
        Eigen::AngleAxisd{0.5 * std::numbers::pi, Eigen::Vector3d::UnitZ()}};
    const Eigen::Vector3d center_body = visual.magnetorquers[0].center_body_m;

    const Eigen::Vector3d center_inertial =
        detumble::rotate_body_to_inertial(quarter_turn, center_body);

    REQUIRE(center_inertial.x() == Catch::Approx(-center_body.y()));
    REQUIRE(center_inertial.y() == Catch::Approx(center_body.x()));
    REQUIRE(center_inertial.z() == Catch::Approx(center_body.z()));
}
