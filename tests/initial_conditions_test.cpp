#include <stdexcept>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/initial_conditions.hpp"

TEST_CASE("a seed reproduces the same initial attitude and angular velocity") {
    const detumble::AttitudeState first =
        detumble::random_attitude_state(42, 0.1, 0.3);
    const detumble::AttitudeState second =
        detumble::random_attitude_state(42, 0.1, 0.3);

    REQUIRE(first.body_to_inertial.coeffs() == second.body_to_inertial.coeffs());
    REQUIRE(
        first.angular_velocity_body_rad_s
        == second.angular_velocity_body_rad_s
    );
}

TEST_CASE("random initial state respects quaternion and speed bounds") {
    const detumble::AttitudeState state =
        detumble::random_attitude_state(7, 0.1, 0.3);

    REQUIRE(state.body_to_inertial.norm()
            == Catch::Approx(1.0).margin(1.0e-12));
    REQUIRE(state.angular_velocity_body_rad_s.norm() >= 0.1);
    REQUIRE(state.angular_velocity_body_rad_s.norm() <= 0.3);
}

TEST_CASE("different seeds generate different states") {
    const detumble::AttitudeState first =
        detumble::random_attitude_state(1, 0.1, 0.3);
    const detumble::AttitudeState second =
        detumble::random_attitude_state(2, 0.1, 0.3);

    REQUIRE_FALSE(
        first.body_to_inertial.coeffs() == second.body_to_inertial.coeffs()
    );
    REQUIRE_FALSE(
        first.angular_velocity_body_rad_s
        == second.angular_velocity_body_rad_s
    );
}

TEST_CASE("random initial state rejects invalid speed bounds") {
    REQUIRE_THROWS_AS(
        detumble::random_attitude_state(1, -0.1, 0.3),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        detumble::random_attitude_state(1, 0.3, 0.1),
        std::invalid_argument
    );
}
