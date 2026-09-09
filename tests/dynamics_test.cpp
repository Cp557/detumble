#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/dynamics.hpp"
#include "detumble/frames.hpp"

namespace {

constexpr double tolerance = 1.0e-10;

void require_vector_approx(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    const double margin = tolerance
) {
    for (Eigen::Index index = 0; index < actual.size(); ++index) {
        REQUIRE(actual[index] == Catch::Approx(expected[index]).margin(margin));
    }
}

detumble::RigidBodyProperties test_body(
    const Eigen::Vector3d& principal_moments_body_kg_m2
) {
    return {
        .mass_kg = 1.0,
        .dimensions_body_m = Eigen::Vector3d::Ones(),
        .center_of_mass_body_m = Eigen::Vector3d::Zero(),
        .principal_moments_body_kg_m2 = principal_moments_body_kg_m2
    };
}

}  // namespace

TEST_CASE("generic 3U properties match a uniform rectangular body") {
    const detumble::RigidBodyProperties body = detumble::generic_3u_cubesat();

    REQUIRE(body.mass_kg == 4.0);
    require_vector_approx(body.dimensions_body_m, {0.10, 0.10, 0.34});
    require_vector_approx(body.center_of_mass_body_m, Eigen::Vector3d::Zero());
    require_vector_approx(
        body.principal_moments_body_kg_m2,
        {0.04186666666666667, 0.04186666666666667, 0.006666666666666668}
    );
}

TEST_CASE("Euler equation includes gyroscopic coupling") {
    const detumble::RigidBodyProperties body = test_body({2.0, 3.0, 4.0});

    const Eigen::Vector3d acceleration =
        detumble::angular_acceleration_body_rad_s2(
            body,
            {1.0, 2.0, 3.0},
            Eigen::Vector3d::Zero()
        );

    require_vector_approx(acceleration, {-3.0, 2.0, -0.5});
}

TEST_CASE("a torque-free spherical body keeps constant angular velocity") {
    const detumble::RigidBodyProperties body = test_body({2.0, 2.0, 2.0});
    detumble::AttitudeState state{
        .body_to_inertial = Eigen::Quaterniond::Identity(),
        .angular_velocity_body_rad_s = {0.1, -0.2, 0.3}
    };
    const Eigen::Vector3d initial_rate = state.angular_velocity_body_rad_s;

    for (int step = 0; step < 1'000; ++step) {
        state = detumble::propagate_rigid_body_rk4(
            body,
            state,
            Eigen::Vector3d::Zero(),
            0.01
        );
    }

    require_vector_approx(state.angular_velocity_body_rad_s, initial_rate);
    REQUIRE(state.body_to_inertial.norm()
            == Catch::Approx(1.0).margin(tolerance));
}

TEST_CASE("asymmetric torque-free motion conserves energy and inertial momentum") {
    const detumble::RigidBodyProperties body = test_body({0.04, 0.03, 0.02});
    detumble::AttitudeState state{
        .body_to_inertial = Eigen::Quaterniond{
            Eigen::AngleAxisd{0.4, Eigen::Vector3d{1.0, 1.0, 0.5}.normalized()}
        },
        .angular_velocity_body_rad_s = {0.4, 0.7, -0.2}
    };
    const detumble::RotationalMetrics initial =
        detumble::rotational_metrics(body, state);

    for (int step = 0; step < 10'000; ++step) {
        state = detumble::propagate_rigid_body_rk4(
            body,
            state,
            Eigen::Vector3d::Zero(),
            0.001
        );
    }
    const detumble::RotationalMetrics final =
        detumble::rotational_metrics(body, state);

    REQUIRE(final.rotational_kinetic_energy_j
            == Catch::Approx(initial.rotational_kinetic_energy_j)
                   .margin(1.0e-12));
    require_vector_approx(
        final.angular_momentum_inertial_kg_m2_s,
        initial.angular_momentum_inertial_kg_m2_s,
        1.0e-11
    );
    REQUIRE(final.quaternion_norm == Catch::Approx(1.0).margin(tolerance));
}

TEST_CASE("constant principal-axis torque produces expected motion") {
    const detumble::RigidBodyProperties body = test_body({0.02, 0.03, 0.04});
    detumble::AttitudeState state;
    const Eigen::Vector3d applied_torque_body_Nm{0.001, 0.0, 0.0};

    for (int step = 0; step < 1'000; ++step) {
        state = detumble::propagate_rigid_body_rk4(
            body,
            state,
            applied_torque_body_Nm,
            0.001
        );
    }

    require_vector_approx(
        state.angular_velocity_body_rad_s,
        {0.05, 0.0, 0.0},
        1.0e-12
    );
    const Eigen::Vector3d rotated_y = detumble::rotate_body_to_inertial(
        state.body_to_inertial,
        Eigen::Vector3d::UnitY()
    );
    require_vector_approx(
        rotated_y,
        {0.0, std::cos(0.025), std::sin(0.025)},
        1.0e-11
    );
}

TEST_CASE("dynamics reject invalid body properties and timesteps") {
    detumble::RigidBodyProperties invalid_body = test_body({1.0, -1.0, 1.0});

    REQUIRE_THROWS_AS(
        detumble::angular_acceleration_body_rad_s2(
            invalid_body,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()
        ),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        detumble::propagate_rigid_body_rk4(
            test_body(Eigen::Vector3d::Ones()),
            detumble::AttitudeState{},
            Eigen::Vector3d::Zero(),
            -0.01
        ),
        std::invalid_argument
    );
}
