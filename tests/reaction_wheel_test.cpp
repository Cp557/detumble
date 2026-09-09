#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/dynamics.hpp"
#include "detumble/reaction_wheel.hpp"
#include "detumble/sun_pointing.hpp"

namespace {

detumble::ReactionWheelClusterConfig orthogonal_cluster_config() {
    return {
        .wheels = {
            detumble::ReactionWheelDefinition{
                .name = "Wheel_X",
                .spin_axis_body = Eigen::Vector3d::UnitX()
            },
            detumble::ReactionWheelDefinition{
                .name = "Wheel_Y",
                .spin_axis_body = Eigen::Vector3d::UnitY()
            },
            detumble::ReactionWheelDefinition{
                .name = "Wheel_Z",
                .spin_axis_body = Eigen::Vector3d::UnitZ()
            }
        }
    };
}

}  // namespace

TEST_CASE("three orthogonal wheels allocate body torque exactly") {
    detumble::ReactionWheelCluster wheels{orthogonal_cluster_config()};
    const Eigen::Vector3d requested{1.0e-4, -2.0e-4, 3.0e-4};

    wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = 0.0,
            .requested_spacecraft_torque_body_Nm = requested,
            .enabled = true
        },
        0.1
    );

    const detumble::ReactionWheelTelemetry& telemetry = wheels.telemetry();
    REQUIRE(telemetry.applied_spacecraft_torque_body_Nm.isApprox(
        requested,
        1.0e-15
    ));
    REQUIRE(telemetry.allocation_error_body_Nm.norm() < 1.0e-15);
    REQUIRE_FALSE(telemetry.allocation_saturated);
    for (std::size_t index = 0; index < 3; ++index) {
        REQUIRE(telemetry.commanded_motor_torque_Nm[index]
                == Catch::Approx(-requested[static_cast<Eigen::Index>(index)]));
    }

    const Eigen::Vector3d net_internal_impulse =
        0.1 * telemetry.applied_spacecraft_torque_body_Nm
        + telemetry.stored_momentum_body_Nm_s;
    REQUIRE(net_internal_impulse.norm() < 1.0e-15);
}

TEST_CASE("four-wheel pyramid tracks an achievable torque") {
    detumble::ReactionWheelCluster wheels;
    const Eigen::Vector3d requested{4.0e-4, -3.0e-4, 5.0e-4};

    wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = 0.0,
            .requested_spacecraft_torque_body_Nm = requested,
            .enabled = true
        },
        0.01
    );

    REQUIRE(wheels.telemetry().applied_spacecraft_torque_body_Nm.isApprox(
        requested,
        1.0e-12
    ));
    REQUIRE_FALSE(wheels.telemetry().allocation_saturated);
    REQUIRE(wheels.telemetry().wheel_speed_rad_s.size() == 4);
}

TEST_CASE("wheel torque limits produce a reported allocation error") {
    detumble::ReactionWheelClusterConfig config = orthogonal_cluster_config();
    config.wheels[0].maximum_motor_torque_Nm = 1.0e-4;
    detumble::ReactionWheelCluster wheels{config};

    wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = 0.0,
            .requested_spacecraft_torque_body_Nm = {2.0e-4, 0.0, 0.0},
            .enabled = true
        },
        0.01
    );

    REQUIRE(wheels.telemetry().applied_spacecraft_torque_body_Nm.x()
            == Catch::Approx(1.0e-4));
    REQUIRE(wheels.telemetry().allocation_error_body_Nm.x()
            == Catch::Approx(1.0e-4));
    REQUIRE(wheels.telemetry().torque_saturated[0]);
    REQUIRE(wheels.telemetry().allocation_saturated);
}

TEST_CASE("wheel speed limit blocks acceleration but allows unloading") {
    detumble::ReactionWheelClusterConfig config{
        .wheels = {
            detumble::ReactionWheelDefinition{
                .name = "Wheel_X",
                .spin_axis_body = Eigen::Vector3d::UnitX(),
                .rotor_inertia_kg_m2 = 1.0,
                .maximum_motor_torque_Nm = 10.0,
                .maximum_speed_rad_s = 10.0,
                .initial_speed_rad_s = 9.5
            }
        }
    };
    detumble::ReactionWheelCluster wheels{config};

    wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = 0.0,
            .requested_spacecraft_torque_body_Nm = {-1.0, 0.0, 0.0},
            .enabled = true
        },
        1.0
    );
    REQUIRE(wheels.telemetry().wheel_speed_rad_s[0]
            == Catch::Approx(10.0));
    REQUIRE(wheels.telemetry().applied_motor_torque_Nm[0]
            == Catch::Approx(0.5));
    REQUIRE(wheels.telemetry().speed_saturated[0]);

    wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = 1.0,
            .requested_spacecraft_torque_body_Nm = {-1.0, 0.0, 0.0},
            .enabled = true
        },
        1.0
    );
    REQUIRE(wheels.telemetry().applied_motor_torque_Nm[0]
            == Catch::Approx(0.0));

    wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = 2.0,
            .requested_spacecraft_torque_body_Nm = {1.0, 0.0, 0.0},
            .enabled = true
        },
        1.0
    );
    REQUIRE(wheels.telemetry().applied_motor_torque_Nm[0]
            == Catch::Approx(-1.0));
    REQUIRE(wheels.telemetry().wheel_speed_rad_s[0]
            == Catch::Approx(9.0));
}

TEST_CASE("four-wheel pyramid retains three-axis control after one failure") {
    detumble::ReactionWheelCluster wheels;
    wheels.set_wheel_failed(0, true);
    const Eigen::Vector3d requested{1.0e-4, 2.0e-4, -1.5e-4};

    wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = 0.0,
            .requested_spacecraft_torque_body_Nm = requested,
            .enabled = true
        },
        0.01
    );

    REQUIRE(wheels.telemetry().failed[0]);
    REQUIRE(wheels.telemetry().applied_motor_torque_Nm[0]
            == Catch::Approx(0.0));
    REQUIRE(wheels.telemetry().applied_spacecraft_torque_body_Nm.isApprox(
        requested,
        1.0e-12
    ));
}

TEST_CASE("reaction-wheel cluster rejects invalid configuration and commands") {
    REQUIRE_THROWS_AS(
        detumble::ReactionWheelCluster{
            detumble::ReactionWheelClusterConfig{}
        },
        std::invalid_argument
    );

    detumble::ReactionWheelCluster wheels;
    REQUIRE_THROWS_AS(
        wheels.update(
            detumble::ReactionWheelCommand{.sample_time_s = 0.0},
            0.0
        ),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(wheels.set_wheel_failed(4, true), std::out_of_range);
}

TEST_CASE("Sun pointing remains stable with the four-wheel pyramid") {
    const detumble::RigidBodyProperties body = detumble::generic_3u_cubesat();
    const detumble::SunPointingControllerConfig controller_config;
    const Eigen::Quaterniond desired =
        detumble::desired_sun_pointing_attitude(
            controller_config,
            Eigen::Vector3d::UnitX()
        );
    const std::array<Eigen::Quaterniond, 3> initial_attitudes{
        Eigen::Quaterniond::Identity(),
        Eigen::Quaterniond{
            Eigen::AngleAxisd{
                95.0 * std::numbers::pi / 180.0,
                Eigen::Vector3d{1.0, 2.0, -1.0}.normalized()
            }
        },
        Eigen::Quaterniond{
            Eigen::AngleAxisd{
                170.0 * std::numbers::pi / 180.0,
                Eigen::Vector3d{-1.0, 0.5, 2.0}.normalized()
            }
        }
    };

    constexpr double time_step_s = 0.01;
    for (const Eigen::Quaterniond& initial : initial_attitudes) {
        detumble::AttitudeState state{
            .body_to_inertial = initial,
            .angular_velocity_body_rad_s = Eigen::Vector3d::Zero()
        };
        detumble::ReactionWheelCluster wheels;
        for (int step = 0; step < 18'000; ++step) {
            const detumble::SunPointingCommand pointing =
                detumble::sun_pointing_command(
                    controller_config,
                    state.body_to_inertial,
                    state.angular_velocity_body_rad_s,
                    desired
                );
            wheels.update(
                detumble::ReactionWheelCommand{
                    .sample_time_s = step * time_step_s,
                    .requested_spacecraft_torque_body_Nm =
                        pointing.limited_torque_body_Nm,
                    .enabled = true
                },
                time_step_s
            );
            state = detumble::propagate_rigid_body_rk4(
                body,
                state,
                wheels.telemetry().applied_spacecraft_torque_body_Nm,
                time_step_s
            );
        }

        const double error_rad =
            detumble::shortest_attitude_error_body_rad(
                state.body_to_inertial,
                desired
            ).norm();
        INFO("initial quaternion = " << initial.coeffs().transpose());
        REQUIRE(error_rad < 0.25 * std::numbers::pi / 180.0);
        REQUIRE(state.angular_velocity_body_rad_s.norm()
                < 0.02 * std::numbers::pi / 180.0);
        for (std::size_t index = 0;
             index < wheels.config().wheels.size();
             ++index) {
            REQUIRE(std::abs(wheels.telemetry().wheel_speed_rad_s[index])
                    <= wheels.config().wheels[index].maximum_speed_rad_s);
        }
    }
}
