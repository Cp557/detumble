#include <array>
#include <numbers>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/dynamics.hpp"
#include "detumble/frames.hpp"
#include "detumble/sun_pointing.hpp"

TEST_CASE("desired Sun attitude fixes pointing and roll axes") {
    const detumble::SunPointingControllerConfig config;
    const Eigen::Vector3d sun_inertial = Eigen::Vector3d::UnitX();

    const Eigen::Quaterniond desired =
        detumble::desired_sun_pointing_attitude(config, sun_inertial);

    REQUIRE(detumble::rotate_body_to_inertial(
                desired,
                config.sun_axis_body
            ).isApprox(sun_inertial, 1.0e-12));
    REQUIRE(detumble::rotate_body_to_inertial(
                desired,
                config.roll_axis_body
            ).isApprox(Eigen::Vector3d::UnitZ(), 1.0e-12));
}

TEST_CASE("attitude error uses the shortest equivalent quaternion rotation") {
    const Eigen::Quaterniond current = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond desired{
        Eigen::AngleAxisd{
            270.0 * std::numbers::pi / 180.0,
            Eigen::Vector3d::UnitZ()
        }
    };
    Eigen::Quaterniond negated_desired = desired;
    negated_desired.coeffs() *= -1.0;

    const Eigen::Vector3d error =
        detumble::shortest_attitude_error_body_rad(current, desired);
    const Eigen::Vector3d negated_error =
        detumble::shortest_attitude_error_body_rad(
            current,
            negated_desired
        );

    REQUIRE(error.x() == Catch::Approx(0.0).margin(1.0e-12));
    REQUIRE(error.y() == Catch::Approx(0.0).margin(1.0e-12));
    REQUIRE(error.z() == Catch::Approx(-0.5 * std::numbers::pi)
                             .margin(1.0e-12));
    REQUIRE(negated_error.isApprox(error, 1.0e-12));
}

TEST_CASE("Sun-pointing PD command damps rate and enforces torque limits") {
    detumble::SunPointingControllerConfig config;
    config.maximum_torque_body_Nm.setConstant(1.0e-4);
    const Eigen::Quaterniond desired{
        Eigen::AngleAxisd{0.5, Eigen::Vector3d::UnitX()}
    };

    const detumble::SunPointingCommand command =
        detumble::sun_pointing_command(
            config,
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d{0.1, 0.0, 0.0},
            desired
        );

    REQUIRE(command.active);
    REQUIRE(command.saturated);
    REQUIRE(command.requested_torque_body_Nm.x() < 0.0);
    REQUIRE(command.limited_torque_body_Nm.x()
            == Catch::Approx(-1.0e-4));
    REQUIRE((command.limited_torque_body_Nm.array().abs()
             <= config.maximum_torque_body_Nm.array()).all());
}

TEST_CASE("Sun-pointing command suppresses chatter inside its deadband") {
    const detumble::SunPointingControllerConfig config;
    const Eigen::Quaterniond tiny_error{
        Eigen::AngleAxisd{
            0.05 * std::numbers::pi / 180.0,
            Eigen::Vector3d::UnitY()
        }
    };

    const detumble::SunPointingCommand command =
        detumble::sun_pointing_command(
            config,
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero(),
            tiny_error
        );

    REQUIRE(command.in_deadband);
    REQUIRE(command.limited_torque_body_Nm.isZero());
}

TEST_CASE("Sun-pointing tracker uses thresholds dwell and hysteresis") {
    detumble::SunPointingTracker tracker{
        detumble::SunPointingControllerConfig{
            .acquisition_threshold_rad = 0.2,
            .steady_pointing_threshold_rad = 0.05,
            .hysteresis_rad = 0.02,
            .dwell_time_s = 2.0
        }
    };

    tracker.update(0.0, 0.1);
    tracker.update(2.0, 0.1);
    REQUIRE(tracker.state().target_acquired);
    REQUIRE_FALSE(tracker.state().steady_pointing);

    tracker.update(3.0, 0.04);
    tracker.update(5.0, 0.04);
    REQUIRE(tracker.state().steady_pointing);

    tracker.update(6.0, 0.06);
    REQUIRE(tracker.state().steady_pointing);
    tracker.update(7.0, 0.08);
    REQUIRE_FALSE(tracker.state().steady_pointing);
}

TEST_CASE("Sun-pointing configuration rejects invalid axes and limits") {
    detumble::SunPointingControllerConfig parallel_axes;
    parallel_axes.roll_axis_body = parallel_axes.sun_axis_body;
    REQUIRE_THROWS_AS(
        detumble::desired_sun_pointing_attitude(
            parallel_axes,
            Eigen::Vector3d::UnitX()
        ),
        std::invalid_argument
    );

    detumble::SunPointingControllerConfig invalid_limit;
    invalid_limit.maximum_torque_body_Nm.x() = 0.0;
    REQUIRE_THROWS_AS(
        detumble::sun_pointing_command(
            invalid_limit,
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity()
        ),
        std::invalid_argument
    );
}

TEST_CASE("ideal torque acquires Sun pointing from detumbled attitudes") {
    const detumble::RigidBodyProperties body = detumble::generic_3u_cubesat();
    const detumble::SunPointingControllerConfig config;
    const Eigen::Quaterniond desired =
        detumble::desired_sun_pointing_attitude(
            config,
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

    for (const Eigen::Quaterniond& initial : initial_attitudes) {
        detumble::AttitudeState state{
            .body_to_inertial = initial,
            .angular_velocity_body_rad_s = Eigen::Vector3d::Zero()
        };
        for (int step = 0; step < 18'000; ++step) {
            const detumble::SunPointingCommand command =
                detumble::sun_pointing_command(
                    config,
                    state.body_to_inertial,
                    state.angular_velocity_body_rad_s,
                    desired
                );
            state = detumble::propagate_rigid_body_rk4(
                body,
                state,
                command.limited_torque_body_Nm,
                0.01
            );
        }

        const Eigen::Vector3d error =
            detumble::shortest_attitude_error_body_rad(
                state.body_to_inertial,
                desired
            );
        INFO("initial quaternion = " << initial.coeffs().transpose());
        REQUIRE(error.norm() < 0.25 * std::numbers::pi / 180.0);
        REQUIRE(state.angular_velocity_body_rad_s.norm()
                < 0.02 * std::numbers::pi / 180.0);
    }
}
