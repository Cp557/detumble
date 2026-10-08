#include "detumble/attitude_estimator.hpp"
#include "detumble/mission.hpp"
#include <Eigen/Eigenvalues>
#include <catch2/catch_test_macros.hpp>
#include <numbers>
TEST_CASE("a single magnetic vector cannot establish attitude or rate confidence") {
    detumble::AttitudeEstimator estimator;
    estimator.update({0.0, {10e-6, 0.0, 30e-6}}, {0.0, 10e-6, 30e-6});
    REQUIRE_FALSE(estimator.estimate().valid);
    REQUIRE_FALSE(estimator.estimate().confident);
    REQUIRE_THROWS(estimator.update({0.0, {10e-6, 0.0, 30e-6}}, {0.0, 10e-6, 30e-6}));
    estimator.reset();
    REQUIRE_FALSE(estimator.estimate().valid);
    REQUIRE_THROWS(
        estimator.predict(-1.0, {0.0, 0.0, 30e-6}, Eigen::Vector3d::Zero(), 0.1));
}
TEST_CASE(
    "magnetic history initialization preserves unobservable attitude uncertainty") {
    detumble::AttitudeEstimatorConfig config;
    config.initialization_window_s = 2.0;
    detumble::AttitudeEstimator estimator{config};
    const Eigen::Vector3d field{0.0, 0.0, 30e-6};
    for (unsigned i = 0; i <= 20; ++i) {
        const double time = 0.1 * i;
        estimator.update({time, field}, field);
        if (i < 20)
            estimator.predict(time, field, Eigen::Vector3d::Zero(), 0.1);
    }
    REQUIRE(estimator.estimate().valid);
    REQUIRE_FALSE(estimator.estimate().confident);
    const Eigen::SelfAdjointEigenSolver<detumble::EstimatorCovariance> covariance{
        estimator.estimate().covariance};
    REQUIRE(covariance.eigenvalues().minCoeff() > -1e-12);
    REQUIRE(estimator.estimate().covariance.block<3, 3>(0, 0).trace() > 1.0);
}

TEST_CASE("gyro-free EKF converges from arbitrary seeded orientations with realistic "
          "errors") {
    for (const std::uint64_t seed : {7ULL, 42ULL, 2025ULL}) {
        detumble::MissionRunConfig config;
        config.seed = seed;
        config.duration_s = 600.0;
        config.flight_software.controller_selection =
            detumble::DetumbleController::estimated_rate;
        const auto metrics = detumble::run_native_mission(config);
        CAPTURE(seed, metrics.final_rate_estimation_error_rad_s,
                metrics.final_attitude_estimation_error_rad);
        REQUIRE(metrics.estimator_convergence_time_s.has_value());
        REQUIRE(metrics.controller_handoff_time_s.has_value());
        REQUIRE(metrics.final_rate_estimation_error_rad_s <
                0.05 * std::numbers::pi / 180.0);
        REQUIRE(metrics.final_attitude_estimation_error_rad <
                2.0 * std::numbers::pi / 180.0);
        REQUIRE(metrics.false_low_rate_events == 0);
    }
}
TEST_CASE("gyro-free initialization handles rotation initially parallel to the "
          "measured field") {
    detumble::Environment environment;
    detumble::AttitudeState truth;
    const Eigen::Vector3d reference = environment.reference_field_inertial_T(0.0);
    truth.angular_velocity_body_rad_s =
        reference.normalized() * 15.0 * std::numbers::pi / 180.0;
    detumble::AttitudeEstimator estimator;
    for (unsigned step = 0; step <= 1200; ++step) {
        const double time = 0.1 * step;
        const auto field = environment.reference_field_inertial_T(time);
        estimator.update({time, truth.body_to_inertial.conjugate() * field}, field);
        if (step < 1200) {
            estimator.predict(time, field, Eigen::Vector3d::Zero(), 0.1);
            truth = detumble::propagate_rigid_body_rk4(
                detumble::generic_3u_cubesat(), truth, Eigen::Vector3d::Zero(), 0.1);
        }
    }
    REQUIRE(estimator.estimate().valid);
    REQUIRE((estimator.estimate().attitude.angular_velocity_body_rad_s -
             truth.angular_velocity_body_rad_s)
                .norm() < 0.05 * std::numbers::pi / 180.0);
    REQUIRE(estimator.estimate().confident);

    // A large finite outlier must remove confidence without correcting toward it.
    const auto before = estimator.estimate().attitude;
    const auto next_field = environment.reference_field_inertial_T(120.1);
    estimator.update({120.1, before.body_to_inertial.conjugate() * next_field +
                                 Eigen::Vector3d{20e-6, 0.0, 0.0}},
                     next_field);
    REQUIRE_FALSE(estimator.estimate().confident);
    REQUIRE(estimator.estimate().status == detumble::EstimatorStatus::rejected);
    REQUIRE(estimator.estimate().attitude.angular_velocity_body_rad_s ==
            before.angular_velocity_body_rad_s);
    for (unsigned i = 2; i <= 20; ++i) {
        const double time = 120.0 + 0.1 * i;
        const auto field = environment.reference_field_inertial_T(time);
        estimator.update({time, before.body_to_inertial.conjugate() * field +
                                    Eigen::Vector3d{20e-6, 0.0, 0.0}},
                         field);
    }
    REQUIRE_FALSE(estimator.estimate().valid);
    REQUIRE(estimator.estimate().status == detumble::EstimatorStatus::collecting);
}
