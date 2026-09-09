#include <cmath>
#include <limits>
#include <numbers>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/attitude.hpp"
#include "detumble/attitude_estimator.hpp"
#include "detumble/frames.hpp"

namespace {

detumble::AttitudeVectorObservation observation(
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& reference_inertial
) {
    return {
        .measured_body = detumble::rotate_inertial_to_body(
            body_to_inertial,
            reference_inertial
        ),
        .reference_inertial = reference_inertial,
        .valid = true
    };
}

detumble::AttitudeEstimatorInput estimator_input(
    const double time_s,
    const Eigen::Quaterniond& true_attitude,
    const Eigen::Vector3d& angular_velocity_body_rad_s
) {
    const Eigen::Vector3d magnetic_reference{0.2, -0.7, 0.5};
    const Eigen::Vector3d sun_reference{1.0, 0.1, -0.2};
    return {
        .sample_time_s = time_s,
        .angular_velocity_body_rad_s = angular_velocity_body_rad_s,
        .gyroscope_valid = true,
        .magnetic_field = observation(true_attitude, magnetic_reference),
        .sun_direction = observation(true_attitude, sun_reference)
    };
}

}  // namespace

TEST_CASE("TRIAD recovers a known body-to-inertial attitude") {
    const Eigen::Quaterniond truth{
        Eigen::AngleAxisd{
            73.0 * std::numbers::pi / 180.0,
            Eigen::Vector3d{1.0, -2.0, 0.5}.normalized()
        }
    };
    const detumble::AttitudeEstimatorInput input =
        estimator_input(0.0, truth, Eigen::Vector3d::Zero());

    const auto triad = detumble::triad_attitude_body_to_inertial(
        *input.magnetic_field,
        *input.sun_direction
    );

    REQUIRE(triad.has_value());
    REQUIRE(detumble::attitude_error_angle_rad(*triad, truth)
            < 1.0e-12);
}

TEST_CASE("attitude estimator correction gain moves toward TRIAD") {
    const Eigen::Quaterniond truth{
        Eigen::AngleAxisd{1.0, Eigen::Vector3d::UnitZ()}
    };
    detumble::AttitudeEstimator estimator{
        detumble::AttitudeEstimatorConfig{.correction_gain = 0.25}
    };

    estimator.update(estimator_input(0.0, truth, Eigen::Vector3d::Zero()));

    REQUIRE(estimator.estimate().valid);
    REQUIRE(estimator.estimate().correction_status
            == detumble::AttitudeCorrectionStatus::accepted);
    REQUIRE(estimator.estimate().accepted_correction_count == 1);
    REQUIRE(detumble::attitude_error_angle_rad(
                estimator.estimate().body_to_inertial,
                truth
            ) == Catch::Approx(0.75).margin(1.0e-12));
}

TEST_CASE("gyro propagation tracks a slowly rotating attitude") {
    const Eigen::Vector3d rate{0.01, -0.02, 0.03};
    Eigen::Quaterniond truth{
        Eigen::AngleAxisd{1.4, Eigen::Vector3d{1.0, 2.0, -1.0}.normalized()}
    };
    detumble::AttitudeEstimator estimator{
        detumble::AttitudeEstimatorConfig{.correction_gain = 0.25}
    };

    estimator.update(estimator_input(0.0, truth, rate));
    for (int sample = 1; sample <= 80; ++sample) {
        truth = detumble::integrate_attitude(truth, rate, 0.1);
        estimator.update(estimator_input(0.1 * sample, truth, rate));
    }

    REQUIRE(detumble::attitude_error_angle_rad(
                estimator.estimate().body_to_inertial,
                truth
            ) < 1.0e-8);
}

TEST_CASE("gyro propagation holds the previous sample over each interval") {
    detumble::AttitudeEstimator estimator{
        detumble::AttitudeEstimatorConfig{.correction_gain = 1.0}
    };
    estimator.update(estimator_input(
        0.0,
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d::UnitX()
    ));

    estimator.update({
        .sample_time_s = 1.0,
        .angular_velocity_body_rad_s = Eigen::Vector3d::Zero(),
        .gyroscope_valid = true
    });

    const Eigen::Quaterniond expected{
        Eigen::AngleAxisd{1.0, Eigen::Vector3d::UnitX()}
    };
    REQUIRE(detumble::attitude_error_angle_rad(
                estimator.estimate().body_to_inertial,
                expected
            ) < 1.0e-12);
}

TEST_CASE("estimator propagates through eclipse and corrects afterward") {
    const Eigen::Vector3d rate{0.005, 0.01, -0.008};
    Eigen::Quaterniond truth{
        Eigen::AngleAxisd{0.8, Eigen::Vector3d{2.0, -1.0, 1.0}.normalized()}
    };
    detumble::AttitudeEstimator estimator{
        detumble::AttitudeEstimatorConfig{.correction_gain = 1.0}
    };
    estimator.update(estimator_input(0.0, truth, rate));

    for (int sample = 1; sample <= 100; ++sample) {
        truth = detumble::integrate_attitude(truth, rate, 0.1);
        auto input = estimator_input(0.1 * sample, truth, rate);
        input.sun_direction->valid = false;
        estimator.update(input);
        REQUIRE(estimator.estimate().correction_status
                == detumble::AttitudeCorrectionStatus::unavailable);
    }

    REQUIRE(estimator.estimate().valid);
    REQUIRE(detumble::attitude_error_angle_rad(
                estimator.estimate().body_to_inertial,
                truth
            ) < 1.0e-7);

    truth = detumble::integrate_attitude(truth, rate, 0.1);
    estimator.update(estimator_input(10.1, truth, rate));
    REQUIRE(estimator.estimate().correction_status
            == detumble::AttitudeCorrectionStatus::accepted);
    REQUIRE(detumble::attitude_error_angle_rad(
                estimator.estimate().body_to_inertial,
                truth
            ) < 1.0e-12);
}

TEST_CASE("TRIAD rejects nearly parallel vector pairs") {
    const detumble::AttitudeVectorObservation magnetic{
        .measured_body = Eigen::Vector3d::UnitX(),
        .reference_inertial = Eigen::Vector3d::UnitY(),
        .valid = true
    };
    const detumble::AttitudeVectorObservation sun{
        .measured_body = Eigen::Vector3d{1.0, 1.0e-4, 0.0},
        .reference_inertial = Eigen::Vector3d{0.0, 1.0, 1.0e-4},
        .valid = true
    };
    detumble::AttitudeEstimator estimator;

    estimator.update({
        .sample_time_s = 0.0,
        .magnetic_field = magnetic,
        .sun_direction = sun
    });

    REQUIRE_FALSE(detumble::triad_attitude_body_to_inertial(
        magnetic,
        sun
    ).has_value());
    REQUIRE_FALSE(estimator.estimate().valid);
    REQUIRE(estimator.estimate().correction_status
            == detumble::AttitudeCorrectionStatus::degenerate_vectors);
}

TEST_CASE("estimator rejects invalid observations without losing propagation") {
    detumble::AttitudeEstimator estimator;
    auto input = estimator_input(
        0.0,
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d::Zero()
    );
    input.magnetic_field->measured_body.x() =
        std::numeric_limits<double>::quiet_NaN();

    estimator.update(input);

    REQUIRE_FALSE(estimator.estimate().valid);
    REQUIRE(estimator.estimate().correction_status
            == detumble::AttitudeCorrectionStatus::invalid_measurement);
    REQUIRE(estimator.estimate().body_to_inertial.w()
            == Catch::Approx(1.0));
}

TEST_CASE("attitude error treats opposite quaternion signs as equal") {
    const Eigen::Quaterniond attitude{
        Eigen::AngleAxisd{0.4, Eigen::Vector3d::UnitY()}
    };
    Eigen::Quaterniond negated = attitude;
    negated.coeffs() *= -1.0;

    REQUIRE(detumble::attitude_error_angle_rad(attitude, negated)
            < 1.0e-12);
}
