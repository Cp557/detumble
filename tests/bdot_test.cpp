#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/bdot.hpp"

TEST_CASE("B-dot estimator differentiates and filters sampled fields") {
    detumble::BdotEstimator estimator{
        detumble::BdotEstimatorConfig{.filter_time_constant_s = 1.0}
    };

    const auto first = estimator.update({
        .sample_time_s = 0.0,
        .magnetic_field_body_T = {0.0, 0.0, 0.0}
    });
    const auto second = estimator.update({
        .sample_time_s = 1.0,
        .magnetic_field_body_T = {2.0e-6, 0.0, 0.0}
    });
    const auto third = estimator.update({
        .sample_time_s = 2.0,
        .magnetic_field_body_T = {2.0e-6, 2.0e-6, 0.0}
    });

    REQUIRE_FALSE(first.has_value());
    REQUIRE(second.has_value());
    REQUIRE(second->raw_bdot_body_T_s.x() == Catch::Approx(2.0e-6));
    REQUIRE(
        second->filtered_bdot_body_T_s.x() == Catch::Approx(2.0e-6)
    );
    REQUIRE(third.has_value());
    REQUIRE(third->raw_bdot_body_T_s.x() == Catch::Approx(0.0));
    REQUIRE(third->raw_bdot_body_T_s.y() == Catch::Approx(2.0e-6));
    REQUIRE(
        third->filtered_bdot_body_T_s.x()
        == Catch::Approx(2.0e-6 * std::exp(-1.0))
    );
    REQUIRE(
        third->filtered_bdot_body_T_s.y()
        == Catch::Approx(2.0e-6 * (1.0 - std::exp(-1.0)))
    );
}

TEST_CASE("B-dot command opposes field change and saturates") {
    const detumble::BdotControllerConfig controller{
        .gain_A_m2_s_per_T = 50'000.0
    };
    const detumble::MagnetorquerConfig magnetorquers;

    const Eigen::Vector3d unsaturated =
        detumble::bdot_dipole_command_body_A_m2(
            controller,
            magnetorquers,
            {2.0e-6, -1.0e-6, 0.0}
        );
    const Eigen::Vector3d saturated =
        detumble::bdot_dipole_command_body_A_m2(
            controller,
            magnetorquers,
            {10.0e-6, -10.0e-6, 0.0}
        );

    REQUIRE(unsaturated.x() == Catch::Approx(-0.1));
    REQUIRE(unsaturated.y() == Catch::Approx(0.05));
    REQUIRE(saturated.x() == Catch::Approx(-0.2));
    REQUIRE(saturated.y() == Catch::Approx(0.2));
}

TEST_CASE("B-dot estimator rejects repeated sample times") {
    detumble::BdotEstimator estimator;
    static_cast<void>(estimator.update({
        .sample_time_s = 1.0,
        .magnetic_field_body_T = Eigen::Vector3d::Zero()
    }));

    REQUIRE_THROWS_AS(
        estimator.update({
            .sample_time_s = 1.0,
            .magnetic_field_body_T = Eigen::Vector3d::Zero()
        }),
        std::invalid_argument
    );
}
