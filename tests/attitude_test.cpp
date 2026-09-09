#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/attitude.hpp"

namespace {

constexpr double tolerance = 1.0e-12;

void require_quaternion_approx(
    const Eigen::Quaterniond& actual,
    const Eigen::Quaterniond& expected
) {
    REQUIRE(actual.w() == Catch::Approx(expected.w()).margin(tolerance));
    REQUIRE(actual.x() == Catch::Approx(expected.x()).margin(tolerance));
    REQUIRE(actual.y() == Catch::Approx(expected.y()).margin(tolerance));
    REQUIRE(actual.z() == Catch::Approx(expected.z()).margin(tolerance));
}

}  // namespace

TEST_CASE("attitude state defaults to identity and zero body rate") {
    const detumble::AttitudeState state;

    require_quaternion_approx(
        state.body_to_inertial,
        Eigen::Quaterniond::Identity()
    );
    REQUIRE(state.angular_velocity_body_rad_s.isZero(tolerance));
}

TEST_CASE("attitude normalization produces a unit quaternion") {
    const Eigen::Quaterniond unnormalized{2.0, 0.0, 0.0, 0.0};

    const Eigen::Quaterniond normalized =
        detumble::normalized_attitude(unnormalized);

    REQUIRE(normalized.norm() == Catch::Approx(1.0).margin(tolerance));
    require_quaternion_approx(normalized, Eigen::Quaterniond::Identity());
}

TEST_CASE("attitude normalization rejects an invalid quaternion") {
    const Eigen::Quaterniond zero{0.0, 0.0, 0.0, 0.0};

    REQUIRE_THROWS_AS(
        detumble::normalized_attitude(zero),
        std::invalid_argument
    );
}

TEST_CASE("attitude integration applies a body-frame angular velocity") {
    constexpr double half_pi_rad = 1.5707963267948966;
    const Eigen::Vector3d body_rate_rad_s{0.0, 0.0, half_pi_rad};

    const Eigen::Quaterniond integrated = detumble::integrate_attitude(
        Eigen::Quaterniond::Identity(),
        body_rate_rad_s,
        1.0
    );
    const Eigen::Quaterniond expected{
        Eigen::AngleAxisd{half_pi_rad, Eigen::Vector3d::UnitZ()}
    };

    require_quaternion_approx(integrated, expected);
    REQUIRE(integrated.norm() == Catch::Approx(1.0).margin(tolerance));
}

TEST_CASE("zero-rate integration preserves and normalizes attitude") {
    const Eigen::Quaterniond scaled_attitude{2.0, 0.0, 0.0, 0.0};

    const Eigen::Quaterniond integrated = detumble::integrate_attitude(
        scaled_attitude,
        Eigen::Vector3d::Zero(),
        10.0
    );

    require_quaternion_approx(integrated, Eigen::Quaterniond::Identity());
}

TEST_CASE("attitude integration rejects invalid inputs") {
    REQUIRE_THROWS_AS(
        detumble::integrate_attitude(
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero(),
            -1.0
        ),
        std::invalid_argument
    );

    const Eigen::Vector3d invalid_rate{
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.0
    };
    REQUIRE_THROWS_AS(
        detumble::integrate_attitude(
            Eigen::Quaterniond::Identity(),
            invalid_rate,
            1.0
        ),
        std::invalid_argument
    );
}
