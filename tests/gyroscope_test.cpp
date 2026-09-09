#include <stdexcept>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/gyroscope.hpp"

TEST_CASE("ideal gyroscope returns the exact body rate") {
    const Eigen::Vector3d true_rate{0.1, -0.2, 0.3};
    const detumble::GyroscopeMeasurement measurement =
        detumble::ideal_gyroscope_measurement(4.2, true_rate);

    REQUIRE(measurement.sample_time_s == 4.2);
    REQUIRE(measurement.angular_velocity_body_rad_s == true_rate);
}

TEST_CASE("ideal gyroscope rejects invalid measurements") {
    REQUIRE_THROWS_AS(
        detumble::ideal_gyroscope_measurement(
            -1.0,
            Eigen::Vector3d::Zero()
        ),
        std::invalid_argument
    );
}

TEST_CASE("ideal gyroscope samples at its configured rate") {
    detumble::IdealGyroscope gyroscope{
        detumble::GyroscopeConfig{.sample_rate_hz = 5.0}
    };
    const Eigen::Vector3d rate{0.1, 0.2, 0.3};

    const auto initial = gyroscope.sample_if_due(0.0, rate);
    const auto early = gyroscope.sample_if_due(0.1, rate);
    const auto next = gyroscope.sample_if_due(0.2, rate);

    REQUIRE(initial.has_value());
    REQUIRE_FALSE(early.has_value());
    REQUIRE(next.has_value());
    REQUIRE(next->sample_time_s == Catch::Approx(0.2));
    REQUIRE(next->angular_velocity_body_rad_s == rate);
}

TEST_CASE("ideal gyroscope resets and rejects invalid scheduling") {
    REQUIRE_THROWS_AS(
        detumble::IdealGyroscope{
            detumble::GyroscopeConfig{.sample_rate_hz = 0.0}
        },
        std::invalid_argument
    );

    detumble::IdealGyroscope gyroscope;
    static_cast<void>(gyroscope.sample_if_due(
        0.1,
        Eigen::Vector3d::Zero()
    ));
    REQUIRE_THROWS_AS(
        gyroscope.sample_if_due(0.09, Eigen::Vector3d::Zero()),
        std::invalid_argument
    );
    gyroscope.reset();
    REQUIRE(gyroscope.sample_if_due(0.0, Eigen::Vector3d::Zero()).has_value());
}
