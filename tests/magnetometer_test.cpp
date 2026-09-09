#include <stdexcept>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/magnetometer.hpp"

TEST_CASE("ideal magnetometer samples the exact field at its configured rate") {
    detumble::IdealMagnetometer magnetometer{
        detumble::MagnetometerConfig{.sample_rate_hz = 10.0}
    };
    const Eigen::Vector3d field_body_T{20.0e-6, -5.0e-6, 30.0e-6};

    const auto initial = magnetometer.sample_if_due(
        0.0,
        field_body_T,
        false
    );
    const auto early = magnetometer.sample_if_due(
        0.05,
        field_body_T,
        true
    );
    const auto next = magnetometer.sample_if_due(
        0.1,
        field_body_T,
        false
    );

    REQUIRE(initial.has_value());
    REQUIRE_FALSE(early.has_value());
    REQUIRE(next.has_value());
    REQUIRE(next->sample_time_s == Catch::Approx(0.1));
    REQUIRE(next->magnetic_field_body_T == field_body_T);
}

TEST_CASE("magnetometer rejects sampling while torquers are energized") {
    detumble::IdealMagnetometer magnetometer;

    REQUIRE_THROWS_AS(
        magnetometer.sample_if_due(
            0.0,
            Eigen::Vector3d::Zero(),
            true
        ),
        std::logic_error
    );
}

TEST_CASE("magnetometer rejects invalid rate and decreasing time") {
    REQUIRE_THROWS_AS(
        detumble::IdealMagnetometer{
            detumble::MagnetometerConfig{.sample_rate_hz = 0.0}
        },
        std::invalid_argument
    );

    detumble::IdealMagnetometer magnetometer;
    static_cast<void>(magnetometer.sample_if_due(
        0.05,
        Eigen::Vector3d::Zero(),
        false
    ));
    REQUIRE_THROWS_AS(
        magnetometer.sample_if_due(
            0.04,
            Eigen::Vector3d::Zero(),
            false
        ),
        std::invalid_argument
    );
}
