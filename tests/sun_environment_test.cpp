#include <numbers>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/environment.hpp"
#include "detumble/orbit.hpp"

TEST_CASE("Earth eclipses a spacecraft behind it but not in front") {
    constexpr double earth_radius_m = 6'378'137.0;
    const double orbit_radius_m = earth_radius_m + 500'000.0;
    const Eigen::Vector3d sun_direction = Eigen::Vector3d::UnitX();

    REQUIRE(detumble::earth_occults_sun(
        {-orbit_radius_m, 0.0, 0.0},
        sun_direction,
        earth_radius_m
    ));
    REQUIRE_FALSE(detumble::earth_occults_sun(
        {orbit_radius_m, 0.0, 0.0},
        sun_direction,
        earth_radius_m
    ));
    REQUIRE_FALSE(detumble::earth_occults_sun(
        {-orbit_radius_m, 1.1 * earth_radius_m, 0.0},
        sun_direction,
        earth_radius_m
    ));
}

TEST_CASE("default orbit crosses the cylindrical Earth shadow") {
    const detumble::EnvironmentConfig config;
    const double half_period_s =
        0.5 * detumble::circular_orbit_period_s(config.orbit);

    const detumble::EnvironmentState sunlit = detumble::sample_environment(
        config,
        Eigen::Quaterniond::Identity(),
        0.0
    );
    const detumble::EnvironmentState eclipse = detumble::sample_environment(
        config,
        Eigen::Quaterniond::Identity(),
        half_period_s
    );

    REQUIRE_FALSE(sunlit.in_eclipse);
    REQUIRE(eclipse.in_eclipse);
    REQUIRE(sunlit.sun_direction_inertial == Eigen::Vector3d::UnitX());
    REQUIRE(sunlit.sun_direction_body == Eigen::Vector3d::UnitX());
}

TEST_CASE("Sun sensor validity follows orbital eclipse state") {
    const detumble::EnvironmentConfig config;
    const double half_period_s =
        0.5 * detumble::circular_orbit_period_s(config.orbit);
    detumble::IdealCoarseSunSensorArray sensors;

    const detumble::EnvironmentState sunlit = detumble::sample_environment(
        config,
        Eigen::Quaterniond::Identity(),
        0.0
    );
    const auto sunlit_measurement = sensors.sample_if_due(
        0.0,
        sunlit.sun_direction_body,
        sunlit.in_eclipse
    );
    sensors.reset();
    const detumble::EnvironmentState eclipse = detumble::sample_environment(
        config,
        Eigen::Quaterniond::Identity(),
        half_period_s
    );
    const auto eclipse_measurement = sensors.sample_if_due(
        half_period_s,
        eclipse.sun_direction_body,
        eclipse.in_eclipse
    );

    REQUIRE(sunlit_measurement.has_value());
    REQUIRE(eclipse_measurement.has_value());
    REQUIRE(sunlit_measurement->sun_direction_body.has_value());
    REQUIRE_FALSE(eclipse_measurement->sun_direction_body.has_value());
}

TEST_CASE("Sun direction transforms from inertial to body coordinates") {
    detumble::EnvironmentConfig config;
    config.sun.direction_inertial = Eigen::Vector3d::UnitY();
    const Eigen::Quaterniond body_to_inertial{
        Eigen::AngleAxisd{
            0.5 * std::numbers::pi,
            Eigen::Vector3d::UnitZ()
        }
    };

    const detumble::EnvironmentState environment =
        detumble::sample_environment(config, body_to_inertial, 0.0);

    REQUIRE(environment.sun_direction_body.isApprox(
        Eigen::Vector3d::UnitX(),
        1.0e-12
    ));
}

TEST_CASE("eclipse geometry rejects invalid inputs") {
    REQUIRE_THROWS_AS(
        detumble::earth_occults_sun(
            Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::Zero(),
            1.0
        ),
        std::invalid_argument
    );
}
