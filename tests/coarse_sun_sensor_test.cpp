#include <cmath>
#include <numbers>
#include <stdexcept>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/coarse_sun_sensor.hpp"

TEST_CASE("coarse Sun sensors reproduce a known body-frame Sun direction") {
    detumble::IdealCoarseSunSensorArray sensors;
    const Eigen::Vector3d sun_direction =
        Eigen::Vector3d{1.0, -2.0, 3.0}.normalized();

    const auto measurement = sensors.sample_if_due(
        0.0,
        sun_direction,
        false
    );

    REQUIRE(measurement.has_value());
    REQUIRE(measurement->sun_direction_body.has_value());
    REQUIRE(
        measurement->sun_direction_body->isApprox(sun_direction, 1.0e-12)
    );
    REQUIRE(measurement->sensor_visible[0]);
    REQUIRE_FALSE(measurement->sensor_visible[1]);
    REQUIRE_FALSE(measurement->sensor_visible[2]);
    REQUIRE(measurement->sensor_visible[3]);
    REQUIRE(measurement->sensor_visible[4]);
    REQUIRE_FALSE(measurement->sensor_visible[5]);
}

TEST_CASE("coarse Sun sensors return no valid direction during eclipse") {
    detumble::IdealCoarseSunSensorArray sensors;
    const auto measurement = sensors.sample_if_due(
        0.0,
        Eigen::Vector3d::UnitX(),
        true
    );

    REQUIRE(measurement.has_value());
    REQUIRE(measurement->in_eclipse);
    REQUIRE_FALSE(measurement->sun_direction_body.has_value());
    for (std::size_t index = 0;
         index < measurement->sensor_visible.size();
         ++index) {
        REQUIRE_FALSE(measurement->sensor_visible[index]);
        REQUIRE(measurement->illumination[index] == 0.0);
    }
}

TEST_CASE("coarse Sun sensor field of view rejects shallow illumination") {
    detumble::CoarseSunSensorArrayConfig config =
        detumble::generic_3u_coarse_sun_sensor_config();
    config.field_of_view_half_angle_rad = 30.0 * std::numbers::pi / 180.0;
    detumble::IdealCoarseSunSensorArray sensors{config};
    const Eigen::Vector3d sun_direction{
        std::cos(45.0 * std::numbers::pi / 180.0),
        std::sin(45.0 * std::numbers::pi / 180.0),
        0.0
    };

    const auto measurement = sensors.sample_if_due(
        0.0,
        sun_direction,
        false
    );

    REQUIRE(measurement.has_value());
    REQUIRE_FALSE(measurement->sensor_visible[0]);
    REQUIRE_FALSE(measurement->sensor_visible[2]);
    REQUIRE_FALSE(measurement->sun_direction_body.has_value());
}

TEST_CASE("coarse Sun sensors use their configured sample rate") {
    detumble::CoarseSunSensorArrayConfig config =
        detumble::generic_3u_coarse_sun_sensor_config();
    config.sample_rate_hz = 5.0;
    detumble::IdealCoarseSunSensorArray sensors{config};

    REQUIRE(sensors.sample_if_due(
        0.0,
        Eigen::Vector3d::UnitX(),
        false
    ).has_value());
    REQUIRE_FALSE(sensors.sample_if_due(
        0.1,
        Eigen::Vector3d::UnitX(),
        false
    ).has_value());
    const auto next = sensors.sample_if_due(
        0.2,
        Eigen::Vector3d::UnitX(),
        false
    );
    REQUIRE(next.has_value());
    REQUIRE(next->sample_time_s == Catch::Approx(0.2));
}

TEST_CASE("coarse Sun sensors reject invalid configuration") {
    detumble::CoarseSunSensorArrayConfig config =
        detumble::generic_3u_coarse_sun_sensor_config();
    config.sensors[0].outward_normal_body = Eigen::Vector3d{2.0, 0.0, 0.0};
    REQUIRE_THROWS_AS(
        detumble::IdealCoarseSunSensorArray{config},
        std::invalid_argument
    );
}
