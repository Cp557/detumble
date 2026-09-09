#include <stdexcept>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/magnetometer.hpp"
#include "detumble/sensor_error.hpp"
#include "detumble/sensor_suite.hpp"

TEST_CASE("vector sensor errors are quantized and saturated") {
    detumble::VectorSensorErrorConfig config;
    config.quantization_step = Eigen::Vector3d::Constant(0.1);
    config.saturation_limit = Eigen::Vector3d::Constant(1.0);
    detumble::VectorSensorErrorModel model{config};

    const Eigen::Vector3d measured = model.apply({2.0, -2.0, 0.26});

    REQUIRE(measured.x() == Catch::Approx(1.0));
    REQUIRE(measured.y() == Catch::Approx(-1.0));
    REQUIRE(measured.z() == Catch::Approx(0.3));
}

TEST_CASE("seeded sensor bias and noise replay after reset") {
    auto config = detumble::realistic_sensor_suite_config(1'234).gyroscope;
    detumble::IdealGyroscope first{config};
    detumble::IdealGyroscope second{config};
    const Eigen::Vector3d truth{0.1, -0.2, 0.3};

    const auto first_sample = first.sample_if_due(0.0, truth);
    const auto second_sample = second.sample_if_due(0.0, truth);
    REQUIRE(first.bias_rad_s().isApprox(second.bias_rad_s(), 0.0));
    REQUIRE(first_sample->angular_velocity_body_rad_s.isApprox(
        second_sample->angular_velocity_body_rad_s,
        0.0
    ));

    first.reset();
    const auto replayed = first.sample_if_due(0.0, truth);
    REQUIRE(replayed->angular_velocity_body_rad_s.isApprox(
        first_sample->angular_velocity_body_rad_s,
        0.0
    ));
}

TEST_CASE("seeded magnetometer errors are reproducible") {
    const auto config =
        detumble::realistic_sensor_suite_config(8'675).magnetometer;
    detumble::IdealMagnetometer first{config};
    detumble::IdealMagnetometer second{config};
    const Eigen::Vector3d truth_T{20.0e-6, -35.0e-6, 42.0e-6};

    const auto first_sample = first.sample_if_due(0.0, truth_T, false);
    const auto second_sample = second.sample_if_due(0.0, truth_T, false);

    REQUIRE(first.bias_T().isApprox(second.bias_T(), 0.0));
    REQUIRE(first_sample->magnetic_field_body_T.isApprox(
        second_sample->magnetic_field_body_T,
        0.0
    ));
}

TEST_CASE("coarse Sun sensor errors remain physical and reproducible") {
    const auto config = detumble::realistic_sensor_suite_config(99).sun_sensor;
    detumble::IdealCoarseSunSensorArray first{config};
    detumble::IdealCoarseSunSensorArray second{config};

    const auto first_sample = first.sample_if_due(
        0.0,
        Eigen::Vector3d{1.0, 0.2, -0.1}.normalized(),
        false
    );
    const auto second_sample = second.sample_if_due(
        0.0,
        Eigen::Vector3d{1.0, 0.2, -0.1}.normalized(),
        false
    );

    REQUIRE(first.illumination_bias() == second.illumination_bias());
    REQUIRE(first_sample->illumination == second_sample->illumination);
    for (const double illumination : first_sample->illumination) {
        REQUIRE(illumination >= 0.0);
        REQUIRE(illumination <= config.maximum_illumination);
    }
    REQUIRE(first_sample->sun_direction_body.has_value());
}

TEST_CASE("invalid sensor error settings are rejected") {
    detumble::VectorSensorErrorConfig config;
    config.noise_standard_deviation.x() = -1.0;
    REQUIRE_THROWS_AS(
        detumble::VectorSensorErrorModel{config},
        std::invalid_argument
    );

    config = {};
    config.saturation_limit.x() = 0.0;
    REQUIRE_THROWS_AS(
        detumble::VectorSensorErrorModel{config},
        std::invalid_argument
    );
}
