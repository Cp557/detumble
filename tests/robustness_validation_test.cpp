#include <numbers>

#include <catch2/catch_test_macros.hpp>

#include "detumble/mission.hpp"
#include "detumble/sensor_suite.hpp"

TEST_CASE("regression seed succeeds despite ending in eclipse") {
    constexpr std::uint64_t regression_seed = 9'203'715'970'740'300'229ULL;
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    detumble::MissionRunConfig config;
    config.seed = regression_seed;
    config.duration_s = 8'000.0;
    config.time_step_s = 0.02;
    config.initial_rate_rad_s = 5.7493772271265273 * degrees_to_radians;
    config.environment.orbit.initial_argument_of_latitude_rad =
        87.286958836250804 * degrees_to_radians;
    config.sensors = detumble::realistic_sensor_suite_config(regression_seed);

    const detumble::MissionMetrics metrics =
        detumble::run_native_mission(config);

    REQUIRE(metrics.success);
    REQUIRE(metrics.detumble_time_s.has_value());
    REQUIRE(metrics.acquisition_time_s.has_value());
    REQUIRE(metrics.steady_pointing_time_s.has_value());
    REQUIRE(metrics.rms_true_pointing_error_rad
            < 2.0 * degrees_to_radians);
    REQUIRE(metrics.final_mode == detumble::FlightMode::safe);
}

TEST_CASE("mission metrics report a short-run detumble timeout") {
    detumble::MissionRunConfig config;
    config.duration_s = 1.0;
    config.time_step_s = 0.02;
    config.sensors = detumble::realistic_sensor_suite_config(config.seed);

    const detumble::MissionMetrics metrics =
        detumble::run_native_mission(config);

    REQUIRE_FALSE(metrics.success);
    REQUIRE(
        metrics.failure_reason
        == detumble::MissionFailureReason::detumble_timeout
    );
}
