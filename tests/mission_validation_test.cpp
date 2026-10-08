#include "detumble/mission.hpp"
#include <catch2/catch_test_macros.hpp>
TEST_CASE("shared mission reset and viewer batches exactly replay headless physics") {
    detumble::MissionRunConfig config;
    config.flight_software.attitude_estimator.enabled = false;
    detumble::DetumbleMission headless{config}, viewer{config};
    for (unsigned i = 0; i < 1000; ++i)
        headless.step();
    for (unsigned frame = 0; frame < 100; ++frame)
        for (unsigned step = 0; step < 10; ++step)
            viewer.step();
    REQUIRE(headless.telemetry().truth.body_to_inertial.coeffs() ==
            viewer.telemetry().truth.body_to_inertial.coeffs());
    REQUIRE(headless.telemetry().truth.angular_velocity_body_rad_s ==
            viewer.telemetry().truth.angular_velocity_body_rad_s);
    viewer.reset();
    for (unsigned i = 0; i < 1000; ++i)
        viewer.step();
    REQUIRE(headless.telemetry().truth.angular_velocity_body_rad_s ==
            viewer.telemetry().truth.angular_velocity_body_rad_s);
    REQUIRE(headless.telemetry().measurement->magnetic_field_body_T ==
            viewer.telemetry().measurement->magnetic_field_body_T);
}
TEST_CASE("mission scoring cannot grant success without enough verification time") {
    detumble::MissionRunConfig config;
    config.duration_s = 60.0;
    config.initial_rate_rad_s = 0.0;
    config.flight_software.attitude_estimator.enabled = false;
    const auto metrics = detumble::run_native_mission(config);
    REQUIRE(metrics.detumble_time_s.has_value());
    REQUIRE_FALSE(metrics.success);
    REQUIRE(metrics.failure_reason ==
            detumble::MissionFailureReason::insufficient_verification);
}
