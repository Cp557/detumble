#include "detumble/mission.hpp"
#include <catch2/catch_test_macros.hpp>
TEST_CASE("short magnetic mission reports a detumble timeout") {
    detumble::MissionRunConfig config;
    config.duration_s = 1.0;
    const auto metrics = detumble::run_native_mission(config);
    REQUIRE_FALSE(metrics.success);
    REQUIRE(metrics.failure_reason == detumble::MissionFailureReason::detumble_timeout);
}
TEST_CASE("navigation mismatch does not change B-dot plant physics") {
    detumble::MissionRunConfig config;
    config.duration_s = 2.0;
    config.flight_software.attitude_estimator.enabled = false;
    detumble::DetumbleMission matched{config};
    config.navigation_environment = config.environment;
    config.navigation_environment->magnetic_field.model =
        detumble::MagneticFieldModel::dipole;
    detumble::DetumbleMission mismatch{config};
    for (unsigned i = 0; i < 100; ++i) {
        matched.step();
        mismatch.step();
    }
    REQUIRE(matched.telemetry().truth.body_to_inertial.coeffs() ==
            mismatch.telemetry().truth.body_to_inertial.coeffs());
    REQUIRE(matched.telemetry().truth.angular_velocity_body_rad_s ==
            mismatch.telemetry().truth.angular_velocity_body_rad_s);
}
