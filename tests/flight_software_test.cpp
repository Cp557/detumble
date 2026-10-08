#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <numbers>
namespace {
void sample(detumble::DetumbleFlightSoftware &software, double time,
            const Eigen::Vector3d &field) {
    software.update(time, detumble::MagnetometerMeasurement{time, field},
                    {0.0, 0.0, 30e-6});
}
} // namespace
TEST_CASE("magnetic-only flight software starts after two valid samples") {
    detumble::DetumbleFlightSoftwareConfig config;
    config.bdot_estimator.filter_time_constant_s = 0.0;
    config.attitude_estimator.enabled = false;
    detumble::DetumbleFlightSoftware software{config};
    sample(software, 0.0, {0.0, 0.0, 30e-6});
    REQUIRE(software.state().mode == detumble::FlightMode::boot);
    sample(software, 0.1, {0.2e-6, 0.0, 30e-6});
    REQUIRE(software.state().mode == detumble::FlightMode::detumble);
    REQUIRE(software.state().commanded_dipole_body_A_m2.x() == Catch::Approx(-0.1));
    REQUIRE_FALSE(software.state().low_rate_confirmed);
}
TEST_CASE("missing and corrupt magnetic data disable rods and recover") {
    detumble::DetumbleFlightSoftware software;
    sample(software, 0.0, {0.0, 0.0, 30e-6});
    sample(software, 0.1, {1e-6, 0.0, 30e-6});
    software.update(0.5, std::nullopt, {0.0, 0.0, 30e-6});
    REQUIRE(software.state().mode == detumble::FlightMode::safe);
    REQUIRE(software.state().commanded_dipole_body_A_m2.isZero());
    sample(software, 0.6, {0.0, 0.0, 30e-6});
    sample(software, 0.7, {1e-6, 0.0, 30e-6});
    REQUIRE(software.state().mode == detumble::FlightMode::detumble);
    sample(software, 0.8, {std::numeric_limits<double>::quiet_NaN(), 0.0, 30e-6});
    REQUIRE(software.state().mode == detumble::FlightMode::safe);
    REQUIRE(software.state().commanded_dipole_body_A_m2.isZero());
}
TEST_CASE("estimated-rate selection falls back before confidence is established") {
    detumble::DetumbleFlightSoftwareConfig config;
    config.controller_selection = detumble::DetumbleController::estimated_rate;
    detumble::DetumbleFlightSoftware software{config};
    sample(software, 0.0, {0.0, 0.0, 30e-6});
    sample(software, 0.1, {1e-6, 0.0, 30e-6});
    REQUIRE_FALSE(software.state().estimated_rate_control_active);
    REQUIRE_FALSE(software.state().commanded_dipole_body_A_m2.isZero());
    REQUIRE_FALSE(software.state().low_rate_confirmed);
}

TEST_CASE("flight software rejects saturation at the configured sensor limit") {
    detumble::VectorSensorErrorConfig sensor;
    sensor.saturation_limit.setConstant(50e-6);
    detumble::DetumbleFlightSoftware software{
        {}, {}, detumble::generic_3u_cubesat(), sensor};
    sample(software, 0.0, {0.0, 0.0, 30e-6});
    sample(software, 0.1, {50e-6, 0.0, 30e-6});
    REQUIRE(software.state().mode == detumble::FlightMode::safe);
    REQUIRE(software.state().commanded_dipole_body_A_m2.isZero());
}

TEST_CASE("an estimator outlier restores B-dot after a successful handoff") {
    detumble::Environment environment;
    detumble::AttitudeState truth;
    truth.angular_velocity_body_rad_s =
        environment.reference_field_inertial_T(0.0).normalized() * 15.0 *
        std::numbers::pi / 180.0;
    detumble::DetumbleFlightSoftwareConfig config;
    config.controller_selection = detumble::DetumbleController::estimated_rate;
    config.handoff_dwell_time_s = 0.0;
    detumble::DetumbleFlightSoftware software{config};
    for (unsigned step = 0; step <= 1200; ++step) {
        const double time = 0.1 * step;
        const auto field = environment.reference_field_inertial_T(time);
        software.update(time,
                        detumble::MagnetometerMeasurement{
                            time, truth.body_to_inertial.conjugate() * field},
                        field);
        if (step < 1200) {
            software.predict(time, field, Eigen::Vector3d::Zero(), 0.1);
            truth = detumble::propagate_rigid_body_rk4(
                detumble::generic_3u_cubesat(), truth, Eigen::Vector3d::Zero(), 0.1);
        }
    }
    REQUIRE(software.state().estimated_rate_control_active);
    const auto field = environment.reference_field_inertial_T(120.1);
    software.update(120.1,
                    detumble::MagnetometerMeasurement{
                        120.1, truth.body_to_inertial.conjugate() * field +
                                   Eigen::Vector3d{20e-6, 0.0, 0.0}},
                    field);
    REQUIRE_FALSE(software.estimate().confident);
    REQUIRE_FALSE(software.state().estimated_rate_control_active);
    REQUIRE(software.state().mode == detumble::FlightMode::detumble);
    REQUIRE(software.state().commanded_dipole_body_A_m2 ==
            detumble::bdot_dipole_command_body_A_m2(
                config.controller, {}, software.state().filtered_bdot_body_T_s));
}
