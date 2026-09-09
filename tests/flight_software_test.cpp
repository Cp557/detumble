#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/flight_software.hpp"

namespace {

void update_flight_software(
    detumble::DetumbleFlightSoftware& flight_software,
    const double time_s,
    const Eigen::Vector3d& field_body_T,
    const double angular_speed_rad_s
) {
    flight_software.update(
        {
            .sample_time_s = time_s,
            .magnetic_field_body_T = field_body_T
        },
        {
            .sample_time_s = time_s,
            .angular_velocity_body_rad_s = {
                angular_speed_rad_s,
                0.0,
                0.0
            }
        }
    );
}

}  // namespace

TEST_CASE("flight software leaves BOOT after two field samples") {
    detumble::DetumbleFlightSoftware flight_software{
        detumble::DetumbleFlightSoftwareConfig{
            .estimator = {.filter_time_constant_s = 0.0},
            .controller = {.gain_A_m2_s_per_T = 50'000.0}
        }
    };

    update_flight_software(
        flight_software,
        0.0,
        Eigen::Vector3d::Zero(),
        0.2
    );
    REQUIRE(
        flight_software.state().mode == detumble::FlightMode::boot
    );
    REQUIRE(flight_software.state().commanded_dipole_body_A_m2.isZero());

    update_flight_software(
        flight_software,
        1.0,
        {2.0e-6, 0.0, 0.0},
        0.2
    );
    REQUIRE(
        flight_software.state().mode == detumble::FlightMode::detumble
    );
    REQUIRE(
        flight_software.state().commanded_dipole_body_A_m2.x()
        == Catch::Approx(-0.1)
    );
}

TEST_CASE("detumble completion uses hysteresis and dwell time") {
    detumble::DetumbleFlightSoftware flight_software{
        detumble::DetumbleFlightSoftwareConfig{
            .estimator = {.filter_time_constant_s = 0.0},
            .controller = {},
            .completion_threshold_rad_s = 0.1,
            .completion_hysteresis_rad_s = 0.05,
            .completion_dwell_time_s = 2.0
        }
    };

    update_flight_software(flight_software, 0.0, {0.0, 0.0, 0.0}, 0.09);
    update_flight_software(flight_software, 1.0, {1.0e-6, 0.0, 0.0}, 0.12);
    REQUIRE(
        flight_software.state().below_threshold_elapsed_s
        == Catch::Approx(1.0)
    );

    update_flight_software(flight_software, 2.0, {2.0e-6, 0.0, 0.0}, 0.16);
    REQUIRE(flight_software.state().below_threshold_elapsed_s == 0.0);

    update_flight_software(flight_software, 3.0, {3.0e-6, 0.0, 0.0}, 0.09);
    update_flight_software(flight_software, 4.0, {4.0e-6, 0.0, 0.0}, 0.12);
    update_flight_software(flight_software, 5.0, {5.0e-6, 0.0, 0.0}, 0.09);

    REQUIRE(flight_software.state().detumble_complete);
    REQUIRE(
        flight_software.state().below_threshold_elapsed_s
        == Catch::Approx(2.0)
    );
    REQUIRE(flight_software.state().commanded_dipole_body_A_m2.isZero());
}
