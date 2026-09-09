#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/flight_software.hpp"

namespace {

detumble::AutonomousFlightSoftwareConfig fast_mission_config() {
    detumble::AutonomousFlightSoftwareConfig config;
    config.detumble.estimator.filter_time_constant_s = 0.0;
    config.detumble.completion_threshold_rad_s = 1.0;
    config.detumble.completion_hysteresis_rad_s = 0.1;
    config.detumble.completion_dwell_time_s = 0.0;
    config.attitude_estimator.correction_gain = 1.0;
    config.sun_pointing.dwell_time_s = 0.0;
    config.boot_to_detumble_dwell_time_s = 0.0;
    config.post_detumble_dwell_time_s = 0.0;
    config.acquire_to_point_dwell_time_s = 0.0;
    config.pointing_loss_dwell_time_s = 0.0;
    config.safe_entry_dwell_time_s = 0.0;
    config.safe_recovery_dwell_time_s = 0.0;
    return config;
}

detumble::AutonomousFlightSoftwareInput mission_input(
    const double sample_time_s,
    const Eigen::Quaterniond& body_to_inertial,
    const bool include_magnetometer = true,
    const bool include_sun_sensor = true,
    const bool in_eclipse = false
) {
    const Eigen::Vector3d magnetic_field_inertial_T{0.0, 0.0, 30.0e-6};
    const Eigen::Vector3d sun_direction_inertial = Eigen::Vector3d::UnitX();
    const Eigen::Quaterniond inertial_to_body = body_to_inertial.conjugate();

    detumble::AutonomousFlightSoftwareInput input{
        .gyroscope = {
            .sample_time_s = sample_time_s,
            .angular_velocity_body_rad_s = Eigen::Vector3d::Zero()
        },
        .magnetic_field_inertial_T = magnetic_field_inertial_T,
        .sun_direction_inertial = sun_direction_inertial,
        .in_eclipse = in_eclipse
    };
    if (include_magnetometer) {
        input.magnetometer = detumble::MagnetometerMeasurement{
            .sample_time_s = sample_time_s,
            .magnetic_field_body_T =
                inertial_to_body * magnetic_field_inertial_T
        };
    }
    if (include_sun_sensor) {
        input.sun_sensor = detumble::CoarseSunSensorMeasurement{
            .sample_time_s = sample_time_s,
            .sun_direction_body = in_eclipse
                ? std::optional<Eigen::Vector3d>{}
                : std::optional<Eigen::Vector3d>{
                      inertial_to_body * sun_direction_inertial
                  },
            .in_eclipse = in_eclipse
        };
    }
    return input;
}

}  // namespace

TEST_CASE("autonomous flight software executes and logs the mission modes") {
    const detumble::AutonomousFlightSoftwareConfig config =
        fast_mission_config();
    detumble::AutonomousFlightSoftware flight_software{config};
    const Eigen::Quaterniond desired =
        detumble::desired_sun_pointing_attitude(
            config.sun_pointing,
            Eigen::Vector3d::UnitX()
        );

    flight_software.update(mission_input(0.0, desired));
    REQUIRE(flight_software.state().mode == detumble::FlightMode::boot);
    REQUIRE_FALSE(
        flight_software.state().actuator_command.magnetorquers_enabled
    );
    REQUIRE_FALSE(
        flight_software.state().actuator_command.reaction_wheels_enabled
    );

    flight_software.update(mission_input(0.1, desired));
    REQUIRE(flight_software.state().mode == detumble::FlightMode::detumble);

    flight_software.update(mission_input(0.2, desired));
    REQUIRE(
        flight_software.state().mode == detumble::FlightMode::sun_acquire
    );
    REQUIRE(
        flight_software.state().actuator_command.reaction_wheels_enabled
    );
    REQUIRE(flight_software.state().sun_pointing.active);

    flight_software.update(mission_input(0.3, desired));
    REQUIRE(flight_software.state().mode == detumble::FlightMode::sun_point);
    REQUIRE(flight_software.events().size() == 3);
    REQUIRE(
        flight_software.events()[0].reason
        == detumble::ModeTransitionReason::sensors_ready
    );
    REQUIRE(
        flight_software.events()[1].reason
        == detumble::ModeTransitionReason::detumble_complete
    );
    REQUIRE(
        flight_software.events()[2].reason
        == detumble::ModeTransitionReason::sun_acquired
    );

    flight_software.update(mission_input(0.4, desired, true, true, true));
    REQUIRE(flight_software.state().mode == detumble::FlightMode::safe);
    REQUIRE_FALSE(
        flight_software.state().actuator_command.magnetorquers_enabled
    );
    REQUIRE_FALSE(
        flight_software.state().actuator_command.reaction_wheels_enabled
    );
    REQUIRE(
        flight_software.events().back().reason
        == detumble::ModeTransitionReason::sun_unavailable
    );

    flight_software.update(mission_input(0.5, desired));
    REQUIRE(
        flight_software.state().mode == detumble::FlightMode::sun_acquire
    );
    REQUIRE(
        flight_software.events().back().reason
        == detumble::ModeTransitionReason::navigation_recovered
    );
}

TEST_CASE("autonomous flight software enters SAFE without an attitude") {
    detumble::AutonomousFlightSoftware flight_software{
        fast_mission_config()
    };
    const Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();

    flight_software.update(mission_input(0.0, attitude, true, false));
    flight_software.update(mission_input(0.1, attitude, true, false));
    REQUIRE(flight_software.state().mode == detumble::FlightMode::detumble);
    REQUIRE_FALSE(flight_software.state().attitude_valid);

    flight_software.update(mission_input(0.2, attitude, false, true));
    REQUIRE(flight_software.state().sun_available);
    REQUIRE_FALSE(flight_software.state().attitude_valid);
    REQUIRE(flight_software.state().mode == detumble::FlightMode::safe);
    REQUIRE(
        flight_software.events().back().reason
        == detumble::ModeTransitionReason::attitude_invalid
    );
}

TEST_CASE("mission transitions require continuous dwell time") {
    detumble::AutonomousFlightSoftwareConfig config = fast_mission_config();
    config.boot_to_detumble_dwell_time_s = 1.0;
    detumble::AutonomousFlightSoftware flight_software{config};
    const Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();

    flight_software.update(mission_input(0.0, attitude));
    flight_software.update(mission_input(0.1, attitude));
    flight_software.update(mission_input(0.9, attitude));
    REQUIRE(flight_software.state().mode == detumble::FlightMode::boot);
    REQUIRE(
        flight_software.state().pending_transition_elapsed_s
        == Catch::Approx(0.8)
    );

    flight_software.update(mission_input(1.1, attitude));
    REQUIRE(flight_software.state().mode == detumble::FlightMode::detumble);
    REQUIRE(flight_software.events().size() == 1);
    REQUIRE(
        flight_software.events().front().sample_time_s
        == Catch::Approx(1.1)
    );
}

TEST_CASE("autonomous flight software rejects invalid timing") {
    detumble::AutonomousFlightSoftware flight_software;
    auto input = mission_input(0.0, Eigen::Quaterniond::Identity());
    input.sun_sensor->sample_time_s = 0.1;
    REQUIRE_THROWS_AS(
        flight_software.update(input),
        std::invalid_argument
    );

    detumble::AutonomousFlightSoftwareConfig config;
    config.safe_recovery_dwell_time_s =
        std::numeric_limits<double>::infinity();
    REQUIRE_THROWS_AS(
        detumble::AutonomousFlightSoftware{config},
        std::invalid_argument
    );
}
