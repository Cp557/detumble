#include <array>
#include <numbers>

#include <catch2/catch_test_macros.hpp>

#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/simulation.hpp"

namespace {

struct ScenarioResult {
    double initial_speed_rad_s{};
    double final_speed_rad_s{};
    double initial_energy_j{};
    double final_energy_j{};
    double completion_time_s{};
    bool complete{};
};

ScenarioResult run_detumble_scenario(
    const std::uint64_t seed,
    const double initial_speed_rad_s
) {
    constexpr double time_step_s = 0.02;
    constexpr double maximum_duration_s = 12'000.0;
    detumble::Simulation simulation{
        detumble::SimulationConfig{
            .seed = seed,
            .time_step_s = time_step_s,
            .minimum_angular_speed_rad_s = initial_speed_rad_s,
            .maximum_angular_speed_rad_s = initial_speed_rad_s
        }
    };
    detumble::MagneticControlCycle magnetic_control;
    detumble::DetumbleFlightSoftware flight_software;
    const detumble::EnvironmentConfig environment_config;
    const double initial_energy_j =
        simulation.metrics().rotational_kinetic_energy_j;

    while (simulation.elapsed_time_s() < maximum_duration_s
           && !flight_software.state().detumble_complete) {
        const detumble::EnvironmentState environment =
            detumble::sample_environment(
                environment_config,
                simulation.state().body_to_inertial,
                simulation.elapsed_time_s()
            );
        const detumble::MagneticControlOutput magnetic_output =
            magnetic_control.update(
                simulation.elapsed_time_s(),
                environment.magnetic_field_body_T,
                flight_software.state().commanded_dipole_body_A_m2
            );
        if (magnetic_output.magnetometer_measurement.has_value()) {
            flight_software.update(
                *magnetic_output.magnetometer_measurement,
                detumble::ideal_gyroscope_measurement(
                    simulation.elapsed_time_s(),
                    simulation.state().angular_velocity_body_rad_s
                )
            );
        }
        simulation.step(magnetic_output.applied_torque_body_Nm);
    }

    return {
        .initial_speed_rad_s = initial_speed_rad_s,
        .final_speed_rad_s =
            simulation.state().angular_velocity_body_rad_s.norm(),
        .initial_energy_j = initial_energy_j,
        .final_energy_j = simulation.metrics().rotational_kinetic_energy_j,
        .completion_time_s = simulation.elapsed_time_s(),
        .complete = flight_software.state().detumble_complete
    };
}

}  // namespace

TEST_CASE("B-dot detumbles representative seeded scenarios") {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    constexpr std::array<std::uint64_t, 3> seeds{7, 42, 2'026};
    constexpr std::array<double, 3> initial_speeds_rad_s{
        5.0 * degrees_to_radians,
        10.0 * degrees_to_radians,
        15.0 * degrees_to_radians
    };

    for (std::size_t index = 0; index < seeds.size(); ++index) {
        const ScenarioResult result = run_detumble_scenario(
            seeds[index],
            initial_speeds_rad_s[index]
        );
        CAPTURE(
            seeds[index],
            result.initial_speed_rad_s,
            result.final_speed_rad_s,
            result.initial_energy_j,
            result.final_energy_j,
            result.completion_time_s
        );
        REQUIRE(result.complete);
        REQUIRE(result.final_speed_rad_s < result.initial_speed_rad_s);
        REQUIRE(result.final_energy_j < result.initial_energy_j);
    }
}
