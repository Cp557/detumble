#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/environment.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/simulation.hpp"

TEST_CASE("magnetic control cycle turns coils off for each field sample") {
    detumble::MagneticControlCycle cycle;
    const Eigen::Vector3d field_body_T{0.0, 0.0, 25.0e-6};
    const Eigen::Vector3d command_body_A_m2{0.3, -0.1, 0.05};

    const detumble::MagneticControlOutput initial = cycle.update(
        0.0,
        field_body_T,
        command_body_A_m2
    );
    const detumble::MagneticControlOutput active = cycle.update(
        0.01,
        field_body_T,
        command_body_A_m2
    );
    const detumble::MagneticControlOutput next_sample = cycle.update(
        0.1,
        field_body_T,
        command_body_A_m2
    );

    REQUIRE(initial.magnetometer_measurement.has_value());
    REQUIRE(initial.torquers_disabled_for_measurement);
    REQUIRE(initial.limited_dipole_body_A_m2.x() == Catch::Approx(0.2));
    REQUIRE(initial.limited_dipole_body_A_m2.y() == Catch::Approx(-0.1));
    REQUIRE(initial.limited_dipole_body_A_m2.z() == Catch::Approx(0.05));
    REQUIRE(initial.applied_dipole_body_A_m2.isZero());
    REQUIRE(initial.applied_torque_body_Nm.isZero());

    REQUIRE_FALSE(active.magnetometer_measurement.has_value());
    REQUIRE_FALSE(active.torquers_disabled_for_measurement);
    REQUIRE(
        active.applied_dipole_body_A_m2
        == active.limited_dipole_body_A_m2
    );
    REQUIRE(active.applied_dipole_body_A_m2.x() == Catch::Approx(0.2));
    REQUIRE(active.applied_dipole_body_A_m2.y() == Catch::Approx(-0.1));
    REQUIRE(active.applied_dipole_body_A_m2.z() == Catch::Approx(0.05));
    REQUIRE_FALSE(active.applied_torque_body_Nm.isZero());

    REQUIRE(next_sample.magnetometer_measurement.has_value());
    REQUIRE(next_sample.torquers_disabled_for_measurement);
    REQUIRE(next_sample.applied_dipole_body_A_m2.isZero());
}

TEST_CASE("magnetic control reset restarts the sample schedule") {
    detumble::MagneticControlCycle cycle;
    static_cast<void>(cycle.update(
        0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
    ));
    static_cast<void>(cycle.update(
        0.01,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
    ));

    cycle.reset();
    const detumble::MagneticControlOutput reset_sample = cycle.update(
        0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
    );

    REQUIRE(reset_sample.magnetometer_measurement.has_value());
    REQUIRE(reset_sample.torquers_disabled_for_measurement);
}

TEST_CASE("applied magnetic torque changes rigid-body motion") {
    detumble::Simulation controlled;
    detumble::Simulation torque_free;
    detumble::MagneticControlCycle cycle;
    const detumble::EnvironmentConfig environment_config;
    const Eigen::Vector3d command_body_A_m2{0.2, 0.1, -0.05};

    for (int step = 0; step < 1'000; ++step) {
        const detumble::EnvironmentState environment =
            detumble::sample_environment(
                environment_config,
                controlled.state().body_to_inertial,
                controlled.elapsed_time_s()
            );
        const detumble::MagneticControlOutput output = cycle.update(
            controlled.elapsed_time_s(),
            environment.magnetic_field_body_T,
            command_body_A_m2
        );
        controlled.step(output.applied_torque_body_Nm);
        torque_free.step();
    }

    REQUIRE(
        (controlled.state().angular_velocity_body_rad_s
         - torque_free.state().angular_velocity_body_rad_s).norm()
        > 1.0e-5
    );
}
