#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/magnetorquer.hpp"

namespace {

void require_vector_approx(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    const double margin = 1.0e-15
) {
    for (Eigen::Index index = 0; index < actual.size(); ++index) {
        REQUIRE(actual[index] == Catch::Approx(expected[index]).margin(margin));
    }
}

}  // namespace

TEST_CASE("three orthogonal magnetorquer commands saturate independently") {
    const detumble::MagnetorquerConfig config{
        .maximum_dipole_body_A_m2 = {0.1, 0.2, 0.3}
    };

    const Eigen::Vector3d applied =
        detumble::saturate_magnetorquer_dipole_body_A_m2(
            config,
            {0.4, -0.4, 0.2}
        );

    require_vector_approx(applied, {0.1, -0.2, 0.2});
}

TEST_CASE("magnetic torque follows dipole cross field") {
    const Eigen::Vector3d dipole_body_A_m2{1.0, 0.0, 0.0};
    const Eigen::Vector3d field_body_T{0.0, 0.0, 20.0e-6};

    const Eigen::Vector3d torque = detumble::magnetic_torque_body_Nm(
        dipole_body_A_m2,
        field_body_T
    );

    require_vector_approx(torque, {0.0, -20.0e-6, 0.0});
    REQUIRE(torque.dot(dipole_body_A_m2) == Catch::Approx(0.0));
    REQUIRE(torque.dot(field_body_T) == Catch::Approx(0.0));
}

TEST_CASE("applied dipole never exceeds configured per-axis limits") {
    const detumble::MagnetorquerConfig config;

    for (int command = -20; command <= 20; ++command) {
        const Eigen::Vector3d applied =
            detumble::saturate_magnetorquer_dipole_body_A_m2(
                config,
                {
                    0.07 * static_cast<double>(command),
                    -0.11 * static_cast<double>(command),
                    0.13 * static_cast<double>(command)
                }
            );
        REQUIRE(
            (applied.cwiseAbs().array()
             <= config.maximum_dipole_body_A_m2.array()).all()
        );
    }
}
