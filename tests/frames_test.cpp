#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/frames.hpp"

namespace {

constexpr double tolerance = 1.0e-12;
constexpr double half_pi_rad = 1.5707963267948966;

void require_vector_approx(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected
) {
    for (Eigen::Index index = 0; index < actual.size(); ++index) {
        REQUIRE(
            actual[index]
            == Catch::Approx(expected[index]).margin(tolerance)
        );
    }
}

}  // namespace

TEST_CASE("identity attitude leaves a body vector unchanged") {
    const Eigen::Vector3d vector_body{1.0, -2.0, 3.0};

    const Eigen::Vector3d vector_inertial = detumble::rotate_body_to_inertial(
        Eigen::Quaterniond::Identity(),
        vector_body
    );

    require_vector_approx(vector_inertial, vector_body);
}

TEST_CASE("positive body Z rotation maps body X toward inertial Y") {
    const Eigen::Quaterniond body_to_inertial{
        Eigen::AngleAxisd{half_pi_rad, Eigen::Vector3d::UnitZ()}
    };

    const Eigen::Vector3d vector_inertial = detumble::rotate_body_to_inertial(
        body_to_inertial,
        Eigen::Vector3d::UnitX()
    );

    require_vector_approx(vector_inertial, Eigen::Vector3d::UnitY());
}

TEST_CASE("inverse frame transforms recover the original vector") {
    const Eigen::Quaterniond body_to_inertial{
        Eigen::AngleAxisd{0.7, Eigen::Vector3d{1.0, 2.0, 3.0}.normalized()}
    };
    const Eigen::Vector3d original_body{2.0, -1.0, 0.5};

    const Eigen::Vector3d vector_inertial = detumble::rotate_body_to_inertial(
        body_to_inertial,
        original_body
    );
    const Eigen::Vector3d recovered_body = detumble::rotate_inertial_to_body(
        body_to_inertial,
        vector_inertial
    );

    require_vector_approx(recovered_body, original_body);
}

TEST_CASE("quaternion multiplication composes rotations right to left") {
    const Eigen::Quaterniond body_to_intermediate{
        Eigen::AngleAxisd{half_pi_rad, Eigen::Vector3d::UnitZ()}
    };
    const Eigen::Quaterniond intermediate_to_inertial{
        Eigen::AngleAxisd{half_pi_rad, Eigen::Vector3d::UnitX()}
    };
    const Eigen::Quaterniond body_to_inertial =
        intermediate_to_inertial * body_to_intermediate;

    const Eigen::Vector3d vector_inertial = detumble::rotate_body_to_inertial(
        body_to_inertial,
        Eigen::Vector3d::UnitX()
    );

    require_vector_approx(vector_inertial, Eigen::Vector3d::UnitZ());
}

TEST_CASE("Eigen quaternion constructor and coefficient storage orders differ") {
    const Eigen::Quaterniond quaternion{1.0, 2.0, 3.0, 4.0};

    REQUIRE(quaternion.w() == 1.0);
    REQUIRE(quaternion.x() == 2.0);
    REQUIRE(quaternion.y() == 3.0);
    REQUIRE(quaternion.z() == 4.0);
    require_vector_approx(
        quaternion.coeffs().head<3>(),
        Eigen::Vector3d{2.0, 3.0, 4.0}
    );
    REQUIRE(quaternion.coeffs()[3] == 1.0);
}
