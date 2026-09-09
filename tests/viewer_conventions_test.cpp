#include <numbers>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <raylib.h>
#include <raymath.h>

TEST_CASE("raylib model rotation agrees with the attitude convention") {
    const Eigen::Quaterniond body_to_inertial{
        Eigen::AngleAxisd{
            std::numbers::pi / 2.0,
            Eigen::Vector3d::UnitZ()
        }
    };
    const Quaternion viewer_rotation{
        static_cast<float>(body_to_inertial.x()),
        static_cast<float>(body_to_inertial.y()),
        static_cast<float>(body_to_inertial.z()),
        static_cast<float>(body_to_inertial.w())
    };
    Vector3 axis{};
    float angle_rad{};
    QuaternionToAxisAngle(viewer_rotation, &axis, &angle_rad);

    const Vector3 rotated_x = Vector3Transform(
        {1.0F, 0.0F, 0.0F},
        MatrixRotate(axis, angle_rad)
    );

    REQUIRE(rotated_x.x == Catch::Approx(0.0F).margin(1.0e-6F));
    REQUIRE(rotated_x.y == Catch::Approx(1.0F).margin(1.0e-6F));
    REQUIRE(rotated_x.z == Catch::Approx(0.0F).margin(1.0e-6F));
}
