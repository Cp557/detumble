#include <cmath>
#include <numbers>
#include <stdexcept>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/orbit.hpp"

namespace {

void require_vector_approx(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    const double margin
) {
    for (Eigen::Index index = 0; index < actual.size(); ++index) {
        REQUIRE(actual[index] == Catch::Approx(expected[index]).margin(margin));
    }
}

}  // namespace

TEST_CASE("default orbit is circular at 500 kilometers") {
    const detumble::CircularOrbitConfig config;
    const detumble::OrbitState state = detumble::circular_orbit_state(
        config,
        1'234.5
    );

    REQUIRE(detumble::circular_orbit_radius_m(config) == 6'871'000.0);
    REQUIRE(
        state.position_inertial_m.norm()
        == Catch::Approx(6'871'000.0).margin(1.0e-8)
    );
    REQUIRE(
        state.position_inertial_m.dot(state.velocity_inertial_m_s)
        == Catch::Approx(0.0).margin(1.0e-5)
    );
    REQUIRE(
        detumble::circular_orbit_period_s(config)
        == Catch::Approx(5'668.144).margin(0.001)
    );
}

TEST_CASE("inclination tilts the circular orbit about inertial X") {
    detumble::CircularOrbitConfig config;
    config.inclination_rad = std::numbers::pi / 2.0;
    const double quarter_period_s =
        detumble::circular_orbit_period_s(config) / 4.0;

    const detumble::OrbitState initial =
        detumble::circular_orbit_state(config, 0.0);
    const detumble::OrbitState quarter =
        detumble::circular_orbit_state(config, quarter_period_s);

    require_vector_approx(
        initial.position_inertial_m,
        {6'871'000.0, 0.0, 0.0},
        1.0e-8
    );
    require_vector_approx(
        quarter.position_inertial_m,
        {0.0, 0.0, 6'871'000.0},
        1.0e-8
    );
}

TEST_CASE("one circular orbit returns to the initial inertial state") {
    const detumble::CircularOrbitConfig config;
    const detumble::OrbitState initial =
        detumble::circular_orbit_state(config, 0.0);
    const detumble::OrbitState final = detumble::circular_orbit_state(
        config,
        detumble::circular_orbit_period_s(config)
    );

    require_vector_approx(
        final.position_inertial_m,
        initial.position_inertial_m,
        1.0e-7
    );
    require_vector_approx(
        final.velocity_inertial_m_s,
        initial.velocity_inertial_m_s,
        1.0e-10
    );
}

TEST_CASE("circular orbit rejects nonphysical configuration") {
    detumble::CircularOrbitConfig config;
    config.altitude_m = -1.0;

    REQUIRE_THROWS_AS(
        detumble::circular_orbit_state(config, 0.0),
        std::invalid_argument
    );
}
