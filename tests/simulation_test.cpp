#include <algorithm>
#include <array>
#include <cstddef>

#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/simulation.hpp"

namespace {

void require_same_attitude(
    const Eigen::Quaterniond& actual,
    const Eigen::Quaterniond& expected
) {
    REQUIRE(actual.angularDistance(expected) == Catch::Approx(0.0).margin(1e-14));
}

void require_same_vector(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected
) {
    for (Eigen::Index index = 0; index < actual.size(); ++index) {
        REQUIRE(actual[index] == expected[index]);
    }
}

}  // namespace

TEST_CASE("simulation reset exactly reproduces its seeded initial state") {
    detumble::Simulation simulation;
    const detumble::AttitudeState initial = simulation.state();

    simulation.step();
    simulation.step();
    simulation.reset();

    require_same_attitude(simulation.state().body_to_inertial,
                          initial.body_to_inertial);
    require_same_vector(simulation.state().angular_velocity_body_rad_s,
                        initial.angular_velocity_body_rad_s);
    REQUIRE(simulation.step_count() == 0);
    REQUIRE(simulation.elapsed_time_s() == 0.0);
}

TEST_CASE("viewer-style step batches match a continuous headless run") {
    detumble::Simulation headless;
    detumble::Simulation viewer;

    constexpr std::size_t total_steps = 1'000;
    for (std::size_t step = 0; step < total_steps; ++step) {
        headless.step();
    }

    constexpr std::array<std::size_t, 7> frame_batches{1, 2, 4, 1, 3, 2, 5};
    std::size_t completed_steps = 0;
    std::size_t frame = 0;
    while (completed_steps < total_steps) {
        const std::size_t batch = std::min(
            frame_batches[frame % frame_batches.size()],
            total_steps - completed_steps
        );
        for (std::size_t step = 0; step < batch; ++step) {
            viewer.step();
        }
        completed_steps += batch;
        ++frame;
    }

    require_same_attitude(viewer.state().body_to_inertial,
                          headless.state().body_to_inertial);
    require_same_vector(viewer.state().angular_velocity_body_rad_s,
                        headless.state().angular_velocity_body_rad_s);
    REQUIRE(viewer.elapsed_time_s() == headless.elapsed_time_s());
}
