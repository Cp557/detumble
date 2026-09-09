#pragma once

#include <cstdint>
#include <numbers>

#include <Eigen/Core>

#include "detumble/attitude.hpp"
#include "detumble/dynamics.hpp"

namespace detumble {

struct SimulationConfig {
    std::uint64_t seed{42};
    double time_step_s{0.01};
    double minimum_angular_speed_rad_s{5.0 * std::numbers::pi / 180.0};
    double maximum_angular_speed_rad_s{15.0 * std::numbers::pi / 180.0};
};

class Simulation {
public:
    explicit Simulation(
        SimulationConfig config = {},
        RigidBodyProperties body = generic_3u_cubesat()
    );

    void reset();
    void step(
        const Eigen::Vector3d& applied_torque_body_Nm =
            Eigen::Vector3d::Zero()
    );

    [[nodiscard]] const SimulationConfig& config() const;
    [[nodiscard]] const RigidBodyProperties& body() const;
    [[nodiscard]] const AttitudeState& state() const;
    [[nodiscard]] RotationalMetrics metrics() const;
    [[nodiscard]] std::uint64_t step_count() const;
    [[nodiscard]] double elapsed_time_s() const;

private:
    SimulationConfig config_;
    RigidBodyProperties body_;
    AttitudeState state_;
    std::uint64_t step_count_{};
};

}  // namespace detumble
