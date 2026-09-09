#include "detumble/simulation.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "detumble/initial_conditions.hpp"

namespace detumble {

Simulation::Simulation(
    SimulationConfig config,
    RigidBodyProperties body
)
    : config_{std::move(config)}, body_{std::move(body)} {
    if (!std::isfinite(config_.time_step_s) || config_.time_step_s <= 0.0) {
        throw std::invalid_argument{
            "Simulation timestep must be finite and positive"
        };
    }

    reset();
}

void Simulation::reset() {
    state_ = random_attitude_state(
        config_.seed,
        config_.minimum_angular_speed_rad_s,
        config_.maximum_angular_speed_rad_s
    );
    step_count_ = 0;
}

void Simulation::step(const Eigen::Vector3d& applied_torque_body_Nm) {
    state_ = propagate_rigid_body_rk4(
        body_,
        state_,
        applied_torque_body_Nm,
        config_.time_step_s
    );
    ++step_count_;
}

const SimulationConfig& Simulation::config() const {
    return config_;
}

const RigidBodyProperties& Simulation::body() const {
    return body_;
}

const AttitudeState& Simulation::state() const {
    return state_;
}

RotationalMetrics Simulation::metrics() const {
    return rotational_metrics(body_, state_);
}

std::uint64_t Simulation::step_count() const {
    return step_count_;
}

double Simulation::elapsed_time_s() const {
    return static_cast<double>(step_count_) * config_.time_step_s;
}

}  // namespace detumble
