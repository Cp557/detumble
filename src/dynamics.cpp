#include "detumble/dynamics.hpp"

#include <cmath>
#include <stdexcept>

#include "detumble/frames.hpp"

namespace detumble {
namespace {

struct StateDerivative {
    Eigen::Quaterniond attitude_rate;
    Eigen::Vector3d angular_acceleration_body_rad_s2;
};

void validate_body(const RigidBodyProperties& body) {
    if (!std::isfinite(body.mass_kg) || body.mass_kg <= 0.0) {
        throw std::invalid_argument{"Rigid-body mass must be finite and positive"};
    }
    if (!body.dimensions_body_m.allFinite()
        || (body.dimensions_body_m.array() <= 0.0).any()) {
        throw std::invalid_argument{
            "Rigid-body dimensions must be finite and positive"
        };
    }
    if (!body.center_of_mass_body_m.allFinite()) {
        throw std::invalid_argument{"Center of mass must be finite"};
    }
    if (!body.principal_moments_body_kg_m2.allFinite()
        || (body.principal_moments_body_kg_m2.array() <= 0.0).any()) {
        throw std::invalid_argument{
            "Principal moments of inertia must be finite and positive"
        };
    }
}

Eigen::Vector3d angular_acceleration_unchecked(
    const RigidBodyProperties& body,
    const Eigen::Vector3d& angular_velocity_body_rad_s,
    const Eigen::Vector3d& applied_torque_body_Nm
) {
    const Eigen::Vector3d angular_momentum_body =
        body.principal_moments_body_kg_m2.cwiseProduct(
            angular_velocity_body_rad_s
        );
    const Eigen::Vector3d gyroscopic_torque =
        angular_velocity_body_rad_s.cross(angular_momentum_body);

    return (applied_torque_body_Nm - gyroscopic_torque).cwiseQuotient(
        body.principal_moments_body_kg_m2
    );
}

StateDerivative state_derivative(
    const RigidBodyProperties& body,
    const AttitudeState& state,
    const Eigen::Vector3d& applied_torque_body_Nm
) {
    const Eigen::Quaterniond angular_velocity_quaternion{
        0.0,
        state.angular_velocity_body_rad_s.x(),
        state.angular_velocity_body_rad_s.y(),
        state.angular_velocity_body_rad_s.z()
    };
    Eigen::Quaterniond attitude_rate =
        state.body_to_inertial * angular_velocity_quaternion;
    attitude_rate.coeffs() *= 0.5;

    return {
        .attitude_rate = attitude_rate,
        .angular_acceleration_body_rad_s2 = angular_acceleration_unchecked(
            body,
            state.angular_velocity_body_rad_s,
            applied_torque_body_Nm
        )
    };
}

AttitudeState add_scaled_derivative(
    const AttitudeState& state,
    const StateDerivative& derivative,
    const double scale_s
) {
    AttitudeState result;
    result.body_to_inertial.coeffs() = state.body_to_inertial.coeffs()
        + scale_s * derivative.attitude_rate.coeffs();
    result.angular_velocity_body_rad_s = state.angular_velocity_body_rad_s
        + scale_s * derivative.angular_acceleration_body_rad_s2;
    return result;
}

}  // namespace

RigidBodyProperties generic_3u_cubesat() {
    constexpr double mass_kg = 4.0;
    const Eigen::Vector3d dimensions_body_m{0.10, 0.10, 0.34};

    const double x_squared = dimensions_body_m.x() * dimensions_body_m.x();
    const double y_squared = dimensions_body_m.y() * dimensions_body_m.y();
    const double z_squared = dimensions_body_m.z() * dimensions_body_m.z();
    constexpr double cuboid_inertia_factor = mass_kg / 12.0;

    return {
        .mass_kg = mass_kg,
        .dimensions_body_m = dimensions_body_m,
        .center_of_mass_body_m = Eigen::Vector3d::Zero(),
        .principal_moments_body_kg_m2 = {
            cuboid_inertia_factor * (y_squared + z_squared),
            cuboid_inertia_factor * (x_squared + z_squared),
            cuboid_inertia_factor * (x_squared + y_squared)
        }
    };
}

Eigen::Vector3d angular_acceleration_body_rad_s2(
    const RigidBodyProperties& body,
    const Eigen::Vector3d& angular_velocity_body_rad_s,
    const Eigen::Vector3d& applied_torque_body_Nm
) {
    validate_body(body);
    if (!angular_velocity_body_rad_s.allFinite()) {
        throw std::invalid_argument{"Angular velocity must be finite"};
    }
    if (!applied_torque_body_Nm.allFinite()) {
        throw std::invalid_argument{"Applied torque must be finite"};
    }

    return angular_acceleration_unchecked(
        body,
        angular_velocity_body_rad_s,
        applied_torque_body_Nm
    );
}

AttitudeState propagate_rigid_body_rk4(
    const RigidBodyProperties& body,
    const AttitudeState& state,
    const Eigen::Vector3d& applied_torque_body_Nm,
    const double time_step_s
) {
    validate_body(body);
    if (!state.angular_velocity_body_rad_s.allFinite()) {
        throw std::invalid_argument{"Angular velocity must be finite"};
    }
    if (!applied_torque_body_Nm.allFinite()) {
        throw std::invalid_argument{"Applied torque must be finite"};
    }
    if (!std::isfinite(time_step_s) || time_step_s < 0.0) {
        throw std::invalid_argument{
            "Rigid-body timestep must be finite and nonnegative"
        };
    }

    const AttitudeState initial{
        .body_to_inertial = normalized_attitude(state.body_to_inertial),
        .angular_velocity_body_rad_s = state.angular_velocity_body_rad_s
    };
    if (time_step_s == 0.0) {
        return initial;
    }

    const StateDerivative k1 = state_derivative(
        body,
        initial,
        applied_torque_body_Nm
    );
    const StateDerivative k2 = state_derivative(
        body,
        add_scaled_derivative(initial, k1, time_step_s * 0.5),
        applied_torque_body_Nm
    );
    const StateDerivative k3 = state_derivative(
        body,
        add_scaled_derivative(initial, k2, time_step_s * 0.5),
        applied_torque_body_Nm
    );
    const StateDerivative k4 = state_derivative(
        body,
        add_scaled_derivative(initial, k3, time_step_s),
        applied_torque_body_Nm
    );

    AttitudeState result;
    result.body_to_inertial.coeffs() = initial.body_to_inertial.coeffs()
        + (time_step_s / 6.0)
            * (k1.attitude_rate.coeffs()
               + 2.0 * k2.attitude_rate.coeffs()
               + 2.0 * k3.attitude_rate.coeffs()
               + k4.attitude_rate.coeffs());
    result.body_to_inertial = normalized_attitude(result.body_to_inertial);
    result.angular_velocity_body_rad_s =
        initial.angular_velocity_body_rad_s
        + (time_step_s / 6.0)
            * (k1.angular_acceleration_body_rad_s2
               + 2.0 * k2.angular_acceleration_body_rad_s2
               + 2.0 * k3.angular_acceleration_body_rad_s2
               + k4.angular_acceleration_body_rad_s2);

    return result;
}

RotationalMetrics rotational_metrics(
    const RigidBodyProperties& body,
    const AttitudeState& state
) {
    validate_body(body);
    if (!state.angular_velocity_body_rad_s.allFinite()) {
        throw std::invalid_argument{"Angular velocity must be finite"};
    }

    const Eigen::Vector3d angular_momentum_body =
        body.principal_moments_body_kg_m2.cwiseProduct(
            state.angular_velocity_body_rad_s
        );

    return {
        .angular_momentum_body_kg_m2_s = angular_momentum_body,
        .angular_momentum_inertial_kg_m2_s = rotate_body_to_inertial(
            state.body_to_inertial,
            angular_momentum_body
        ),
        .rotational_kinetic_energy_j = 0.5
            * state.angular_velocity_body_rad_s.dot(angular_momentum_body),
        .quaternion_norm = state.body_to_inertial.norm()
    };
}

}  // namespace detumble
