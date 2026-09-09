#include "detumble/attitude_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "detumble/attitude.hpp"

namespace detumble {
namespace {

bool finite_vector(const Eigen::Vector3d& vector) {
    return vector.array().isFinite().all();
}

bool valid_observation(
    const AttitudeVectorObservation& observation,
    const double minimum_vector_norm
) {
    return observation.valid
        && finite_vector(observation.measured_body)
        && finite_vector(observation.reference_inertial)
        && observation.measured_body.norm() >= minimum_vector_norm
        && observation.reference_inertial.norm() >= minimum_vector_norm;
}

double separation_sine(
    const Eigen::Vector3d& first,
    const Eigen::Vector3d& second
) {
    return first.normalized().cross(second.normalized()).norm();
}

Eigen::Matrix3d triad_basis(
    const Eigen::Vector3d& first,
    const Eigen::Vector3d& second
) {
    const Eigen::Vector3d primary = first.normalized();
    const Eigen::Vector3d normal = primary.cross(second).normalized();
    const Eigen::Vector3d transverse = normal.cross(primary);

    Eigen::Matrix3d basis;
    basis.col(0) = primary;
    basis.col(1) = transverse;
    basis.col(2) = normal;
    return basis;
}

AttitudeCorrectionStatus correction_status(
    const std::optional<AttitudeVectorObservation>& first,
    const std::optional<AttitudeVectorObservation>& second,
    const AttitudeEstimatorConfig& config
) {
    if (!first.has_value() || !second.has_value()
        || !first->valid || !second->valid) {
        return AttitudeCorrectionStatus::unavailable;
    }
    if (!valid_observation(*first, config.minimum_vector_norm)
        || !valid_observation(*second, config.minimum_vector_norm)) {
        return AttitudeCorrectionStatus::invalid_measurement;
    }
    if (separation_sine(first->measured_body, second->measured_body)
            < config.minimum_vector_separation_sine
        || separation_sine(
               first->reference_inertial,
               second->reference_inertial
           ) < config.minimum_vector_separation_sine) {
        return AttitudeCorrectionStatus::degenerate_vectors;
    }
    return AttitudeCorrectionStatus::accepted;
}

}  // namespace

std::optional<Eigen::Quaterniond> triad_attitude_body_to_inertial(
    const AttitudeVectorObservation& first,
    const AttitudeVectorObservation& second,
    const double minimum_vector_norm,
    const double minimum_vector_separation_sine
) {
    if (!std::isfinite(minimum_vector_norm) || minimum_vector_norm <= 0.0
        || !std::isfinite(minimum_vector_separation_sine)
        || minimum_vector_separation_sine < 0.0
        || minimum_vector_separation_sine >= 1.0
        || !valid_observation(first, minimum_vector_norm)
        || !valid_observation(second, minimum_vector_norm)
        || separation_sine(first.measured_body, second.measured_body)
            < minimum_vector_separation_sine
        || separation_sine(
               first.reference_inertial,
               second.reference_inertial
           ) < minimum_vector_separation_sine) {
        return std::nullopt;
    }

    const Eigen::Matrix3d body_basis =
        triad_basis(first.measured_body, second.measured_body);
    const Eigen::Matrix3d inertial_basis =
        triad_basis(first.reference_inertial, second.reference_inertial);
    return normalized_attitude(
        Eigen::Quaterniond{inertial_basis * body_basis.transpose()}
    );
}

double attitude_error_angle_rad(
    const Eigen::Quaterniond& estimated_body_to_inertial,
    const Eigen::Quaterniond& true_body_to_inertial
) {
    const Eigen::Quaterniond estimated =
        normalized_attitude(estimated_body_to_inertial);
    const Eigen::Quaterniond truth =
        normalized_attitude(true_body_to_inertial);
    const Eigen::Quaterniond difference = estimated.conjugate() * truth;
    const double scalar = std::clamp(std::abs(difference.w()), 0.0, 1.0);
    return 2.0 * std::acos(scalar);
}

std::string_view to_string(const AttitudeCorrectionStatus status) {
    switch (status) {
        case AttitudeCorrectionStatus::unavailable:
            return "NO CORRECTION";
        case AttitudeCorrectionStatus::accepted:
            return "TRIAD ACCEPTED";
        case AttitudeCorrectionStatus::invalid_measurement:
            return "INVALID MEASUREMENT";
        case AttitudeCorrectionStatus::degenerate_vectors:
            return "VECTORS NEARLY PARALLEL";
    }
    return "UNKNOWN";
}

AttitudeEstimator::AttitudeEstimator(AttitudeEstimatorConfig config)
    : config_{config} {
    if (!std::isfinite(config_.correction_gain)
        || config_.correction_gain < 0.0
        || config_.correction_gain > 1.0) {
        throw std::invalid_argument{
            "Estimator correction gain must be between zero and one"
        };
    }
    if (!std::isfinite(config_.minimum_vector_norm)
        || config_.minimum_vector_norm <= 0.0) {
        throw std::invalid_argument{
            "Estimator minimum vector norm must be finite and positive"
        };
    }
    if (!std::isfinite(config_.minimum_vector_separation_sine)
        || config_.minimum_vector_separation_sine < 0.0
        || config_.minimum_vector_separation_sine >= 1.0) {
        throw std::invalid_argument{
            "Estimator vector-separation sine must be in [0, 1)"
        };
    }
}

void AttitudeEstimator::reset() {
    estimate_ = {};
    has_sample_time_ = false;
    has_valid_gyroscope_measurement_ = false;
}

void AttitudeEstimator::update(const AttitudeEstimatorInput& input) {
    if (!std::isfinite(input.sample_time_s) || input.sample_time_s < 0.0) {
        throw std::invalid_argument{
            "Estimator sample time must be finite and nonnegative"
        };
    }
    if (has_sample_time_
        && input.sample_time_s < estimate_.sample_time_s) {
        throw std::invalid_argument{
            "Estimator inputs must use nondecreasing timestamps"
        };
    }

    if (input.gyroscope_valid) {
        if (!finite_vector(input.angular_velocity_body_rad_s)) {
            throw std::invalid_argument{
                "Valid gyroscope measurements must be finite"
            };
        }
    }

    const double time_step_s = has_sample_time_
        ? input.sample_time_s - estimate_.sample_time_s
        : 0.0;
    if (has_valid_gyroscope_measurement_) {
        estimate_.body_to_inertial = integrate_attitude(
            estimate_.body_to_inertial,
            estimate_.angular_velocity_body_rad_s,
            time_step_s
        );
    }
    if (input.gyroscope_valid) {
        estimate_.angular_velocity_body_rad_s =
            input.angular_velocity_body_rad_s;
    }
    has_valid_gyroscope_measurement_ = input.gyroscope_valid;

    estimate_.sample_time_s = input.sample_time_s;
    has_sample_time_ = true;
    estimate_.correction_status = correction_status(
        input.magnetic_field,
        input.sun_direction,
        config_
    );

    if (estimate_.correction_status == AttitudeCorrectionStatus::accepted) {
        const std::optional<Eigen::Quaterniond> triad =
            triad_attitude_body_to_inertial(
                *input.magnetic_field,
                *input.sun_direction,
                config_.minimum_vector_norm,
                config_.minimum_vector_separation_sine
            );
        if (!triad.has_value()) {
            throw std::logic_error{
                "Accepted TRIAD observations did not produce an attitude"
            };
        }
        estimate_.body_to_inertial = normalized_attitude(
            estimate_.body_to_inertial.slerp(
                config_.correction_gain,
                *triad
            )
        );
        estimate_.valid = true;
        ++estimate_.accepted_correction_count;
    }
}

const AttitudeEstimatorConfig& AttitudeEstimator::config() const {
    return config_;
}

const AttitudeEstimate& AttitudeEstimator::estimate() const {
    return estimate_;
}

}  // namespace detumble
