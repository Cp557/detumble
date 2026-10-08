#include "detumble/attitude_estimator.hpp"
#include "detumble/magnetorquer.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
namespace detumble {
namespace {
using Vector9 = Eigen::Matrix<double, 9, 1>;
using Matrix39 = Eigen::Matrix<double, 3, 9>;
Eigen::Matrix3d cross_matrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return matrix;
}
Eigen::Quaterniond rotation_increment(const Eigen::Vector3d &rotation) {
    const double angle = rotation.norm();
    return angle < 1e-12
               ? Eigen::Quaterniond::Identity()
               : Eigen::Quaterniond{Eigen::AngleAxisd{angle, rotation / angle}};
}
EstimatorCovariance error_transition(const RigidBodyProperties &body,
                                     const AttitudeState &state,
                                     const Eigen::Vector3d &reference,
                                     const Eigen::Vector3d &dipole, double dt) {
    const auto inertia = body.principal_moments_body_kg_m2.asDiagonal();
    const auto inverse_inertia =
        body.principal_moments_body_kg_m2.cwiseInverse().asDiagonal();
    const auto &omega = state.angular_velocity_body_rad_s;
    const Eigen::Vector3d field = state.body_to_inertial.conjugate() * reference;
    EstimatorCovariance dynamics = EstimatorCovariance::Zero();
    dynamics.block<3, 3>(0, 0) = -cross_matrix(omega);
    dynamics.block<3, 3>(0, 3).setIdentity();
    dynamics.block<3, 3>(3, 0) =
        inverse_inertia * cross_matrix(dipole) * cross_matrix(field);
    dynamics.block<3, 3>(3, 3) = inverse_inertia * (cross_matrix(inertia * omega) -
                                                    cross_matrix(omega) * inertia);
    return EstimatorCovariance::Identity() + dt * dynamics +
           0.5 * dt * dt * dynamics * dynamics;
}
AttitudeState propagate(const RigidBodyProperties &body, const AttitudeState &state,
                        const Eigen::Vector3d &reference, const Eigen::Vector3d &dipole,
                        double dt) {
    return propagate_rigid_body_rk4(
        body, state, dipole.cross(state.body_to_inertial.conjugate() * reference), dt);
}
} // namespace
std::string_view to_string(const EstimatorStatus status) {
    switch (status) {
    case EstimatorStatus::collecting:
        return "collecting magnetic history";
    case EstimatorStatus::ambiguous:
        return "orientation/rate ambiguous";
    case EstimatorStatus::tracking:
        return "tracking";
    case EstimatorStatus::rejected:
        return "measurement rejected";
    }
    return "unknown";
}
double attitude_error_angle_rad(const Eigen::Quaterniond &estimate,
                                const Eigen::Quaterniond &truth) {
    return normalized_attitude(estimate).angularDistance(normalized_attitude(truth));
}
AttitudeEstimator::AttitudeEstimator(AttitudeEstimatorConfig config,
                                     RigidBodyProperties body,
                                     VectorSensorErrorConfig sensor)
    : config_{config}, body_{std::move(body)} {
    const double values[] = {config_.initialization_window_s,
                             config_.maximum_initial_rate_rad_s,
                             config_.field_model_standard_deviation_T,
                             config_.rate_random_walk_rad_s_sqrt_s,
                             config_.bias_random_walk_T_sqrt_s,
                             config_.innovation_gate,
                             config_.confidence_rate_standard_deviation_rad_s};
    for (double value : values)
        if (!std::isfinite(value) || value < 0.0)
            throw std::invalid_argument{
                "Estimator settings must be finite and nonnegative"};
    if (config_.initialization_window_s < 2.0 || config_.innovation_gate == 0.0 ||
        config_.maximum_initial_rate_rad_s == 0.0) {
        throw std::invalid_argument{
            "Estimator window, gate and rate envelope must be positive"};
    }
    static_cast<void>(angular_acceleration_body_rad_s2(body_, Eigen::Vector3d::Zero(),
                                                       Eigen::Vector3d::Zero()));
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        measurement_variance_T2_[axis] =
            std::max(1e-18, std::pow(sensor.noise_standard_deviation[axis], 2) +
                                std::pow(sensor.quantization_step[axis], 2) / 12.0 +
                                std::pow(config_.field_model_standard_deviation_T, 2));
        bias_prior_T_[axis] = std::max(100e-9, sensor.bias_standard_deviation[axis]);
    }
}
void AttitudeEstimator::reset() {
    estimate_ = {};
    history_.clear();
    candidates_.clear();
    dipole_integral_.setZero();
    prediction_interval_s_ = 0.0;
    last_measurement_time_s_ = -1.0;
}
void AttitudeEstimator::predict(const double time_s, const Eigen::Vector3d &reference,
                                const Eigen::Vector3d &dipole, const double dt) {
    if (!config_.enabled)
        return;
    if (!std::isfinite(time_s) || time_s < 0.0 || !std::isfinite(dt) || dt < 0.0 ||
        !reference.allFinite() || !dipole.allFinite())
        throw std::invalid_argument{"Invalid estimator prediction"};
    dipole_integral_ += dt * dipole;
    prediction_interval_s_ += dt;
    for (auto &candidate : candidates_) {
        const auto transition =
            error_transition(body_, candidate.attitude, reference, dipole, dt);
        candidate.covariance =
            transition * candidate.covariance * transition.transpose();
        // Continuous angular-acceleration noise integrated through angle and rate.
        const double variance = std::pow(config_.rate_random_walk_rad_s_sqrt_s, 2);
        candidate.covariance.block<3, 3>(0, 0).diagonal().array() +=
            variance * dt * dt * dt / 3.0;
        candidate.covariance.block<3, 3>(0, 3).diagonal().array() +=
            variance * dt * dt / 2.0;
        candidate.covariance.block<3, 3>(3, 0).diagonal().array() +=
            variance * dt * dt / 2.0;
        candidate.covariance.block<3, 3>(3, 3).diagonal().array() += variance * dt;
        candidate.covariance.block<3, 3>(6, 6).diagonal().array() +=
            std::pow(config_.bias_random_walk_T_sqrt_s, 2) * dt;
        candidate.attitude =
            propagate(body_, candidate.attitude, reference, dipole, dt);
    }
    if (!candidates_.empty())
        publish(time_s + dt);
}
void AttitudeEstimator::update(const MagnetometerMeasurement &measurement,
                               const Eigen::Vector3d &reference) {
    if (!config_.enabled)
        return;
    if (!std::isfinite(measurement.sample_time_s) ||
        measurement.sample_time_s <= last_measurement_time_s_ ||
        !measurement.magnetic_field_body_T.allFinite() || !reference.allFinite() ||
        reference.norm() < 1e-6) {
        throw std::invalid_argument{
            "Estimator measurements must be valid and chronological"};
    }
    last_measurement_time_s_ = measurement.sample_time_s;
    Observation observation{
        measurement.sample_time_s, measurement.magnetic_field_body_T, reference,
        prediction_interval_s_ > 0.0
            ? Eigen::Vector3d{dipole_integral_ / prediction_interval_s_}
            : Eigen::Vector3d::Zero()};
    dipole_integral_.setZero();
    prediction_interval_s_ = 0.0;
    if (candidates_.empty()) {
        history_.push_back(observation);
        if (history_.back().time_s - history_.front().time_s >=
            config_.initialization_window_s)
            initialize();
        publish(measurement.sample_time_s);
        return;
    }
    for (auto &candidate : candidates_) {
        const Eigen::Vector3d field =
            candidate.attitude.body_to_inertial.conjugate() * reference;
        Matrix39 observation_matrix = Matrix39::Zero();
        observation_matrix.block<3, 3>(0, 0) = cross_matrix(field);
        observation_matrix.block<3, 3>(0, 6).setIdentity();
        const Eigen::Matrix3d residual_covariance =
            observation_matrix * candidate.covariance * observation_matrix.transpose() +
            measurement_variance_T2_.asDiagonal().toDenseMatrix();
        const Eigen::Vector3d residual =
            measurement.magnetic_field_body_T - field - candidate.bias;
        const Eigen::LDLT<Eigen::Matrix3d> decomposition{residual_covariance};
        candidate.innovation = residual.dot(decomposition.solve(residual));
        candidate.score =
            0.995 * candidate.score + std::min(100.0, candidate.innovation);
        if (!std::isfinite(candidate.innovation) ||
            candidate.innovation > config_.innovation_gate) {
            ++candidate.rejected_samples;
            continue;
        }
        candidate.rejected_samples = 0;
        const Eigen::Matrix<double, 9, 3> gain =
            decomposition
                .solve(
                    (candidate.covariance * observation_matrix.transpose()).transpose())
                .transpose();
        const Vector9 correction = gain * residual;
        candidate.attitude.body_to_inertial =
            normalized_attitude(candidate.attitude.body_to_inertial *
                                rotation_increment(correction.head<3>()));
        candidate.attitude.angular_velocity_body_rad_s += correction.segment<3>(3);
        candidate.bias += correction.tail<3>();
        const EstimatorCovariance remainder =
            EstimatorCovariance::Identity() - gain * observation_matrix;
        candidate.covariance =
            remainder * candidate.covariance * remainder.transpose() +
            gain * measurement_variance_T2_.asDiagonal() * gain.transpose();
        EstimatorCovariance reset = EstimatorCovariance::Identity();
        reset.block<3, 3>(0, 0) -= 0.5 * cross_matrix(correction.head<3>());
        candidate.covariance = reset * candidate.covariance * reset.transpose();
        candidate.covariance =
            (0.5 * (candidate.covariance + candidate.covariance.transpose())).eval();
    }
    std::erase_if(candidates_, [](const Candidate &candidate) {
        return candidate.rejected_samples >= 20 || !candidate.covariance.allFinite() ||
               !candidate.attitude.angular_velocity_body_rad_s.allFinite();
    });
    if (candidates_.empty()) {
        reset();
        history_.push_back(observation);
        last_measurement_time_s_ = observation.time_s;
    }
    publish(measurement.sample_time_s);
}
void AttitudeEstimator::initialize() {
    double magnitude_error_squared{};
    for (const auto &observation : history_) {
        magnitude_error_squared +=
            std::pow(observation.measured.norm() - observation.reference.norm(), 2);
    }
    const double magnitude_tolerance =
        5.0 * measurement_variance_T2_.cwiseSqrt().maxCoeff() +
        3.0 * bias_prior_T_.norm();
    if (std::sqrt(magnitude_error_squared / static_cast<double>(history_.size())) >
        magnitude_tolerance) {
        const double cutoff =
            history_.back().time_s - 0.5 * config_.initialization_window_s;
        while (history_.front().time_s < cutoff)
            history_.pop_front();
        return;
    }
    // Fit a short prefix first to avoid rate aliases, then extend to the full window.
    // Unknowns use dimensionless scales: 1 rad, 0.1 rad/s, and 1 microtesla.
    const auto perturb = [](Candidate candidate, const Vector9 &change) {
        candidate.attitude.body_to_inertial = normalized_attitude(
            candidate.attitude.body_to_inertial * rotation_increment(change.head<3>()));
        candidate.attitude.angular_velocity_body_rad_s += 0.1 * change.segment<3>(3);
        candidate.bias += 1e-6 * change.tail<3>();
        return candidate;
    };
    const auto residuals = [&](const Candidate &start, std::size_t count) {
        Eigen::VectorXd residual(static_cast<Eigen::Index>(3 * count + 3));
        auto state = start.attitude;
        for (std::size_t i = 0; i < count; ++i) {
            const auto &observation = history_[i];
            if (i > 0) {
                const double dt = observation.time_s - history_[i - 1].time_s;
                state =
                    propagate(body_, state,
                              0.5 * (observation.reference + history_[i - 1].reference),
                              observation.dipole, dt);
            }
            residual.segment<3>(static_cast<Eigen::Index>(3 * i)) =
                (state.body_to_inertial.conjugate() * observation.reference +
                 start.bias - observation.measured)
                    .cwiseQuotient(measurement_variance_T2_.cwiseSqrt());
        }
        residual.tail<3>() = start.bias.cwiseQuotient(bias_prior_T_);
        return residual;
    };
    const auto jacobian = [&](const Candidate &candidate, std::size_t count,
                              const Eigen::VectorXd &residual) {
        Eigen::MatrixXd jac(residual.size(), 9);
        for (Eigen::Index axis = 0; axis < 9; ++axis) {
            Vector9 delta = Vector9::Zero();
            delta[axis] = 1e-5;
            jac.col(axis) =
                (residuals(perturb(candidate, delta), count) - residual) / 1e-5;
        }
        return jac;
    };
    const auto &first = history_.front();
    const std::size_t derivative_end = std::min<std::size_t>(4, history_.size() - 1);
    const double derivative_time = history_[derivative_end].time_s - first.time_s;
    const Eigen::Vector3d derivative =
        (history_[derivative_end].measured - first.measured) / derivative_time;
    const Eigen::Vector3d transverse_rate =
        derivative.cross(first.measured) / first.measured.squaredNorm();
    const Eigen::Quaterniond aligned =
        Eigen::Quaterniond::FromTwoVectors(first.measured, first.reference);
    std::vector<Candidate> fitted;
    for (int roll = 0; roll < 12; ++roll) {
        for (int rate = -2; rate <= 2; ++rate) {
            Candidate candidate;
            candidate.attitude.body_to_inertial =
                Eigen::Quaterniond{Eigen::AngleAxisd{roll * std::numbers::pi / 6.0,
                                                     first.reference.normalized()}} *
                aligned;
            candidate.attitude.angular_velocity_body_rad_s =
                transverse_rate + (0.5 * rate * config_.maximum_initial_rate_rad_s) *
                                      first.measured.normalized();
            for (const double window : {2.0, 10.0, config_.initialization_window_s}) {
                const auto end = std::upper_bound(
                    history_.begin(), history_.end(), first.time_s + window,
                    [](double time, const Observation &observation) {
                        return time < observation.time_s;
                    });
                const auto count = static_cast<std::size_t>(end - history_.begin());
                double damping = 1e-3;
                for (int iteration = 0; iteration < 18; ++iteration) {
                    const Eigen::VectorXd residual = residuals(candidate, count);
                    const Eigen::MatrixXd jac = jacobian(candidate, count, residual);
                    const EstimatorCovariance normal = jac.transpose() * jac;
                    EstimatorCovariance regularized = normal;
                    regularized.diagonal().array() +=
                        damping * (normal.diagonal().array() + 1.0);
                    Vector9 step =
                        regularized.ldlt().solve(-jac.transpose() * residual);
                    // Bound optimizer excursions without imposing a truth-based rate
                    // prior.
                    const double scale = std::max({1.0, step.head<3>().norm() / 0.5,
                                                   step.segment<3>(3).norm() / 0.5,
                                                   step.tail<3>().norm() / 1.0});
                    step /= scale;
                    const auto trial = perturb(candidate, step);
                    if (residuals(trial, count).squaredNorm() <
                        residual.squaredNorm()) {
                        candidate = trial;
                        damping = std::max(1e-8, damping * 0.3);
                        if (step.norm() < 1e-5)
                            break;
                    } else
                        damping = std::min(1e8, damping * 10.0);
                }
            }
            const auto residual = residuals(candidate, history_.size());
            candidate.score = residual.squaredNorm();
            if (!std::isfinite(candidate.score))
                continue;
            const auto jac = jacobian(candidate, history_.size(), residual);
            const EstimatorCovariance information = jac.transpose() * jac;
            const Eigen::SelfAdjointEigenSolver<EstimatorCovariance> decomposition{
                information};
            Vector9 scales;
            scales << 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 1e-6, 1e-6, 1e-6;
            candidate.covariance =
                scales.asDiagonal() * decomposition.eigenvectors() *
                decomposition.eigenvalues().cwiseMax(1e-6).cwiseInverse().asDiagonal() *
                decomposition.eigenvectors().transpose() * scales.asDiagonal();
            for (std::size_t i = 1; i < history_.size(); ++i) {
                const auto &observation = history_[i];
                const Eigen::Vector3d reference =
                    0.5 * (observation.reference + history_[i - 1].reference);
                const double dt = observation.time_s - history_[i - 1].time_s;
                const auto transition = error_transition(
                    body_, candidate.attitude, reference, observation.dipole, dt);
                candidate.covariance =
                    transition * candidate.covariance * transition.transpose();
                candidate.attitude = propagate(body_, candidate.attitude, reference,
                                               observation.dipole, dt);
            }
            fitted.push_back(candidate);
        }
    }
    std::sort(fitted.begin(), fitted.end(),
              [](const Candidate &a, const Candidate &b) { return a.score < b.score; });
    const double sample_count = static_cast<double>(3 * history_.size());
    if (!fitted.empty() && fitted.front().score / sample_count < 4.0) {
        const double best = fitted.front().score;
        for (auto &candidate : fitted) {
            if (candidate.score > best + 25.0)
                break;
            const bool duplicate = std::any_of(
                candidates_.begin(), candidates_.end(), [&](const Candidate &existing) {
                    return existing.attitude.body_to_inertial.angularDistance(
                               candidate.attitude.body_to_inertial) < 1e-3 &&
                           (existing.attitude.angular_velocity_body_rad_s -
                            candidate.attitude.angular_velocity_body_rad_s)
                                   .norm() < 1e-5 &&
                           (existing.bias - candidate.bias).norm() < 1e-8;
                });
            if (!duplicate) {
                candidate.score -= best;
                candidates_.push_back(candidate);
            }
        }
        history_.clear();
    } else {
        // Slide the window by half its width and retry with fresh data.
        const double cutoff =
            history_.back().time_s - 0.5 * config_.initialization_window_s;
        while (history_.front().time_s < cutoff)
            history_.pop_front();
    }
}
void AttitudeEstimator::publish(const double time_s) {
    estimate_.sample_time_s = time_s;
    if (candidates_.empty()) {
        estimate_.valid = false;
        estimate_.confident = false;
        estimate_.status = EstimatorStatus::collecting;
        return;
    }
    const auto best = std::min_element(
        candidates_.begin(), candidates_.end(),
        [](const Candidate &a, const Candidate &b) { return a.score < b.score; });
    estimate_.attitude = best->attitude;
    estimate_.bias_body_T = best->bias;
    estimate_.covariance = best->covariance;
    estimate_.normalized_innovation = best->innovation;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covariance{
        best->covariance.block<3, 3>(3, 3)};
    double deviation = std::sqrt(std::max(0.0, covariance.eigenvalues().maxCoeff()));
    double spread{};
    for (const auto &candidate : candidates_) {
        if (candidate.score <= best->score + 25.0) {
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> alternative_covariance{
                candidate.covariance.block<3, 3>(3, 3)};
            deviation = std::max(
                deviation, std::sqrt(std::max(
                               0.0, alternative_covariance.eigenvalues().maxCoeff())));
            spread = std::max(spread, (candidate.attitude.angular_velocity_body_rad_s -
                                       best->attitude.angular_velocity_body_rad_s)
                                          .norm());
        }
    }
    estimate_.rate_standard_deviation_rad_s = deviation + spread;
    estimate_.valid =
        best->covariance.allFinite() && covariance.eigenvalues().minCoeff() >= -1e-15;
    estimate_.confident = estimate_.valid && best->rejected_samples == 0 &&
                          estimate_.rate_standard_deviation_rad_s <
                              config_.confidence_rate_standard_deviation_rad_s;
    estimate_.status = best->rejected_samples > 0 ? EstimatorStatus::rejected
                       : estimate_.confident      ? EstimatorStatus::tracking
                                                  : EstimatorStatus::ambiguous;
}
} // namespace detumble
