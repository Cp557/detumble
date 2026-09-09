#pragma once

#include <cstddef>
#include <optional>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace detumble {

struct AttitudeVectorObservation {
    Eigen::Vector3d measured_body{Eigen::Vector3d::Zero()};
    Eigen::Vector3d reference_inertial{Eigen::Vector3d::Zero()};
    bool valid{};
};

struct AttitudeEstimatorInput {
    double sample_time_s{};
    Eigen::Vector3d angular_velocity_body_rad_s{Eigen::Vector3d::Zero()};
    bool gyroscope_valid{true};
    std::optional<AttitudeVectorObservation> magnetic_field;
    std::optional<AttitudeVectorObservation> sun_direction;
};

enum class AttitudeCorrectionStatus {
    unavailable,
    accepted,
    invalid_measurement,
    degenerate_vectors
};

struct AttitudeEstimatorConfig {
    double correction_gain{0.25};
    double minimum_vector_norm{1.0e-12};
    double minimum_vector_separation_sine{0.05};
};

struct AttitudeEstimate {
    double sample_time_s{};
    Eigen::Quaterniond body_to_inertial{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_velocity_body_rad_s{Eigen::Vector3d::Zero()};
    bool valid{};
    AttitudeCorrectionStatus correction_status{
        AttitudeCorrectionStatus::unavailable
    };
    std::size_t accepted_correction_count{};
};

[[nodiscard]] std::optional<Eigen::Quaterniond>
triad_attitude_body_to_inertial(
    const AttitudeVectorObservation& first,
    const AttitudeVectorObservation& second,
    double minimum_vector_norm = 1.0e-12,
    double minimum_vector_separation_sine = 0.05
);

[[nodiscard]] double attitude_error_angle_rad(
    const Eigen::Quaterniond& estimated_body_to_inertial,
    const Eigen::Quaterniond& true_body_to_inertial
);

[[nodiscard]] std::string_view to_string(AttitudeCorrectionStatus status);

class AttitudeEstimator {
public:
    explicit AttitudeEstimator(AttitudeEstimatorConfig config = {});

    void reset();
    void update(const AttitudeEstimatorInput& input);

    [[nodiscard]] const AttitudeEstimatorConfig& config() const;
    [[nodiscard]] const AttitudeEstimate& estimate() const;

private:
    AttitudeEstimatorConfig config_;
    AttitudeEstimate estimate_;
    bool has_sample_time_{};
    bool has_valid_gyroscope_measurement_{};
};

}  // namespace detumble
