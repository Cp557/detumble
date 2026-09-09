#pragma once

#include <optional>

#include <Eigen/Core>

#include "detumble/sensor_error.hpp"

namespace detumble {

struct GyroscopeConfig {
    double sample_rate_hz{10.0};
    VectorSensorErrorConfig error;
};

struct GyroscopeMeasurement {
    double sample_time_s{};
    Eigen::Vector3d angular_velocity_body_rad_s{Eigen::Vector3d::Zero()};
};

[[nodiscard]] GyroscopeMeasurement ideal_gyroscope_measurement(
    double sample_time_s,
    const Eigen::Vector3d& true_angular_velocity_body_rad_s
);

class IdealGyroscope {
public:
    explicit IdealGyroscope(GyroscopeConfig config = {});

    void reset();

    [[nodiscard]] bool sample_due(double elapsed_time_s) const;
    [[nodiscard]] std::optional<GyroscopeMeasurement> sample_if_due(
        double elapsed_time_s,
        const Eigen::Vector3d& true_angular_velocity_body_rad_s
    );

    [[nodiscard]] const GyroscopeConfig& config() const;
    [[nodiscard]] const std::optional<GyroscopeMeasurement>&
    last_measurement() const;
    [[nodiscard]] const Eigen::Vector3d& bias_rad_s() const;

private:
    GyroscopeConfig config_;
    VectorSensorErrorModel error_model_;
    double next_sample_time_s_{};
    double last_update_time_s_{};
    bool has_update_time_{};
    std::optional<GyroscopeMeasurement> last_measurement_;
};

}  // namespace detumble
