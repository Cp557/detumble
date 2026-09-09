#pragma once

#include <optional>

#include <Eigen/Core>

#include "detumble/sensor_error.hpp"

namespace detumble {

struct MagnetometerConfig {
    double sample_rate_hz{10.0};
    VectorSensorErrorConfig error;
};

struct MagnetometerMeasurement {
    double sample_time_s{};
    Eigen::Vector3d magnetic_field_body_T{Eigen::Vector3d::Zero()};
};

class IdealMagnetometer {
public:
    explicit IdealMagnetometer(MagnetometerConfig config = {});

    void reset();

    [[nodiscard]] bool sample_due(double elapsed_time_s) const;

    [[nodiscard]] std::optional<MagnetometerMeasurement> sample_if_due(
        double elapsed_time_s,
        const Eigen::Vector3d& magnetic_field_body_T,
        bool torquers_enabled
    );

    [[nodiscard]] const MagnetometerConfig& config() const;
    [[nodiscard]] const std::optional<MagnetometerMeasurement>&
    last_measurement() const;
    [[nodiscard]] const Eigen::Vector3d& bias_T() const;

private:
    MagnetometerConfig config_;
    VectorSensorErrorModel error_model_;
    double next_sample_time_s_{};
    double last_update_time_s_{};
    bool has_update_time_{};
    std::optional<MagnetometerMeasurement> last_measurement_;
};

}  // namespace detumble
