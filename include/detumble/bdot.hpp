#pragma once

#include <optional>

#include <Eigen/Core>

#include "detumble/magnetometer.hpp"
#include "detumble/magnetorquer.hpp"

namespace detumble {

struct BdotEstimatorConfig {
    double filter_time_constant_s{0.2};
};

struct BdotEstimate {
    double sample_time_s{};
    Eigen::Vector3d raw_bdot_body_T_s{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_bdot_body_T_s{Eigen::Vector3d::Zero()};
};

class BdotEstimator {
public:
    explicit BdotEstimator(BdotEstimatorConfig config = {});

    void reset();

    [[nodiscard]] std::optional<BdotEstimate> update(
        const MagnetometerMeasurement& measurement
    );

    [[nodiscard]] const BdotEstimatorConfig& config() const;
    [[nodiscard]] const std::optional<BdotEstimate>& last_estimate() const;

private:
    BdotEstimatorConfig config_;
    std::optional<MagnetometerMeasurement> previous_measurement_;
    std::optional<BdotEstimate> last_estimate_;
};

struct BdotControllerConfig {
    double gain_A_m2_s_per_T{50'000.0};
};

[[nodiscard]] Eigen::Vector3d bdot_dipole_command_body_A_m2(
    const BdotControllerConfig& controller_config,
    const MagnetorquerConfig& magnetorquer_config,
    const Eigen::Vector3d& filtered_bdot_body_T_s
);

}  // namespace detumble
