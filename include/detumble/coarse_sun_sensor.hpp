#pragma once

#include <array>
#include <cstdint>
#include <numbers>
#include <optional>
#include <random>
#include <string_view>

#include <Eigen/Core>

namespace detumble {

struct CoarseSunSensorDefinition {
    std::string_view name;
    Eigen::Vector3d outward_normal_body{Eigen::Vector3d::UnitX()};
};

struct CoarseSunSensorArrayConfig {
    double sample_rate_hz{10.0};
    double field_of_view_half_angle_rad{0.5 * std::numbers::pi};
    std::array<CoarseSunSensorDefinition, 6> sensors;
    std::uint64_t error_seed{};
    double illumination_noise_standard_deviation{};
    double illumination_bias_standard_deviation{};
    double illumination_quantization_step{};
    double maximum_illumination{1.0};
};

struct CoarseSunSensorMeasurement {
    double sample_time_s{};
    std::array<double, 6> illumination{};
    std::array<bool, 6> sensor_visible{};
    std::optional<Eigen::Vector3d> sun_direction_body;
    bool in_eclipse{};
};

[[nodiscard]] CoarseSunSensorArrayConfig
generic_3u_coarse_sun_sensor_config();

class IdealCoarseSunSensorArray {
public:
    explicit IdealCoarseSunSensorArray(
        CoarseSunSensorArrayConfig config =
            generic_3u_coarse_sun_sensor_config()
    );

    void reset();

    [[nodiscard]] bool sample_due(double elapsed_time_s) const;
    [[nodiscard]] std::optional<CoarseSunSensorMeasurement> sample_if_due(
        double elapsed_time_s,
        const Eigen::Vector3d& sun_direction_body,
        bool in_eclipse
    );

    [[nodiscard]] const CoarseSunSensorArrayConfig& config() const;
    [[nodiscard]] const std::optional<CoarseSunSensorMeasurement>&
    last_measurement() const;
    [[nodiscard]] const std::array<double, 6>& illumination_bias() const;

private:
    CoarseSunSensorArrayConfig config_;
    std::mt19937_64 generator_;
    std::array<double, 6> illumination_bias_{};
    double next_sample_time_s_{};
    double last_update_time_s_{};
    bool has_update_time_{};
    std::optional<CoarseSunSensorMeasurement> last_measurement_;
};

}  // namespace detumble
