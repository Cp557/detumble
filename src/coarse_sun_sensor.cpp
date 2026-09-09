#include "detumble/coarse_sun_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "detumble/sensor_error.hpp"

namespace detumble {
namespace {

constexpr double schedule_tolerance_s = 1.0e-12;

void validate_time(const double elapsed_time_s) {
    if (!std::isfinite(elapsed_time_s) || elapsed_time_s < 0.0) {
        throw std::invalid_argument{
            "Sun-sensor time must be finite and nonnegative"
        };
    }
}

}  // namespace

CoarseSunSensorArrayConfig generic_3u_coarse_sun_sensor_config() {
    return {
        .sample_rate_hz = 10.0,
        .field_of_view_half_angle_rad = 0.5 * std::numbers::pi,
        .sensors = {
            CoarseSunSensorDefinition{"CSS_PosX", Eigen::Vector3d::UnitX()},
            CoarseSunSensorDefinition{"CSS_NegX", -Eigen::Vector3d::UnitX()},
            CoarseSunSensorDefinition{"CSS_PosY", Eigen::Vector3d::UnitY()},
            CoarseSunSensorDefinition{"CSS_NegY", -Eigen::Vector3d::UnitY()},
            CoarseSunSensorDefinition{"CSS_PosZ", Eigen::Vector3d::UnitZ()},
            CoarseSunSensorDefinition{"CSS_NegZ", -Eigen::Vector3d::UnitZ()}
        },
        .error_seed = 0,
        .illumination_noise_standard_deviation = 0.0,
        .illumination_bias_standard_deviation = 0.0,
        .illumination_quantization_step = 0.0,
        .maximum_illumination = 1.0
    };
}

IdealCoarseSunSensorArray::IdealCoarseSunSensorArray(
    CoarseSunSensorArrayConfig config
)
    : config_{std::move(config)}, generator_{config_.error_seed} {
    if (!std::isfinite(config_.sample_rate_hz)
        || config_.sample_rate_hz <= 0.0
        || !std::isfinite(config_.field_of_view_half_angle_rad)
        || config_.field_of_view_half_angle_rad <= 0.0
        || config_.field_of_view_half_angle_rad > 0.5 * std::numbers::pi
        || !std::isfinite(config_.illumination_noise_standard_deviation)
        || config_.illumination_noise_standard_deviation < 0.0
        || !std::isfinite(config_.illumination_bias_standard_deviation)
        || config_.illumination_bias_standard_deviation < 0.0
        || !std::isfinite(config_.illumination_quantization_step)
        || config_.illumination_quantization_step < 0.0
        || !std::isfinite(config_.maximum_illumination)
        || config_.maximum_illumination <= 0.0) {
        throw std::invalid_argument{
            "Sun-sensor rate and field of view must be physically valid"
        };
    }
    for (const auto& sensor : config_.sensors) {
        if (sensor.name.empty()
            || !sensor.outward_normal_body.allFinite()
            || std::abs(sensor.outward_normal_body.norm() - 1.0) > 1.0e-12) {
            throw std::invalid_argument{
                "Sun-sensor definitions require names and unit normals"
            };
        }
    }
    reset();
}

void IdealCoarseSunSensorArray::reset() {
    generator_.seed(config_.error_seed);
    for (double& bias : illumination_bias_) {
        bias = seeded_gaussian_sample(
            generator_,
            config_.illumination_bias_standard_deviation
        );
    }
    next_sample_time_s_ = 0.0;
    last_update_time_s_ = 0.0;
    has_update_time_ = false;
    last_measurement_.reset();
}

bool IdealCoarseSunSensorArray::sample_due(
    const double elapsed_time_s
) const {
    validate_time(elapsed_time_s);
    return elapsed_time_s + schedule_tolerance_s >= next_sample_time_s_;
}

std::optional<CoarseSunSensorMeasurement>
IdealCoarseSunSensorArray::sample_if_due(
    const double elapsed_time_s,
    const Eigen::Vector3d& sun_direction_body,
    const bool in_eclipse
) {
    validate_time(elapsed_time_s);
    if (!sun_direction_body.allFinite() || sun_direction_body.isZero()) {
        throw std::invalid_argument{
            "Body-frame Sun direction must be finite and nonzero"
        };
    }
    if (has_update_time_
        && elapsed_time_s + schedule_tolerance_s < last_update_time_s_) {
        throw std::invalid_argument{
            "Sun-sensor updates must use nondecreasing time"
        };
    }
    last_update_time_s_ = elapsed_time_s;
    has_update_time_ = true;

    if (!sample_due(elapsed_time_s)) {
        return std::nullopt;
    }

    CoarseSunSensorMeasurement measurement{
        .sample_time_s = elapsed_time_s,
        .illumination = {},
        .sensor_visible = {},
        .sun_direction_body = std::nullopt,
        .in_eclipse = in_eclipse
    };
    if (!in_eclipse) {
        const Eigen::Vector3d sun_direction = sun_direction_body.normalized();
        const double minimum_cosine =
            std::cos(config_.field_of_view_half_angle_rad);
        Eigen::Vector3d weighted_direction = Eigen::Vector3d::Zero();
        for (std::size_t index = 0; index < config_.sensors.size(); ++index) {
            const double cosine_response = config_.sensors[index]
                .outward_normal_body.dot(sun_direction);
            const bool visible = cosine_response > 0.0
                && cosine_response + 1.0e-12 >= minimum_cosine;
            measurement.sensor_visible[index] = visible;
            double illumination = visible
                ? cosine_response
                : 0.0;
            illumination += illumination_bias_[index];
            illumination += seeded_gaussian_sample(
                generator_,
                config_.illumination_noise_standard_deviation
            );
            if (config_.illumination_quantization_step > 0.0) {
                illumination = config_.illumination_quantization_step
                    * std::round(
                        illumination
                        / config_.illumination_quantization_step
                    );
            }
            measurement.illumination[index] = std::clamp(
                illumination,
                0.0,
                config_.maximum_illumination
            );
            weighted_direction += measurement.illumination[index]
                * config_.sensors[index].outward_normal_body;
        }
        if (!weighted_direction.isZero()) {
            measurement.sun_direction_body = weighted_direction.normalized();
        }
    }

    last_measurement_ = measurement;
    const double sample_period_s = 1.0 / config_.sample_rate_hz;
    do {
        next_sample_time_s_ += sample_period_s;
    } while (next_sample_time_s_
             <= elapsed_time_s + schedule_tolerance_s);
    return last_measurement_;
}

const CoarseSunSensorArrayConfig& IdealCoarseSunSensorArray::config() const {
    return config_;
}

const std::optional<CoarseSunSensorMeasurement>&
IdealCoarseSunSensorArray::last_measurement() const {
    return last_measurement_;
}

const std::array<double, 6>&
IdealCoarseSunSensorArray::illumination_bias() const {
    return illumination_bias_;
}

}  // namespace detumble
