#include "detumble/sensor_error.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

namespace detumble {
namespace {

void validate_nonnegative_finite(
    const Eigen::Vector3d& value,
    const char* name
) {
    if (!value.allFinite() || (value.array() < 0.0).any()) {
        throw std::invalid_argument{
            std::string{name} + " must be finite and nonnegative"
        };
    }
}

double uniform_open_unit_interval(std::mt19937_64& generator) {
    constexpr double inverse_two_to_53 = 1.0 / 9007199254740992.0;
    return (static_cast<double>(generator() >> 11U) + 0.5)
        * inverse_two_to_53;
}

}  // namespace

double seeded_gaussian_sample(
    std::mt19937_64& generator,
    const double standard_deviation
) {
    if (!std::isfinite(standard_deviation) || standard_deviation < 0.0) {
        throw std::invalid_argument{
            "Gaussian standard deviation must be finite and nonnegative"
        };
    }
    if (standard_deviation == 0.0) {
        return 0.0;
    }
    const double radius = std::sqrt(
        -2.0 * std::log(uniform_open_unit_interval(generator))
    );
    const double angle = 2.0 * std::numbers::pi
        * uniform_open_unit_interval(generator);
    return standard_deviation * radius * std::cos(angle);
}

VectorSensorErrorModel::VectorSensorErrorModel(
    VectorSensorErrorConfig config
)
    : config_{std::move(config)}, generator_{config_.seed} {
    validate_nonnegative_finite(
        config_.noise_standard_deviation,
        "Sensor noise standard deviation"
    );
    validate_nonnegative_finite(
        config_.bias_standard_deviation,
        "Sensor bias standard deviation"
    );
    validate_nonnegative_finite(
        config_.quantization_step,
        "Sensor quantization step"
    );
    if ((config_.saturation_limit.array() <= 0.0).any()
        || config_.saturation_limit.array().isNaN().any()) {
        throw std::invalid_argument{
            "Sensor saturation limits must be positive"
        };
    }
    reset();
}

void VectorSensorErrorModel::reset() {
    generator_.seed(config_.seed);
    for (Eigen::Index axis = 0; axis < bias_.size(); ++axis) {
        bias_[axis] = seeded_gaussian_sample(
            generator_,
            config_.bias_standard_deviation[axis]
        );
    }
}

Eigen::Vector3d VectorSensorErrorModel::apply(
    const Eigen::Vector3d& true_value
) {
    if (!true_value.allFinite()) {
        throw std::invalid_argument{"True sensor value must be finite"};
    }

    Eigen::Vector3d measured = true_value + bias_;
    for (Eigen::Index axis = 0; axis < measured.size(); ++axis) {
        measured[axis] += seeded_gaussian_sample(
            generator_,
            config_.noise_standard_deviation[axis]
        );
        const double step = config_.quantization_step[axis];
        if (step > 0.0) {
            measured[axis] = step * std::round(measured[axis] / step);
        }
        measured[axis] = std::clamp(
            measured[axis],
            -config_.saturation_limit[axis],
            config_.saturation_limit[axis]
        );
    }
    return measured;
}

const VectorSensorErrorConfig& VectorSensorErrorModel::config() const {
    return config_;
}

const Eigen::Vector3d& VectorSensorErrorModel::bias() const {
    return bias_;
}

}  // namespace detumble
