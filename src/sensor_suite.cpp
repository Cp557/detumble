#include "detumble/sensor_suite.hpp"

#include <numbers>

namespace detumble {
namespace {

VectorSensorErrorConfig vector_error(
    const std::uint64_t seed,
    const double noise_standard_deviation,
    const double bias_standard_deviation,
    const double quantization_step,
    const double saturation_limit
) {
    return {
        .seed = seed,
        .noise_standard_deviation = Eigen::Vector3d::Constant(
            noise_standard_deviation
        ),
        .bias_standard_deviation = Eigen::Vector3d::Constant(
            bias_standard_deviation
        ),
        .quantization_step = Eigen::Vector3d::Constant(quantization_step),
        .saturation_limit = Eigen::Vector3d::Constant(saturation_limit)
    };
}

}  // namespace

SensorSuiteConfig ideal_sensor_suite_config(const std::uint64_t seed) {
    SensorSuiteConfig config;
    config.gyroscope.error.seed = seed ^ 0x4759524fULL;
    config.magnetometer.error.seed = seed ^ 0x4d414755ULL;
    config.sun_sensor.error_seed = seed ^ 0x53554e55ULL;
    return config;
}

SensorSuiteConfig realistic_sensor_suite_config(const std::uint64_t seed) {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    SensorSuiteConfig config = ideal_sensor_suite_config(seed);
    config.gyroscope.error = vector_error(
        seed ^ 0x4759524fULL,
        0.005 * degrees_to_radians,
        0.02 * degrees_to_radians,
        0.001 * degrees_to_radians,
        250.0 * degrees_to_radians
    );
    config.magnetometer.error = vector_error(
        seed ^ 0x4d414755ULL,
        100.0e-9,
        300.0e-9,
        10.0e-9,
        100.0e-6
    );
    config.sun_sensor.illumination_noise_standard_deviation = 0.005;
    config.sun_sensor.illumination_bias_standard_deviation = 0.01;
    config.sun_sensor.illumination_quantization_step = 1.0 / 4095.0;
    config.sun_sensor.maximum_illumination = 1.0;
    return config;
}

}  // namespace detumble
