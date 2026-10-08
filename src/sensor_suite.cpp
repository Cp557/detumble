#include "detumble/sensor_suite.hpp"
namespace detumble {
SensorSuiteConfig ideal_sensor_suite_config(const std::uint64_t seed) {
    SensorSuiteConfig config;
    config.magnetometer.error.seed = seed ^ 0x4d414755ULL;
    return config;
}
SensorSuiteConfig realistic_sensor_suite_config(const std::uint64_t seed) {
    auto config = ideal_sensor_suite_config(seed);
    config.magnetometer.error.noise_standard_deviation.setConstant(100e-9);
    config.magnetometer.error.bias_standard_deviation.setConstant(300e-9);
    config.magnetometer.error.quantization_step.setConstant(10e-9);
    config.magnetometer.error.saturation_limit.setConstant(100e-6);
    return config;
}
} // namespace detumble
