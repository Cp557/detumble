#pragma once
#include "detumble/magnetometer.hpp"
#include <cstdint>
namespace detumble {
struct SensorSuiteConfig {
    MagnetometerConfig magnetometer;
};
[[nodiscard]] SensorSuiteConfig ideal_sensor_suite_config(std::uint64_t seed = 0);
[[nodiscard]] SensorSuiteConfig realistic_sensor_suite_config(std::uint64_t seed);
} // namespace detumble
