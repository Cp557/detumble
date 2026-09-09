#pragma once

#include <cstdint>

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/magnetometer.hpp"

namespace detumble {

struct SensorSuiteConfig {
    GyroscopeConfig gyroscope;
    MagnetometerConfig magnetometer;
    CoarseSunSensorArrayConfig sun_sensor{
        generic_3u_coarse_sun_sensor_config()
    };
};

[[nodiscard]] SensorSuiteConfig ideal_sensor_suite_config(
    std::uint64_t seed = 0
);

[[nodiscard]] SensorSuiteConfig realistic_sensor_suite_config(
    std::uint64_t seed
);

}  // namespace detumble
