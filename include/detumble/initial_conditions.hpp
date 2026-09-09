#pragma once

#include <cstdint>

#include "detumble/attitude.hpp"

namespace detumble {

[[nodiscard]] AttitudeState random_attitude_state(
    std::uint64_t seed,
    double minimum_angular_speed_rad_s,
    double maximum_angular_speed_rad_s
);

}  // namespace detumble
