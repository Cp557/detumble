#include "detumble/mission.hpp"
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <numbers>
TEST_CASE("magnetic-only B-dot detumbles representative scenarios") {
    constexpr std::array<std::uint64_t, 3> seeds{7, 42, 2026};
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        detumble::MissionRunConfig config;
        config.seed = seeds[i];
        config.initial_rate_rad_s =
            (5.0 + 5.0 * static_cast<double>(i)) * std::numbers::pi / 180.0;
        config.flight_software.attitude_estimator.enabled = false;
        const auto result = detumble::run_native_mission(config);
        CAPTURE(config.seed, result.final_rate_rad_s, result.detumble_time_s);
        REQUIRE(result.success);
        REQUIRE(result.detumble_time_s.has_value());
        REQUIRE(result.final_window_maximum_rate_rad_s <
                config.success.maximum_final_window_rate_rad_s);
    }
}
