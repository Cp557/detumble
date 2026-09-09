#pragma once

#include <cstdint>
#include <functional>
#include <numbers>
#include <optional>
#include <string_view>

#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include "detumble/sensor_suite.hpp"

namespace detumble {

struct MissionSuccessCriteria {
    double maximum_final_rate_rad_s{0.1 * std::numbers::pi / 180.0};
    double maximum_rms_pointing_error_rad{
        2.0 * std::numbers::pi / 180.0
    };
    bool require_steady_pointing{true};
};

struct MissionRunConfig {
    std::uint64_t seed{42};
    double duration_s{6'000.0};
    double time_step_s{0.02};
    double initial_rate_rad_s{10.0 * std::numbers::pi / 180.0};
    EnvironmentConfig environment;
    std::optional<SensorSuiteConfig> sensors;
    AutonomousFlightSoftwareConfig flight_software;
    MissionSuccessCriteria success;
};

enum class MissionFailureReason {
    none,
    detumble_timeout,
    sun_acquisition_timeout,
    pointing_not_steady,
    final_rate_exceeded,
    rms_pointing_error_exceeded
};

[[nodiscard]] std::string_view to_string(MissionFailureReason reason);

struct MissionMetrics {
    std::uint64_t seed{};
    bool success{};
    MissionFailureReason failure_reason{MissionFailureReason::none};
    FlightMode final_mode{FlightMode::boot};
    std::optional<double> detumble_time_s;
    std::optional<double> acquisition_time_s;
    std::optional<double> steady_pointing_time_s;
    double final_rate_rad_s{};
    double final_true_pointing_error_rad{};
    double final_estimated_pointing_error_rad{};
    double rms_true_pointing_error_rad{};
    double maximum_true_pointing_error_rad{};
    double final_attitude_estimation_error_rad{};
    double maximum_wheel_speed_rad_s{};
    std::size_t wheel_saturation_steps{};
    std::size_t pointing_sample_count{};
};

struct MissionTelemetrySample {
    double time_s{};
    FlightMode mode{FlightMode::boot};
    double body_rate_rad_s{};
    double true_pointing_error_rad{};
    double estimated_pointing_error_rad{};
    double attitude_estimation_error_rad{};
    bool in_eclipse{};
    bool detumble_complete{};
    bool target_acquired{};
    bool steady_pointing{};
};

using MissionTelemetryCallback =
    std::function<void(const MissionTelemetrySample&)>;

[[nodiscard]] MissionMetrics run_native_mission(
    const MissionRunConfig& config,
    const MissionTelemetryCallback& telemetry_callback = {}
);

}  // namespace detumble
