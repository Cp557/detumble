#pragma once
#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/sensor_suite.hpp"
#include "detumble/simulation.hpp"
#include <deque>
#include <functional>
namespace detumble {
struct MissionSuccessCriteria {
    double maximum_rate_rad_s{0.5 * 3.141592653589793 / 180.0};
    double dwell_time_s{30.0};
    double final_window_s{300.0};
    double maximum_final_window_rate_rad_s{0.7 * 3.141592653589793 / 180.0};
};
struct MissionRunConfig {
    std::uint64_t seed{42};
    double duration_s{12'000.0};
    double time_step_s{0.02};
    double initial_rate_rad_s{10.0 * 3.141592653589793 / 180.0};
    EnvironmentConfig environment;
    std::optional<EnvironmentConfig> navigation_environment;
    std::optional<SensorSuiteConfig> sensors;
    DetumbleFlightSoftwareConfig flight_software;
    MissionSuccessCriteria success;
};
enum class MissionFailureReason {
    none,
    detumble_timeout,
    insufficient_verification,
    residual_rate_exceeded
};
[[nodiscard]] std::string_view to_string(MissionFailureReason reason);
struct MissionMetrics {
    std::uint64_t seed{};
    bool success{};
    MissionFailureReason failure_reason{MissionFailureReason::detumble_timeout};
    std::optional<double> detumble_time_s, estimator_convergence_time_s,
        controller_handoff_time_s;
    double final_rate_rad_s{}, final_window_maximum_rate_rad_s{};
    double final_rate_estimation_error_rad_s{}, final_attitude_estimation_error_rad{};
    double rms_rate_estimation_error_rad_s{};
    double integrated_dipole_squared_A2_m4_s{};
    std::size_t saturation_steps{}, false_low_rate_events{},
        estimated_rate_control_steps{}, estimate_samples{};
};
struct MissionTelemetrySample {
    double time_s{};
    AttitudeState truth;
    EnvironmentState environment;
    MagneticControlOutput magnetic;
    std::optional<MagnetometerMeasurement> measurement;
    AttitudeEstimate estimate;
    DetumbleFlightSoftwareState flight;
    double rotational_energy_J{};
    bool goal_achieved{};
};
using MissionTelemetryCallback = std::function<void(const MissionTelemetrySample &)>;
class DetumbleMission {
  public:
    explicit DetumbleMission(MissionRunConfig config = {});
    void step();
    void reset();
    [[nodiscard]] const MissionRunConfig &config() const {
        return config_;
    }
    [[nodiscard]] const MissionTelemetrySample &telemetry() const {
        return telemetry_;
    }
    [[nodiscard]] MissionMetrics metrics() const;
    [[nodiscard]] const std::vector<ModeTransitionEvent> &events() const {
        return flight_software_.events();
    }

  private:
    MissionRunConfig config_;
    Simulation simulation_;
    Environment environment_, navigation_;
    MagneticControlCycle magnetic_control_;
    DetumbleFlightSoftware flight_software_;
    MissionTelemetrySample telemetry_;
    MissionMetrics metrics_;
    std::optional<double> below_threshold_since_s_;
    std::deque<std::pair<double, double>> maximum_rates_;
    double squared_rate_error_sum_{};
    FlightMode preceding_mode_{FlightMode::boot};
};
[[nodiscard]] MissionMetrics
run_native_mission(const MissionRunConfig &config,
                   const MissionTelemetryCallback &callback = {});
} // namespace detumble
