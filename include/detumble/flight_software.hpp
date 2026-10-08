#pragma once
#include "detumble/attitude_estimator.hpp"
#include "detumble/bdot.hpp"
#include <optional>
#include <string_view>
#include <vector>
namespace detumble {
enum class FlightMode { boot, detumble, low_rate, safe };
enum class DetumbleController { bdot, estimated_rate };
[[nodiscard]] std::string_view to_string(FlightMode mode);
[[nodiscard]] std::string_view to_string(DetumbleController controller);
[[nodiscard]] DetumbleController parse_controller(std::string_view name);
struct ModeTransitionEvent {
    double time_s{};
    FlightMode old_mode{};
    FlightMode new_mode{};
};
struct DetumbleFlightSoftwareConfig {
    BdotEstimatorConfig bdot_estimator;
    BdotControllerConfig controller;
    AttitudeEstimatorConfig attitude_estimator;
    DetumbleController controller_selection{DetumbleController::bdot};
    double completion_threshold_rad_s{0.5 * 3.141592653589793 / 180.0};
    double completion_hysteresis_rad_s{0.2 * 3.141592653589793 / 180.0};
    double completion_dwell_time_s{30.0};
    double handoff_dwell_time_s{30.0};
    double measurement_timeout_s{0.3};
};
struct DetumbleFlightSoftwareState {
    FlightMode mode{FlightMode::boot};
    Eigen::Vector3d commanded_dipole_body_A_m2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_bdot_body_T_s{Eigen::Vector3d::Zero()};
    bool estimated_rate_control_active{};
    bool low_rate_confirmed{};
};
class DetumbleFlightSoftware {
  public:
    DetumbleFlightSoftware(DetumbleFlightSoftwareConfig config = {},
                           MagnetorquerConfig magnetorquer = {},
                           RigidBodyProperties body = generic_3u_cubesat(),
                           VectorSensorErrorConfig sensor = {});
    void reset();
    void predict(double time_s, const Eigen::Vector3d &reference,
                 const Eigen::Vector3d &applied_dipole, double dt);
    void update(double time_s,
                const std::optional<MagnetometerMeasurement> &measurement,
                const Eigen::Vector3d &reference_field_inertial_T);
    [[nodiscard]] const DetumbleFlightSoftwareState &state() const {
        return state_;
    }
    [[nodiscard]] const AttitudeEstimate &estimate() const {
        return attitude_estimator_.estimate();
    }
    [[nodiscard]] const std::vector<ModeTransitionEvent> &events() const {
        return events_;
    }

  private:
    void transition(FlightMode mode, double time_s);
    DetumbleFlightSoftwareConfig config_;
    MagnetorquerConfig magnetorquer_;
    Eigen::Vector3d measurement_saturation_limit_;
    BdotEstimator bdot_estimator_;
    AttitudeEstimator attitude_estimator_;
    DetumbleFlightSoftwareState state_;
    std::vector<ModeTransitionEvent> events_;
    std::optional<double> confident_since_s_, below_threshold_since_s_, last_sample_s_;
    double last_update_s_{-1.0};
};
} // namespace detumble
