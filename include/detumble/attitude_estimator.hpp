#pragma once
#include "detumble/attitude.hpp"
#include "detumble/dynamics.hpp"
#include "detumble/magnetometer.hpp"
#include <deque>
#include <limits>
#include <string_view>
#include <vector>
namespace detumble {
using EstimatorCovariance = Eigen::Matrix<double, 9, 9>;
enum class EstimatorStatus { collecting, ambiguous, tracking, rejected };
[[nodiscard]] std::string_view to_string(EstimatorStatus status);
struct AttitudeEstimatorConfig {
    bool enabled{true};
    double initialization_window_s{60.0};
    double maximum_initial_rate_rad_s{20.0 * 3.141592653589793 / 180.0};
    double field_model_standard_deviation_T{100e-9};
    double rate_random_walk_rad_s_sqrt_s{1e-5}; // Rate diffusion: rad/s per sqrt(s).
    double bias_random_walk_T_sqrt_s{1e-10};
    double innovation_gate{16.3}; // 99.9% chi-square threshold, 3 measurements.
    double confidence_rate_standard_deviation_rad_s{0.05 * 3.141592653589793 / 180.0};
};
struct AttitudeEstimate {
    double sample_time_s{};
    AttitudeState attitude;
    Eigen::Vector3d bias_body_T{Eigen::Vector3d::Zero()};
    EstimatorCovariance covariance{EstimatorCovariance::Identity()};
    double rate_standard_deviation_rad_s{std::numeric_limits<double>::infinity()};
    double normalized_innovation{};
    EstimatorStatus status{EstimatorStatus::collecting};
    bool valid{};
    bool confident{};
};
// No truth-state input: prediction uses navigation references and known dipoles.
class AttitudeEstimator {
  public:
    AttitudeEstimator(AttitudeEstimatorConfig config = {},
                      RigidBodyProperties body = generic_3u_cubesat(),
                      VectorSensorErrorConfig sensor = {});
    void reset();
    void predict(double time_s, const Eigen::Vector3d &reference_field_inertial_T,
                 const Eigen::Vector3d &applied_dipole_body_A_m2, double time_step_s);
    void update(const MagnetometerMeasurement &measurement,
                const Eigen::Vector3d &reference_field_inertial_T);
    [[nodiscard]] const AttitudeEstimate &estimate() const {
        return estimate_;
    }

  private:
    struct Observation {
        double time_s{};
        Eigen::Vector3d measured{Eigen::Vector3d::Zero()};
        Eigen::Vector3d reference{Eigen::Vector3d::Zero()};
        Eigen::Vector3d dipole{
            Eigen::Vector3d::Zero()}; // Average since preceding observation.
    };
    struct Candidate {
        AttitudeState attitude;
        Eigen::Vector3d bias{Eigen::Vector3d::Zero()};
        EstimatorCovariance covariance{EstimatorCovariance::Identity()};
        double score{};
        double innovation{};
        unsigned rejected_samples{};
    };
    void initialize();
    void publish(double time_s);
    AttitudeEstimatorConfig config_;
    RigidBodyProperties body_;
    Eigen::Vector3d measurement_variance_T2_;
    Eigen::Vector3d bias_prior_T_;
    AttitudeEstimate estimate_;
    std::deque<Observation> history_;
    std::vector<Candidate> candidates_;
    Eigen::Vector3d dipole_integral_{Eigen::Vector3d::Zero()};
    double prediction_interval_s_{};
    double last_measurement_time_s_{-1.0};
};
[[nodiscard]] double attitude_error_angle_rad(const Eigen::Quaterniond &estimate,
                                              const Eigen::Quaterniond &truth);
} // namespace detumble
