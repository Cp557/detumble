#pragma once
#include "options.hpp"
#include <string_view>
namespace detumble_app {
inline void write_vector(std::ostream &output, const Eigen::Vector3d &vector) {
    output << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
}
inline void write_telemetry_header(std::ostream &output) {
    output
        << "seed,controller,field_model,reference_field_model,epoch,duration_s,"
        << "time_step_s,initial_rate_deg_s,gain_A_m2_s_per_T,bdot_filter_s,"
        << "estimator_enabled,sensor_profile,time_s,"
        << "q_w,q_x,q_y,q_z,omega_x_body_rad_s,omega_y_body_rad_s,omega_z_body_rad_s,"
        << "rotational_energy_J,B_x_body_T,B_y_body_T,B_z_body_T,"
        << "mag_sample_time_s,measured_B_x_body_T,measured_B_y_body_T,measured_B_z_"
           "body_T,"
        << "command_x_A_m2,command_y_A_m2,command_z_A_m2,applied_x_A_m2,applied_y_A_m2,"
           "applied_z_A_m2,"
        << "torque_x_Nm,torque_y_Nm,torque_z_Nm,coils_blanked,flight_mode,goal_"
           "achieved,"
        << "estimator_valid,estimator_confident,estimated_rate_active,estimator_time_s,"
        << "estimated_q_w,estimated_q_x,estimated_q_y,estimated_q_z,"
        << "estimated_omega_x_rad_s,estimated_omega_y_rad_s,estimated_omega_z_rad_s,"
        << "estimated_bias_x_T,estimated_bias_y_T,estimated_bias_z_T,"
        << "rate_standard_deviation_rad_s,normalized_innovation,estimator_status\n";
}
inline void write_telemetry(std::ostream &output,
                            const detumble::MissionRunConfig &config,
                            const detumble::MissionTelemetrySample &sample,
                            const std::string_view sensor_profile) {
    const auto &q = sample.truth.body_to_inertial;
    const auto &estimate = sample.estimate;
    output << config.seed << ','
           << detumble::to_string(config.flight_software.controller_selection) << ','
           << detumble::to_string(config.environment.magnetic_field.model) << ','
           << detumble::to_string(
                  config.navigation_environment.value_or(config.environment)
                      .magnetic_field.model)
           << ',' << config.environment.magnetic_field.epoch << ',' << config.duration_s
           << ',' << config.time_step_s << ','
           << config.initial_rate_rad_s * radians_to_degrees << ','
           << config.flight_software.controller.gain_A_m2_s_per_T << ','
           << config.flight_software.bdot_estimator.filter_time_constant_s << ','
           << config.flight_software.attitude_estimator.enabled << ',' << sensor_profile
           << ',' << sample.time_s << ',' << q.w() << ',' << q.x() << ',' << q.y()
           << ',' << q.z();
    write_vector(output, sample.truth.angular_velocity_body_rad_s);
    output << ',' << sample.rotational_energy_J;
    write_vector(output, sample.environment.magnetic_field_body_T);
    output << ',' << (sample.measurement ? sample.measurement->sample_time_s : -1.0);
    write_vector(output, sample.measurement
                             ? sample.measurement->magnetic_field_body_T
                             : Eigen::Vector3d::Constant(
                                   std::numeric_limits<double>::quiet_NaN()));
    write_vector(output, sample.magnetic.commanded_dipole_body_A_m2);
    write_vector(output, sample.magnetic.applied_dipole_body_A_m2);
    write_vector(output, sample.magnetic.applied_torque_body_Nm);
    output << ',' << sample.magnetic.torquers_disabled_for_measurement << ','
           << detumble::to_string(sample.flight.mode) << ',' << sample.goal_achieved
           << ',' << estimate.valid << ',' << estimate.confident << ','
           << sample.flight.estimated_rate_control_active << ','
           << estimate.sample_time_s;
    const auto &eq = estimate.attitude.body_to_inertial;
    output << ',' << eq.w() << ',' << eq.x() << ',' << eq.y() << ',' << eq.z();
    write_vector(output, estimate.attitude.angular_velocity_body_rad_s);
    write_vector(output, estimate.bias_body_T);
    output << ',' << estimate.rate_standard_deviation_rad_s << ','
           << estimate.normalized_innovation << ','
           << detumble::to_string(estimate.status) << '\n';
}
} // namespace detumble_app
