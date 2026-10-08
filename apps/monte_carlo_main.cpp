#include "options.hpp"
#include "telemetry.hpp"
#include <algorithm>
#include <random>
#include <vector>
namespace {
double unit(std::mt19937_64 &generator) {
    return static_cast<double>(generator() >> 11U) / 9007199254740992.0;
}
void header(std::ostream &output) {
    output << "run,seed,controller,field_model,reference_field_model,epoch,sensor_"
              "profile,duration_s,time_step_s,"
           << "initial_rate_deg_s,initial_orbit_phase_deg,success,failure_reason,"
              "detumble_time_s,"
           << "final_rate_deg_s,final_window_maximum_deg_s,estimator_convergence_s,"
              "handoff_s,"
           << "rate_estimation_error_deg_s,attitude_estimation_error_deg,rms_rate_"
              "estimation_error_deg_s,"
           << "dipole_effort_A2_m4_s,saturation_steps,false_low_rate_events,estimated_"
              "rate_control_steps,master_seed,gain_A_m2_s_per_T,bdot_filter_s,"
              "estimator_enabled,goal_rate_deg_s,goal_dwell_s,final_window_s,"
              "final_window_limit_deg_s\n";
}
void result(std::ostream &output, std::size_t index,
            const detumble_app::Options &options,
            const detumble::MissionRunConfig &config,
            const detumble::MissionMetrics &metrics) {
    const double deg = detumble_app::radians_to_degrees;
    output << index << ',' << config.seed << ','
           << detumble::to_string(config.flight_software.controller_selection) << ','
           << detumble::to_string(config.environment.magnetic_field.model) << ','
           << detumble::to_string(
                  config.navigation_environment.value_or(config.environment)
                      .magnetic_field.model)
           << ',' << config.environment.magnetic_field.epoch << ','
           << options.sensor_profile << ',' << config.duration_s << ','
           << config.time_step_s << ',' << config.initial_rate_rad_s * deg << ','
           << config.environment.orbit.initial_argument_of_latitude_rad * deg << ','
           << metrics.success << ',' << detumble::to_string(metrics.failure_reason)
           << ',' << metrics.detumble_time_s.value_or(-1.0) << ','
           << metrics.final_rate_rad_s * deg << ','
           << metrics.final_window_maximum_rate_rad_s * deg << ','
           << metrics.estimator_convergence_time_s.value_or(-1.0) << ','
           << metrics.controller_handoff_time_s.value_or(-1.0) << ','
           << metrics.final_rate_estimation_error_rad_s * deg << ','
           << metrics.final_attitude_estimation_error_rad * deg << ','
           << metrics.rms_rate_estimation_error_rad_s * deg << ','
           << metrics.integrated_dipole_squared_A2_m4_s << ','
           << metrics.saturation_steps << ',' << metrics.false_low_rate_events << ','
           << metrics.estimated_rate_control_steps << ',' << options.master_seed << ','
           << config.flight_software.controller.gain_A_m2_s_per_T << ','
           << config.flight_software.bdot_estimator.filter_time_constant_s << ','
           << config.flight_software.attitude_estimator.enabled << ','
           << config.success.maximum_rate_rad_s * deg << ','
           << config.success.dwell_time_s << ',' << config.success.final_window_s << ','
           << config.success.maximum_final_window_rate_rad_s * deg << '\n';
}
} // namespace
int main(int argc, char **argv) {
    try {
        const auto options = detumble_app::parse_options(argc, argv, true);
        if (options.help) {
            detumble_app::print_usage(true);
            return 0;
        }
        auto output = detumble_app::open_output(options.results);
        header(output);
        auto aggregate = detumble_app::open_output(options.aggregate);
        aggregate
            << "controller,runs,successes,mean_detumble_time_s,worst_final_rate_deg_s,"
            << "mean_dipole_effort_A2_m4_s,converged_estimators,handoffs,false_low_"
               "rate_events\n";
        std::mt19937_64 generator{options.master_seed};
        // Each scenario consumes exactly three draws, independent of controller count.
        for (std::size_t skipped = 0; skipped < options.start_run; ++skipped) {
            static_cast<void>(generator());
            static_cast<void>(generator());
            static_cast<void>(generator());
        }
        std::vector<std::pair<detumble::DetumbleController, detumble::MissionMetrics>>
            results;
        bool all_pass = true;
        for (std::size_t i = 0; i < options.runs; ++i) {
            auto config = options.mission;
            config.seed = generator();
            config.initial_rate_rad_s =
                (5.0 + 10.0 * unit(generator)) / detumble_app::radians_to_degrees;
            config.environment.orbit.initial_argument_of_latitude_rad =
                2.0 * std::numbers::pi * unit(generator);
            if (config.navigation_environment)
                config.navigation_environment->orbit = config.environment.orbit;
            detumble_app::configure_sensors(config, options.sensor_profile);
            const std::vector<detumble::DetumbleController> controllers =
                options.compare
                    ? std::vector{detumble::DetumbleController::bdot,
                                  detumble::DetumbleController::estimated_rate}
                    : std::vector{config.flight_software.controller_selection};
            for (const auto controller : controllers) {
                config.flight_software.controller_selection = controller;
                std::ofstream telemetry;
                if (!options.telemetry_directory.empty()) {
                    telemetry = detumble_app::open_output(
                        std::filesystem::path{options.telemetry_directory} /
                        (std::to_string(options.start_run + i) + "_" +
                         std::string{detumble::to_string(controller)} + ".csv"));
                    detumble_app::write_telemetry_header(telemetry);
                }
                double next_sample{};
                const auto metrics =
                    detumble::run_native_mission(config, [&](const auto &sample) {
                        if (telemetry.is_open() && sample.time_s >= next_sample) {
                            detumble_app::write_telemetry(telemetry, config, sample,
                                                          options.sensor_profile);
                            next_sample += 1.0;
                        }
                    });
                result(output, options.start_run + i, options, config, metrics);
                output.flush();
                results.emplace_back(controller, metrics);
                all_pass = all_pass && metrics.success;
                std::cout << "Run " << i + 1 << '/' << options.runs << ' '
                          << detumble::to_string(controller) << ": "
                          << (metrics.success ? "PASS" : "FAIL") << ", detumble "
                          << metrics.detumble_time_s.value_or(-1.0) << " s, handoff "
                          << metrics.controller_handoff_time_s.value_or(-1.0) << " s\n"
                          << std::flush;
            }
        }
        for (const auto controller : {detumble::DetumbleController::bdot,
                                      detumble::DetumbleController::estimated_rate}) {
            std::size_t count{}, successes{}, detumbled{}, converged{}, handoffs{},
                false_events{};
            double time{}, effort{}, worst_rate{};
            for (const auto &[selected, metrics] : results) {
                if (selected != controller)
                    continue;
                ++count;
                successes += metrics.success;
                converged += metrics.estimator_convergence_time_s.has_value();
                handoffs += metrics.controller_handoff_time_s.has_value();
                false_events += metrics.false_low_rate_events;
                if (metrics.detumble_time_s) {
                    time += *metrics.detumble_time_s;
                    ++detumbled;
                }
                effort += metrics.integrated_dipole_squared_A2_m4_s;
                worst_rate = std::max(worst_rate, metrics.final_rate_rad_s);
            }
            if (count)
                aggregate << detumble::to_string(controller) << ',' << count << ','
                          << successes << ','
                          << (detumbled ? time / static_cast<double>(detumbled) : -1.0)
                          << ',' << worst_rate * detumble_app::radians_to_degrees << ','
                          << effort / static_cast<double>(count) << ',' << converged
                          << ',' << handoffs << ',' << false_events << '\n';
        }
        return all_pass ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
