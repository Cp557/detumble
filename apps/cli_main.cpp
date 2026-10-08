#include "detumble/version.hpp"
#include "options.hpp"
#include "telemetry.hpp"
int main(int argc, char **argv) {
    try {
        auto options = detumble_app::parse_options(argc, argv, false);
        if (options.help) {
            detumble_app::print_usage(false);
            return 0;
        }
        detumble_app::configure_sensors(options.mission, options.sensor_profile);
        std::ofstream output;
        if (!options.output.empty()) {
            output = detumble_app::open_output(options.output);
            detumble_app::write_telemetry_header(output);
        }
        const auto metrics =
            detumble::run_native_mission(options.mission, [&](const auto &sample) {
                if (output.is_open())
                    detumble_app::write_telemetry(output, options.mission, sample,
                                                  options.sensor_profile);
            });
        std::cout << "Detumble " << detumble::version() << " magnetic recovery\n"
                  << "Result: " << (metrics.success ? "PASS" : "FAIL") << " ("
                  << detumble::to_string(metrics.failure_reason) << ")\n"
                  << "Detumble time: " << metrics.detumble_time_s.value_or(-1.0)
                  << " s\n"
                  << "Final rotation: "
                  << metrics.final_rate_rad_s * detumble_app::radians_to_degrees
                  << " deg/s\n"
                  << "Final 300 s maximum: "
                  << metrics.final_window_maximum_rate_rad_s *
                         detumble_app::radians_to_degrees
                  << " deg/s\n"
                  << "Estimator convergence: "
                  << metrics.estimator_convergence_time_s.value_or(-1.0) << " s\n"
                  << "Rate estimation error: "
                  << metrics.final_rate_estimation_error_rad_s *
                         detumble_app::radians_to_degrees
                  << " deg/s\n"
                  << "Attitude estimation error: "
                  << metrics.final_attitude_estimation_error_rad *
                         detumble_app::radians_to_degrees
                  << " deg\n"
                  << "Controller handoff: "
                  << metrics.controller_handoff_time_s.value_or(-1.0) << " s\n"
                  << "False low-rate events: " << metrics.false_low_rate_events << '\n';
        return metrics.success ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
