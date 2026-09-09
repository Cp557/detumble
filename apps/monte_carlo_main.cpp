#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "detumble/mission.hpp"
#include "detumble/version.hpp"

namespace {

struct Options {
    std::size_t run_count{20};
    std::uint64_t seed{2'026};
    double duration_s{8'000.0};
    double time_step_s{0.02};
    double attitude_correction_gain{0.25};
    double bdot_filter_time_constant_s{0.2};
    double pointing_gain_scale{1.0};
    std::string results_path{"runs/monte_carlo_runs.csv"};
    std::string aggregate_path{"runs/monte_carlo_aggregate.csv"};
    std::optional<std::string> telemetry_directory;
    bool show_help{};
};

void print_usage(std::ostream& output) {
    output
        << "Usage: detumble_monte_carlo [options]\n\n"
        << "Options:\n"
        << "  --runs COUNT              Number of runs (default: 20)\n"
        << "  --seed INTEGER            Master seed (default: 2026)\n"
        << "  --duration SECONDS        Duration per run (default: 8000)\n"
        << "  --time-step SECONDS       Fixed timestep (default: 0.02)\n"
        << "  --attitude-gain GAIN      TRIAD correction gain (default: 0.25)\n"
        << "  --bdot-filter SECONDS     B-dot filter time constant (default: 0.2)\n"
        << "  --pointing-gain-scale X   Scale default PD gains (default: 1.0)\n"
        << "  --results PATH            Per-run metrics CSV\n"
        << "  --aggregate PATH          Aggregate metrics CSV\n"
        << "  --telemetry-dir PATH      Save reduced telemetry for each run\n"
        << "  --help                    Show this message\n";
}

std::uint64_t parse_integer(
    const std::string_view option,
    const std::string& value
) {
    if (value.empty() || value.front() == '-') {
        throw std::invalid_argument{
            std::string{option} + " requires a nonnegative integer"
        };
    }
    std::size_t parsed_characters{};
    const auto parsed = std::stoull(value, &parsed_characters);
    if (parsed_characters != value.size()) {
        throw std::invalid_argument{
            std::string{option} + " requires a nonnegative integer"
        };
    }
    return parsed;
}

double parse_number(
    const std::string_view option,
    const std::string& value
) {
    std::size_t parsed_characters{};
    const double parsed = std::stod(value, &parsed_characters);
    if (parsed_characters != value.size() || !std::isfinite(parsed)) {
        throw std::invalid_argument{
            std::string{option} + " requires a finite number"
        };
    }
    return parsed;
}

Options parse_options(const int argument_count, char** arguments) {
    Options options;
    for (int index = 1; index < argument_count; ++index) {
        const std::string_view option{arguments[index]};
        if (option == "--help") {
            options.show_help = true;
            continue;
        }
        if (index + 1 >= argument_count) {
            throw std::invalid_argument{
                std::string{option} + " requires a value"
            };
        }
        const std::string value{arguments[++index]};
        if (option == "--runs") {
            options.run_count = parse_integer(option, value);
        } else if (option == "--seed") {
            options.seed = parse_integer(option, value);
        } else if (option == "--duration") {
            options.duration_s = parse_number(option, value);
        } else if (option == "--time-step") {
            options.time_step_s = parse_number(option, value);
        } else if (option == "--attitude-gain") {
            options.attitude_correction_gain = parse_number(option, value);
        } else if (option == "--bdot-filter") {
            options.bdot_filter_time_constant_s = parse_number(option, value);
        } else if (option == "--pointing-gain-scale") {
            options.pointing_gain_scale = parse_number(option, value);
        } else if (option == "--results") {
            options.results_path = value;
        } else if (option == "--aggregate") {
            options.aggregate_path = value;
        } else if (option == "--telemetry-dir") {
            options.telemetry_directory = value;
        } else {
            throw std::invalid_argument{"Unknown option: " + std::string{option}};
        }
    }
    if (options.run_count == 0 || options.duration_s <= 0.0
        || options.time_step_s <= 0.0
        || options.attitude_correction_gain < 0.0
        || options.attitude_correction_gain > 1.0
        || options.bdot_filter_time_constant_s < 0.0
        || options.pointing_gain_scale <= 0.0) {
        throw std::invalid_argument{
            "Run count, duration, and timestep must be positive"
        };
    }
    return options;
}

double uniform_unit_interval(std::mt19937_64& generator) {
    constexpr double inverse_two_to_53 = 1.0 / 9007199254740992.0;
    return static_cast<double>(generator() >> 11U) * inverse_two_to_53;
}

void prepare_parent(const std::filesystem::path& path) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
}

double seconds_or_negative(const std::optional<double>& value) {
    return value.value_or(-1.0);
}

void write_results_header(std::ostream& output) {
    output
        << "run,seed,initial_rate_deg_s,initial_orbit_phase_deg,success,"
        << "failure_reason,final_mode,detumble_time_s,acquisition_time_s,"
        << "steady_pointing_time_s,final_rate_deg_s,"
        << "final_true_pointing_error_deg,"
        << "final_estimated_pointing_error_deg,"
        << "rms_true_pointing_error_deg,"
        << "maximum_true_pointing_error_deg,"
        << "final_attitude_estimation_error_deg,maximum_wheel_speed_rpm,"
        << "wheel_saturation_steps,pointing_samples\n";
}

void write_result(
    std::ostream& output,
    const std::size_t run,
    const detumble::MissionRunConfig& config,
    const detumble::MissionMetrics& metrics
) {
    constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
    output << run << ',' << config.seed << ','
           << config.initial_rate_rad_s * radians_to_degrees << ','
           << config.environment.orbit.initial_argument_of_latitude_rad
                  * radians_to_degrees
           << ',' << (metrics.success ? 1 : 0) << ','
           << detumble::to_string(metrics.failure_reason) << ','
           << detumble::to_string(metrics.final_mode) << ','
           << seconds_or_negative(metrics.detumble_time_s) << ','
           << seconds_or_negative(metrics.acquisition_time_s) << ','
           << seconds_or_negative(metrics.steady_pointing_time_s) << ','
           << metrics.final_rate_rad_s * radians_to_degrees << ','
           << metrics.final_true_pointing_error_rad * radians_to_degrees
           << ','
           << metrics.final_estimated_pointing_error_rad * radians_to_degrees
           << ',' << metrics.rms_true_pointing_error_rad * radians_to_degrees
           << ','
           << metrics.maximum_true_pointing_error_rad * radians_to_degrees
           << ','
           << metrics.final_attitude_estimation_error_rad
                  * radians_to_degrees
           << ',' << metrics.maximum_wheel_speed_rad_s * 60.0
                  / (2.0 * std::numbers::pi)
           << ',' << metrics.wheel_saturation_steps << ','
           << metrics.pointing_sample_count << '\n';
}

detumble::MissionTelemetryCallback telemetry_writer(
    const Options& options,
    const std::size_t run,
    std::ofstream& output
) {
    if (!options.telemetry_directory.has_value()) {
        return {};
    }
    const std::filesystem::path path =
        std::filesystem::path{*options.telemetry_directory}
        / ("run_" + std::to_string(run) + ".csv");
    std::filesystem::create_directories(path.parent_path());
    output.open(path);
    if (!output) {
        throw std::runtime_error{
            "Could not open telemetry file: " + path.string()
        };
    }
    output << std::setprecision(17)
           << "time_s,mode,body_rate_rad_s,true_pointing_error_rad,"
           << "estimated_pointing_error_rad,attitude_error_rad,in_eclipse,"
           << "detumble_complete,target_acquired,steady_pointing\n";
    return [&output](const detumble::MissionTelemetrySample& sample) {
        output << sample.time_s << ',' << detumble::to_string(sample.mode)
               << ',' << sample.body_rate_rad_s << ','
               << sample.true_pointing_error_rad << ','
               << sample.estimated_pointing_error_rad << ','
               << sample.attitude_estimation_error_rad << ','
               << (sample.in_eclipse ? 1 : 0) << ','
               << (sample.detumble_complete ? 1 : 0) << ','
               << (sample.target_acquired ? 1 : 0) << ','
               << (sample.steady_pointing ? 1 : 0) << '\n';
    };
}

void write_aggregate(
    const Options& options,
    const std::vector<detumble::MissionMetrics>& results
) {
    const auto successful = std::count_if(
        results.begin(),
        results.end(),
        [](const auto& result) { return result.success; }
    );
    double detumble_sum{};
    double acquisition_sum{};
    double rms_pointing_sum{};
    double worst_rms_pointing{};
    double worst_final_pointing{};
    std::size_t detumble_count{};
    std::size_t acquisition_count{};
    for (const auto& result : results) {
        if (result.detumble_time_s.has_value()) {
            detumble_sum += *result.detumble_time_s;
            ++detumble_count;
        }
        if (result.acquisition_time_s.has_value()) {
            acquisition_sum += *result.acquisition_time_s;
            ++acquisition_count;
        }
        rms_pointing_sum += result.rms_true_pointing_error_rad;
        worst_rms_pointing = std::max(
            worst_rms_pointing,
            result.rms_true_pointing_error_rad
        );
        worst_final_pointing = std::max(
            worst_final_pointing,
            result.final_true_pointing_error_rad
        );
    }
    const std::filesystem::path path{options.aggregate_path};
    prepare_parent(path);
    std::ofstream output{path};
    if (!output) {
        throw std::runtime_error{
            "Could not open aggregate output: " + path.string()
        };
    }
    constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
    output << std::setprecision(17) << "metric,value,unit\n"
           << "master_seed," << options.seed << ",count\n"
           << "run_count," << results.size() << ",count\n"
           << "attitude_correction_gain,"
           << options.attitude_correction_gain << ",unitless\n"
           << "bdot_filter_time_constant,"
           << options.bdot_filter_time_constant_s << ",s\n"
           << "pointing_gain_scale," << options.pointing_gain_scale
           << ",unitless\n"
           << "success_count," << successful << ",count\n"
           << "success_rate," << 100.0 * static_cast<double>(successful)
                  / static_cast<double>(results.size())
           << ",percent\n"
           << "mean_detumble_time," << (detumble_count == 0 ? -1.0
                  : detumble_sum / static_cast<double>(detumble_count))
           << ",s\n"
           << "mean_acquisition_time," << (acquisition_count == 0 ? -1.0
                  : acquisition_sum / static_cast<double>(acquisition_count))
           << ",s\n"
           << "mean_rms_true_pointing_error,"
           << rms_pointing_sum / static_cast<double>(results.size())
                  * radians_to_degrees
           << ",deg\n"
           << "worst_rms_true_pointing_error,"
           << worst_rms_pointing * radians_to_degrees << ",deg\n"
           << "worst_final_true_pointing_error,"
           << worst_final_pointing * radians_to_degrees << ",deg\n";
}

int run(const Options& options) {
    const std::filesystem::path results_path{options.results_path};
    prepare_parent(results_path);
    std::ofstream results_output{results_path};
    if (!results_output) {
        throw std::runtime_error{
            "Could not open results output: " + results_path.string()
        };
    }
    results_output << std::setprecision(17);
    write_results_header(results_output);

    std::mt19937_64 generator{options.seed};
    std::vector<detumble::MissionMetrics> results;
    results.reserve(options.run_count);
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    for (std::size_t run_index = 0; run_index < options.run_count; ++run_index) {
        const std::uint64_t run_seed = generator();
        detumble::MissionRunConfig config;
        config.seed = run_seed;
        config.duration_s = options.duration_s;
        config.time_step_s = options.time_step_s;
        config.initial_rate_rad_s = (5.0 + 10.0 * uniform_unit_interval(generator))
            * degrees_to_radians;
        config.environment.orbit.initial_argument_of_latitude_rad =
            2.0 * std::numbers::pi * uniform_unit_interval(generator);
        config.sensors = detumble::realistic_sensor_suite_config(run_seed);
        config.flight_software.attitude_estimator.correction_gain =
            options.attitude_correction_gain;
        config.flight_software.detumble.estimator.filter_time_constant_s =
            options.bdot_filter_time_constant_s;
        config.flight_software.sun_pointing.proportional_gain_body_Nm_per_rad
            *= options.pointing_gain_scale;
        config.flight_software.sun_pointing.derivative_gain_body_Nm_s_per_rad
            *= options.pointing_gain_scale;

        std::ofstream telemetry_output;
        const auto metrics = detumble::run_native_mission(
            config,
            telemetry_writer(options, run_index, telemetry_output)
        );
        write_result(results_output, run_index, config, metrics);
        results.push_back(metrics);
        std::cout << "Run " << (run_index + 1) << '/' << options.run_count
                  << ": " << (metrics.success ? "PASS" : "FAIL")
                  << " (" << detumble::to_string(metrics.failure_reason)
                  << ")\n";
    }
    write_aggregate(options, results);
    const auto success_count = std::count_if(
        results.begin(),
        results.end(),
        [](const auto& result) { return result.success; }
    );
    std::cout << "\nDetumble " << detumble::version()
              << " Monte Carlo\nSuccess: " << success_count << '/'
              << results.size() << " ("
              << 100.0 * static_cast<double>(success_count)
                     / static_cast<double>(results.size())
              << "%)\nResults: " << options.results_path
              << "\nAggregate: " << options.aggregate_path << '\n';
    return success_count == static_cast<std::ptrdiff_t>(results.size()) ? 0 : 1;
}

}  // namespace

int main(const int argument_count, char** arguments) {
    try {
        const Options options = parse_options(argument_count, arguments);
        if (options.show_help) {
            print_usage(std::cout);
            return 0;
        }
        return run(options);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n\n";
        print_usage(std::cerr);
        return 2;
    }
}
