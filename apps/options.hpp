#pragma once
#include "detumble/mission.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>
namespace detumble_app {
constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
struct Options {
    detumble::MissionRunConfig mission;
    std::size_t runs{25};
    std::size_t start_run{};
    std::uint64_t master_seed{2026};
    std::string sensor_profile{"realistic"};
    std::string output, results{"runs/magnetic_runs.csv"},
        aggregate{"runs/magnetic_aggregate.csv"}, telemetry_directory;
    bool compare{}, help{};
};
inline double number(const std::string &value) {
    std::size_t size{};
    const double result = std::stod(value, &size);
    if (size != value.size() || !std::isfinite(result))
        throw std::invalid_argument{"Expected finite number"};
    return result;
}
inline std::uint64_t integer(const std::string &value) {
    if (value.empty() || value[0] == '-')
        throw std::invalid_argument{"Expected nonnegative integer"};
    std::size_t size{};
    const auto result = std::stoull(value, &size);
    if (size != value.size())
        throw std::invalid_argument{"Expected integer"};
    return result;
}
inline Options parse_options(int argc, char **argv, bool campaign) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--help") {
            options.help = true;
            continue;
        }
        if (campaign && option == "--compare") {
            options.compare = true;
            continue;
        }
        if (++i >= argc)
            throw std::invalid_argument{option + " requires a value"};
        const std::string value = argv[i];
        if (option == "--duration")
            options.mission.duration_s = number(value);
        else if (option == "--time-step")
            options.mission.time_step_s = number(value);
        else if (option == "--seed") {
            options.mission.seed = integer(value);
            options.master_seed = integer(value);
        } else if (option == "--initial-rate-deg-s" && !campaign)
            options.mission.initial_rate_rad_s = number(value) / radians_to_degrees;
        else if (option == "--controller")
            options.mission.flight_software.controller_selection =
                detumble::parse_controller(value);
        else if (option == "--field-model")
            options.mission.environment.magnetic_field.model =
                detumble::parse_field_model(value);
        else if (option == "--reference-field-model") {
            options.mission.navigation_environment = options.mission.environment;
            options.mission.navigation_environment->magnetic_field.model =
                detumble::parse_field_model(value);
        } else if (option == "--epoch")
            options.mission.environment.magnetic_field.epoch = value;
        else if (option == "--bdot-filter")
            options.mission.flight_software.bdot_estimator.filter_time_constant_s =
                number(value);
        else if (option == "--gain")
            options.mission.flight_software.controller.gain_A_m2_s_per_T =
                number(value);
        else if (option == "--sensor-profile")
            options.sensor_profile = value;
        else if (option == "--estimator" && (value == "on" || value == "off"))
            options.mission.flight_software.attitude_estimator.enabled = value == "on";
        else if (!campaign && option == "--output")
            options.output = value;
        else if (campaign && option == "--start-run")
            options.start_run = integer(value);
        else if (campaign && option == "--runs")
            options.runs = integer(value);
        else if (campaign && option == "--results")
            options.results = value;
        else if (campaign && option == "--aggregate")
            options.aggregate = value;
        else if (campaign && option == "--telemetry-dir")
            options.telemetry_directory = value;
        else
            throw std::invalid_argument{"Unknown option or invalid value: " + option};
    }
    if (options.sensor_profile != "realistic" && options.sensor_profile != "ideal")
        throw std::invalid_argument{"Sensor profile must be realistic or ideal"};
    if (options.mission.duration_s <= 0.0 || options.mission.time_step_s <= 0.0 ||
        options.mission.initial_rate_rad_s < 0.0 || options.runs == 0)
        throw std::invalid_argument{
            "Duration, timestep and run count must be positive; rate nonnegative"};
    if (options.mission.navigation_environment) {
        const auto model = options.mission.navigation_environment->magnetic_field.model;
        options.mission.navigation_environment = options.mission.environment;
        options.mission.navigation_environment->magnetic_field.model = model;
    }
    return options;
}
inline void configure_sensors(detumble::MissionRunConfig &config,
                              const std::string &profile) {
    config.sensors = profile == "ideal"
                         ? detumble::ideal_sensor_suite_config(config.seed)
                         : detumble::realistic_sensor_suite_config(config.seed);
}
inline std::ofstream open_output(const std::filesystem::path &path) {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream stream{path};
    if (!stream)
        throw std::runtime_error{"Cannot write " + path.string()};
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream.precision(17);
    return stream;
}
inline void print_usage(bool campaign) {
    std::cout << "Usage: " << (campaign ? "detumble_monte_carlo" : "detumble")
              << " [options]\n"
              << "--duration SECONDS (12000)   --time-step SECONDS (0.02)\n"
              << "--seed INTEGER (42; campaign 2026)\n"
              << "--controller bdot|estimated-rate (bdot)\n"
              << "--field-model dipole|igrf14 (igrf14)\n"
              << "--reference-field-model dipole|igrf14 (same as truth)\n"
              << "--epoch YYYY-MM-DDTHH:MM:SSZ (2025-01-01T00:00:00Z)\n"
              << "--sensor-profile realistic|ideal (realistic)\n"
              << "--estimator on|off (on)   --bdot-filter SECONDS (0.2)   --gain VALUE "
                 "(50000)\n";
    if (campaign)
        std::cout << "--runs COUNT (25)   --start-run INDEX (0)   --compare (paired "
                     "controllers)\n"
                  << "--results PATH   --aggregate PATH   --telemetry-dir PATH "
                     "(optional, 1 Hz)\n";
    else
        std::cout << "--initial-rate-deg-s RATE (10)   --output PATH (full-rate CSV)\n";
}
} // namespace detumble_app
