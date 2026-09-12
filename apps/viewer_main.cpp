#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <imgui.h>
#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>

#include "detumble/coarse_sun_sensor.hpp"
#include "detumble/environment.hpp"
#include "detumble/flight_software.hpp"
#include "detumble/frames.hpp"
#include "detumble/gyroscope.hpp"
#include "detumble/magnetic_control.hpp"
#include "detumble/orbit.hpp"
#include "detumble/reaction_wheel.hpp"
#include "detumble/sensor_suite.hpp"
#include "detumble/simulation.hpp"
#include "detumble/spacecraft_visual.hpp"
#include "detumble/sun_pointing.hpp"

namespace {

constexpr double earth_display_radius = 2.0;
constexpr double orbit_display_altitude = 0.65;
constexpr double meters_to_spacecraft_display_units = 10.0;
constexpr float camera_transition_rate = 7.0F;
constexpr float orbit_camera_overview_radius = 10.5F;
constexpr Color sun_color{255, 135, 30, 255};

Vector3 to_raylib(const Eigen::Vector3d& vector) {
    return {
        static_cast<float>(vector.x()),
        static_cast<float>(vector.y()),
        static_cast<float>(vector.z())
    };
}

struct ViewerScenario {
    const char* name;
    const char* description;
    std::uint64_t seed;
    double initial_rate_deg_s;
    float suggested_speed;
};

constexpr std::array<ViewerScenario, 3> viewer_scenarios{{
    {
        "Nominal mission",
        "10 deg/s tumble with realistic seeded sensors",
        42,
        10.0,
        50.0F
    },
    {
        "Gentle tumble",
        "5 deg/s starting rate for a shorter detumble",
        7,
        5.0,
        25.0F
    },
    {
        "Fast tumble",
        "15 deg/s upper design case",
        2025,
        15.0,
        100.0F
    }
}};

struct ViewerFrame {
    double time_s{};
    Eigen::Quaterniond body_to_inertial{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_velocity_body_rad_s{Eigen::Vector3d::Zero()};
    detumble::EnvironmentState environment;
    detumble::FlightMode mode{detumble::FlightMode::boot};
    double sun_pointing_error_deg{std::numeric_limits<double>::quiet_NaN()};
    bool detumble_complete{};
    bool steady_pointing{};
    bool magnetorquers_enabled{};
    bool reaction_wheels_enabled{};
    Eigen::Vector3d limited_dipole_body_A_m2{Eigen::Vector3d::Zero()};
    std::array<double, 4> wheel_speed_rad_s{};
    std::optional<detumble::CoarseSunSensorMeasurement> sun_measurement;
};

ViewerFrame capture_viewer_frame(
    const detumble::Simulation& simulation,
    const detumble::EnvironmentState& environment,
    const detumble::AutonomousFlightSoftware& flight_software,
    const detumble::MagneticControlOutput& magnetic_output,
    const detumble::ReactionWheelTelemetry& wheel_telemetry,
    const std::optional<detumble::CoarseSunSensorMeasurement>& sun_measurement
) {
    const detumble::AutonomousFlightSoftwareState& flight_state =
        flight_software.state();
    ViewerFrame frame{
        .time_s = simulation.elapsed_time_s(),
        .body_to_inertial = simulation.state().body_to_inertial,
        .angular_velocity_body_rad_s =
            simulation.state().angular_velocity_body_rad_s,
        .environment = environment,
        .mode = flight_state.mode,
        .sun_pointing_error_deg = flight_state.sun_pointing.active
            ? flight_state.sun_pointing.pointing_error_rad
                * 180.0 / std::numbers::pi
            : std::numeric_limits<double>::quiet_NaN(),
        .detumble_complete =
            flight_software.detumble().state().detumble_complete,
        .steady_pointing =
            flight_software.sun_pointing_tracker().state().steady_pointing,
        .magnetorquers_enabled =
            flight_state.actuator_command.magnetorquers_enabled,
        .reaction_wheels_enabled =
            flight_state.actuator_command.reaction_wheels_enabled,
        .limited_dipole_body_A_m2 =
            magnetic_output.limited_dipole_body_A_m2,
        .sun_measurement = sun_measurement
    };
    const std::size_t wheel_count = std::min(
        frame.wheel_speed_rad_s.size(),
        wheel_telemetry.wheel_speed_rad_s.size()
    );
    std::copy_n(
        wheel_telemetry.wheel_speed_rad_s.begin(),
        wheel_count,
        frame.wheel_speed_rad_s.begin()
    );
    return frame;
}

struct MissionVisualState {
    void reset() {
        *this = {};
    }

    void update(const ViewerFrame& frame, const double frame_time_s) {
        goal_achieved = goal_achieved || frame.steady_pointing;
        const bool current_pointing_phase = frame.detumble_complete;
        constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
        const double target_value = current_pointing_phase
            ? frame.sun_pointing_error_deg
            : radians_to_degrees
                * frame.angular_velocity_body_rad_s.norm();
        if (!std::isfinite(target_value)) {
            value_valid = false;
            pointing_phase = current_pointing_phase;
            return;
        }

        if (!value_valid || pointing_phase != current_pointing_phase) {
            smoothed_value = target_value;
            displayed_value = target_value;
            display_update_accumulator_s = 0.0;
            value_valid = true;
            pointing_phase = current_pointing_phase;
            return;
        }

        const double bounded_frame_time_s = std::clamp(
            frame_time_s,
            0.0,
            0.1
        );
        constexpr double smoothing_time_s = 0.6;
        const double blend = 1.0 - std::exp(
            -bounded_frame_time_s / smoothing_time_s
        );
        smoothed_value += blend * (target_value - smoothed_value);
        display_update_accumulator_s += bounded_frame_time_s;
        constexpr double display_update_period_s = 0.25;
        if (display_update_accumulator_s >= display_update_period_s) {
            displayed_value = smoothed_value;
            display_update_accumulator_s = 0.0;
        }
    }

    bool goal_achieved{};
    bool value_valid{};
    bool pointing_phase{};
    double smoothed_value{};
    double displayed_value{};
    double display_update_accumulator_s{};
};

struct ActuatorVisualState {
    void reset() {
        rod_activity.fill(0.0F);
        wheel_phase_rad.fill(0.0F);
    }

    void update(
        const ViewerFrame& frame,
        const detumble::MagnetorquerConfig& magnetorquer_config,
        const detumble::ReactionWheelClusterConfig& wheel_config,
        const double frame_time_s,
        const bool animation_running
    ) {
        const double bounded_frame_time_s = std::clamp(
            frame_time_s,
            0.0,
            0.1
        );
        const float blend = static_cast<float>(
            1.0 - std::exp(-bounded_frame_time_s / 0.18)
        );
        for (std::size_t index = 0; index < rod_activity.size(); ++index) {
            const double limit =
                magnetorquer_config.maximum_dipole_body_A_m2[
                    static_cast<Eigen::Index>(index)
                ];
            const float target = frame.magnetorquers_enabled && limit > 0.0
                ? static_cast<float>(std::clamp(
                      std::abs(frame.limited_dipole_body_A_m2[
                          static_cast<Eigen::Index>(index)
                      ]) / limit,
                      0.0,
                      1.0
                  ))
                : 0.0F;
            rod_activity[index] += blend * (target - rod_activity[index]);
        }

        if (!animation_running) {
            return;
        }
        const std::size_t wheel_count = std::min(
            wheel_phase_rad.size(),
            wheel_config.wheels.size()
        );
        for (std::size_t index = 0; index < wheel_count; ++index) {
            const double speed = frame.wheel_speed_rad_s[index];
            const double maximum_speed =
                wheel_config.wheels[index].maximum_speed_rad_s;
            if (std::abs(speed) < 1.0e-6 || maximum_speed <= 0.0) {
                continue;
            }
            const double speed_fraction = std::clamp(
                std::abs(speed) / maximum_speed,
                0.0,
                1.0
            );
            const double display_speed_rad_s = std::copysign(
                0.8 + 5.2 * std::sqrt(speed_fraction),
                speed
            );
            wheel_phase_rad[index] = static_cast<float>(std::remainder(
                static_cast<double>(wheel_phase_rad[index])
                    + display_speed_rad_s * bounded_frame_time_s,
                2.0 * std::numbers::pi
            ));
        }
    }

    std::array<float, 3> rod_activity{};
    std::array<float, 4> wheel_phase_rad{};
};

detumble::EnvironmentState current_environment(
    const detumble::Simulation& simulation,
    const detumble::EnvironmentConfig& config
) {
    return detumble::sample_environment(
        config,
        simulation.state().body_to_inertial,
        simulation.elapsed_time_s()
    );
}

detumble::MagneticControlOutput advance_simulation(
    detumble::Simulation& simulation,
    const detumble::EnvironmentConfig& environment_config,
    detumble::MagneticControlCycle& magnetic_control,
    detumble::IdealGyroscope& gyroscope,
    detumble::IdealCoarseSunSensorArray& sun_sensors,
    detumble::AutonomousFlightSoftware& flight_software,
    detumble::ReactionWheelCluster& reaction_wheels
) {
    const detumble::EnvironmentState environment = current_environment(
        simulation,
        environment_config
    );
    const std::optional<detumble::GyroscopeMeasurement>
        gyroscope_measurement = gyroscope.sample_if_due(
            simulation.elapsed_time_s(),
            simulation.state().angular_velocity_body_rad_s
        );
    const std::optional<detumble::CoarseSunSensorMeasurement>
        sun_sensor_measurement = sun_sensors.sample_if_due(
        simulation.elapsed_time_s(),
        environment.sun_direction_body,
        environment.in_eclipse
    );
    detumble::MagneticControlOutput output = magnetic_control.update(
        simulation.elapsed_time_s(),
        environment.magnetic_field_body_T,
        flight_software.state()
            .actuator_command.magnetorquer_dipole_body_A_m2
    );
    if (gyroscope_measurement.has_value()) {
        flight_software.update(detumble::AutonomousFlightSoftwareInput{
            .gyroscope = *gyroscope_measurement,
            .magnetometer = output.magnetometer_measurement,
            .sun_sensor = sun_sensor_measurement,
            .magnetic_field_inertial_T =
                environment.magnetic_field_inertial_T,
            .sun_direction_inertial = environment.sun_direction_inertial,
            .in_eclipse = environment.in_eclipse
        });
    }
    const Eigen::Vector3d& wheel_request = flight_software.state()
        .actuator_command.reaction_wheel_torque_body_Nm;
    const bool wheels_enabled = flight_software.state()
        .actuator_command.reaction_wheels_enabled;
    reaction_wheels.update(
        detumble::ReactionWheelCommand{
            .sample_time_s = simulation.elapsed_time_s(),
            .requested_spacecraft_torque_body_Nm = wheel_request,
            .enabled = wheels_enabled
        },
        simulation.config().time_step_s
    );
    simulation.step(
        output.applied_torque_body_Nm
        + reaction_wheels.telemetry().applied_spacecraft_torque_body_Nm
    );
    return output;
}

struct OrbitCamera {
    float azimuth_rad{0.75F};
    float elevation_rad{0.35F};
    float radius{orbit_camera_overview_radius};
    float target_azimuth_rad{azimuth_rad};
    float target_elevation_rad{elevation_rad};
    float target_radius{radius};

    [[nodiscard]] Camera3D camera() const {
        const float horizontal_radius = radius * std::cos(elevation_rad);
        return {
            .position = {
                horizontal_radius * std::cos(azimuth_rad),
                radius * std::sin(elevation_rad),
                horizontal_radius * std::sin(azimuth_rad)
            },
            .target = {0.0F, 0.0F, 0.0F},
            .up = {0.0F, 1.0F, 0.0F},
            .fovy = 45.0F,
            .projection = CAMERA_PERSPECTIVE
        };
    }

    void orbit_overview() {
        target_azimuth_rad = 0.75F;
        target_elevation_rad = 0.45F;
        target_radius = orbit_camera_overview_radius;
    }

    void update() {
        const float frame_time_s = std::clamp(GetFrameTime(), 0.0F, 0.1F);
        const float blend = 1.0F - std::exp(
            -camera_transition_rate * frame_time_s
        );
        azimuth_rad += blend * (target_azimuth_rad - azimuth_rad);
        elevation_rad += blend * (target_elevation_rad - elevation_rad);
        radius += blend * (target_radius - radius);

        if (ImGui::GetIO().WantCaptureMouse) {
            return;
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            const Vector2 delta = GetMouseDelta();
            azimuth_rad -= delta.x * 0.008F;
            elevation_rad = std::clamp(
                elevation_rad + delta.y * 0.008F,
                -1.35F,
                1.35F
            );
            target_azimuth_rad = azimuth_rad;
            target_elevation_rad = elevation_rad;
        }

        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0F) {
            radius = std::clamp(
                radius * std::exp(-wheel * 0.12F),
                4.5F,
                14.0F
            );
            target_radius = radius;
        }
    }
};

struct SpacecraftModel {
    Model model{};
    float uniform_display_scale{1.0F};
    bool loaded_custom_glb{};
};

SpacecraftModel load_spacecraft_model() {
    const std::filesystem::path asset_path =
        std::filesystem::path{GetApplicationDirectory()}
        / "assets/models/detumble_3u.glb";

    SpacecraftModel spacecraft;
    spacecraft.model = LoadModel(asset_path.string().c_str());
    spacecraft.loaded_custom_glb = IsModelValid(spacecraft.model);

    if (!spacecraft.loaded_custom_glb) {
        TraceLog(
            LOG_WARNING,
            "Could not load %s; using a generated cube",
            asset_path.string().c_str()
        );
        spacecraft.model = LoadModelFromMesh(GenMeshCube(1.0F, 1.0F, 1.0F));
    }

    const BoundingBox bounds = GetModelBoundingBox(spacecraft.model);
    const Vector3 dimensions = Vector3Subtract(bounds.max, bounds.min);
    if (spacecraft.loaded_custom_glb) {
        spacecraft.uniform_display_scale =
            static_cast<float>(meters_to_spacecraft_display_units);
        TraceLog(
            LOG_INFO,
            "MODEL: Detumble 3U bounds %.4f x %.4f x %.4f m",
            dimensions.x,
            dimensions.y,
            dimensions.z
        );
        if (dimensions.z <= std::max(dimensions.x, dimensions.y)) {
            TraceLog(
                LOG_WARNING,
                "MODEL: expected body +Z to be the custom model's long axis"
            );
        }
    } else {
        const Vector3 center = Vector3Scale(
            Vector3Add(bounds.min, bounds.max),
            0.5F
        );
        spacecraft.model.transform = MatrixTranslate(
            -center.x,
            -center.y,
            -center.z
        );
    }
    return spacecraft;
}

void draw_spacecraft(
    const SpacecraftModel& spacecraft,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    const Quaternion rotation{
        static_cast<float>(body_to_inertial.x()),
        static_cast<float>(body_to_inertial.y()),
        static_cast<float>(body_to_inertial.z()),
        static_cast<float>(body_to_inertial.w())
    };
    Vector3 rotation_axis{};
    float rotation_angle_rad{};
    QuaternionToAxisAngle(rotation, &rotation_axis, &rotation_angle_rad);
    const float rotation_angle_deg = rotation_angle_rad * 180.0F / PI;

    const float model_scale = static_cast<float>(
        spacecraft.uniform_display_scale * visual_scale
    );
    const Vector3 draw_scale = spacecraft.loaded_custom_glb
        ? Vector3{model_scale, model_scale, model_scale}
        : Vector3{model_scale, model_scale, 3.4F * model_scale};
    DrawModelEx(
        spacecraft.model,
        to_raylib(origin_inertial),
        rotation_axis,
        rotation_angle_deg,
        draw_scale,
        WHITE
    );
}

void draw_body_outline(
    const detumble::SpacecraftVisualConfig& visual_config,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    const Eigen::Vector3d half_dimensions =
        0.5 * meters_to_spacecraft_display_units
        * visual_config.body_dimensions_m;
    const std::array<Eigen::Vector3d, 8> corners_body{
        Eigen::Vector3d{
            -half_dimensions.x(),
            -half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            -half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            -half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            -half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            half_dimensions.y(),
            half_dimensions.z()
        }
    };
    constexpr std::array<std::array<std::size_t, 2>, 12> edges{
        std::array<std::size_t, 2>{0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    std::array<Vector3, 8> corners_inertial{};
    for (std::size_t index = 0; index < corners_body.size(); ++index) {
        corners_inertial[index] = to_raylib(
            origin_inertial
            + visual_scale * detumble::rotate_body_to_inertial(
                  body_to_inertial,
                  corners_body[index]
              )
        );
    }
    for (const auto& edge : edges) {
        DrawLine3D(
            corners_inertial[edge[0]],
            corners_inertial[edge[1]],
            Color{110, 200, 255, 255}
        );
    }
}

Eigen::Vector3d component_center_inertial(
    const Eigen::Vector3d& center_body_m,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    return origin_inertial
        + visual_scale * meters_to_spacecraft_display_units
            * detumble::rotate_body_to_inertial(
                body_to_inertial,
                center_body_m
            );
}

void draw_configured_surface_sensor(
    const detumble::SurfaceSensorComponent& sensor,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale,
    const Color color
) {
    const Eigen::Vector3d center = component_center_inertial(
        sensor.center_body_m,
        body_to_inertial,
        origin_inertial,
        visual_scale
    );
    const Eigen::Vector3d normal = detumble::rotate_body_to_inertial(
        body_to_inertial,
        sensor.outward_normal_body
    );
    const double half_thickness = 0.00075 * visual_scale
        * meters_to_spacecraft_display_units;
    const float half_width = static_cast<float>(
        0.5 * visual_scale * meters_to_spacecraft_display_units
        * sensor.side_length_m
    );
    DrawCylinderEx(
        to_raylib(center - half_thickness * normal),
        to_raylib(center + half_thickness * normal),
        half_width,
        half_width,
        4,
        color
    );
}

void draw_sun_sensor_status(
    const detumble::SpacecraftVisualConfig& visual_config,
    const std::optional<detumble::CoarseSunSensorMeasurement>& measurement,
    const Eigen::Quaterniond& body_to_inertial,
    const Eigen::Vector3d& origin_inertial,
    const double visual_scale
) {
    for (std::size_t index = 0;
         index < visual_config.coarse_sun_sensors.size();
         ++index) {
        Color color{35, 115, 130, 255};
        if (measurement.has_value() && measurement->in_eclipse) {
            color = Color{100, 105, 115, 255};
        } else if (measurement.has_value()
                   && measurement->sensor_visible[index]) {
            color = sun_color;
        }
        detumble::SurfaceSensorComponent status_sensor =
            visual_config.coarse_sun_sensors[index];
        status_sensor.center_body_m +=
            0.0015 * status_sensor.outward_normal_body;
        draw_configured_surface_sensor(
            status_sensor,
            body_to_inertial,
            origin_inertial,
            visual_scale,
            color
        );
    }
}

Eigen::Vector3d orbit_position_display(
    const detumble::EnvironmentConfig&,
    const detumble::EnvironmentState& environment
) {
    return (earth_display_radius + orbit_display_altitude)
        * environment.orbit.position_inertial_m.normalized();
}

void draw_orbit_path(
    const detumble::EnvironmentConfig& config,
    const double current_time_s
) {
    constexpr int segment_count = 180;
    const double period_s = detumble::circular_orbit_period_s(config.orbit);
    const double display_radius = earth_display_radius
        + orbit_display_altitude;
    const double current_phase = std::fmod(current_time_s / period_s, 1.0);

    Eigen::Vector3d previous = display_radius
        * detumble::circular_orbit_state(config.orbit, 0.0)
              .position_inertial_m.normalized();
    for (int segment = 1; segment <= segment_count; ++segment) {
        const double elapsed_time_s = period_s
            * static_cast<double>(segment)
            / static_cast<double>(segment_count);
        const Eigen::Vector3d current = display_radius
            * detumble::circular_orbit_state(config.orbit, elapsed_time_s)
                  .position_inertial_m.normalized();
        const double segment_phase = static_cast<double>(segment)
            / static_cast<double>(segment_count);
        const double age_fraction = std::fmod(
            current_phase - segment_phase + 1.0,
            1.0
        );
        const bool recent_trail = age_fraction < 0.18;
        const unsigned char alpha = recent_trail
            ? static_cast<unsigned char>(
                235.0 - 155.0 * age_fraction / 0.18
            )
            : 55;
        DrawLine3D(
            to_raylib(previous),
            to_raylib(current),
            Color{80, 175, 235, alpha}
        );
        previous = current;
    }
}

void draw_earth() {
    DrawSphere({}, earth_display_radius, Color{24, 66, 120, 255});
    DrawSphereWires(
        {},
        static_cast<float>(earth_display_radius * 1.002),
        18,
        36,
        Color{70, 125, 180, 180}
    );
}

void draw_starfield() {
    const int width = GetScreenWidth();
    const int height = GetScreenHeight();
    if (width <= 0 || height <= 0) {
        return;
    }

    std::uint32_t state = 0x5A17C9E3U;
    constexpr int star_count = 260;
    for (int index = 0; index < star_count; ++index) {
        state = 1'664'525U * state + 1'013'904'223U;
        const int x = static_cast<int>(
            state % static_cast<std::uint32_t>(width)
        );
        state = 1'664'525U * state + 1'013'904'223U;
        const int y = static_cast<int>(
            state % static_cast<std::uint32_t>(height)
        );
        state = 1'664'525U * state + 1'013'904'223U;
        const unsigned char brightness = static_cast<unsigned char>(
            105U + state % 130U
        );
        const Color color{
            brightness,
            static_cast<unsigned char>(std::min(255, brightness + 8)),
            static_cast<unsigned char>(std::min(255, brightness + 18)),
            210
        };
        if (index % 23 == 0) {
            DrawCircle(x, y, 1.4F, color);
        } else {
            DrawPixel(x, y, color);
        }
    }
}

void draw_sun_marker(const Eigen::Vector3d& sun_direction_inertial) {
    const Eigen::Vector3d direction = sun_direction_inertial.normalized();
    DrawSphere(
        to_raylib(5.2 * direction),
        0.32F,
        sun_color
    );
}

void reset_viewer_scenario(
    const ViewerScenario& scenario,
    detumble::Simulation& simulation,
    detumble::MagneticControlCycle& magnetic_control,
    detumble::IdealGyroscope& gyroscope,
    detumble::IdealCoarseSunSensorArray& sun_sensors,
    detumble::AutonomousFlightSoftware& flight_software,
    detumble::ReactionWheelCluster& reaction_wheels,
    detumble::MagneticControlOutput& magnetic_output
) {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    detumble::SimulationConfig simulation_config;
    simulation_config.seed = scenario.seed;
    simulation_config.minimum_angular_speed_rad_s =
        scenario.initial_rate_deg_s * degrees_to_radians;
    simulation_config.maximum_angular_speed_rad_s =
        scenario.initial_rate_deg_s * degrees_to_radians;
    simulation = detumble::Simulation{simulation_config};

    const detumble::SensorSuiteConfig sensors =
        detumble::realistic_sensor_suite_config(scenario.seed);
    magnetic_control = detumble::MagneticControlCycle{sensors.magnetometer};
    gyroscope = detumble::IdealGyroscope{sensors.gyroscope};
    sun_sensors = detumble::IdealCoarseSunSensorArray{sensors.sun_sensor};
    flight_software = detumble::AutonomousFlightSoftware{};
    reaction_wheels = detumble::ReactionWheelCluster{};
    magnetic_output = {};
}

const char* actuator_status(const ViewerFrame& frame) {
    if (frame.magnetorquers_enabled) {
        return "magnetorquers active";
    }
    if (frame.reaction_wheels_enabled) {
        return "reaction wheels active";
    }
    return "actuators idle";
}

struct InternalsProjection {
    ImVec2 center;
    float pixels_per_meter{};

    [[nodiscard]] ImVec2 project(const Eigen::Vector3d& point_body_m) const {
        return {
            center.x + pixels_per_meter * static_cast<float>(
                point_body_m.x() - 0.65 * point_body_m.y()
            ),
            center.y - pixels_per_meter * static_cast<float>(
                point_body_m.z()
                + 0.28 * point_body_m.x()
                + 0.28 * point_body_m.y()
            )
        };
    }
};

ImU32 blended_color(
    const ImVec4& inactive,
    const ImVec4& active,
    const float activity
) {
    const float amount = std::clamp(activity, 0.0F, 1.0F);
    return ImGui::ColorConvertFloat4ToU32({
        inactive.x + amount * (active.x - inactive.x),
        inactive.y + amount * (active.y - inactive.y),
        inactive.z + amount * (active.z - inactive.z),
        inactive.w + amount * (active.w - inactive.w)
    });
}

void draw_internal_body(
    ImDrawList& draw_list,
    const InternalsProjection& projection,
    const Eigen::Vector3d& dimensions_m
) {
    const Eigen::Vector3d half_dimensions = 0.5 * dimensions_m;
    const std::array<Eigen::Vector3d, 8> corners_body{
        Eigen::Vector3d{
            -half_dimensions.x(),
            -half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            -half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            half_dimensions.y(),
            -half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            -half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            -half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            half_dimensions.x(),
            half_dimensions.y(),
            half_dimensions.z()
        },
        Eigen::Vector3d{
            -half_dimensions.x(),
            half_dimensions.y(),
            half_dimensions.z()
        }
    };
    std::array<ImVec2, 8> corners{};
    for (std::size_t index = 0; index < corners.size(); ++index) {
        corners[index] = projection.project(corners_body[index]);
    }

    const std::array<ImVec2, 4> side_face{
        corners[0], corners[1], corners[5], corners[4]
    };
    draw_list.AddConvexPolyFilled(
        side_face.data(),
        static_cast<int>(side_face.size()),
        IM_COL32(65, 78, 96, 55)
    );
    constexpr std::array<std::array<std::size_t, 2>, 12> edges{
        std::array<std::size_t, 2>{0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };
    for (const auto& edge : edges) {
        draw_list.AddLine(
            corners[edge[0]],
            corners[edge[1]],
            IM_COL32(145, 165, 190, 210),
            1.5F
        );
    }
}

void draw_internal_magnetorquers(
    ImDrawList& draw_list,
    const InternalsProjection& projection,
    const detumble::SpacecraftVisualConfig& config,
    const ActuatorVisualState& visual_state
) {
    constexpr ImVec4 inactive{0.20F, 0.42F, 0.47F, 0.85F};
    constexpr ImVec4 active{0.10F, 0.95F, 1.00F, 1.00F};
    for (std::size_t index = 0; index < config.magnetorquers.size(); ++index) {
        const detumble::RodComponent& rod = config.magnetorquers[index];
        const Eigen::Vector3d half_axis =
            0.5 * rod.length_m * rod.axis_body.normalized();
        const ImVec2 start = projection.project(rod.center_body_m - half_axis);
        const ImVec2 end = projection.project(rod.center_body_m + half_axis);
        const float activity = visual_state.rod_activity[index];
        if (activity > 0.03F) {
            draw_list.AddLine(
                start,
                end,
                IM_COL32(
                    35,
                    225,
                    245,
                    static_cast<int>(75.0F * activity)
                ),
                9.0F
            );
        }
        draw_list.AddLine(
            start,
            end,
            blended_color(inactive, active, activity),
            3.0F + 2.0F * activity
        );
    }
}

void draw_internal_reaction_wheels(
    ImDrawList& draw_list,
    const ImVec2& canvas_start,
    const ImVec2& canvas_size,
    const detumble::SpacecraftVisualConfig& config,
    const ActuatorVisualState& visual_state,
    const bool active
) {
    constexpr int segment_count = 24;
    constexpr std::array<ImVec2, 4> center_fractions{
        ImVec2{0.30F, 0.30F},
        ImVec2{0.70F, 0.30F},
        ImVec2{0.30F, 0.70F},
        ImVec2{0.70F, 0.70F}
    };
    const ImU32 rim_color = active
        ? IM_COL32(255, 175, 55, 255)
        : IM_COL32(125, 105, 70, 220);
    const ImU32 spoke_color = active
        ? IM_COL32(255, 215, 125, 255)
        : IM_COL32(145, 130, 100, 210);
    for (std::size_t index = 0; index < config.reaction_wheels.size(); ++index) {
        const detumble::ReactionWheelComponent& wheel =
            config.reaction_wheels[index];
        const Eigen::Vector3d axis = wheel.spin_axis_body.normalized();
        const Eigen::Vector3d basis_u = axis.unitOrthogonal().normalized();
        const Eigen::Vector3d basis_v = axis.cross(basis_u).normalized();
        const ImVec2 wheel_center{
            canvas_start.x + canvas_size.x * center_fractions[index].x,
            canvas_start.y + canvas_size.y * center_fractions[index].y
        };
        const float displayed_radius = std::min(
            0.17F * canvas_size.x,
            0.14F * canvas_size.y
        );
        const float pixels_per_meter = displayed_radius
            / static_cast<float>(wheel.radius_m);
        const auto project_offset = [wheel_center, pixels_per_meter](
            const Eigen::Vector3d& offset_body_m
        ) {
            return ImVec2{
                wheel_center.x + pixels_per_meter
                    * static_cast<float>(offset_body_m.x()),
                wheel_center.y - pixels_per_meter
                    * static_cast<float>(offset_body_m.y())
            };
        };
        std::array<ImVec2, segment_count> rim{};
        for (int segment = 0; segment < segment_count; ++segment) {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(segment)
                / static_cast<double>(segment_count);
            rim[static_cast<std::size_t>(segment)] = project_offset(
                wheel.radius_m
                * (std::cos(angle) * basis_u + std::sin(angle) * basis_v)
            );
        }
        draw_list.AddPolyline(
            rim.data(),
            static_cast<int>(rim.size()),
            rim_color,
            ImDrawFlags_Closed,
            active ? 2.5F : 1.8F
        );

        const double phase = visual_state.wheel_phase_rad[index];
        const Eigen::Vector3d spoke_direction =
            std::cos(phase) * basis_u + std::sin(phase) * basis_v;
        draw_list.AddLine(
            project_offset(-wheel.radius_m * spoke_direction),
            project_offset(wheel.radius_m * spoke_direction),
            spoke_color,
            active ? 2.0F : 1.2F
        );
        draw_list.AddCircleFilled(
            wheel_center,
            2.5F,
            spoke_color
        );
    }
}

void draw_actuator_internals(
    const detumble::SpacecraftVisualConfig& config,
    const ViewerFrame& frame,
    const ActuatorVisualState& visual_state
) {
    const ImVec2 canvas_start = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    canvas_size.x = std::max(canvas_size.x, 50.0F);
    canvas_size.y = std::max(canvas_size.y, 80.0F);
    const ImVec2 canvas_end{
        canvas_start.x + canvas_size.x,
        canvas_start.y + canvas_size.y
    };
    ImDrawList& draw_list = *ImGui::GetWindowDrawList();
    draw_list.AddRectFilled(
        canvas_start,
        canvas_end,
        IM_COL32(6, 11, 19, 180),
        4.0F
    );
    draw_list.PushClipRect(canvas_start, canvas_end, true);

    constexpr float card_gap = 8.0F;
    constexpr float card_title_height = 24.0F;
    const ImVec2 card_size{
        0.5F * (canvas_size.x - card_gap),
        canvas_size.y
    };
    const ImVec2 magnetorquer_card_start = canvas_start;
    const ImVec2 wheel_card_start{
        canvas_start.x + card_size.x + card_gap,
        canvas_start.y
    };
    for (const ImVec2& card_start : {
             magnetorquer_card_start,
             wheel_card_start
         }) {
        draw_list.AddRectFilled(
            card_start,
            {card_start.x + card_size.x, card_start.y + card_size.y},
            IM_COL32(7, 14, 24, 230),
            4.0F
        );
        draw_list.AddRect(
            card_start,
            {card_start.x + card_size.x, card_start.y + card_size.y},
            IM_COL32(55, 72, 94, 190),
            4.0F
        );
    }
    draw_list.AddText(
        {magnetorquer_card_start.x + 7.0F,
         magnetorquer_card_start.y + 5.0F},
        IM_COL32(30, 235, 250, 255),
        "MAGNETORQUERS"
    );
    draw_list.AddText(
        {wheel_card_start.x + 7.0F, wheel_card_start.y + 5.0F},
        IM_COL32(255, 175, 55, 255),
        "REACTION WHEELS"
    );

    const ImVec2 subsystem_size{
        card_size.x,
        card_size.y - card_title_height
    };
    const ImVec2 magnetorquer_start{
        magnetorquer_card_start.x,
        magnetorquer_card_start.y + card_title_height
    };
    const InternalsProjection projection{
        .center = {
            magnetorquer_start.x + 0.5F * subsystem_size.x,
            magnetorquer_start.y + 0.50F * subsystem_size.y
        },
        .pixels_per_meter = std::min(
            subsystem_size.x / 0.22F,
            subsystem_size.y / 0.42F
        )
    };
    draw_internal_body(draw_list, projection, config.body_dimensions_m);
    draw_internal_magnetorquers(
        draw_list,
        projection,
        config,
        visual_state
    );
    draw_internal_reaction_wheels(
        draw_list,
        {wheel_card_start.x, wheel_card_start.y + card_title_height},
        subsystem_size,
        config,
        visual_state,
        frame.reaction_wheels_enabled
    );

    draw_list.PopClipRect();
    ImGui::Dummy(canvas_size);
}

void draw_mission_progress(
    const ViewerScenario& scenario,
    const MissionVisualState& visual_state
);

void draw_scenario_status_panel(
    const ViewerScenario& scenario,
    const ViewerFrame& frame,
    const detumble::SpacecraftVisualConfig& spacecraft_config,
    const ActuatorVisualState& actuator_visual_state,
    const MissionVisualState& mission_visual_state
) {
    const float right_aligned_x = std::max(
        390.0F,
        static_cast<float>(GetScreenWidth()) - 376.0F
    );
    const float panel_height = std::clamp(
        static_cast<float>(GetScreenHeight()) - 32.0F,
        420.0F,
        560.0F
    );
    ImGui::SetNextWindowPos({right_aligned_x, 16.0F}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({360.0F, panel_height}, ImGuiCond_Always);
    if (ImGui::Begin("Mission status")) {
        ImGui::TextColored(
            ImVec4{0.45F, 1.0F, 0.55F, 1.0F},
            "LIVE  |  %s",
            detumble::to_string(frame.mode).data()
        );
        ImGui::Text(
            "Seed %llu  |  %llu s",
            static_cast<unsigned long long>(scenario.seed),
            static_cast<unsigned long long>(frame.time_s)
        );

        ImGui::Separator();
        ImGui::Text(
            "Sun: %s",
            frame.environment.in_eclipse ? "ECLIPSE" : "AVAILABLE"
        );
        ImGui::Text("Active control: %s", actuator_status(frame));
        if (mission_visual_state.goal_achieved) {
            ImGui::TextColored(
                ImVec4{0.30F, 0.95F, 0.45F, 1.0F},
                "MISSION GOAL ACHIEVED"
            );
        }

        ImGui::Separator();
        ImGui::TextDisabled("MISSION PROGRESS");
        draw_mission_progress(scenario, mission_visual_state);

        ImGui::Separator();
        ImGui::TextDisabled("SPACECRAFT INTERNALS");
        draw_actuator_internals(
            spacecraft_config,
            frame,
            actuator_visual_state
        );
    }
    ImGui::End();
}
void draw_help_window(bool& show_help) {
    if (!show_help) {
        return;
    }
    ImGui::SetNextWindowSize({410.0F, 240.0F}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Help", &show_help)) {
        ImGui::Text("Space   pause or resume");
        ImGui::Text("R       reset the selected scenario");
        ImGui::Text("O       reset the orbit camera");
        ImGui::Text("H       show or hide this help");
        ImGui::Separator();
        ImGui::Text("Right-drag   orbit the camera");
        ImGui::Text("Wheel        zoom");
        ImGui::Separator();
        ImGui::TextWrapped(
            "Mission: magnetorquers remove the initial tumble, then "
            "reaction wheels aim the solar arrays toward the Sun."
        );
    }
    ImGui::End();
}
void draw_control_panel(
    detumble::Simulation& simulation,
    detumble::MagneticControlCycle& magnetic_control,
    detumble::IdealGyroscope& gyroscope,
    detumble::IdealCoarseSunSensorArray& sun_sensors,
    detumble::AutonomousFlightSoftware& flight_software,
    detumble::ReactionWheelCluster& reaction_wheels,
    detumble::MagneticControlOutput& magnetic_output,
    ActuatorVisualState& actuator_visual_state,
    MissionVisualState& mission_visual_state,
    OrbitCamera& camera,
    bool& playing,
    float& playback_speed,
    double& time_accumulator_s,
    int& selected_scenario_index,
    int& requested_scenario_index,
    bool& show_help
) {
    ImGui::SetNextWindowPos({16.0F, 16.0F}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({360.0F, 175.0F}, ImGuiCond_Always);
    if (ImGui::Begin("Controls")) {
        if (ImGui::Button(playing ? "Pause" : "Resume")) {
            playing = !playing;
        }

        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            playing = false;
            time_accumulator_s = 0.0;
            simulation.reset();
            magnetic_control.reset();
            gyroscope.reset();
            sun_sensors.reset();
            flight_software.reset();
            reaction_wheels.reset();
            magnetic_output = {};
            actuator_visual_state.reset();
            mission_visual_state.reset();
        }

        ImGui::SliderFloat(
            "Simulation speed",
            &playback_speed,
            0.1F,
            100.0F,
            "%.1fx",
            ImGuiSliderFlags_Logarithmic
        );
        if (ImGui::BeginCombo(
                "Scenario",
                viewer_scenarios[static_cast<std::size_t>(
                    selected_scenario_index
                )].name
            )) {
            for (std::size_t index = 0;
                 index < viewer_scenarios.size();
                 ++index) {
                const bool selected =
                    static_cast<int>(index) == selected_scenario_index;
                if (ImGui::Selectable(
                        viewer_scenarios[index].name,
                        selected
                    )) {
                    requested_scenario_index = static_cast<int>(index);
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::TextDisabled(
            "%s",
            viewer_scenarios[static_cast<std::size_t>(
                selected_scenario_index
            )].description
        );

        if (ImGui::Button("Help (H)")) {
            show_help = !show_help;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset camera (O)")) {
            camera.orbit_overview();
        }
    }
    ImGui::End();
}
void draw_mission_progress(
    const ViewerScenario& scenario,
    const MissionVisualState& visual_state
) {
    const bool pointing_phase = visual_state.pointing_phase;
    const double goal = pointing_phase ? 2.0 : 0.5;

    if (!visual_state.value_valid) {
        ImGui::TextWrapped("Waiting for sensors to detect the Sun.");
        return;
    }

    const double value = visual_state.displayed_value;
    const double gauge_maximum = pointing_phase
        ? 10.0
        : std::max(scenario.initial_rate_deg_s, goal + 1.0);
    const double remaining_fraction = std::clamp(
        value / gauge_maximum,
        0.0,
        1.0
    );
    const bool goal_reached = value <= goal;
    const char* value_text = TextFormat(
        "%.2f deg%s",
        value,
        pointing_phase ? "" : "/s"
    );
    const float value_x = ImGui::GetWindowContentRegionMax().x
        - ImGui::CalcTextSize(value_text).x;
    ImGui::TextUnformatted(
        pointing_phase ? "Solar-array pointing error" : "Angular speed"
    );
    ImGui::SameLine(value_x);
    ImGui::TextColored(
        goal_reached
            ? ImVec4{0.30F, 0.90F, 0.45F, 1.0F}
            : ImVec4{0.85F, 0.88F, 0.92F, 1.0F},
        "%s",
        value_text
    );
    ImGui::PushStyleColor(
        ImGuiCol_PlotHistogram,
        goal_reached
            ? ImVec4{0.30F, 0.90F, 0.45F, 1.0F}
            : pointing_phase
                ? ImVec4{1.0F, 0.55F, 0.15F, 1.0F}
                : ImVec4{0.30F, 0.75F, 1.0F, 1.0F}
    );
    ImGui::ProgressBar(
        static_cast<float>(remaining_fraction),
        {-1.0F, 24.0F},
        ""
    );
    ImGui::PopStyleColor();
    if (pointing_phase) {
        ImGui::TextDisabled("Goal: at or below %.1f deg", goal);
    } else if (goal_reached) {
        ImGui::TextDisabled("Goal reached; holding for the 30 s dwell time");
    } else {
        ImGui::TextDisabled("Goal: at or below %.1f deg/s", goal);
    }
}
}  // namespace

int main() {
    constexpr int initial_width = 1440;
    constexpr int initial_height = 900;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(
        initial_width,
        initial_height,
        "Detumble - CubeSat attitude viewer"
    );
    SetTargetFPS(120);
    rlImGuiSetup(true);
    ImGui::GetIO().IniFilename = nullptr;

    SpacecraftModel spacecraft = load_spacecraft_model();
    const detumble::SpacecraftVisualConfig spacecraft_visual_config =
        detumble::generic_3u_visual_config();
    detumble::Simulation simulation;
    const detumble::EnvironmentConfig environment_config;
    const detumble::SensorSuiteConfig sensors =
        detumble::realistic_sensor_suite_config(simulation.config().seed);
    detumble::MagneticControlCycle magnetic_control{sensors.magnetometer};
    detumble::IdealGyroscope gyroscope{sensors.gyroscope};
    detumble::IdealCoarseSunSensorArray sun_sensors{sensors.sun_sensor};
    detumble::AutonomousFlightSoftware flight_software;
    detumble::ReactionWheelCluster reaction_wheels;
    detumble::MagneticControlOutput magnetic_output;
    ActuatorVisualState actuator_visual_state;
    MissionVisualState mission_visual_state;
    OrbitCamera orbit_camera;

    bool playing = true;
    int selected_scenario_index = 0;
    int requested_scenario_index = -1;
    bool show_help = false;
    float playback_speed = viewer_scenarios.front().suggested_speed;
    double time_accumulator_s = 0.0;
    reset_viewer_scenario(
        viewer_scenarios.front(),
        simulation,
        magnetic_control,
        gyroscope,
        sun_sensors,
        flight_software,
        reaction_wheels,
        magnetic_output
    );

    while (!WindowShouldClose()) {
        if (requested_scenario_index >= 0) {
            selected_scenario_index = requested_scenario_index;
            requested_scenario_index = -1;
            const ViewerScenario& scenario = viewer_scenarios[
                static_cast<std::size_t>(selected_scenario_index)
            ];
            reset_viewer_scenario(
                scenario,
                simulation,
                magnetic_control,
                gyroscope,
                sun_sensors,
                flight_software,
                reaction_wheels,
                magnetic_output
            );
            actuator_visual_state.reset();
            mission_visual_state.reset();
            playback_speed = scenario.suggested_speed;
            time_accumulator_s = 0.0;
            playing = true;
        }

        if (!ImGui::GetIO().WantCaptureKeyboard) {
            if (IsKeyPressed(KEY_H)) {
                show_help = !show_help;
            }
            if (IsKeyPressed(KEY_SPACE)) {
                playing = !playing;
            }
            if (IsKeyPressed(KEY_R)) {
                requested_scenario_index = selected_scenario_index;
            }
            if (IsKeyPressed(KEY_O)) {
                orbit_camera.orbit_overview();
            }
        }

        orbit_camera.update();
        const double frame_time_s = static_cast<double>(GetFrameTime());

        if (playing) {
            time_accumulator_s += frame_time_s
                * static_cast<double>(playback_speed);
            while (time_accumulator_s >= simulation.config().time_step_s) {
                magnetic_output = advance_simulation(
                    simulation,
                    environment_config,
                    magnetic_control,
                    gyroscope,
                    sun_sensors,
                    flight_software,
                    reaction_wheels
                );
                time_accumulator_s -= simulation.config().time_step_s;
            }
        }

        const detumble::EnvironmentState live_environment =
            current_environment(simulation, environment_config);
        const ViewerFrame display_frame = capture_viewer_frame(
            simulation,
            live_environment,
            flight_software,
            magnetic_output,
            reaction_wheels.telemetry(),
            sun_sensors.last_measurement()
        );
        mission_visual_state.update(display_frame, frame_time_s);
        actuator_visual_state.update(
            display_frame,
            magnetic_control.magnetorquer_config(),
            reaction_wheels.config(),
            frame_time_s,
            playing
        );
        const detumble::EnvironmentState& environment =
            display_frame.environment;
        const Eigen::Vector3d spacecraft_origin = orbit_position_display(
            environment_config,
            environment
        );
        constexpr double spacecraft_visual_scale = 0.14;
        const Eigen::Quaterniond& body_to_inertial =
            display_frame.body_to_inertial;
        const Camera3D camera = orbit_camera.camera();
        BeginDrawing();
        ClearBackground(Color{3, 7, 15, 255});
        draw_starfield();

        BeginMode3D(camera);
        draw_orbit_path(environment_config, display_frame.time_s);
        draw_earth();
        draw_sun_marker(environment.sun_direction_inertial);
        draw_spacecraft(
            spacecraft,
            body_to_inertial,
            spacecraft_origin,
            spacecraft_visual_scale
        );
        if (!spacecraft.loaded_custom_glb) {
            draw_body_outline(
                spacecraft_visual_config,
                body_to_inertial,
                spacecraft_origin,
                spacecraft_visual_scale
            );
        }
        draw_sun_sensor_status(
            spacecraft_visual_config,
            display_frame.sun_measurement,
            body_to_inertial,
            spacecraft_origin,
            spacecraft_visual_scale
        );
        EndMode3D();

        rlImGuiBegin();
        draw_control_panel(
            simulation,
            magnetic_control,
            gyroscope,
            sun_sensors,
            flight_software,
            reaction_wheels,
            magnetic_output,
            actuator_visual_state,
            mission_visual_state,
            orbit_camera,
            playing,
            playback_speed,
            time_accumulator_s,
            selected_scenario_index,
            requested_scenario_index,
            show_help
        );
        draw_scenario_status_panel(
            viewer_scenarios[static_cast<std::size_t>(
                selected_scenario_index
            )],
            display_frame,
            spacecraft_visual_config,
            actuator_visual_state,
            mission_visual_state
        );
        draw_help_window(show_help);
        rlImGuiEnd();

        EndDrawing();
    }

    UnloadModel(spacecraft.model);
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
