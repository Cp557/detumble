#include "detumble/mission.hpp"
#include "detumble/spacecraft_visual.hpp"
#include "rod_view.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <imgui.h>
#include <numbers>
#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>
#include <string>
#include <string_view>
namespace {
constexpr double playback_speed = 20.0;
constexpr double earth_display_radius = 2.0;
constexpr double orbit_display_altitude = 0.65;
constexpr double meters_to_spacecraft_display_units = 10.0;
constexpr float camera_transition_rate = 7.0F;
constexpr float initial_camera_radius = 10.5F;
Vector3 to_raylib(const Eigen::Vector3d &v) {
    return {static_cast<float>(v.x()), static_cast<float>(v.y()),
            static_cast<float>(v.z())};
}
struct Scenario {
    const char *name;
    std::uint64_t seed;
    double rate_deg_s;
};
constexpr std::array<Scenario, 3> scenarios{
    {{"Gentle", 7, 5.0}, {"Nominal", 42, 10.0}, {"Fast", 2025, 15.0}}};
detumble::MissionRunConfig scenario_config(int index) {
    detumble::MissionRunConfig config;
    const auto &scenario = scenarios[static_cast<std::size_t>(index)];
    config.seed = scenario.seed;
    config.initial_rate_rad_s = scenario.rate_deg_s * std::numbers::pi / 180.0;
    return config;
}
struct OrbitCamera {
    float azimuth_rad{0.75F};
    float elevation_rad{0.35F};
    float radius{initial_camera_radius};
    float target_azimuth_rad{azimuth_rad};
    float target_elevation_rad{elevation_rad};
    float target_radius{radius};

    [[nodiscard]] Camera3D camera() const {
        const float horizontal_radius = radius * std::cos(elevation_rad);
        return {.position = {horizontal_radius * std::cos(azimuth_rad),
                             radius * std::sin(elevation_rad),
                             horizontal_radius * std::sin(azimuth_rad)},
                .target = {0.0F, 0.0F, 0.0F},
                .up = {0.0F, 1.0F, 0.0F},
                .fovy = 45.0F,
                .projection = CAMERA_PERSPECTIVE};
    }

    void update() {
        const float frame_time_s = std::clamp(GetFrameTime(), 0.0F, 0.1F);
        const float blend = 1.0F - std::exp(-camera_transition_rate * frame_time_s);
        azimuth_rad += blend * (target_azimuth_rad - azimuth_rad);
        elevation_rad += blend * (target_elevation_rad - elevation_rad);
        radius += blend * (target_radius - radius);

        if (ImGui::GetIO().WantCaptureMouse) {
            return;
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            const Vector2 delta = GetMouseDelta();
            azimuth_rad -= delta.x * 0.008F;
            elevation_rad = std::clamp(elevation_rad + delta.y * 0.008F, -1.35F, 1.35F);
            target_azimuth_rad = azimuth_rad;
            target_elevation_rad = elevation_rad;
        }

        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0F) {
            radius = std::clamp(radius * std::exp(-wheel * 0.12F), 4.5F, 14.0F);
            target_radius = radius;
        }
    }
};

struct SpacecraftModel {
    Model model{};
    float uniform_display_scale{1.0F};
    bool loaded_glb{};
};

SpacecraftModel load_spacecraft_model() {
    SpacecraftModel spacecraft;
    const auto model_directory =
        std::filesystem::path{GetApplicationDirectory()} / "assets/models";
    for (const char *filename : {"icecube_3u.glb", "detumble_3u.glb"}) {
        const auto path = model_directory / filename;
        spacecraft.model = LoadModel(path.string().c_str());
        spacecraft.loaded_glb = IsModelValid(spacecraft.model);
        if (spacecraft.loaded_glb) {
            TraceLog(LOG_INFO, "MODEL: Using %s for the orbital spacecraft", filename);
            break;
        }
        TraceLog(LOG_WARNING, "Could not load %s; trying fallback",
                 path.string().c_str());
        UnloadModel(spacecraft.model);
    }
    if (!spacecraft.loaded_glb) {
        spacecraft.model = LoadModelFromMesh(GenMeshCube(1.0F, 1.0F, 1.0F));
    }

    const BoundingBox bounds = GetModelBoundingBox(spacecraft.model);
    const Vector3 dimensions = Vector3Subtract(bounds.max, bounds.min);
    if (spacecraft.loaded_glb) {
        spacecraft.uniform_display_scale =
            static_cast<float>(meters_to_spacecraft_display_units);
        TraceLog(LOG_INFO, "MODEL: Spacecraft bounds %.4f x %.4f x %.4f m",
                 dimensions.x, dimensions.y, dimensions.z);
        if (dimensions.z <= std::max(dimensions.x, dimensions.y)) {
            TraceLog(LOG_WARNING,
                     "MODEL: expected body +Z to be the model's long axis");
        }
    } else {
        const Vector3 center = Vector3Scale(Vector3Add(bounds.min, bounds.max), 0.5F);
        spacecraft.model.transform = MatrixTranslate(-center.x, -center.y, -center.z);
    }
    return spacecraft;
}

void draw_spacecraft(const SpacecraftModel &spacecraft,
                     const Eigen::Quaterniond &body_to_inertial,
                     const Eigen::Vector3d &origin_inertial,
                     const double visual_scale) {
    const Quaternion rotation{static_cast<float>(body_to_inertial.x()),
                              static_cast<float>(body_to_inertial.y()),
                              static_cast<float>(body_to_inertial.z()),
                              static_cast<float>(body_to_inertial.w())};
    Vector3 rotation_axis{};
    float rotation_angle_rad{};
    QuaternionToAxisAngle(rotation, &rotation_axis, &rotation_angle_rad);
    const float rotation_angle_deg = rotation_angle_rad * 180.0F / PI;

    const float model_scale =
        static_cast<float>(spacecraft.uniform_display_scale * visual_scale);
    const Vector3 draw_scale =
        spacecraft.loaded_glb ? Vector3{model_scale, model_scale, model_scale}
                              : Vector3{model_scale, model_scale, 3.4F * model_scale};
    DrawModelEx(spacecraft.model, to_raylib(origin_inertial), rotation_axis,
                rotation_angle_deg, draw_scale, WHITE);
}

Eigen::Vector3d orbit_position_display(const detumble::EnvironmentConfig &,
                                       const detumble::EnvironmentState &environment) {
    return (earth_display_radius + orbit_display_altitude) *
           environment.orbit.position_inertial_m.normalized();
}

void draw_orbit_path(const detumble::EnvironmentConfig &config,
                     const double current_time_s) {
    constexpr int segment_count = 180;
    const double period_s = detumble::circular_orbit_period_s(config.orbit);
    const double display_radius = earth_display_radius + orbit_display_altitude;
    const double current_phase = std::fmod(current_time_s / period_s, 1.0);

    Eigen::Vector3d previous =
        display_radius * detumble::circular_orbit_state(config.orbit, 0.0)
                             .position_inertial_m.normalized();
    for (int segment = 1; segment <= segment_count; ++segment) {
        const double elapsed_time_s = period_s * static_cast<double>(segment) /
                                      static_cast<double>(segment_count);
        const Eigen::Vector3d current =
            display_radius *
            detumble::circular_orbit_state(config.orbit, elapsed_time_s)
                .position_inertial_m.normalized();
        const double segment_phase =
            static_cast<double>(segment) / static_cast<double>(segment_count);
        const double age_fraction = std::fmod(current_phase - segment_phase + 1.0, 1.0);
        const bool recent_trail = age_fraction < 0.18;
        const unsigned char alpha =
            recent_trail
                ? static_cast<unsigned char>(235.0 - 155.0 * age_fraction / 0.18)
                : 55;
        DrawLine3D(to_raylib(previous), to_raylib(current), Color{80, 175, 235, alpha});
        previous = current;
    }
}

void draw_earth() {
    DrawSphere({}, earth_display_radius, Color{24, 66, 120, 255});
    DrawSphereWires({}, static_cast<float>(earth_display_radius * 1.002), 18, 36,
                    Color{70, 125, 180, 180});
}

void draw_starfield(const int width, const int height) {
    if (width <= 0 || height <= 0) {
        return;
    }

    std::uint32_t state = 0x5A17C9E3U;
    constexpr int star_count = 260;
    for (int index = 0; index < star_count; ++index) {
        state = 1'664'525U * state + 1'013'904'223U;
        const int x = static_cast<int>(state % static_cast<std::uint32_t>(width));
        state = 1'664'525U * state + 1'013'904'223U;
        const int y = static_cast<int>(state % static_cast<std::uint32_t>(height));
        state = 1'664'525U * state + 1'013'904'223U;
        const unsigned char brightness =
            static_cast<unsigned char>(105U + state % 130U);
        const Color color{
            brightness, static_cast<unsigned char>(std::min(255, brightness + 8)),
            static_cast<unsigned char>(std::min(255, brightness + 18)), 210};
        if (index % 23 == 0) {
            DrawCircle(x, y, 1.4F, color);
        } else {
            DrawPixel(x, y, color);
        }
    }
}

void apply_sidebar_palette() {
    ImGui::StyleColorsLight();
    auto &style = ImGui::GetStyle();
    style.FrameRounding = 3.0F;
    style.FontSizeBase = 15.3F;
    auto &colors = style.Colors;
    colors[ImGuiCol_WindowBg] = {0.90F, 0.925F, 0.95F, 1.0F};
    colors[ImGuiCol_Text] = {0.12F, 0.18F, 0.26F, 1.0F};
    colors[ImGuiCol_TextDisabled] = colors[ImGuiCol_Text];
    colors[ImGuiCol_FrameBg] = {0.78F, 0.83F, 0.89F, 1.0F};
    colors[ImGuiCol_Button] = {0.79F, 0.85F, 0.92F, 1.0F};
    colors[ImGuiCol_ButtonHovered] = {0.69F, 0.78F, 0.88F, 1.0F};
    colors[ImGuiCol_ButtonActive] = {0.60F, 0.72F, 0.85F, 1.0F};
    colors[ImGuiCol_Separator] = {0.66F, 0.72F, 0.80F, 1.0F};
    colors[ImGuiCol_PlotHistogram] = {0.16F, 0.38F, 0.67F, 1.0F};
}

bool resize_view(RenderTexture2D &view, int width, int height) {
    if (view.texture.width == width && view.texture.height == height)
        return true;
    if (IsRenderTextureValid(view))
        UnloadRenderTexture(view);
    view = LoadRenderTexture(width, height);
    if (!IsRenderTextureValid(view)) {
        TraceLog(LOG_ERROR, "Unable to allocate a viewer render texture");
        return false;
    }
    SetTextureFilter(view.texture, TEXTURE_FILTER_BILINEAR);
    return true;
}

// Presentation only: physical measurements and commands remain full rate.
struct ViewerReadout {
    Eigen::Vector3d rod_activity{Eigen::Vector3d::Zero()};
    double time_s{};
    double rate_deg_s{};
    double last_refresh_real_s{-1.0};

    void step(const detumble::MissionTelemetrySample &sample, double dt) {
        // Half a real second at 20x keeps the applied-strength bars readable.
        const double blend = -std::expm1(-dt / 10.0);
        const Eigen::Vector3d strength =
            sample.magnetic.applied_dipole_body_A_m2.cwiseAbs() / 0.2;
        rod_activity += blend * (strength - rod_activity);
    }

    void refresh(const detumble::MissionTelemetrySample &sample, double real_time_s) {
        if (last_refresh_real_s < 0.0 || real_time_s - last_refresh_real_s >= 0.5) {
            time_s = sample.time_s;
            rate_deg_s = sample.truth.angular_velocity_body_rad_s.norm() * 180.0 /
                         std::numbers::pi;
            last_refresh_real_s = real_time_s;
        }
    }
};

void draw_rod_dimensions(const detumble::RodComponent &component) {
    const Eigen::Vector3d center_mm = 1000.0 * component.center_body_m;
    ImGui::Text("Center: (%.0f, %.0f, %.0f) mm", center_mm.x(), center_mm.y(),
                center_mm.z());
    ImGui::Text("Length: %.0f mm | Diameter: %.0f mm", 1000.0 * component.length_m,
                2000.0 * component.radius_m);
}

void draw_rods(const ViewerReadout &readout, const RenderTexture2D &cutaway) {
    const ImVec2 image_origin = ImGui::GetCursorScreenPos();
    // Downsample the 2x render for smooth cylinder edges and frame rails.
    rlImGuiImageRect(&cutaway.texture, cutaway.texture.width / 2,
                     cutaway.texture.height / 2,
                     {0.0F, 0.0F, static_cast<float>(cutaway.texture.width),
                      -static_cast<float>(cutaway.texture.height)});
    const auto labels = detumble::viewer::rod_label_boxes(cutaway.texture.width,
                                                          cutaway.texture.height);
    const ImU32 text_color = ImGui::GetColorU32(ImGuiCol_Text);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        const char label[]{static_cast<char>('X' + i), '\0'};
        const ImVec2 text_size = ImGui::CalcTextSize(label);
        ImGui::GetWindowDrawList()->AddText(
            {image_origin.x + (labels[i].x + labels[i].width * 0.5F) * 0.5F -
                 text_size.x * 0.5F,
             image_origin.y + (labels[i].y + labels[i].height * 0.5F) * 0.5F -
                 text_size.y * 0.5F},
            text_color, label);
    }
    const auto config = detumble::generic_3u_visual_config();
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        const auto &component = config.magnetorquers[static_cast<std::size_t>(axis)];
        const auto color = ColorNormalize(
            detumble::viewer::rod_colors[static_cast<std::size_t>(axis)]);
        ImGui::PushID(static_cast<int>(axis));
        ImGui::Text("%c", static_cast<int>('X' + axis));
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            draw_rod_dimensions(component);
            ImGui::EndTooltip();
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                              ImVec4{color.x, color.y, color.z, color.w});
        ImGui::ProgressBar(
            static_cast<float>(std::clamp(readout.rod_activity[axis], 0.0, 1.0)),
            {-1.0F, 12.0F}, "");
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            draw_rod_dimensions(component);
            ImGui::Separator();
            ImGui::TextUnformatted("Recent average applied magnetic strength\n"
                                   "Full bar: 0.2 A m^2 | Not electrical power");
            ImGui::EndTooltip();
        }
        ImGui::PopID();
    }
    ImGui::TextUnformatted("Pink: magnetometer");
    if (ImGui::IsItemHovered()) {
        const Eigen::Vector3d center_mm = 1000.0 * config.magnetometer.center_body_m;
        ImGui::SetTooltip("Magnetometer center: (%.0f, %.0f, %.0f) mm", center_mm.x(),
                          center_mm.y(), center_mm.z());
    }
}

void draw_panel(const detumble::DetumbleMission &mission, const ViewerReadout &readout,
                const RenderTexture2D &cutaway, bool &playing, int &selected,
                bool &restart, float width, float height) {
    ImGui::SetNextWindowPos({0.0F, 0.0F}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({width, height}, ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {16.0F, 16.0F});
    constexpr auto flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                           ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("##Sidebar", nullptr, flags)) {
        const auto &sample = mission.telemetry();
        ImGui::Text("Time: %.0f s  |  20x playback", readout.time_s);
        ImGui::Text("Rotation: %.1f deg/s", readout.rate_deg_s);
        ImGui::ProgressBar(
            static_cast<float>(
                std::clamp(readout.rate_deg_s /
                               scenarios[static_cast<std::size_t>(selected)].rate_deg_s,
                           0.0, 1.0)),
            {-1.0F, 16.0F}, "");
        if (sample.flight.mode == detumble::FlightMode::safe)
            ImGui::TextWrapped("Waiting for magnetic measurements");
        else if (sample.goal_achieved)
            ImGui::TextUnformatted("Detumble goal achieved");
        else
            ImGui::TextWrapped("Goal: below 0.5 deg/s for 30 seconds");
        ImGui::Separator();
        ImGui::TextUnformatted("Magnetic hardware");
        draw_rods(readout, cutaway);
        ImGui::Spacing();
        constexpr float button_height = 32.0F;
        const float controls_height =
            2.0F * button_height + 2.0F * ImGui::GetStyle().ItemSpacing.y + 1.0F;
        ImGui::SetCursorPosY(
            std::max(ImGui::GetCursorPosY(), height - 16.0F - controls_height));
        ImGui::Separator();
        const float button_width = (ImGui::GetContentRegionAvail().x -
                                    2.0F * ImGui::GetStyle().ItemSpacing.x) /
                                   3.0F;
        for (std::size_t i = 0; i < scenarios.size(); ++i) {
            if (i > 0)
                ImGui::SameLine();
            const bool active = selected == static_cast<int>(i);
            if (active) {
                ImGui::PushStyleColor(ImGuiCol_Button, {0.53F, 0.68F, 0.85F, 1.0F});
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                      {0.48F, 0.64F, 0.82F, 1.0F});
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                      {0.43F, 0.59F, 0.77F, 1.0F});
            }
            if (ImGui::Button(scenarios[i].name, {button_width, button_height})) {
                selected = static_cast<int>(i);
                restart = true;
            }
            if (active)
                ImGui::PopStyleColor(3);
        }
        const float playback_button_width =
            (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2.0F;
        if (ImGui::Button(playing ? "Pause" : "Play",
                          {playback_button_width, button_height}))
            playing = !playing;
        ImGui::SameLine();
        if (ImGui::Button("Reset", {playback_button_width, button_height}))
            restart = true;
    }
    ImGui::End();
    ImGui::PopStyleVar(3);
}
} // namespace
int main(int argc, char **argv) {
    int screenshot_frame = -1;
    std::string screenshot_path;
    int window_width = 1440;
    int window_height = 900;
    // Capture a frame for visual QA; ordinary launches remain interactive.
    if ((argc == 4 || argc == 6) && std::string_view{argv[1]} == "--screenshot") {
        screenshot_path = argv[2];
        screenshot_frame = std::stoi(argv[3]);
        if (argc == 6) {
            window_width = std::max(1200, std::stoi(argv[4]));
            window_height = std::max(720, std::stoi(argv[5]));
        }
    }
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(window_width, window_height, "Detumble");
    SetWindowMinSize(1200, 720);
    SetTargetFPS(60);
    rlImGuiSetup(true);
    apply_sidebar_palette();
    ImGui::GetIO().IniFilename = nullptr;
    auto spacecraft = load_spacecraft_model();
    int selected = 1;
    bool playing = true, restart = false;
    ViewerReadout readout;
    double accumulator = 0.0;
    OrbitCamera orbit_camera;
    detumble::DetumbleMission mission{scenario_config(selected)};
    int frame_index{};
    int exit_code{};
    RenderTexture2D orbital_view{};
    RenderTexture2D rod_view{};
    while (!WindowShouldClose()) {
        if (restart) {
            mission = detumble::DetumbleMission{scenario_config(selected)};
            accumulator = 0.0;
            readout = {};
            restart = false;
        }
        if (!ImGui::GetIO().WantCaptureKeyboard) {
            if (IsKeyPressed(KEY_SPACE))
                playing = !playing;
            if (IsKeyPressed(KEY_R))
                restart = true;
        }
        orbit_camera.update();
        if (playing) {
            accumulator += std::clamp(static_cast<double>(GetFrameTime()), 0.0, 0.1) *
                           playback_speed;
            // Budget each frame without discarding accumulated physical steps.
            unsigned steps{};
            while (accumulator >= mission.config().time_step_s && steps++ < 500) {
                mission.step();
                readout.step(mission.telemetry(), mission.config().time_step_s);
                accumulator -= mission.config().time_step_s;
            }
        }
        const auto &sample = mission.telemetry();
        readout.refresh(sample, GetTime());
        const int width = std::max(1, GetScreenWidth());
        const int height = std::max(1, GetScreenHeight());
        const int sidebar_width = width / 4;
        const int scene_width = width - sidebar_width;
        const int cutaway_width = std::max(1, sidebar_width - 32);
        const int cutaway_height = std::max(1, std::min(460, height - 430));
        if (!resize_view(orbital_view, scene_width, height) ||
            !resize_view(rod_view, cutaway_width * 2, cutaway_height * 2)) {
            exit_code = 1;
            break;
        }
        BeginTextureMode(rod_view);
        detumble::viewer::draw_rod_schematic(
            readout.rod_activity, rod_view.texture.width, rod_view.texture.height);
        EndTextureMode();
        // A separate render target gives the 3D camera the right-hand area's aspect
        // ratio and center, rather than cropping a full-window view behind the UI.
        BeginTextureMode(orbital_view);
        ClearBackground({3, 7, 15, 255});
        draw_starfield(scene_width, height);
        BeginMode3D(orbit_camera.camera());
        draw_orbit_path(mission.config().environment, sample.time_s);
        draw_earth();
        draw_spacecraft(
            spacecraft, sample.truth.body_to_inertial,
            orbit_position_display(mission.config().environment, sample.environment),
            0.14);
        EndMode3D();
        EndTextureMode();
        BeginDrawing();
        ClearBackground({3, 7, 15, 255});
        DrawTexturePro(
            orbital_view.texture,
            {0.0F, 0.0F, static_cast<float>(scene_width), -static_cast<float>(height)},
            {static_cast<float>(sidebar_width), 0.0F, static_cast<float>(scene_width),
             static_cast<float>(height)},
            {}, 0.0F, WHITE);
        rlImGuiBegin();
        draw_panel(mission, readout, rod_view, playing, selected, restart,
                   static_cast<float>(sidebar_width), static_cast<float>(height));
        rlImGuiEnd();
        EndDrawing();
        if (frame_index++ == screenshot_frame) {
            TakeScreenshot(screenshot_path.c_str());
            break;
        }
    }
    if (IsRenderTextureValid(orbital_view))
        UnloadRenderTexture(orbital_view);
    if (IsRenderTextureValid(rod_view))
        UnloadRenderTexture(rod_view);
    UnloadModel(spacecraft.model);
    rlImGuiShutdown();
    CloseWindow();
    return exit_code;
}
