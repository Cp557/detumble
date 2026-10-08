#include "rod_view.hpp"
#include "detumble/spacecraft_visual.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <raymath.h>

namespace detumble::viewer {
namespace {
constexpr Camera3D cutaway_camera{.position = {0.7F, -1.0F, 0.50F},
                                  .target = {0, 0, 0},
                                  .up = {0, 0, 1},
                                  .fovy = 0.43F,
                                  .projection = CAMERA_ORTHOGRAPHIC};

Vector3 vector(const Eigen::Vector3d &value) {
    return {static_cast<float>(value.x()), static_cast<float>(value.y()),
            static_cast<float>(value.z())};
}

Color shade(Color color, Vector3 normal) {
    const Vector3 light = Vector3Normalize({-0.4F, -0.6F, 0.8F});
    const float brightness =
        0.58F + 0.42F * std::max(0.0F, Vector3DotProduct(normal, light));
    return {static_cast<unsigned char>(color.r * brightness),
            static_cast<unsigned char>(color.g * brightness),
            static_cast<unsigned char>(color.b * brightness), color.a};
}

void quad(const std::array<Vector3, 4> &points, Vector3 normal, Color color) {
    const Color lit = shade(color, normal);
    DrawTriangle3D(points[0], points[1], points[2], lit);
    DrawTriangle3D(points[0], points[2], points[3], lit);
}

void box(Vector3 center, Vector3 size, Color color) {
    const Vector3 half = Vector3Scale(size, 0.5F);
    const float x0 = center.x - half.x, x1 = center.x + half.x;
    const float y0 = center.y - half.y, y1 = center.y + half.y;
    const float z0 = center.z - half.z, z1 = center.z + half.z;
    quad({Vector3{x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}}, {1, 0, 0},
         color);
    quad({Vector3{x0, y1, z0}, {x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}}, {-1, 0, 0},
         color);
    quad({Vector3{x1, y1, z0}, {x0, y1, z0}, {x0, y1, z1}, {x1, y1, z1}}, {0, 1, 0},
         color);
    quad({Vector3{x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}}, {0, -1, 0},
         color);
    quad({Vector3{x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}}, {0, 0, 1},
         color);
    quad({Vector3{x0, y1, z0}, {x1, y1, z0}, {x1, y0, z0}, {x0, y0, z0}}, {0, 0, -1},
         color);
}

void cylinder(Vector3 center, Vector3 axis, float length, float radius, Color color) {
    constexpr int segments = 24;
    const Vector3 reference =
        std::abs(axis.z) < 0.9F ? Vector3{0, 0, 1} : Vector3{0, 1, 0};
    const Vector3 u = Vector3Normalize(Vector3CrossProduct(reference, axis));
    const Vector3 v = Vector3CrossProduct(axis, u);
    const Vector3 start = Vector3Subtract(center, Vector3Scale(axis, length * 0.5F));
    const Vector3 end = Vector3Add(center, Vector3Scale(axis, length * 0.5F));
    for (int i = 0; i < segments; ++i) {
        const float a = 2.0F * PI * static_cast<float>(i) / segments;
        const float b = 2.0F * PI * static_cast<float>(i + 1) / segments;
        const Vector3 radial_a =
            Vector3Add(Vector3Scale(u, std::cos(a)), Vector3Scale(v, std::sin(a)));
        const Vector3 radial_b =
            Vector3Add(Vector3Scale(u, std::cos(b)), Vector3Scale(v, std::sin(b)));
        const Vector3 s0 = Vector3Add(start, Vector3Scale(radial_a, radius));
        const Vector3 s1 = Vector3Add(start, Vector3Scale(radial_b, radius));
        const Vector3 e0 = Vector3Add(end, Vector3Scale(radial_a, radius));
        const Vector3 e1 = Vector3Add(end, Vector3Scale(radial_b, radius));
        quad({s0, s1, e1, e0}, Vector3Normalize(Vector3Add(radial_a, radial_b)), color);
        DrawTriangle3D(start, s1, s0, shade(color, Vector3Negate(axis)));
        DrawTriangle3D(end, e0, e1, shade(color, axis));
    }
}

void wire(Vector3 start, Vector3 end, Color color) {
    const Vector3 direction = Vector3Subtract(end, start);
    const float length = Vector3Length(direction);
    if (length > 0.0F)
        cylinder(Vector3Scale(Vector3Add(start, end), 0.5F),
                 Vector3Scale(direction, 1.0F / length), length, 0.0007F, color);
}

void body_outline(const Eigen::Vector3d &dimensions) {
    const Vector3 half = Vector3Scale(vector(dimensions), 0.5F);
    constexpr Color outline{155, 172, 190, 255};
    for (const float x : {-half.x, half.x})
        for (const float y : {-half.y, half.y})
            wire({x, y, -half.z}, {x, y, half.z}, outline);
    for (const float z : {-half.z, half.z}) {
        for (const float y : {-half.y, half.y})
            wire({-half.x, y, z}, {half.x, y, z}, outline);
        for (const float x : {-half.x, half.x})
            wire({x, -half.y, z}, {x, half.y, z}, outline);
    }
}

void rod(const RodComponent &component, Color accent, float activity) {
    const Vector3 center = vector(component.center_body_m);
    const Vector3 axis = vector(component.axis_body);
    const float length = static_cast<float>(component.length_m);
    const float radius = static_cast<float>(component.radius_m);
    const float strength = std::clamp(std::abs(activity), 0.0F, 1.0F);
    const Color sleeve = strength > 1e-10F ? ColorLerp({75, 88, 106, 255}, accent,
                                                       0.45F + 0.55F * strength)
                                           : Color{100, 113, 130, 255};
    cylinder(center, axis, length, radius, sleeve);
    // End collars suggest a coil assembly without adding obstructing electronics.
    for (const float fraction : {-0.44F, 0.44F})
        cylinder(Vector3Add(center, Vector3Scale(axis, length * fraction)), axis,
                 0.004F, radius * 1.12F, {56, 68, 84, 255});
}
} // namespace

std::array<Rectangle, 3> rod_label_boxes(int width, int height) {
    const auto config = generic_3u_visual_config();
    const float pixel_scale = static_cast<float>(height) / 300.0F;
    std::array<Rectangle, 3> labels{};
    for (std::size_t i = 0; i < labels.size(); ++i) {
        const auto &component = config.magnetorquers[i];
        const double sign = i == 1 ? -1.0 : 1.0;
        const Vector2 tip = GetWorldToScreenEx(
            vector(component.center_body_m +
                   sign * 0.5 * component.length_m * component.axis_body),
            cutaway_camera, width, height);
        labels[i] = {static_cast<float>(width) * (i == 0 ? 0.80F : 0.20F) -
                         4.0F * pixel_scale,
                     tip.y - (i == 2 ? 20.0F : 11.0F) * pixel_scale,
                     20.0F * pixel_scale, 20.0F * pixel_scale};
    }
    return labels;
}

void draw_rod_schematic(const Eigen::Vector3d &activity, int width, int height) {
    const auto config = generic_3u_visual_config();
    ClearBackground({219, 228, 238, 255});
    BeginMode3D(cutaway_camera);
    body_outline(config.body_dimensions_m);
    for (std::size_t i = 0; i < config.magnetorquers.size(); ++i)
        rod(config.magnetorquers[i], rod_colors[i],
            static_cast<float>(
                std::clamp(activity[static_cast<Eigen::Index>(i)], -1.0, 1.0)));
    const Vector3 sensor = vector(config.magnetometer.center_body_m);
    const float sensor_size = static_cast<float>(config.magnetometer.side_length_m);
    wire({0.04F, sensor.y, sensor.z}, sensor, {140, 155, 174, 255});
    box(sensor, {sensor_size, sensor_size, sensor_size}, {155, 55, 128, 255});
    box({sensor.x + 0.51F * sensor_size, sensor.y, sensor.z},
        {0.001F, 0.75F * sensor_size, 0.75F * sensor_size}, {236, 178, 218, 255});
    EndMode3D();
    const float pixel_scale = static_cast<float>(height) / 300.0F;
    const auto labels = rod_label_boxes(width, height);
    for (std::size_t i = 0; i < config.magnetorquers.size(); ++i) {
        const auto &component = config.magnetorquers[i];
        // Y's negative tip is visible from this fixed camera.
        const double sign = i == 1 ? -1.0 : 1.0;
        const Vector2 tip = GetWorldToScreenEx(
            vector(component.center_body_m +
                   sign * 0.5 * component.length_m * component.axis_body),
            cutaway_camera, width, height);
        // Put labels outside the frame so rails cannot obscure the lettering.
        const Rectangle label = labels[i];
        const Vector2 leader_end{label.x + (i == 0 ? 0.0F : label.width),
                                 label.y + label.height * 0.5F};
        DrawLineEx(tip, leader_end, pixel_scale, rod_colors[i]);
        DrawCircleV(tip, 2.0F * pixel_scale,
                    std::abs(activity[static_cast<Eigen::Index>(i)]) > 1e-10
                        ? rod_colors[i]
                        : Color{100, 113, 130, 255});
        DrawRectangleRounded(label, 0.2F, 4, {232, 238, 245, 255});
    }
    const Vector2 sensor_tip =
        GetWorldToScreenEx(sensor, cutaway_camera, width, height);
    DrawCircleV(sensor_tip, 3.0F * pixel_scale, {171, 57, 140, 255});
}
} // namespace detumble::viewer
