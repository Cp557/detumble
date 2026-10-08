#pragma once

#include <Eigen/Core>
#include <array>
#include <raylib.h>

namespace detumble::viewer {

inline constexpr std::array<Color, 3> rod_colors{
    Color{180, 55, 72, 255}, Color{0, 119, 99, 255}, Color{40, 98, 177, 255}};

// Label boxes in render-texture pixels; the UI centers its own readable font.
std::array<Rectangle, 3> rod_label_boxes(int width, int height);

// Draw into the caller's active render texture. Hardware stays in body coordinates.
void draw_rod_schematic(const Eigen::Vector3d &activity, int width, int height);

} // namespace detumble::viewer
