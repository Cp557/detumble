#pragma once

#include <chrono>
#include <numbers>
#include <string>
#include <string_view>

#include <Eigen/Core>

namespace detumble {

struct TiltedDipoleConfig {
    double reference_radius_m{6'371'000.0};
    double equatorial_surface_field_T{3.12e-5};
    double tilt_rad{11.0 * std::numbers::pi / 180.0};
    double earth_rotation_rate_rad_s{7.2921150e-5};
    double initial_rotation_phase_rad{};
};

[[nodiscard]] Eigen::Vector3d
magnetic_dipole_axis_inertial(const TiltedDipoleConfig &config, double elapsed_time_s);

[[nodiscard]] Eigen::Vector3d
tilted_dipole_field_inertial_T(const TiltedDipoleConfig &config,
                               const Eigen::Vector3d &position_inertial_m,
                               double elapsed_time_s);

} // namespace detumble

// IGRF uses geocentric coordinates and SI units at this public boundary.
namespace detumble {
enum class MagneticFieldModel { dipole, igrf14 };
[[nodiscard]] std::string_view to_string(MagneticFieldModel model);
[[nodiscard]] MagneticFieldModel parse_field_model(std::string_view name);
struct MagneticFieldConfig {
    MagneticFieldModel model{MagneticFieldModel::igrf14};
    std::string epoch{"2025-01-01T00:00:00Z"};
    TiltedDipoleConfig dipole;
};
[[nodiscard]] Eigen::Vector3d
igrf14_field_earth_fixed_T(const Eigen::Vector3d &position_earth_fixed_m,
                           double decimal_year);
class MagneticField {
  public:
    explicit MagneticField(MagneticFieldConfig config = {});
    [[nodiscard]] Eigen::Vector3d
    sample_inertial_T(const Eigen::Vector3d &position_inertial_m,
                      double elapsed_time_s) const;
    [[nodiscard]] const MagneticFieldConfig &config() const {
        return config_;
    }

  private:
    MagneticFieldConfig config_;
    std::chrono::sys_seconds epoch_;
};
} // namespace detumble
