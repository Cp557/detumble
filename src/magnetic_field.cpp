#include "detumble/magnetic_field.hpp"

#include "igrf14_coefficients.hpp"
#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>

#include <stdexcept>

namespace detumble {
namespace {

void validate_config(const TiltedDipoleConfig &config) {
    if (!std::isfinite(config.reference_radius_m) || config.reference_radius_m <= 0.0) {
        throw std::invalid_argument{
            "Magnetic reference radius must be finite and positive"};
    }
    if (!std::isfinite(config.equatorial_surface_field_T) ||
        config.equatorial_surface_field_T <= 0.0) {
        throw std::invalid_argument{
            "Equatorial magnetic field must be finite and positive"};
    }
    if (!std::isfinite(config.tilt_rad) || config.tilt_rad < 0.0 ||
        config.tilt_rad > std::numbers::pi) {
        throw std::invalid_argument{
            "Magnetic dipole tilt must be between zero and pi radians"};
    }
    if (!std::isfinite(config.earth_rotation_rate_rad_s) ||
        !std::isfinite(config.initial_rotation_phase_rad)) {
        throw std::invalid_argument{"Earth rotation rate and phase must be finite"};
    }
}

} // namespace

Eigen::Vector3d magnetic_dipole_axis_inertial(const TiltedDipoleConfig &config,
                                              const double elapsed_time_s) {
    validate_config(config);
    if (!std::isfinite(elapsed_time_s)) {
        throw std::invalid_argument{"Magnetic-field time must be finite"};
    }

    const double rotation_phase_rad =
        std::remainder(config.initial_rotation_phase_rad +
                           config.earth_rotation_rate_rad_s * elapsed_time_s,
                       2.0 * std::numbers::pi);
    const double horizontal_component = std::sin(config.tilt_rad);

    return {horizontal_component * std::cos(rotation_phase_rad),
            horizontal_component * std::sin(rotation_phase_rad),
            -std::cos(config.tilt_rad)};
}

Eigen::Vector3d
tilted_dipole_field_inertial_T(const TiltedDipoleConfig &config,
                               const Eigen::Vector3d &position_inertial_m,
                               const double elapsed_time_s) {
    validate_config(config);
    if (!position_inertial_m.allFinite()) {
        throw std::invalid_argument{"Magnetic-field position must be finite"};
    }

    const double radius_m = position_inertial_m.norm();
    if (radius_m < config.reference_radius_m) {
        throw std::invalid_argument{
            "Magnetic-field position must be at or above Earth surface"};
    }

    const Eigen::Vector3d radial_direction = position_inertial_m / radius_m;
    const Eigen::Vector3d dipole_axis =
        magnetic_dipole_axis_inertial(config, elapsed_time_s);
    const double radial_scale = std::pow(config.reference_radius_m / radius_m, 3.0);

    return config.equatorial_surface_field_T * radial_scale *
           (3.0 * dipole_axis.dot(radial_direction) * radial_direction - dipole_axis);
}

} // namespace detumble

namespace detumble {
namespace {
std::chrono::sys_seconds parse_epoch(const std::string &epoch) {
    constexpr std::string_view pattern{"0000-00-00T00:00:00Z"};
    if (epoch.size() != pattern.size())
        throw std::invalid_argument{"Epoch must be YYYY-MM-DDTHH:MM:SSZ"};
    for (std::size_t i = 0; i < epoch.size(); ++i) {
        if (pattern[i] == '0' ? (epoch[i] < '0' || epoch[i] > '9')
                              : epoch[i] != pattern[i])
            throw std::invalid_argument{"Epoch must be YYYY-MM-DDTHH:MM:SSZ"};
    }
    const int year = std::stoi(epoch.substr(0, 4));
    const int month = std::stoi(epoch.substr(5, 2));
    const int day = std::stoi(epoch.substr(8, 2));
    const int hour = std::stoi(epoch.substr(11, 2));
    const int minute = std::stoi(epoch.substr(14, 2));
    const int second = std::stoi(epoch.substr(17, 2));
    if (hour > 23 || minute > 59 || second > 59)
        throw std::invalid_argument{"Invalid epoch time"};
    const std::chrono::year_month_day date{
        std::chrono::year{year}, std::chrono::month{static_cast<unsigned>(month)},
        std::chrono::day{static_cast<unsigned>(day)}};
    if (!date.ok())
        throw std::invalid_argument{"Invalid epoch date"};
    const auto value = std::chrono::sys_days{date} + std::chrono::hours{hour} +
                       std::chrono::minutes{minute} + std::chrono::seconds{second};
    if (value < std::chrono::sys_days{std::chrono::year{2025} / 1 / 1} ||
        value > std::chrono::sys_days{std::chrono::year{2030} / 1 / 1}) {
        throw std::invalid_argument{"IGRF scenarios support 2025 through 2030"};
    }
    return value;
}
} // namespace
std::string_view to_string(const MagneticFieldModel model) {
    switch (model) {
    case MagneticFieldModel::dipole:
        return "dipole";
    case MagneticFieldModel::igrf14:
        return "igrf14";
    }
    throw std::invalid_argument{"Invalid magnetic field model"};
}
MagneticFieldModel parse_field_model(const std::string_view name) {
    if (name == "dipole")
        return MagneticFieldModel::dipole;
    if (name == "igrf14")
        return MagneticFieldModel::igrf14;
    throw std::invalid_argument{"Field model must be dipole or igrf14"};
}
// Schmidt quasi-normal recurrence adapted from the official igrf14syn routine.
// Its north/east/down components are converted to Earth-fixed Cartesian here.
Eigen::Vector3d igrf14_field_earth_fixed_T(const Eigen::Vector3d &position,
                                           const double date) {
    const double radius = position.norm();
    if (!position.allFinite() || !std::isfinite(radius) || radius < 6'371'000.0 ||
        !std::isfinite(date) || date < 2025.0 || date > 2030.0) {
        throw std::invalid_argument{
            "IGRF requires a surface-or-higher position and 2025-2030 date"};
    }
    const double ct = position.z() / radius;
    const double st = std::hypot(position.x(), position.y()) / radius;
    const double longitude = std::atan2(position.y(), position.x());
    const double cl = std::cos(longitude), sl = std::sin(longitude);
    std::array<double, 14> cos_m{}, sin_m{};
    cos_m[0] = 1.0;
    for (std::size_t m = 1; m < cos_m.size(); ++m) {
        cos_m[m] = cos_m[m - 1] * cl - sin_m[m - 1] * sl;
        sin_m[m] = sin_m[m - 1] * cl + cos_m[m - 1] * sl;
    }
    const double ratio = 6'371'200.0 / radius;
    double radial_scale = ratio * ratio;
    std::array<double, 106> p{}, q{};
    p[1] = 1.0;
    p[3] = st;
    q[3] = ct;
    double north{}, east{}, down{};
    for (int n = 1; n <= 13; ++n) {
        radial_scale *= ratio;
        for (int m = 0; m <= n; ++m) {
            const auto k = static_cast<std::size_t>(n * (n + 1) / 2 + m + 1);
            if (n == m) {
                if (n != 1) {
                    const double factor = std::sqrt(1.0 - 0.5 / m);
                    const auto j = k - static_cast<std::size_t>(n + 1);
                    p[k] = factor * st * p[j];
                    q[k] = factor * (st * q[j] + ct * p[j]);
                }
            } else {
                const double one = std::sqrt(static_cast<double>(n * n - m * m));
                const double two =
                    std::sqrt(static_cast<double>((n - 1) * (n - 1) - m * m)) / one;
                const double three = (2.0 * n - 1.0) / one;
                const auto i = k - static_cast<std::size_t>(n);
                const auto j = i - static_cast<std::size_t>(n - 1);
                p[k] = three * ct * p[i] - two * p[j];
                q[k] = three * (ct * q[i] - st * p[i]) - two * q[j];
            }
            const auto &coefficient = detail::igrf14_coefficients[k - 2];
            const double g =
                (coefficient.g + (date - 2025.0) * coefficient.dg) * radial_scale;
            const double h =
                (coefficient.h + (date - 2025.0) * coefficient.dh) * radial_scale;
            const double cosine = cos_m[static_cast<std::size_t>(m)];
            const double sine = sin_m[static_cast<std::size_t>(m)];
            const double field = g * cosine + h * sine;
            north += field * q[k];
            down -= (n + 1.0) * field * p[k];
            if (m != 0) {
                const double polar_limit = st > 1e-10 ? m * p[k] / st : q[k] * ct;
                east += (g * sine - h * cosine) * polar_limit;
            }
        }
    }
    return 1e-9 * (north * Eigen::Vector3d{-ct * cl, -ct * sl, st} +
                   east * Eigen::Vector3d{-sl, cl, 0.0} +
                   down * Eigen::Vector3d{-st * cl, -st * sl, -ct});
}
MagneticField::MagneticField(MagneticFieldConfig config)
    : config_{std::move(config)}, epoch_{parse_epoch(config_.epoch)} {
    static_cast<void>(to_string(config_.model));
    static_cast<void>(magnetic_dipole_axis_inertial(config_.dipole, 0.0));
}
Eigen::Vector3d MagneticField::sample_inertial_T(const Eigen::Vector3d &position,
                                                 const double time_s) const {
    if (!std::isfinite(time_s) || time_s < 0.0) {
        throw std::invalid_argument{"Field time must be finite and nonnegative"};
    }
    // Continuous UTC-like scenario time; leap seconds/precession are not modeled.
    const auto time = epoch_ + std::chrono::duration_cast<std::chrono::seconds>(
                                   std::chrono::duration<double>{time_s});
    const auto day = std::chrono::floor<std::chrono::days>(time);
    const auto year = std::chrono::year_month_day{day}.year();
    const auto start = std::chrono::sys_days{year / 1 / 1};
    const auto next = std::chrono::sys_days{(year + std::chrono::years{1}) / 1 / 1};
    const double date =
        static_cast<int>(year) +
        (std::chrono::duration<double>{epoch_ - start}.count() + time_s) /
            std::chrono::duration<double>{next - start}.count();
    if (date > 2030.0)
        throw std::invalid_argument{"Scenario exceeds IGRF date range"};
    if (config_.model == MagneticFieldModel::dipole) {
        return tilted_dipole_field_inertial_T(config_.dipole, position, time_s);
    }
    const Eigen::AngleAxisd earth_rotation{
        config_.dipole.earth_rotation_rate_rad_s * time_s, Eigen::Vector3d::UnitZ()};
    const Eigen::Vector3d fixed = earth_rotation.inverse() * position;
    return earth_rotation * igrf14_field_earth_fixed_T(fixed, date);
}
} // namespace detumble
