#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "detumble/environment.hpp"
#include "detumble/magnetic_field.hpp"
#include "detumble/orbit.hpp"

namespace {

void require_vector_approx(const Eigen::Vector3d &actual,
                           const Eigen::Vector3d &expected,
                           const double margin = 1.0e-12) {
    for (Eigen::Index index = 0; index < actual.size(); ++index) {
        REQUIRE(actual[index] == Catch::Approx(expected[index]).margin(margin));
    }
}

} // namespace

TEST_CASE("untilted dipole has expected equatorial and polar fields") {
    detumble::TiltedDipoleConfig config;
    config.tilt_rad = 0.0;
    config.earth_rotation_rate_rad_s = 0.0;

    const Eigen::Vector3d equatorial_field = detumble::tilted_dipole_field_inertial_T(
        config, {config.reference_radius_m, 0.0, 0.0}, 0.0);
    const Eigen::Vector3d north_pole_field = detumble::tilted_dipole_field_inertial_T(
        config, {0.0, 0.0, config.reference_radius_m}, 0.0);

    require_vector_approx(equatorial_field,
                          {0.0, 0.0, config.equatorial_surface_field_T});
    require_vector_approx(north_pole_field,
                          {0.0, 0.0, -2.0 * config.equatorial_surface_field_T});
}

TEST_CASE("dipole field magnitude decreases with inverse radius cubed") {
    detumble::TiltedDipoleConfig config;
    config.tilt_rad = 0.0;
    config.earth_rotation_rate_rad_s = 0.0;
    const double orbit_radius_m = config.reference_radius_m + 500'000.0;

    const Eigen::Vector3d field = detumble::tilted_dipole_field_inertial_T(
        config, {orbit_radius_m, 0.0, 0.0}, 0.0);
    const double expected_magnitude_T =
        config.equatorial_surface_field_T *
        std::pow(config.reference_radius_m / orbit_radius_m, 3.0);

    REQUIRE(field.norm() == Catch::Approx(expected_magnitude_T).epsilon(1.0e-12));
}

TEST_CASE("environment transforms inertial magnetic field into body frame") {
    const detumble::EnvironmentConfig config;
    const Eigen::Quaterniond body_to_inertial{
        Eigen::AngleAxisd{std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()}};

    const detumble::EnvironmentState environment =
        detumble::sample_environment(config, body_to_inertial, 100.0);
    const Eigen::Vector3d reconstructed_inertial =
        body_to_inertial * environment.magnetic_field_body_T;

    require_vector_approx(reconstructed_inertial, environment.magnetic_field_inertial_T,
                          1.0e-15);
}

TEST_CASE("orbital magnetic field stays plausible and changes smoothly") {
    detumble::EnvironmentConfig config;
    config.magnetic_field.model = detumble::MagneticFieldModel::dipole;
    const double orbit_period_s = detumble::circular_orbit_period_s(config.orbit);
    constexpr int sample_count = 720;

    double minimum_magnitude_T = std::numeric_limits<double>::infinity();
    double maximum_magnitude_T = 0.0;
    double maximum_step_change_T = 0.0;
    Eigen::Vector3d previous_field = Eigen::Vector3d::Zero();

    for (int sample = 0; sample <= sample_count; ++sample) {
        const double elapsed_time_s = orbit_period_s * static_cast<double>(sample) /
                                      static_cast<double>(sample_count);
        const detumble::EnvironmentState environment = detumble::sample_environment(
            config, Eigen::Quaterniond::Identity(), elapsed_time_s);
        const double magnitude_T = environment.magnetic_field_inertial_T.norm();
        minimum_magnitude_T = std::min(minimum_magnitude_T, magnitude_T);
        maximum_magnitude_T = std::max(maximum_magnitude_T, magnitude_T);

        if (sample > 0) {
            maximum_step_change_T = std::max(
                maximum_step_change_T,
                (environment.magnetic_field_inertial_T - previous_field).norm());
        }
        previous_field = environment.magnetic_field_inertial_T;
    }

    REQUIRE(minimum_magnitude_T > 20.0e-6);
    REQUIRE(maximum_magnitude_T < 60.0e-6);
    REQUIRE(maximum_step_change_T < 1.0e-6);
}

TEST_CASE("tilted dipole rejects positions below Earth surface") {
    const detumble::TiltedDipoleConfig config;

    REQUIRE_THROWS_AS(
        detumble::tilted_dipole_field_inertial_T(config, Eigen::Vector3d::Zero(), 0.0),
        std::invalid_argument);
}

TEST_CASE("IGRF-14 agrees with independent official Fortran synthesis fixtures") {
    struct Fixture {
        double year, radius_km, latitude_deg, longitude_deg, north_nT, east_nT, down_nT;
    };
    // Generated with NOAA/BGS igrf14syn, isv=0, itype=2; stored independently of C++.
    const std::array<Fixture, 6> fixtures{
        {{2025, 6871, 0, 0, 21621.61069575, -1689.36488891, -10875.03039207},
         {2027.5, 6371, 45, -90, 17542.04723473, -838.98425769, 51167.43231603},
         {2026.25, 7471, -60, 140, 2545.02228665, 1246.89829721, -39503.19553621},
         {2025, 6871, 90, 0, 1047.92354097, 46.56305814, 46030.85688334},
         {2025, 6871, -90, 90, -6897.48251539, -10065.64128602, -41037.24712569},
         {2030, 6371, 10, 270, 27414.21314468, 43.44457619, 19828.26407336}}};
    for (const auto &f : fixtures) {
        const double latitude = f.latitude_deg * std::numbers::pi / 180.0;
        const double longitude = f.longitude_deg * std::numbers::pi / 180.0;
        const double ct = std::sin(latitude), st = std::cos(latitude),
                     cl = std::cos(longitude), sl = std::sin(longitude);
        const Eigen::Vector3d radial{st * cl, st * sl, ct};
        const Eigen::Vector3d field =
            detumble::igrf14_field_earth_fixed_T(1000.0 * f.radius_km * radial, f.year);
        const Eigen::Vector3d components{
            field.dot(Eigen::Vector3d{-ct * cl, -ct * sl, st}),
            field.dot(Eigen::Vector3d{-sl, cl, 0.0}), field.dot(-radial)};
        CAPTURE(f.year, f.latitude_deg, f.longitude_deg);
        REQUIRE(components.x() * 1e9 == Catch::Approx(f.north_nT).margin(0.02));
        REQUIRE(components.y() * 1e9 == Catch::Approx(f.east_nT).margin(0.02));
        REQUIRE(components.z() * 1e9 == Catch::Approx(f.down_nT).margin(0.02));
    }
}
TEST_CASE("IGRF epochs and Earth rotation use the documented frame convention") {
    detumble::MagneticFieldConfig config;
    detumble::MagneticField field{config};
    const Eigen::Vector3d position{6871000.0, 0.0, 0.0};
    REQUIRE(field.sample_inertial_T(position, 0.0)
                .isApprox(detumble::igrf14_field_earth_fixed_T(position, 2025), 1e-12));
    REQUIRE((field.sample_inertial_T(position, 3600.0) -
             field.sample_inertial_T(position, 0.0))
                .norm() > 1e-7);
    for (const auto &epoch :
         {"2024-12-31T00:00:00Z", "2031-01-01T00:00:00Z", "2025-02-30T00:00:00Z",
          "2025-01-01", "2025-+1-01T00:00:00Z", "2025-01-01T24:00:00Z"}) {
        config.epoch = epoch;
        REQUIRE_THROWS(detumble::MagneticField{config});
    }
    config.epoch = "2030-01-01T00:00:00Z";
    const detumble::MagneticField endpoint{config};
    REQUIRE_NOTHROW(endpoint.sample_inertial_T(position, 0.0));
    REQUIRE_THROWS(endpoint.sample_inertial_T(position, 1.0));
}
