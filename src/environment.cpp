#include "detumble/environment.hpp"
#include "detumble/frames.hpp"
namespace detumble {
Environment::Environment(EnvironmentConfig config)
    : config_{std::move(config)}, field_{config_.magnetic_field} {
    static_cast<void>(circular_orbit_state(config_.orbit, 0.0));
}
Eigen::Vector3d Environment::reference_field_inertial_T(const double time_s) const {
    return field_.sample_inertial_T(
        circular_orbit_state(config_.orbit, time_s).position_inertial_m, time_s);
}
EnvironmentState Environment::sample(const Eigen::Quaterniond &attitude,
                                     const double time_s) const {
    const auto orbit = circular_orbit_state(config_.orbit, time_s);
    const auto field = field_.sample_inertial_T(orbit.position_inertial_m, time_s);
    return {orbit, field, rotate_inertial_to_body(attitude, field)};
}
EnvironmentState sample_environment(const EnvironmentConfig &config,
                                    const Eigen::Quaterniond &attitude,
                                    const double time_s) {
    return Environment{config}.sample(attitude, time_s);
}
} // namespace detumble
