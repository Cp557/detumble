#pragma once
#include "detumble/magnetic_field.hpp"
#include "detumble/orbit.hpp"
#include <Eigen/Geometry>
namespace detumble {
struct EnvironmentConfig {
    CircularOrbitConfig orbit;
    MagneticFieldConfig magnetic_field;
};
struct EnvironmentState {
    OrbitState orbit;
    Eigen::Vector3d magnetic_field_inertial_T{Eigen::Vector3d::Zero()};
    Eigen::Vector3d magnetic_field_body_T{Eigen::Vector3d::Zero()};
};
class Environment {
  public:
    explicit Environment(EnvironmentConfig config = {});
    [[nodiscard]] Eigen::Vector3d reference_field_inertial_T(double time_s) const;
    [[nodiscard]] EnvironmentState sample(const Eigen::Quaterniond &attitude,
                                          double time_s) const;
    [[nodiscard]] const EnvironmentConfig &config() const {
        return config_;
    }

  private:
    EnvironmentConfig config_;
    MagneticField field_;
};
[[nodiscard]] EnvironmentState
sample_environment(const EnvironmentConfig &config,
                   const Eigen::Quaterniond &body_to_inertial, double elapsed_time_s);
} // namespace detumble
