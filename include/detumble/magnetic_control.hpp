#pragma once

#include <optional>

#include <Eigen/Core>

#include "detumble/magnetometer.hpp"
#include "detumble/magnetorquer.hpp"

namespace detumble {

struct MagneticControlOutput {
    Eigen::Vector3d commanded_dipole_body_A_m2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d limited_dipole_body_A_m2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d applied_dipole_body_A_m2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d applied_torque_body_Nm{Eigen::Vector3d::Zero()};
    std::optional<MagnetometerMeasurement> magnetometer_measurement;
    bool torquers_disabled_for_measurement{};
};

class MagneticControlCycle {
public:
    explicit MagneticControlCycle(
        MagnetometerConfig magnetometer_config = {},
        MagnetorquerConfig magnetorquer_config = {}
    );

    void reset();

    [[nodiscard]] MagneticControlOutput update(
        double elapsed_time_s,
        const Eigen::Vector3d& magnetic_field_body_T,
        const Eigen::Vector3d& commanded_dipole_body_A_m2
    );

    [[nodiscard]] const IdealMagnetometer& magnetometer() const;
    [[nodiscard]] const MagnetorquerConfig& magnetorquer_config() const;

private:
    IdealMagnetometer magnetometer_;
    MagnetorquerConfig magnetorquer_config_;
};

}  // namespace detumble
