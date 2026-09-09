#include "detumble/magnetic_control.hpp"

#include <utility>

namespace detumble {

MagneticControlCycle::MagneticControlCycle(
    MagnetometerConfig magnetometer_config,
    MagnetorquerConfig magnetorquer_config
)
    : magnetometer_{std::move(magnetometer_config)},
      magnetorquer_config_{std::move(magnetorquer_config)} {
    static_cast<void>(saturate_magnetorquer_dipole_body_A_m2(
        magnetorquer_config_,
        Eigen::Vector3d::Zero()
    ));
}

void MagneticControlCycle::reset() {
    magnetometer_.reset();
}

MagneticControlOutput MagneticControlCycle::update(
    const double elapsed_time_s,
    const Eigen::Vector3d& magnetic_field_body_T,
    const Eigen::Vector3d& commanded_dipole_body_A_m2
) {
    const bool sample_due = magnetometer_.sample_due(elapsed_time_s);
    const std::optional<MagnetometerMeasurement> measurement =
        magnetometer_.sample_if_due(
            elapsed_time_s,
            magnetic_field_body_T,
            !sample_due
        );
    const Eigen::Vector3d limited_dipole_body_A_m2 =
        saturate_magnetorquer_dipole_body_A_m2(
            magnetorquer_config_,
            commanded_dipole_body_A_m2
        );
    const Eigen::Vector3d applied_dipole_body_A_m2 = sample_due
        ? Eigen::Vector3d::Zero()
        : limited_dipole_body_A_m2;

    return {
        .commanded_dipole_body_A_m2 = commanded_dipole_body_A_m2,
        .limited_dipole_body_A_m2 = limited_dipole_body_A_m2,
        .applied_dipole_body_A_m2 = applied_dipole_body_A_m2,
        .applied_torque_body_Nm = magnetic_torque_body_Nm(
            applied_dipole_body_A_m2,
            magnetic_field_body_T
        ),
        .magnetometer_measurement = measurement,
        .torquers_disabled_for_measurement = sample_due
    };
}

const IdealMagnetometer& MagneticControlCycle::magnetometer() const {
    return magnetometer_;
}

const MagnetorquerConfig& MagneticControlCycle::magnetorquer_config() const {
    return magnetorquer_config_;
}

}  // namespace detumble
