#pragma once

#include <cstdint>
#include <limits>
#include <random>

#include <Eigen/Core>

namespace detumble {

[[nodiscard]] double seeded_gaussian_sample(
    std::mt19937_64& generator,
    double standard_deviation
);

struct VectorSensorErrorConfig {
    std::uint64_t seed{};
    Eigen::Vector3d noise_standard_deviation{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bias_standard_deviation{Eigen::Vector3d::Zero()};
    Eigen::Vector3d quantization_step{Eigen::Vector3d::Zero()};
    Eigen::Vector3d saturation_limit{
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::infinity()
        )
    };
};

class VectorSensorErrorModel {
public:
    explicit VectorSensorErrorModel(VectorSensorErrorConfig config = {});

    void reset();

    [[nodiscard]] Eigen::Vector3d apply(
        const Eigen::Vector3d& true_value
    );
    [[nodiscard]] const VectorSensorErrorConfig& config() const;
    [[nodiscard]] const Eigen::Vector3d& bias() const;

private:
    VectorSensorErrorConfig config_;
    std::mt19937_64 generator_;
    Eigen::Vector3d bias_{Eigen::Vector3d::Zero()};
};

}  // namespace detumble
