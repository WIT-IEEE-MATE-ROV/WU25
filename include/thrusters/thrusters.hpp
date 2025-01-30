#pragma once

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "thrusters/thruster_data.hpp"

using namespace Eigen;

class Thrusters {
public:
    using PWMValue = uint16_t;
    using ThrusterDecomp = CompleteOrthogonalDecomposition<Matrix<float, 3, 4> >;

    class ThrusterOutputs {
    public:
        Vector4f horizontal_;
        Vector4f vertical_;

        ThrusterOutputs() = default;

        explicit ThrusterOutputs(const Vector<float, 8> &outputs);

        ThrusterOutputs(Vector4f horizontal, Vector4f vertical);

        ThrusterOutputs(const Solve<ThrusterDecomp, Vector3f> &horizontal,
                        const Solve<ThrusterDecomp, Vector3f> &vertical);

        ~ThrusterOutputs();

        void Desaturate(float max_thrust_kgf);

        std::string ToString();

        [[nodiscard]] float flh() const {
            return horizontal_[0];
        }

        [[nodiscard]] float frh() const {
            return horizontal_[1];
        }

        [[nodiscard]] float blh() const {
            return horizontal_[2];
        }

        [[nodiscard]] float brh() const {
            return horizontal_[3];
        }

        [[nodiscard]] float flv() const {
            return vertical_[0];
        }

        [[nodiscard]] float frv() const {
            return vertical_[1];
        }

        [[nodiscard]] float blv() const {
            return vertical_[2];
        }

        [[nodiscard]] float brv() const {
            return vertical_[3];
        }

        float &operator[](size_t i);

        float operator[](size_t i) const;

        float &operator()(const Index i) {
            return operator[](static_cast<size_t>(i));
        }

        float operator()(const Index i) const {
            return operator[](static_cast<size_t>(i));
        }

        float &operator[](const int i) {
            return operator[](static_cast<size_t>(i));
        }

        float operator[](const int i) const {
            return operator[](static_cast<size_t>(i));
        }
    };

    class ThrustVector {
    public:
        // XYZ
        Vector<float, 3> linear_;
        // RPY
        Vector<float, 3> angular_;

        ThrustVector() {
            linear_ = Vector<float, 3>::Zero();
            angular_ = Vector<float, 3>::Zero();
        }

        explicit ThrustVector(const Vector<float, 6> &vec) {
            for (int i = 0; i < 3; i++) {
                linear_[i] = vec[i];
            }
            for (int i = 3; i < 6; i++) {
                angular_[i - 3] = vec[i];
            }
        }

        ThrustVector(const Vector<float, 3> &linear, const Vector<float, 3> &angular) {
            linear_ = linear;
            angular_ = angular;
        }

        [[nodiscard]] ThrusterOutputs GetThrusterOutputs(const ThrusterDecomp &horizontal_decomp,
                                                         const ThrusterDecomp &vertical_decomp) const;

        float operator[](const Index i) {
            assert(i < 6);

            if (i >= 3) {
                return angular_[i - 3];
            }

            return linear_[i];
        }

        float operator[](const Index i) const {
            assert(i < 6);

            if (i >= 3) {
                return angular_[i - 3];
            }

            return linear_[i];
        }

        float operator[](const int i) const {
            return operator[](static_cast<Index>(i));
        }

        float operator[](const int i) {
            return operator[](static_cast<Index>(i));
        }
    };

    Thrusters();

    ~Thrusters();

    void Init();

    ThrusterOutputs Update();

    void SetThrustVector(const ThrustVector &thrust_vector);

    void SetRotation(const Quaternionf &q);

    void SetDesiredRotation(const Quaternionf &q);

private:
    void GetPWMOutputs(const ThrusterOutputs &thruster_outputs, std::array<PWMValue, 8> &pwm_outputs) const;

    void PlotPWMVsThrust();

    void PlotThrustVsPWM();

    const float THRUSTER_LENGTH_HORIZONTAL_M = 0.381;
    const float THRUSTER_WIDTH_HORIZONTAL_M = 0.3175;

    const float THRUSTER_DIAGONAL_HORIZONTAL_M =
            sqrtf(std::pow(THRUSTER_LENGTH_HORIZONTAL_M, 2) + std::pow(THRUSTER_WIDTH_HORIZONTAL_M, 2));

    const float HALF_DIAGONAL_HORIZONTAL_M = THRUSTER_DIAGONAL_HORIZONTAL_M / 2.f;

    const float LENGTH_DIAGONAL_ANGLE_HORIZONTAL_RAD = -std::acos(
        THRUSTER_LENGTH_HORIZONTAL_M / THRUSTER_DIAGONAL_HORIZONTAL_M);

    const float THRUSTER_LENGTH_VERTICAL_M = 0.2032;
    const float THRUSTER_WIDTH_VERTICAL_M = 0.3429;
    const float THRUSTER_DIAGONAL_VERTICAL_M =
            sqrtf(std::pow(THRUSTER_LENGTH_VERTICAL_M, 2) + std::pow(THRUSTER_WIDTH_VERTICAL_M, 2));

    const float HALF_LENGTH_VERTICAL_M = THRUSTER_LENGTH_VERTICAL_M / 2.f;
    const float HALF_WIDTH_VERTICAL_M = THRUSTER_WIDTH_VERTICAL_M / 2.f;

    const float THRUSTER_ANGLE_RAD = 40.f * M_PI / 180.f;

    const std::string DATA_PATH = "/home/foamstein/ros2_ws/src/WU25/data/T200-Public-Performance-Data.csv";

    ThrusterData thruster_data_;
    ThrustVector thrust_vector_;
    Quaternionf current_rotation_{1, 0, 0, 0};
    Quaternionf desired_rotation_{1, 0, 0, 0};
    ThrusterDecomp decomp_horizontal_;
    ThrusterDecomp decomp_vertical_;
    std::array<float, 8> pwm_outputs_{};
    // TODO: Change to axis-angle for PID
};
