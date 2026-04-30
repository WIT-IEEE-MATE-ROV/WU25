#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <util/quaternion_pid.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <util/pid_controller.hpp>

#include "thrusters/thruster_data.hpp"

using namespace std::chrono_literals;

class Thrusters {
public:
    using PWMValue = uint16_t;
    using PCAOutputs = std::array<PWMValue, 8>;
    using ThrusterDecomp = Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<float, 3, 4> >;

    class ThrusterOutputs {
    public:
        Eigen::Vector4f horizontal_;
        Eigen::Vector4f vertical_;

        ThrusterOutputs() = default;

        explicit ThrusterOutputs(const Eigen::Vector<float, 8> &outputs);

        ThrusterOutputs(Eigen::Vector4f horizontal, Eigen::Vector4f vertical);

        ThrusterOutputs(const Eigen::Solve<ThrusterDecomp, Eigen::Vector3f> &horizontal,
                        const Eigen::Solve<ThrusterDecomp, Eigen::Vector3f> &vertical);

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

        float &operator()(const Eigen::Index i) {
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
        Eigen::Vector3f linear_;
        // RPY
        Eigen::Vector3f angular_;

        ThrustVector() {
            linear_ = Eigen::Vector3f::Zero();
            angular_ = Eigen::Vector3f::Zero();
        }

        explicit ThrustVector(const Eigen::Vector<float, 6> &vec) {
            for (int i = 0; i < 3; i++) {
                linear_[i] = vec[i];
            }
            for (int i = 3; i < 6; i++) {
                angular_[i - 3] = vec[i];
            }
        }

        ThrustVector(const Eigen::Vector3f &linear, const Eigen::Vector3f &angular) {
            linear_ = linear;
            angular_ = angular;
        }

        [[nodiscard]] ThrusterOutputs GetThrusterOutputs(const ThrusterDecomp &horizontal_decomp,
                                                         const ThrusterDecomp &vertical_decomp) const;

        float operator[](const Eigen::Index i) {
            assert(i < 6);

            if (i >= 3) {
                return angular_[i - 3];
            }

            return linear_[i];
        }

        float operator[](const Eigen::Index i) const {
            assert(i < 6);

            if (i >= 3) {
                return angular_[i - 3];
            }

            return linear_[i];
        }

        float operator[](const int i) const {
            return operator[](static_cast<Eigen::Index>(i));
        }

        float operator[](const int i) {
            return operator[](static_cast<Eigen::Index>(i));
        }
    };

    Thrusters();

    ~Thrusters();

    void Init();

    ThrusterOutputs Update();

    void SetHoldIdleRotation(bool enabled);

    void SetAngVelControl(bool enabled);

    void SetHoldIdleDepth(bool enabled);

    void SetDepthLock(bool enabled);

    void SetThrustVector(const ThrustVector &thrust_vector);

    void SetRotation(const Eigen::Quaternionf &q);

    void SetDesiredRotation(const Eigen::Quaternionf &q);

    void SetRotationPIDGains(float p, float i, float d,
                             float i_zone = std::numeric_limits<float>::max(),
                             float max_output = std::numeric_limits<float>::max());

    void SetDepthPIDGains(float kp, float ki, float kd);

    static Eigen::Vector3f CalculateAngVel(const Eigen::Quaternionf &q1, const Eigen::Quaternionf &q2, float dt);

    static float Thrusters::CalculateInclination(const Eigen::Quaternionf& rot);

    static float Thrusters::CalculateInclination(const Eigen::Vector3f& plane);

    [[nodiscard]] std::array<PWMValue, 8> GetPWMOutputs(const ThrusterOutputs &thruster_outputs) const;

private:
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

    const float MAX_THRUST_KGF = 2.4f;
    const float MAX_ANGVEL_X_RPS = 90 * M_PIf / 180.f;
    const float MAX_ANGVEL_Y_RPS = 90 * M_PIf / 180.f;
    const float MAX_ANGVEL_Z_RPS = 90 * M_PIf / 180.f;

    constexpr float ZERO_THRESHOLD = 0.0001f;
    constexpr float DEPTH_COMMAND_THRESHOLD_DEG = 3.5f;


    //    const std::string DATA_PATH = "/home/foamstein/ros2_ws/src/WU25/data/T200-Public-Performance-Data.csv";
    const std::string DATA_PATH = ament_index_cpp::get_package_share_directory("wu25")
                                  + "/data/T200-Public-Performance-Data.csv";

    ThrusterData thruster_data_;
    ThrustVector thrust_vector_;

    Eigen::Quaternionf current_rotation_{1, 0, 0, 0};
    Eigen::Quaternionf desired_rotation_{1, 0, 0, 0};
    Eigen::Quaternionf previous_rotation_{1, 0, 0, 0};

    ThrusterDecomp decomp_horizontal_;
    ThrusterDecomp decomp_vertical_;

    QuatPIDController::PIDParams x_params_{.p = 1.0f};
    QuatPIDController::PIDParams y_params_{.p = 1.0f};
    QuatPIDController::PIDParams z_params_{.p = 1.0f};
    QuatPIDController idle_rotation_controller_;

    PIDController x_omega_controller_{0, 0, 0};
    PIDController y_omega_controller_{0, 0, 0};
    PIDController z_omega_controller_{0, 0, 0};
    PIDController idle_depth_controller_{0, 0, 0};

    std::chrono::high_resolution_clock::time_point rotation_recieved_time_ns_;
    std::chrono::high_resolution_clock::time_point previous_rotation_recieved_time_;

    std::array<float, 8> pwm_outputs_{};

    float current_depth{0};

    bool hold_idle_rotation_{false};
    bool ang_vel_control_{false};
    bool hold_idle_depth_{false};
    bool depth_lock_{false};

    bool currently_depthing_{false};
    bool currently_rotating_{false};

    float idle_depth_setpoint_{0};
};
