#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iostream>
#include <bits/stdc++.h>
#include <cmath>

#include <matplot/freestanding/plot.h>

#include "thrusters/thrusters.hpp"
#include "thrusters/thruster_data.hpp"
#include "util/quaternion_pid.hpp"


Thrusters::Thrusters(): thruster_data_(DATA_PATH), idle_rotation_controller_(x_params_, y_params_, z_params_) {
    Vector4f horizontal_angles(
        THRUSTER_ANGLE_RAD, -THRUSTER_ANGLE_RAD, 2.f * M_PIf - THRUSTER_ANGLE_RAD, -(2.f * M_PIf - THRUSTER_ANGLE_RAD)
    );

    // Constrain angles to +-180
    for (float &angle: horizontal_angles) {
        angle = std::fmod(angle, M_PIf);
    }

    const float yaw_weight = HALF_DIAGONAL_HORIZONTAL_M * std::sin(
                                 THRUSTER_ANGLE_RAD - LENGTH_DIAGONAL_ANGLE_HORIZONTAL_RAD);

    Matrix<float, 3, 4> thruster_config_horizontal;
    thruster_config_horizontal.row(0) <<
            std::cos(horizontal_angles[0]),
            std::cos(horizontal_angles[1]),
            std::cos(horizontal_angles[2]),
            std::cos(horizontal_angles[3]);
    thruster_config_horizontal.row(1) <<
            std::sin(horizontal_angles[0]),
            std::sin(horizontal_angles[1]),
            std::sin(horizontal_angles[2]),
            std::sin(horizontal_angles[3]);
    thruster_config_horizontal.row(2) << yaw_weight, -yaw_weight, -yaw_weight, yaw_weight;

    Matrix<float, 3, 4> thruster_config_vertical;
    thruster_config_vertical.row(0) << 1.f, 1.f, 1.f, 1.f;
    thruster_config_vertical.row(1) <<
            HALF_LENGTH_VERTICAL_M, HALF_LENGTH_VERTICAL_M, -HALF_LENGTH_VERTICAL_M, -HALF_LENGTH_VERTICAL_M;
    thruster_config_vertical.row(2) <<
            HALF_WIDTH_VERTICAL_M, -HALF_WIDTH_VERTICAL_M, HALF_WIDTH_VERTICAL_M, -HALF_WIDTH_VERTICAL_M;

    decomp_horizontal_ = ThrusterDecomp(thruster_config_horizontal);
    decomp_vertical_ = ThrusterDecomp(thruster_config_vertical);

    x_params_ = {.p = 1};
    y_params_ = {.p = 1};
    z_params_ = {.p = 1};
    // rotation_controller_ = {x_params_, y_params_, z_params_};

    // X, Y, Yaw
    const Vector3f horizontal_vector(1, 0, 0);
    // Z, Pitch, Roll
    const Vector3f vertical_vector(1, 1, 0);

    const Solve horizontal_outputs = decomp_horizontal_.solve(horizontal_vector);
    const Solve vertical_outputs = decomp_vertical_.solve(vertical_vector);

    // std::cout << "Horizontal outputs: \n" << horizontal_outputs << std::endl;
    // std::cout << "Vertical outputs: \n" << vertical_outputs << std::endl;
    //
    // const auto horizontal_check = thruster_config_horizontal * horizontal_outputs;
    // std::cout << "Horizontal Check: \n" << horizontal_check << std::endl;
    //
    // const auto vertical_check = thruster_config_vertical * vertical_outputs;
    // std::cout << "Vertical Check: \n" << vertical_check << std::endl;

    // PlotPWMVsThrust();
}

Thrusters::ThrusterOutputs Thrusters::Update() {
    Vector3f depth_linear_output = {0, 0, 0};

    if (ang_vel_control_) {
        // Assume thrust vector angular is angular velocities in rad/s
        // Set PID omega controller setpoints
        x_omega_controller_.SetSetpoint(thrust_vector_.angular_.x());
        y_omega_controller_.SetSetpoint(thrust_vector_.angular_.y());
        z_omega_controller_.SetSetpoint(thrust_vector_.angular_.z());

        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>
                (rotation_recieved_time_ns_ - previous_rotation_recieved_time_).count();

        // Calculate current angular velocity
        Vector3f angVel = CalculateAngVel(previous_rotation_, current_rotation_, static_cast<float>(dt));

        // Set angular thrust vector to pid calculated kgf.
        thrust_vector_.angular_ = {
            x_omega_controller_.Calculate(angVel.x()),
            y_omega_controller_.Calculate(angVel.y()),
            z_omega_controller_.Calculate(angVel.z())
        };
    }

    if (depth_lock_) {
        auto rot_mat = current_rotation_.toRotationMatrix();
        Vector3f current_euler = rot_mat.canonicalEulerAngles(2, 1, 0);
        current_euler[0] = 0;

        AngleAxisf rotZ(current_euler[0], Vector3f::UnitZ());
        AngleAxisf rotY(current_euler[1], Vector3f::UnitY());
        AngleAxisf rotX(current_euler[2], Vector3f::UnitX());
        Quaternionf no_yaw = rotZ * rotY * rotX;

        Vector3f desired_direction{thrust_vector_.linear_.x(), thrust_vector_.linear_.y(), 0};

        thrust_vector_.linear_ += no_yaw * desired_direction;
    }


    // Apply rotation hold
    if (hold_idle_rotation_ && !currently_rotating_) {
        Vector3f holdRotOutput = idle_rotation_controller_.Calculate(current_rotation_);
        if (std::fabs(thrust_vector_.angular_.x()) < ZERO_THRESHOLD) {
            thrust_vector_.angular_.x() = holdRotOutput.x();
        }
        if (std::fabs(thrust_vector_.angular_.y()) < ZERO_THRESHOLD) {
            thrust_vector_.angular_.y() = holdRotOutput.y();
        }
        if (std::fabs(thrust_vector_.angular_.z()) < ZERO_THRESHOLD) {
            thrust_vector_.angular_.z() = holdRotOutput.z();
        }
    }

    if (hold_idle_depth_) {
        const float depth_command = idle_depth_controller_.Calculate(current_depth);
        const Vector3f depth_vec{0, 0, depth_command};
        thrust_vector_.linear_ += current_rotation_ * depth_vec;
    }

    ThrusterOutputs outputs = thrust_vector_.GetThrusterOutputs(decomp_horizontal_, decomp_vertical_);
    std::array<PWMValue, 8> pwms{};

    for (size_t i = 0; i < pwms.size(); i++) {
        pwms[i] = thruster_data_.ThrustToPWM(outputs[i]);
    }

    return outputs;
}

Thrusters::~Thrusters() = default;

void Thrusters::Init() {
}

// Hold rotation when no angular thrust vector is given
void Thrusters::SetHoldIdleRotation(const bool enabled) {
    hold_idle_rotation_ = enabled;
}

// Treat desired thrust vector angular component as angular velocities instead of net kgf moment
void Thrusters::SetAngVelControl(const bool enabled) {
    ang_vel_control_ = enabled;
}

// Hold depth when not commanding a thrust vector that would affect depth
void Thrusters::SetHoldIdleDepth(const bool enabled) {
    hold_idle_depth_ = enabled;
}

// Rotate thrust vectors to global XY plane only, as to not affect depth
void Thrusters::SetDepthLock(const bool enabled) {
    depth_lock_ = enabled;
}


void Thrusters::SetThrustVector(const ThrustVector &thrust_vector) {
    thrust_vector_ = thrust_vector;

    const float angular_norm = thrust_vector_.angular_.norm();

    const bool rotating_now = angular_norm > ZERO_THRESHOLD;
    if (!rotating_now) {
        thrust_vector_.angular_ = {0, 0, 0};
    }

    // If you stop rotating then set idle depth setpoint
    if (currently_rotating_ && !rotating_now) {
        idle_rotation_controller_.SetSetpoint(current_rotation_);
    }

    const bool depthing_now = CalculateInclination(thrust_vector_.linear_) > DEPTH_COMMAND_THRESHOLD_DEG;
    if (currently_depthing_ && !depthing_now) {
        idle_depth_controller_.SetSetpoint(current_depth);
    }

    currently_depthing_ = depthing_now;
    currently_rotating_ = rotating_now;
}

void Thrusters::SetRotation(const Quaternionf &q) {
    previous_rotation_recieved_time_ = rotation_recieved_time_ns_;
    rotation_recieved_time_ns_ = std::chrono::high_resolution_clock::now();
    current_rotation_ = q;
}

void Thrusters::SetDesiredRotation(const Quaternionf &q) {
    desired_rotation_ = q;
    idle_rotation_controller_.SetSetpoint(q);
}

Vector3f Thrusters::CalculateAngVel(const Quaternionf &q1, const Quaternionf &q2, const float dt) {
    // Calculate relative rotation (error)
    auto q_delta = q1.inverse() * q2;
    q_delta.normalize();

    // Convert to axis angle
    AngleAxis<float> axisAngle(q_delta);

    // Calculate axis angle omega
    Vector3f omega = axisAngle.angle() / dt * axisAngle.axis();

    return omega;
}

float Thrusters::CalculateInclination(const Quaternionf &rov_rot) {
    const Vector3f reference_plane{0, 0, 1};
    auto rov_plane = reference_plane * rov_rot;
    // Orbital inclination formula
    return std::acos(rov_plane[2] / rov_plane.norm());
}

float Thrusters::CalculateInclination(const Vector3f &plane) {
    return std::acos(plane[2] / plane.norm());
}

Thrusters::PCAOutputs Thrusters::GetPWMOutputs(const ThrusterOutputs &thruster_outputs) const {
    PCAOutputs pwm_outputs{};
    for (int i = 0; i < 8; i++) {
        pwm_outputs[i] = thruster_data_.ThrustToPWM(thruster_outputs[i]);
    }
    return pwm_outputs;
}

void Thrusters::PlotPWMVsThrust() {
    auto &pwm_values = thruster_data_.GetPWMValues();
    auto &thrust_values = thruster_data_.GetThrustValues();
    auto &thrust_left_values = thruster_data_.GetThrustLeftValues();
    auto &thrust_right_values = thruster_data_.GetThrustRightValues();
    auto &pwm_left_values = thruster_data_.GetPWMLeftValues();
    auto &pwm_right_values = thruster_data_.GetPWMRightValues();

    std::vector<float> thrust_fit_left(thrust_left_values.size());
    std::vector<float> thrust_fit_right(thrust_right_values.size());

    for (size_t i = 0; i < thrust_fit_left.size(); i++) {
        const float x = pwm_left_values[i];
        thrust_fit_left[i] = thruster_data_.PWMToThrust(x);
    }

    for (size_t i = 0; i < thrust_fit_right.size(); i++) {
        const float x = pwm_right_values[i];
        thrust_fit_right[i] = thruster_data_.PWMToThrust(x);
    }

    matplot::plot(pwm_values, thrust_values, pwm_left_values, thrust_fit_left, pwm_right_values, thrust_fit_right);
    matplot::show();
}

void Thrusters::PlotThrustVsPWM() {
    auto &pwm_values = thruster_data_.GetPWMValues();
    auto &thrust_values = thruster_data_.GetThrustValues();

    std::vector<float> pwm_fit(thrust_values.size());

    for (size_t i = 0; i < thrust_values.size(); i++) {
        const float x = pwm_values[i];
        pwm_fit[i] = thruster_data_.ThrustToPWM(x);
    }

    matplot::plot(thrust_values, pwm_values, thrust_values, pwm_fit);
    matplot::show();
}


Thrusters::ThrusterOutputs Thrusters::ThrustVector::GetThrusterOutputs(const ThrusterDecomp &horizontal_decomp,
                                                                       const ThrusterDecomp &vertical_decomp) const {
    const Vector3f horizontal_(linear_[0], linear_[1], angular_[2]);
    const Vector3f vertical_(linear_[2], angular_[1], angular_[0]);

    return {horizontal_decomp.solve(horizontal_), vertical_decomp.solve(vertical_)};
}

Thrusters::ThrusterOutputs::ThrusterOutputs(const Vector<float, 8> &outputs) {
    for (Index i = 0; i < 4; i++) {
        horizontal_[i] = outputs[i];
    }
    for (Index i = 4; i < 8; i++) {
        vertical_[i - 4] = outputs[i];
    }
}

Thrusters::ThrusterOutputs::ThrusterOutputs(const Solve<ThrusterDecomp, Vector3f> &horizontal,
                                            const Solve<ThrusterDecomp, Vector3f> &vertical) {
    horizontal_ = horizontal;
    vertical_ = vertical;
}

Thrusters::ThrusterOutputs::~ThrusterOutputs() = default;

void Thrusters::ThrusterOutputs::Desaturate(const float max_thrust_kgf) {
    float real_max_thrust_kgf;
    for (float value: horizontal_) {
        real_max_thrust_kgf = std::max(real_max_thrust_kgf, value);
    }
    for (float value: vertical_) {
        real_max_thrust_kgf = std::max(real_max_thrust_kgf, value);
    }

    const float ratio = real_max_thrust_kgf * max_thrust_kgf;
    if (real_max_thrust_kgf > max_thrust_kgf) {
        for (float &value: horizontal_) {
            value /= ratio;
        }
        for (float &value: vertical_) {
            value /= ratio;
        }
    }
}

std::string Thrusters::ThrusterOutputs::ToString() {
    std::stringstream ss;
    std::string x_direction, y_direction, z_direction;
    std::string roll_direction, pitch_direction, yaw_direction;
    // TODO: Implement
    return ss.str();
}

float &Thrusters::ThrusterOutputs::operator[](const size_t i) {
    if (i >= 8) {
        return vertical_[vertical_.size() - 1];
    }
    if (i >= 4) {
        return vertical_[i - 4];
    }
    return horizontal_[i];
}

float Thrusters::ThrusterOutputs::operator[](const size_t i) const {
    if (i >= 8) {
        return vertical_[vertical_.size() - 1];
    }
    if (i >= 4) {
        return vertical_[i - 4];
    }
    return horizontal_[i];
}
