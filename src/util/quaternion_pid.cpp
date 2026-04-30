//
// Created by foamstein on 1/30/25.
//

#include "util/quaternion_pid.hpp"
#include <iostream>


QuatPIDController::QuatPIDController(const PIDParams &x_params, const PIDParams &y_params, const PIDParams &z_params) {
    x_params_ = x_params;
    y_params_ = y_params;
    z_params_ = z_params;
    prev_time_ = std::chrono::system_clock::now();
}

void QuatPIDController::SetSetpoint(const Eigen::Quaternionf &setpoint) {
    setpoint_ = setpoint;
}

void QuatPIDController::SetParams(float p, float i, float d, float i_zone, float i_max_accum, float max_output) {
    PIDParams params;
    params.p = p;
    params.i = i;
    params.d = d;
    params.i_zone = i_zone;
    params.i_max_accum = i_max_accum;
    params.max_output = max_output;
    params.i_accum = 0.0f;
    x_params_ = params;
    y_params_ = params;
    z_params_ = params;
    prev_x_error_ = 0.0f;
    prev_y_error_ = 0.0f;
    prev_z_error_ = 0.0f;
}

Eigen::Vector3f QuatPIDController::Calculate(const Eigen::Quaternionf &current_rotation) {
    current_ = current_rotation;
    // Compute error quaternion: rotation from current -> setpoint
    Eigen::Quaternionf q_err = setpoint_ * current_.inverse();

    // Ensure shortest rotation
    if (q_err.w() < 0.0f) {
        q_err.coeffs() = -q_err.coeffs();
    }

    // Convert to angle-axis (angle in radians)
    const float w = std::clamp(q_err.w(), -1.0f, 1.0f);
    const float angle = 2.0f * std::acos(w);
    const float sin_half = std::sqrt(std::max(0.0f, 1.0f - w * w));

    Eigen::Vector3f axis;
    if (sin_half < 1e-6f) {
        axis = Eigen::Vector3f::Zero();
    } else {
        axis = q_err.vec() / sin_half;
    }

    // axis * angle gives an angle-axis vector (radians) suitable as error signal
    Eigen::Vector3f angle_axis = axis * angle;

    const auto current_time = std::chrono::system_clock::now();
    const std::chrono::duration<float> dt_duration = current_time - prev_time_;
    const float dtf = dt_duration.count();
    prev_time_ = current_time;

    if (dtf <= 0.0f) {
        return Eigen::Vector3f::Zero();
    }

    return {
        CalculateAxisOutput(angle_axis.x(), prev_x_error_, dtf, x_params_),
        CalculateAxisOutput(angle_axis.y(), prev_y_error_, dtf, y_params_),
        CalculateAxisOutput(angle_axis.z(), prev_z_error_, dtf, z_params_)
    };
}

Eigen::Quaternionf QuatPIDController::GetError() const {
    return setpoint_ * current_.inverse();
}

float QuatPIDController::CalculateAxisOutput(const float error, float &prev_error, const float dt, PIDParams &params) {
    // Guard dt
    if (dt <= 0.0f) {
        return 0.0f;
    }

    // Proportional
    const float p_out = params.p * error;

    // Integral (only accumulate inside i_zone)
    if (std::abs(error) <= params.i_zone) {
        params.i_accum += error * dt;
        if (params.i_accum > params.i_max_accum) params.i_accum = params.i_max_accum;
        if (params.i_accum < -params.i_max_accum) params.i_accum = -params.i_max_accum;
    }
    const float i_out = params.i * params.i_accum;

    // Derivative
    const float de = (error - prev_error) / dt;
    const float d_out = params.d * de;

    prev_error = error;

    float out = p_out + i_out + d_out;

    // Clamp by symmetric max output if provided
    if (params.max_output < std::numeric_limits<float>::max()) {
        out = std::clamp(out, -params.max_output, params.max_output);
    }

    return out;
}
