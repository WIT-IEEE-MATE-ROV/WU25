//
// Created by foamstein on 1/30/25.
//

#include "util/quaternion_pid.hpp"
#include <iostream>

/

QuatPIDController::QuatPIDController(const PIDParams &x_params, const PIDParams &y_params, const PIDParams &z_params) {
    x_params_ = x_params;
    y_params_ = y_params;
    z_params_ = z_params;
    prev_time_ = std::chrono::high_resolution_clock::now();

    std::cout << ""
}

void QuatPIDController::SetSetpoint(const Eigen::Quaternionf &setpoint) {
    setpoint_ = setpoint;
}

Eigen::Vector3f QuatPIDController::Calculate(const Eigen::Quaternionf &current_rotation) {
    current_ = current_rotation;
    Eigen::Quaternionf error = GetError();

    if (error.w() < 0) {
        error = Eigen::Quaternionf{-error.w(), -error.x(), -error.y(), -error.z()};
    }

    const auto current_time = std::chrono::high_resolution_clock::now();

    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - prev_time_);
    const float dtf = static_cast<float>(dt.count());
    prev_time_ = current_time;


    return {
        CalculateAxisOutput(error.x(), prev_x_error_, dtf, x_params_),
        CalculateAxisOutput(error.y(), prev_y_error_, dtf, y_params_),
        CalculateAxisOutput(error.z(), prev_z_error_, dtf, z_params_)
    };
}

Eigen::Quaternionf QuatPIDController::GetError() const {
    return current_.inverse() * setpoint_;
}

float QuatPIDController::CalculateAxisOutput(const float error, float &prev_error, const float dt, PIDParams &params) {
    params.i_accum += error * dt;
    const float de = (error - prev_error) / dt;

    const float p_out = error * params.p;
    const float i_out = params.i_accum * params.i;
    const float d_out = de * params.d;

    prev_error = error;
    return p_out + i_out + d_out;
}
