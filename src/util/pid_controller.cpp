//
// Created by foamstein on 5/18/25.
//

#include "util/pid_controller.hpp"

PIDController::PIDController(const float kP, const float kI, const float kD, const float kFF, const float max_output)
    : kP_(kP), kI_(kI), kD_(kD), kFF_(kFF), max_output_(max_output) {
    prev_time_ms_ = std::chrono::high_resolution_clock::now();
}

void PIDController::SetSetpoint(const float setpoint) {
    setpoint_ = setpoint;
}

float PIDController::Calculate(const float current_state) {
    const auto now = std::chrono::high_resolution_clock::now();
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev_time_ms_).count();
    prev_time_ms_ = now;

    const auto error = setpoint_ - current_state;
    i_accum_ += error * static_cast<float>(dt);

    const auto p_out = kP_ * error;
    const auto i_out = kI_ * i_accum_;
    float d_out = 0;
    if (dt != 0) {
        d_out = kD_ * (error / static_cast<float>(dt));
    }
    const auto ff_out = kFF_ * setpoint_;

    prev_error_ = error;

    return p_out + i_out + d_out + ff_out;
}
