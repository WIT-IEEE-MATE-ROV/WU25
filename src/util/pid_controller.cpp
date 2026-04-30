//
// Created by foamstein on 5/18/25.
//

#include "util/pid_controller.hpp"
#include <algorithm>

PIDController::PIDController(const float kP, const float kI, const float kD, const float kFF, const float max_output)
    : kP_(kP), kI_(kI), kD_(kD), kFF_(kFF), max_output_(max_output) {
    prev_time_ms_ = std::chrono::system_clock::now();
}

void PIDController::SetSetpoint(const float setpoint) {
    setpoint_ = setpoint;
}

void PIDController::SetGains(const float kP, const float kI, const float kD, const float kFF) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    kFF_ = kFF;
    i_accum_ = 0.0f;
    prev_error_ = 0.0f;
}

float PIDController::Calculate(const float current_state) {
    const auto now = std::chrono::system_clock::now();
    const std::chrono::duration<float> dt_duration = now - prev_time_ms_;
    const float dt_s = dt_duration.count();
    prev_time_ms_ = now;

    // Guard: if dt is zero or extremely small, don't update derivative/integral
    if (dt_s <= 0.0f) {
        return 0.0f;
    }

    const float error = setpoint_ - current_state;

    // Integrate with seconds units
    i_accum_ += error * dt_s;

    // Anti-windup: clamp integral term if kI_ non-zero and max_output_ finite
    if (kI_ != 0.0f && max_output_ < std::numeric_limits<float>::max()) {
        const float i_limit = std::abs(max_output_ / kI_);
        i_accum_ = std::clamp(i_accum_, -i_limit, i_limit);
    }

    const float p_out = kP_ * error;
    const float i_out = kI_ * i_accum_;

    float d_out = 0.0f;
    if (dt_s > 1e-6f) {
        d_out = kD_ * ((error - prev_error_) / dt_s);
    }
    const float ff_out = kFF_ * setpoint_;

    prev_error_ = error;

    float output = p_out + i_out + d_out + ff_out;

    // Final output clamp
    if (max_output_ < std::numeric_limits<float>::max()) {
        output = std::clamp(output, -max_output_, max_output_);
    }

    return output;
}
