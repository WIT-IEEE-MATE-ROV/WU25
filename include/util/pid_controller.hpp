//
// Created by foamstein on 5/18/25.
//

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <limits>
#include <chrono>

class PIDController {
public:
    PIDController(float kP, float kI, float kD, float kFF = 0, float max_output = std::numeric_limits<float>::max());

    void SetSetpoint(float setpoint);

    float Calculate(float current_state);

private:
    float kP_, kI_, kD_, kFF_;
    float max_output_;

    float setpoint_{0};
    float prev_error_{0};
    float i_accum_{0};

    std::chrono::system_clock::time_point prev_time_ms_;
};

#endif //PID_CONTROLLER_HPP
