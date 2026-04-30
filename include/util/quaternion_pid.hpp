//
// Created by foamstein on 1/30/25.
//

#ifndef QUATERNION_PID_HPP
#define QUATERNION_PID_HPP

#include <chrono>
#include <Eigen/Geometry>

class QuatPIDController {
public:
    typedef struct {
        float p = 0;
        float i = 0;
        float d = 0;
        float f = 0;
        float i_max_accum = std::numeric_limits<float>::max();
        float i_zone = std::numeric_limits<float>::max();
        float max_output = std::numeric_limits<float>::max();
        float min_output = 0;
        float error_threshold = 0;

        // Current integrated I accumulation
        float i_accum = 0;
    } PIDParams;

    QuatPIDController(const PIDParams &x_params, const PIDParams &y_params, const PIDParams &z_params);

    void SetSetpoint(const Eigen::Quaternionf &setpoint);

    void SetParams(float p, float i, float d,
                   float i_zone = std::numeric_limits<float>::max(),
                   float i_max_accum = std::numeric_limits<float>::max(),
                   float max_output = std::numeric_limits<float>::max());

    Eigen::Vector3f Calculate(const Eigen::Quaternionf &current_rotation);

    Eigen::Quaternionf GetError() const;

private:
    float CalculateAxisOutput(float error, float &prev_error, float dt, PIDParams &params);

    PIDParams x_params_;
    PIDParams y_params_;
    PIDParams z_params_;
    Eigen::Quaternionf current_{1, 0, 0, 0};
    Eigen::Quaternionf setpoint_{1, 0, 0, 0};
    float prev_x_error_ = 0;
    float prev_y_error_ = 0;
    float prev_z_error_ = 0;
    std::chrono::time_point<std::chrono::system_clock> prev_time_;
};

#endif //QUATERNION_PID_HPP
