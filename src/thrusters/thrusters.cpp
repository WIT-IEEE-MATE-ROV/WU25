#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iostream>
#include <bits/stdc++.h>
#include <cmath>

#include <matplot/freestanding/plot.h>

#include "thrusters/thrusters.hpp"
#include "thrusters/thruster_data.hpp"

Thrusters::Thrusters(): thruster_data_(DATA_PATH) {
    Vector4f horizontal_angles = {
        THRUSTER_ANGLE_RAD, -THRUSTER_ANGLE_RAD, 2.f * M_PIf - THRUSTER_ANGLE_RAD, -(2.f * M_PIf - THRUSTER_ANGLE_RAD)
    };

    for (float &angle: horizontal_angles) {
        angle = std::fmod(angle, M_PIf);
    }

    // for (size_t i = 0; i < thruster_data_.GetPWMValues().size(); i++) {
    //     std::cout << "PWM: " << thruster_data_.GetPWMValues()[i] << '\t';
    //     std::cout << "Current: " << thruster_data_.GetCurrentValues()[i] << '\t';
    //     std::cout << "Thrust: " << thruster_data_.GetThrustValues()[i] << std::endl;
    // }

    // std::cout << "Angles: " << std::endl;
    // for (int i = 0; i < 4; i++) {
    //     std::cout << horizontal_angles[i] * 180.f / M_PIf << " ";
    // }
    // std::cout << std::endl;

    const float yaw_weight = HALF_DIAGONAL_HORIZONTAL_M * std::sin(
                                 THRUSTER_ANGLE_RAD - LENGTH_DIAGONAL_ANGLE_HORIZONTAL_RAD);

    Matrix<float, 3, 4> thruster_config_horizontal;
    thruster_config_horizontal.row(0) << std::cos(horizontal_angles[0]), std::cos(horizontal_angles[1]),
            std::cos(horizontal_angles[2]), std::cos(horizontal_angles[3]);
    thruster_config_horizontal.row(1) << std::sin(horizontal_angles[0]), std::sin(horizontal_angles[1]),
            std::sin(horizontal_angles[2]), std::sin(horizontal_angles[3]);
    thruster_config_horizontal.row(2) << yaw_weight, -yaw_weight, -yaw_weight, yaw_weight;

    Matrix<float, 3, 4> thruster_config_vertical;
    thruster_config_vertical.row(0) << 1.f, 1.f, 1.f, 1.f;
    thruster_config_vertical.row(1) << HALF_LENGTH_VERTICAL_M, HALF_LENGTH_VERTICAL_M, -HALF_LENGTH_VERTICAL_M, -
            HALF_LENGTH_VERTICAL_M;
    thruster_config_vertical.row(2) << HALF_WIDTH_VERTICAL_M, -HALF_WIDTH_VERTICAL_M, HALF_WIDTH_VERTICAL_M, -
            HALF_WIDTH_VERTICAL_M;

    decomp_horizontal_ = ThrusterDecomp(thruster_config_horizontal);
    decomp_vertical_ = ThrusterDecomp(thruster_config_vertical);

    // X, Y, Yaw
    const Vector3f horizontal_vector = {1, 0, 0};
    // Z, Pitch, Roll
    const Vector3f vertical_vector = {1, 1, 0};

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

Thrusters::~Thrusters() = default;

void Thrusters::Init() {
    const ThrustVector t({1, 0, 0}, {0, 0, 0});

    const ThrusterOutputs outputs = t.GetThrusterOutputs(decomp_horizontal_, decomp_vertical_);
    std::array<PWMValue, 8> pwm_outputs{};
    GetPWMOutputs(outputs, pwm_outputs);
    // std::cout << "GetThrusterOutputs: \n" << outputs.horizontal_ << '\n' << outputs.vertical_ << std::endl;
    // std::cout << "PWM values: \n";
    // for (int i = 0; i < 8; i++) {
    //     std::cout << pwm_outputs[i] << ' ';
    // }
    // std::cout << std::endl;
}

Thrusters::ThrusterOutputs Thrusters::Update() {
    ThrusterOutputs outputs = thrust_vector_.GetThrusterOutputs(decomp_horizontal_, decomp_vertical_);
    std::array<PWMValue, 8> pwms{};

    for (size_t i = 0; i < pwms.size(); i++) {
        pwms[i] = thruster_data_.ThrustToPWM(outputs[i]);
    }

    // std::cout << "PWM outputs: " << std::endl;
    // for (const PWMValue output: pwms) {
    //     std::cout << output << " \n";
    // }
    // std::cout << std::endl;


    std::cout << "Current: " << current_rotation_ << "\nDesired: " << desired_rotation_ << std::endl;

    const Quaternionf quat_error = desired_rotation_.inverse() * current_rotation_;

    std::cout << "Error: " << quat_error << std::endl;

    const Vector3f error_e = quat_error.toRotationMatrix().canonicalEulerAngles(0, 1, 2);
    std::cout << "Error euler: \n" << error_e * 180.f / M_PIf << std::endl;


    ThrusterOutputs outputs_calculated;
    for (int i = 0; i < 8; i++) {
        outputs_calculated[i] = thruster_data_.PWMToThrust(pwms[i]);
    }

    return outputs_calculated;
}

void Thrusters::SetThrustVector(const ThrustVector &thrust_vector) {
    thrust_vector_ = thrust_vector;
    // std::cout << "thrust_vector_:\n";
    // std::cout << "X:\t" << thrust_vector_[0] << "\nY:\t" << thrust_vector_[1] << "\nZ:\t" << thrust_vector_[2]
    //         << "\nR:\t" << thrust_vector_[3] << "\nP:\t" << thrust_vector_[4] << "\nY:\t" << thrust_vector_[5]
    //         << std::endl;
}

void Thrusters::SetRotation(const Quaternionf &q) {
    current_rotation_ = q;
}

void Thrusters::SetDesiredRotation(const Quaternionf &q) {
    desired_rotation_ = q;
}

void Thrusters::GetPWMOutputs(const ThrusterOutputs &thruster_outputs, std::array<PWMValue, 8> &pwm_outputs) const {
    for (int i = 0; i < 8; i++) {
        pwm_outputs[i] = thruster_data_.ThrustToPWM(thruster_outputs[i]);
    }
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

//


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
