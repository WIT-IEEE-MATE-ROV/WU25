//
// Created by foamstein on 1/21/25.
//

#include <array>
#include <cstdint>
#include <complex>

#include <csv.hpp>

#include "thruster_data.hpp"
#include "PolynomialRegression.hpp"

using namespace csv;

ThrusterData::ThrusterData(const std::string &file_path) {
    static constexpr uint32_t pwm_field_idx = 0, current_field_idx = 2, thrust_field_idx = 5;
    CSVReader reader(file_path);

    uint32_t idx = 0;
    for (CSVRow &row: reader) {
        pwm_value_[idx] = row[pwm_field_idx].get<float>();
        current_[idx] = row[current_field_idx].get<float>();
        thrust_kgf_[idx] = row[thrust_field_idx].get<float>();

        ++idx;
    }

    // std::cout << "right_start = " << RIGHT_START << " thrust_kgf[right_start] = " << thrust_kgf_[RIGHT_START] <<
    //         std::endl;
    // std::cout << "left_end = " << LEFT_END << " thrust_kgf[left_end] " << thrust_kgf_[LEFT_END] << std::endl;
    for (size_t i = 0; i < LEFT_END; i++) {
        thrust_left_[i] = thrust_kgf_[i];
        pwm_left_[i] = pwm_value_[i];
    }

    for (size_t i = RIGHT_START; i < DATA_LEN; i++) {
        thrust_right_[i - RIGHT_START] = thrust_kgf_[i];
        pwm_right_[i - RIGHT_START] = pwm_value_[i];
    }

    std::vector pwm_left_vec(pwm_left_.begin(), pwm_left_.end());
    std::vector pwm_right_vec(pwm_right_.begin(), pwm_right_.end());
    std::vector thrust_left_vec(thrust_left_.begin(), thrust_left_.end());
    std::vector thrust_right_vec(thrust_right_.begin(), thrust_right_.end());

    PolynomialRegression<float> regression;
    regression.fitIt(pwm_left_vec, thrust_left_vec, 2, pwm_to_thrust_fit_left_coeffs_);
    regression.fitIt(pwm_right_vec, thrust_right_vec, 2, pwm_to_thrust_fit_right_coeffs_);

    std::cout << "PWM to Thrust Left coeffs:\n";
    for (const float c: pwm_to_thrust_fit_left_coeffs_) {
        std::cout << c << ' ';
    }
    std::cout << std::endl;

    std::cout << "PWM to Thrust Left coeffs:\n";
    for (const float c: pwm_to_thrust_fit_left_coeffs_) {
        std::cout << c << ' ';
    }
    std::cout << std::endl;
}

uint16_t ThrusterData::ThrustToPWM(const float thrust_kgf) const {
    auto quadratic_solve_one_root = [](const float y, const float a, const float b, const float c) -> float {
        const float sqrt_discriminant = std::sqrt(b * b - 4.f * a * (c - y));
        return (-b + sqrt_discriminant) / (2.f * a);
    };

    float pwmf;
    if (thrust_kgf < 0) {
        const auto &c = pwm_to_thrust_fit_left_coeffs_;
        pwmf = quadratic_solve_one_root(thrust_kgf, c[2], c[1], c[0]);
        pwmf = std::clamp(pwmf, static_cast<float>(PWM_MIN), static_cast<float>(PWM_ZERO));
    } else if (thrust_kgf > 0) {
        const auto &c = pwm_to_thrust_fit_right_coeffs_;
        pwmf = quadratic_solve_one_root(thrust_kgf, c[2], c[1], c[0]);
        pwmf = std::clamp(pwmf, static_cast<float>(PWM_ZERO), static_cast<float>(PWM_MAX));
    } else {
        return PWM_ZERO;
    }

    if (fabsf(pwmf) > UINT16_MAX) {
        std::cout << "Couldn't convert PWM float to uint16_t" << std::endl;
        return PWM_ZERO;
    }

    return static_cast<uint16_t>(roundf(pwmf));
}

float ThrusterData::PWMToThrust(const float pwm) const {
    float thrust = 0;
    if (pwm <= pwm_left_.back()) {
        const auto &c = pwm_to_thrust_fit_left_coeffs_;
        for (size_t order = 0; order < c.size(); order++) {
            thrust += c[order] * powf(pwm, order);
        }
        thrust = std::clamp(thrust, thrust_left_.front(), 0.f);
    } else if (pwm >= pwm_right_.front()) {
        const auto &c = pwm_to_thrust_fit_right_coeffs_;
        for (size_t order = 0; order < c.size(); order++) {
            thrust += c[order] * powf(pwm, order);
        }
        thrust = std::clamp(thrust, 0.f, thrust_right_.back());
    } else {
        return 0;
    }

    return thrust;
}
