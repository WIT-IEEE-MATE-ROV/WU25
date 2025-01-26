//
// Created by foamstein on 1/21/25.
//

#ifndef THRUSTER_DATA_HPP
#define THRUSTER_DATA_HPP

class ThrusterData {
public:
    static constexpr size_t DATA_LEN = 201;
    static constexpr size_t HALF_DATA_LEN = DATA_LEN / 2;
    static constexpr size_t RIGHT_START = HALF_DATA_LEN + 9;
    static constexpr size_t LEFT_END = HALF_DATA_LEN - 8;
    static constexpr size_t LEFT_LEN = LEFT_END;
    static constexpr size_t RIGHT_LEN = DATA_LEN - RIGHT_START;
    static constexpr uint16_t PWM_MIN = 1100;
    static constexpr uint16_t PWM_ZERO = 1500;
    static constexpr uint16_t PWM_MAX = 1900;

    explicit ThrusterData(const std::string &file_path);

    ~ThrusterData() = default;

    [[nodiscard]] std::array<float, DATA_LEN> const &GetPWMValues() const {
        return pwm_value_;
    }

    [[nodiscard]] std::array<float, DATA_LEN> const &GetThrustValues() const {
        return thrust_kgf_;
    }

    [[nodiscard]] std::array<float, DATA_LEN> const &GetCurrentValues() const {
        return current_;
    }

    [[nodiscard]] std::array<float, LEFT_END> const &GetThrustLeftValues() const {
        return thrust_left_;
    }

    [[nodiscard]] std::array<float, DATA_LEN - RIGHT_START> const &GetThrustRightValues() const {
        return thrust_right_;
    }

    [[nodiscard]] std::array<float, LEFT_END> const &GetPWMLeftValues() const {
        return pwm_left_;
    }

    [[nodiscard]] std::array<float, DATA_LEN - RIGHT_START> const &GetPWMRightValues() const {
        return pwm_right_;
    }

    [[nodiscard]] uint16_t ThrustToPWM(float thrust_kgf) const;

    [[nodiscard]] float PWMToThrust(float pwm) const ;

private:
    std::array<float, DATA_LEN> pwm_value_{};
    std::array<float, DATA_LEN> thrust_kgf_{};
    std::array<float, DATA_LEN> current_{};
    std::array<float, LEFT_LEN> thrust_left_{};
    std::array<float, RIGHT_LEN> thrust_right_{};
    std::array<float, LEFT_LEN> pwm_left_{};
    std::array<float, RIGHT_LEN> pwm_right_{};

    std::vector<float> pwm_to_current_fit_coeffs_;
    std::vector<float> thrust_to_pwm_fit_coeffs_;
    std::vector<float> pwm_to_thrust_fit_left_coeffs_;
    std::vector<float> pwm_to_thrust_fit_right_coeffs_;
};

#endif //THRUSTER_DATA_HPP
