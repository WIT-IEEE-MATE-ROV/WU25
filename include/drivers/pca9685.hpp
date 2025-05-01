//
// Created by ubuntu on 5/1/25.
//

#ifndef WU25_PCA9685_H
#define WU25_PCA9685_H

#include <stdint.h>

enum PCAError {
    PCA_OK = 0,
    PCA_FAIL = 1
};

class PCA9685 {
public:
    PCAError init(uint8_t address, float frequency_hz);

    PCAError restart();

    PCAError software_reset();

    PCAError set_pwm_frequency(float pwm_freq_hz);

    PCAError set_sleep(bool sleep_state);

    PCAError set_period(uint8_t control_num, uint32_t period_us);

private:
    int bus_fd_;
    float frequency_hz_;
    float osc_frequency_hz_;
    uint8_t address_;
};


#endif //WU25_PCA9685_H
