//
// Created by ubuntu on 5/1/25.
//

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <linux/i2c-dev.h>
#include <thread>
#include <chrono>
#include <cmath>

#include "drivers/pca9685.hpp"

#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

#define PCA_DEFAULT_ADDRESS 0x41

#define PCA_REG_PRE_SCALE 0xFE
#define PCA_REG_MODE_1 0x00
#define PCA_REG_LED0_ON_L 6

#define PCA_M1_RESTART 1 << 7
#define PCA_M1_EXTCLK 1 << 6
#define PCA_M1_AUTO_INC 1 << 5
#define PCA_M1_SLEEP 1 << 4
#define PCA_CTRL_REG_OFFSET 0x06


using namespace std::chrono_literals;
uint8_t i2c_tx[255];

PCAError i2c_write(int bus_fd, uint8_t *data, uint32_t data_len) {
    ssize_t write_ret = write(bus_fd, i2c_tx, data_len);
    if (write_ret != data_len) {
        perror("Failed to write i2c");
        return PCA_FAIL;
    }

    return PCA_OK;
}

PCAError reg_write(int bus_fd, uint8_t reg, uint8_t *data, uint32_t data_len) {
    i2c_tx[0] = reg;
    memcpy(i2c_tx + 1, data, data_len);

    ssize_t write_ret = write(bus_fd, i2c_tx, data_len + 1);
    if (write_ret != data_len + 1) {
        perror("Failed to write to the I2C device");
        return PCA_FAIL;
    }

    return PCA_OK;
}

PCAError reg_read(int bus_fd, uint8_t reg, uint8_t *data, uint32_t data_len) {
    if (write(bus_fd, &reg, 1) != 1) {
        perror("Failed to write register address to the I2C device");
//        close(bus_fd);
        return PCA_FAIL;
    }

    if (read(bus_fd, data, data_len) != 1) {
        perror("Failed to read from the I2C device");
//        close(bus_fd);
        return PCA_FAIL;
    }

    return PCA_OK;
}

PCA9685::PCA9685() {
    std::string filename = "/dev/i2c-2";
    bus_fd_ = open(filename.c_str(), O_RDWR);
    if (bus_fd_ < 0) {
        perror("Failed to open the I2C bus");
    }
}

PCA9685::~PCA9685() {
    if (bus_fd_ > 0) {
//        restart();
        set_sleep(true);
        close(bus_fd_);
    }
}

// TODO: Move arguments to constructor
PCAError PCA9685::init(uint8_t address, float frequency_hz) {
    address_ = address;
    osc_frequency_hz_ = 25;

    if (ioctl(bus_fd_, I2C_SLAVE, address) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(bus_fd_);
        return PCA_FAIL;
    }

    frequency_hz_ = frequency_hz;
    measured_frequency_hz_ = 104.45;

    restart();
    std::this_thread::sleep_for(10ms);

    uint8_t mode_1 = PCA_M1_AUTO_INC | 1;
    reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    set_pwm_frequency(frequency_hz_);

    return PCA_OK;
}

PCAError PCA9685::restart() {
    uint8_t mode_1;
    PCAError ret = reg_read(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    if (ret) {
        std::cerr << "Couldn't read restart mode 1\n";
        return PCA_FAIL;
    }

    printf("Mode 1 before = 0x%02X\n", mode_1);
    std::cout << "Sleep bit = " << !!(mode_1 & PCA_M1_SLEEP) << std::endl;

    if (mode_1 & PCA_M1_SLEEP) {
        mode_1 &= ~PCA_M1_SLEEP;
        std::cout << "Clearing sleep bit" << std::endl;
        reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
        std::this_thread::sleep_for(500us);
    }

    mode_1 |= PCA_M1_RESTART;
    ret = reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    if (ret) {
        std::cerr << "Couldn't write restart mode 1\n";
        return PCA_FAIL;
    }

    return ret;
}

PCAError PCA9685::software_reset() {
    if (ioctl(bus_fd_, I2C_SLAVE, 0x00) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        return PCA_FAIL;
    }

    uint8_t reset_code = 6;
    PCAError ret = i2c_write(bus_fd_, &reset_code, 1);

    if (ioctl(bus_fd_, I2C_SLAVE, address_) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        return PCA_FAIL;
    }

    return ret;
}

PCAError PCA9685::set_pwm_frequency(float pwm_freq_hz) {
    pwm_freq_hz = MAX(MIN(pwm_freq_hz, 1526), 26);

    uint8_t mode_1;
    PCAError ret = reg_read(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);

    uint8_t sleep_state = mode_1 & PCA_M1_SLEEP;

    if (!sleep_state) {
        mode_1 |= PCA_M1_SLEEP;
        ret = reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    }

    double prescale_val = round((osc_frequency_hz_ * 1000000.0) / (4096.0 * pwm_freq_hz)) - 1.0;
    auto prescale_data = static_cast<uint8_t>(prescale_val);
    ret = reg_write(bus_fd_, PCA_REG_PRE_SCALE, &prescale_data, 1);

    if (!sleep_state) {
        mode_1 &= ~PCA_M1_SLEEP;
        ret = reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    }

    frequency_hz_ = pwm_freq_hz;

    return PCA_OK;
}

PCAError PCA9685::set_sleep(bool sleep_on) {
    uint8_t mode_1;
    PCAError ret = reg_read(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);

    uint8_t sleep_state = mode_1 & PCA_M1_SLEEP;
    if (sleep_on && !sleep_state) {
        mode_1 |= PCA_M1_SLEEP;
        ret = reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    } else if (!sleep_on && sleep_state) {
        mode_1 &= ~PCA_M1_SLEEP;
        ret = reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    }

    return PCA_OK;
}

PCAError PCA9685::set_period(uint8_t control_num, uint32_t period_us) {
    uint8_t reg = control_num * 4 + PCA_REG_LED0_ON_L;
    float pwm_period_us = (1.f / measured_frequency_hz_) * 1000000.f;

    float us_on_ratio = static_cast<float>(period_us) / pwm_period_us;
    auto off_counts = static_cast<uint16_t>(roundf(us_on_ratio * 4096.f));

    uint8_t ctrl_regs[4] = {0};

    ctrl_regs[2] = off_counts & 0xFF;
    ctrl_regs[3] = (off_counts >> 8) & 0x0F;

    PCAError ret = reg_write(bus_fd_, reg, ctrl_regs, 4);

    return ret;
}


PCAError PCA9685::set_period(uint8_t control_num, std::vector<uint16_t> &period_us) {
    uint8_t start_reg = control_num * 4 + PCA_REG_LED0_ON_L;
    uint32_t reg_len = period_us.size() * 4;
    float pwm_period_us = (1.f / measured_frequency_hz_) * 1000000.f;

    for (uint32_t i = 0; i < period_us.size(); i++) {
        uint16_t period = period_us[i];

        float us_on_ratio = static_cast<float>(period) / pwm_period_us;
        auto off_counts = static_cast<uint16_t>(roundf(us_on_ratio * 4096.f));

        uint32_t reg_idx = i * 4;
        reg_tx_[reg_idx] = 0;
        reg_tx_[reg_idx + 1] = 0;
        reg_tx_[reg_idx + 2] = off_counts & 0xFF;
        reg_tx_[reg_idx + 3] = (off_counts >> 8) & 0x0F;
    }

    PCAError ret = reg_write(bus_fd_, start_reg, reg_tx_, reg_len);
    return ret;
}

uint8_t PCA9685::get_mode1_reg() {
    uint8_t mode1;
    PCAError ret = reg_read(bus_fd_, PCA_REG_MODE_1, &mode1, 1);
    if (ret != PCA_OK) {
        std::cerr << "Failed to read mode 1 register" << std::endl;
    }

    return mode1;
}
