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

PCAError PCA9685::init(uint8_t address, float frequency_hz) {
    address_ = address;
    osc_frequency_hz_ = 25;

    std::string filename = "/dev/i2c-2";
    bus_fd_ = open(filename.c_str(), O_RDWR);
    if (bus_fd_ < 0) {
        perror("Failed to open the I2C bus");
        return PCA_FAIL;
    }

    if (ioctl(bus_fd_, I2C_SLAVE, address) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(bus_fd_);
        return PCA_FAIL;
    }

    frequency_hz_ = frequency_hz;

    restart();
    std::this_thread::sleep_for(10ms);

    uint8_t mode_1 = PCA_M1_AUTO_INC | 1;
    reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    set_pwm_frequency(frequency_hz_);

}

PCAError PCA9685::restart() {
    uint8_t mode_1;
    PCAError ret = reg_read(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
    mode_1 |= PCA_M1_RESTART;
    ret = reg_write(bus_fd_, PCA_REG_MODE_1, &mode_1, 1);
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
    uint16_t on_counts;q

    return PCA_OK;
}

