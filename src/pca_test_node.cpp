//
// Created by ubuntu on 5/1/25.
//

#include <cstdio>
#include <iostream>
#include <thread>
#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "drivers/pca9685.hpp"

using namespace std::chrono_literals;

PCA9685 pca{};
volatile bool interrupted = false;

void sighandler(int signal)
{
    std::cout << "Restarting" << std::endl;
    if (signal == SIGINT) {
//        pca.restart();
//        pca.set_sleep(true);
        interrupted = true;
    }
}

class PCANode : public rclcpp::Node {
public:
    PCANode() : Node("pca_test"), logger_(this->get_logger()) {
//        timer_ = create_wall_timer(10ms, std::bind(&PCANode::timer_callback, this));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PCANode::joy_callback, this, std::placeholders::_1));

        pca.init(0x40, 100);

        std::this_thread::sleep_for(100ms);

        RCLCPP_INFO(logger_, "Sending 1500us for 3 seconds...");
        pca.set_period(0, 1500);
        std::this_thread::sleep_for(3s);
        RCLCPP_INFO(logger_, "Initialized");
    }

    void timer_callback() {

    }

    void joy_callback(const sensor_msgs::msg::Joy msg) {
        RCLCPP_INFO(logger_, "Joy left x = %f", msg.axes[0]);

        const float leftX = msg.axes[0];
        const uint32_t period = static_cast<int32_t>(((leftX + 1.f) / 2.f) * 800.f + 1100.f);
        RCLCPP_INFO(logger_, "Period = %u", period);
        pca.set_period(0, period);
    }

private:
//    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Logger logger_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCANode>());
    rclcpp::shutdown();
    return 0;
/*

    signal(SIGINT, sighandler);

    std::cout << "PCA Test" << std::endl;

    pca.init(0x40, 100);

    std::this_thread::sleep_for(100ms);

    uint8_t mode_1 = pca.get_mode1_reg();
    printf("Mode 1 = 0x%02X\n", mode_1);

    std::cout << "Sending 1500us for 3 seconds...\n";
    pca.set_period(0, 1500);
    std::this_thread::sleep_for(3s);


    uint32_t period = 1500;
    while (!interrupted) {
        pca.set_period(0, period = 1300);
        std::cout << period << std::endl;
        std::this_thread::sleep_for(2000ms);

        pca.set_period(0, period = 1800);
        std::cout << period << std::endl;
        std::this_thread::sleep_for(2000ms);


//        pca.set_period(0, period);
//        printf("%u\n", period);
//        period += 5;
//        std::this_thread::sleep_for(500ms);

        pca.set_period(0, period = 1500);
        std::cout << period << std::endl;
        std::this_thread::sleep_for(2000ms);


        pca.set_period(0, period = 1600);
        std::cout << period << std::endl;
        std::this_thread::sleep_for(2000ms);

    }

    std::cout << "Exiting" << std::endl;
    return 0;
*/
}
