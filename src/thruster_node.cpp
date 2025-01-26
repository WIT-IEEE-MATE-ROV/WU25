#include <chrono>
#include <memory>
#include <string>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>

#include <wiringPi.h>
#include <gpiod.h>
#include <sh2.h>
#include <sh2_util.h>
#include <euler.h>
#include <sh2_err.h>
#include <sh2_SensorValue.h>
#include <thrusters.hpp>

using namespace std::chrono_literals;

class ThrusterNode final : public rclcpp::Node {
public:
    ThrusterNode() : Node("bno_node") {
        RCLCPP_INFO(this->get_logger(), "Starting thruster node");
        // timer_ = this->create_wall_timer(10ms, [this] { TimerCallback(); });

        thrusters_ = std::make_unique<Thrusters>();
        thrusters_->Init();

        printf("Initialize finished\n");

        rclcpp::shutdown();
    }

    void TimerCallback() {
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::unique_ptr<Thrusters> thrusters_;
    size_t count_;
};

int main(const int argc, char *argv[]) {
    std::cout << "Thruster node argc = " << argc << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterNode>());
    rclcpp::shutdown();
    return 0;
}
