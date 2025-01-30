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

#include "drivers/bno08x.hpp"

using namespace std::chrono_literals;

class BNONode : public rclcpp::Node
{
public:
    BNO08x bno;

    BNONode() : Node("bno_node")
    {
        RCLCPP_INFO(this->get_logger(), "fart ass bitch");
        timer_ = this->create_wall_timer(10ms, std::bind(&BNONode::timer_callback, this));

        bno.initialize();
        printf("Initialize finished\n");
    }

    void timer_callback()
    {
        
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNONode>());
    rclcpp::shutdown();
    return 0;
}
