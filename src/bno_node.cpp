#include <chrono>
#include <memory>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>

#include <gpiod.h>
#include <sh2.h>

#include "drivers/BNO08x_ada.hpp"

using namespace std::chrono_literals;

class BNONode : public rclcpp::Node {
public:
    Adafruit_BNO08x bno;
    sh2_SensorValue_t sensorValue;

    BNONode() : Node("bno_node") {
        timer_ = this->create_wall_timer(10ms, std::bind(&BNONode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Creating BNO object");
        bno.init();
        bno.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
        printf("Initialize finished\n");
    }

    void timer_callback() {
        if (bno.wasReset()) {
            printf("BNO was reset\n");
        }
//        std::cout << "Servicing" << std::endl;
        if (bno.getSensorEvent(&sensorValue)) {
//            printf("Got sensor event: status=%u\n", sensorValue.status);
            sh2_RotationVectorWAcc *quat = &sensorValue.un.arvrStabilizedRV;
            printf("w: %f, i: %f, j: %f, k:%f\n",
                   quat->real, quat->i, quat->j, quat->k);
        }
//        bno.service();
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_{};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNONode>());
    rclcpp::shutdown();
    return 0;
}
