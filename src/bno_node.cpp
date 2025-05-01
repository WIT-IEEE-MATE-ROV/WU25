#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <gpiod.h>
#include <sh2.h>

#include "drivers/BNO08x_ada.hpp"

using namespace std::chrono_literals;

class BNONode : public rclcpp::Node {
public:
    Adafruit_BNO08x bno;
    sh2_SensorValue_t sensorValue{};
    uint64_t loops = 0;


    BNONode() : Node("bno_node"), log_(get_logger()) {
        // TODO:  Read page 51 of datasheet to see report rates for each report type to not use extra CPU
        timer_ = create_wall_timer(10ms, std::bind(&BNONode::timer_callback, this));

        quat_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("bno/quat", 2);

        RCLCPP_INFO(log_, "Creating BNO object");
        bno.init();
        bno.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
        printf("Initialize finished\n");
    }

    void timer_callback() {
        if (bno.wasReset()) {
            printf("BNO was reset\n");
        }

        if (bno.getSensorEvent(&sensorValue)) {
            sh2_RotationVector_t *game_rot = &sensorValue.un.gameRotationVector;

            if (loops++ % 5 == 0) {
                RCLCPP_INFO(log_, "w: %f, i: %f, j: %f, k:%f",
                            game_rot->real, game_rot->i, game_rot->j, game_rot->k);
            }

            quat_.w = game_rot->real;
            quat_.x = game_rot->i;
            quat_.y = game_rot->j;
            quat_.z = game_rot->k;

            hdr_.stamp = get_clock()->now();

            quat_stamped_.header = hdr_;
            quat_stamped_.quaternion = quat_;

            quat_pub_->publish(quat_stamped_);
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quat_pub_;
    std_msgs::msg::Header hdr_{};
    geometry_msgs::msg::Quaternion quat_{};
    geometry_msgs::msg::QuaternionStamped quat_stamped_{};
    rclcpp::Logger log_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNONode>());
    rclcpp::shutdown();
    return 0;
}
