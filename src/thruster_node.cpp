#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "thrusters.hpp"
#include "controller_map.hpp"

using namespace std::chrono_literals;

class ThrusterNode final : public rclcpp::Node {
public:
    ThrusterNode() : Node("thruster_node"), log_(this->get_logger()) {
        RCLCPP_INFO(log_, "Starting thruster node");
        timer_ = this->create_wall_timer(20ms, [this] { Loop(); });

        joy_sub_ =
                this->create_subscription<sensor_msgs::msg::Joy>(
                    "joy", 10, std::bind(&ThrusterNode::JoyCallback, this, std::placeholders::_1));

        thrusters_ = std::make_unique<Thrusters>();
        thrusters_->Init();

        RCLCPP_INFO(log_, "Initialize finished\n");
    }

    void Loop() {
        Thrusters::ThrusterOutputs applied_outputs = thrusters_->Update();

        RCLCPP_INFO(log_, "%.02f\t%.02f\t%.02f\t%.02f\t%.02f\t%.02f\t%.02f\t%.02f\t",
                    applied_outputs[0], applied_outputs[1], applied_outputs[2], applied_outputs[3],
                    applied_outputs[4], applied_outputs[5], applied_outputs[6], applied_outputs[7]);
    }

    void JoyCallback(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        using namespace XBoxController;

        // float x_translation = msg->axes[static_cast<int>(Axis::LEFT_Y)];
        const float x_translation = msg->axes[std::to_underlying(Axis::LEFT_Y)];
        const float y_translation = msg->axes[std::to_underlying(Axis::LEFT_X)];
        const float z_rotation = -msg->axes[std::to_underlying(Axis::RIGHT_X)];
        const float y_rotation = -msg->axes[std::to_underlying(Axis::RIGHT_Y)];

        float left_trigger = msg->axes[std::to_underlying(Axis::LEFT_TRIGGER)];
        float right_trigger = msg->axes[std::to_underlying(Axis::RIGHT_TRIGGER)];

        // Remap triggers [1:-1] -> [0:1]
        left_trigger = -(left_trigger - 1.f);
        right_trigger = -(right_trigger - 1.f);

        const float z_translation = (right_trigger - left_trigger) / 2.f;

        // RCLCPP_INFO(log_, "X: %.03f\tY: %.03f\t Z: %.03f, Yaw: %.03f\t Pitch: %.03f",
        //             x_translation, y_translation, z_translation, z_rotation, y_rotation);

        // thrusters_->SetThrustVector({{x_translation, y_translation, z_translation, 0, y_rotation, z_rotation}});
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    const rclcpp::Logger log_;

    std::unique_ptr<Thrusters> thrusters_;
    size_t count_{};
    Thrusters::ThrustVector joy_tvec_;
};

int main(const int argc, char *argv[]) {
    std::cout << "Thruster node argc = " << argc << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterNode>());
    rclcpp::shutdown();
    return 0;
}
