#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "thrusters/thrusters.hpp"
#include "util/controller_map.hpp"
#include "drivers/pca9685.hpp"


using namespace std::chrono_literals;

class ThrusterNode final : public rclcpp::Node {
public:
    ThrusterNode() : Node("thruster_node"), log_(this->get_logger()) {
        RCLCPP_INFO(log_, "Starting thruster node");
        timer_ = this->create_wall_timer(20ms, [this] { Loop(); });

        joy_sub_ =
                this->create_subscription<sensor_msgs::msg::Joy>(
                        "joy", 3, std::bind(&ThrusterNode::JoyCallback, this, std::placeholders::_1));

        thrusters_ = std::make_unique<Thrusters>();
        thrusters_->Init();

        pca_ = std::make_unique<PCA9685>();
        PCAError ret = pca_->init(0x40, 100);
        if (ret != PCA_OK) {
            RCLCPP_INFO(log_, "Failed to initialize PCA9685");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(log_, "Initializing thrusters...");
        std::vector<uint16_t> init_vec{1500, 1500, 1500, 1500, 1500, 1500, 1500};
        pca_->set_period(0, init_vec);
        std::this_thread::sleep_for(2s);

        RCLCPP_INFO(log_, "Initialize finished\n");
//
//        Vector<float, 6> v{1, 0, 0, 0, 0, 0};
//        std::cout << "Calling constructor" << std::endl;
//        Thrusters::ThrustVector tvec(v);
//        thrusters_->SetThrustVector(tvec);
//        Quaternionf desired =
//                AngleAxisf(0, Vector3f::UnitX()) *
//                AngleAxisf(0, Vector3f::UnitY()) *
//                AngleAxisf(90.f * M_PIf / 180.f, Vector3f::UnitZ());
//
//        thrusters_->SetDesiredRotation(desired);
    }

    uint16_t negatePeriod(uint16_t period) {
        uint16_t difference = period - 1500;
        return 1500 - difference;
    }

    void Loop() {
        Thrusters::ThrusterOutputs applied_outputs = thrusters_->Update();

//        RCLCPP_INFO(log_, "%.02f\t%.02f\t%.02f\t%.02f\t%.02f\t%.02f\t%.02f\t%.02f",
//                    applied_outputs[0], applied_outputs[1], applied_outputs[2], applied_outputs[3],
//                    applied_outputs[4], applied_outputs[5], applied_outputs[6], applied_outputs[7]);

        Thrusters::PCAOutputs pca_outputs = thrusters_->GetPWMOutputs(applied_outputs);
        RCLCPP_INFO(log_, "%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u",
                    pca_outputs[0], pca_outputs[1], pca_outputs[2], pca_outputs[3],
                    pca_outputs[4], pca_outputs[5], pca_outputs[6], pca_outputs[7]);

//        const int FLH = 4;
//        const int FRH = 3;
//        const int BLH = 7;
//        const int BRH = 2;
//
//        const int FLV = 6;
//        const int FRV = 1;
//        const int BLV = 5;
//        const int BRV = 0; // 0

        const int FLH = 6;
        const int FRH = 1;
        const int BLH = 7;
        const int BRH = 2;

        const int FLV = 4;
        const int FRV = 3;
        const int BLV = 5;
        const int BRV = 0; // 0

        for (auto &period : pca_outputs) {
            if (period > 1490 && period < 1510) {
                period = 1505;
            }else if (period == 1505) {
                period = 1495;
            } else if (period == 1495) {
                period = 1505;
            }
        }

        std::vector<uint16_t> remapped(8);
        remapped[FLH] = negatePeriod(pca_outputs[0]);
        remapped[FRH] = negatePeriod(pca_outputs[1]);
        remapped[BLH] = negatePeriod(pca_outputs[2]);
        remapped[BRH] = pca_outputs[3];

        remapped[FLV] = pca_outputs[4];
        remapped[FRV] = pca_outputs[5];
        remapped[BLV] = pca_outputs[6];
        remapped[BRV] = pca_outputs[7];


//        std::vector outs(pca_outputs.begin(), pca_outputs.end());
        PCAError ret = pca_->set_period(0, remapped);
//        PCAError ret = pca_->set_period(6, 1700);
        if (ret != PCA_OK) {
            RCLCPP_ERROR(log_, "Failed to set PCA outputs");
        }

    }

    void JoyCallback(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        using namespace XBoxController;

//        const float x_translation = GetExponential(msg->axes[std::to_underlying(Axis::LEFT_Y)], 1, 1, STICK_DEADBAND);
//        const float y_translation = GetExponential(msg->axes[std::to_underlying(Axis::LEFT_X)], 1, 1, STICK_DEADBAND);
//        const float z_rotation =    GetExponential(-msg->axes[std::to_underlying(Axis::RIGHT_X)], 1, 1, STICK_DEADBAND);
//        const float y_rotation =    GetExponential(-msg->axes[std::to_underlying(Axis::RIGHT_Y)], 1, 1, STICK_DEADBAND);
//
//        float left_trigger =  GetExponential(msg->axes[std::to_underlying(Axis::LEFT_TRIGGER)], 1, 1, TRIGGER_DEADBAND);
//        float right_trigger = GetExponential(msg->axes[std::to_underlying(Axis::RIGHT_TRIGGER)], 1, 1,
//                                             TRIGGER_DEADBAND);
        const float x_translation = Deadband(msg->axes[std::to_underlying(Axis::LEFT_Y)], STICK_DEADBAND);
        const float y_translation = Deadband(-msg->axes[std::to_underlying(Axis::LEFT_X)], STICK_DEADBAND);
        const float z_rotation =    Deadband(-msg->axes[std::to_underlying(Axis::RIGHT_X)], STICK_DEADBAND);
        const float y_rotation =    Deadband(-msg->axes[std::to_underlying(Axis::RIGHT_Y)], STICK_DEADBAND);

        float left_trigger =  Deadband(msg->axes[std::to_underlying(Axis::LEFT_TRIGGER)], TRIGGER_DEADBAND);
        float right_trigger = Deadband(msg->axes[std::to_underlying(Axis::RIGHT_TRIGGER)], TRIGGER_DEADBAND);

        int32_t right_bumper = msg->buttons[std::to_underlying(Button::RIGHT_SHOULDER)];
        int32_t left_bumper = msg->buttons[std::to_underlying(Button::LEFT_SHOULDER)];

//        int32_t


        // Remap triggers [1:-1] -> [0:1]
        left_trigger = -(left_trigger - 1.f);
        right_trigger = -(right_trigger - 1.f);

        const float z_translation = -(right_trigger - left_trigger) / 2.f;

         RCLCPP_INFO(log_, "X: %.03f\tY: %.03f\t Z: %.03f, Yaw: %.03f\t Pitch: %.03f",
                     x_translation, y_translation, z_translation, z_rotation, y_rotation);

        thrusters_->SetThrustVector({
                                            {x_translation * 3.f, y_translation * 3.f,     z_translation * 4.5f},
                                            {0,             -y_rotation, z_rotation}});
        if (right_bumper) {
            pca_->set_period(8, 1700);
        } else if (left_bumper) {
            pca_->set_period(8, 1300);
        }
    }

    template<typename T>
    int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    float Deadband(float input, float deadband) {
        if (std::fabs(input) < deadband) {
            return 0.f;
        }

        return input;
    }

    float GetExponential(float input, float exponent, float weight, float deadband) {
        float v = std::fabs(input);
        if (v < deadband) {
            return 0;
        }

        int sign = sgn(input);

        float a = weight * powf(v, exponent) + (1.f - weight) * v;
        float b = weight * powf(deadband, exponent) + (1.f - weight) * deadband;
        v = (a - 1.f * b) / (1.f - b);

        return v * static_cast<float>(sign);
    }

private:
    const float STICK_DEADBAND = 0.1;
    const float TRIGGER_DEADBAND = 0.1;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    const rclcpp::Logger log_;
    std::unique_ptr<Thrusters> thrusters_;
    std::unique_ptr<PCA9685> pca_;
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
