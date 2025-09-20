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
        // RCLCPP_INFO(log_, "%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u",
        //             pca_outputs[0], pca_outputs[1], pca_outputs[2], pca_outputs[3],
        //             pca_outputs[4], pca_outputs[5], pca_outputs[6], pca_outputs[7]);

//        const int FLH = 4;
//        const int FRH = 3;
//        const int BLH = 7;
//        const int BRH = 2;
//
//        const int FLV = 6;
//        const int FRV = 1;
//        const int BLV = 5;
//        const int BRV = 0; // 0

        // const int FLH = 6;
        // const int FRH = 1;
        // const int BLH = 7;
        // const int BRH = 2;

        // const int FLV = 4;
        // const int FRV = 3;
        // const int BLV = 5;
        // const int BRV = 0; // 0

        const int FLH = 5;
        const int FRH = 0; // 0
        const int BLH = 1;
        const int BRH = 2;

        const int FLV = 7;
        const int FRV = 4;
        const int BLV = 3;
        const int BRV = 6;

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
        
        


    //    std::vector outs(pca_outputs.begin(), pca_outputs.end());
        PCAError ret = pca_->set_period(0, remapped);
    //    PCAError ret = pca_->set_period(7, 1700);
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

        int32_t right_stick = msg->buttons[std::to_underlying(Button::RIGHT_STICK)];
        int32_t left_stick = msg->buttons[std::to_underlying(Button::LEFT_STICK)];

        int32_t right_bumper = msg->buttons[std::to_underlying(Button::RIGHT_BUMPER)];
        int32_t left_bumper = msg->buttons[std::to_underlying(Button::LEFT_BUMPER)];
        int32_t a_button = msg->buttons[std::to_underlying(Button::A)];
        int32_t b_button = msg->buttons[std::to_underlying(Button::B)];
        int32_t y_button = msg->buttons[std::to_underlying(Button::Y)];

        int32_t up_dpad = msg->axes[std::to_underlying(Axis::DPAD_UP_DOWN)] > 0;
        int32_t down_dpad = msg->axes[std::to_underlying(Axis::DPAD_UP_DOWN)] < 0;
        int32_t left_dpad = msg->axes[std::to_underlying(Axis::DPAD_LEFT_RIGHT)] > 0;
        int32_t right_dpad = msg->axes[std::to_underlying(Axis::DPAD_LEFT_RIGHT)] < 0;

        // Remap triggers [1:-1] -> [0:1]
        left_trigger = -(left_trigger - 1.f);
        right_trigger = -(right_trigger - 1.f);

        const float z_translation = -(right_trigger - left_trigger) / 2.f;

         RCLCPP_INFO(log_, "X: %.03f\tY: %.03f\t Z: %.03f, Yaw: %.03f\t Pitch: %.03f",
                     x_translation, y_translation, z_translation, z_rotation, y_rotation);

        thrusters_->SetThrustVector({
                                    {x_translation * 3.f, y_translation * 3.f,     z_translation * 4.5f},
                                    {0,             -y_rotation, z_rotation}});

        if (right_stick) {
            RCLCPP_INFO(log_, "Right Stick");
            pca_->set_period(8, 1800);
        } else if (left_stick) {
            RCLCPP_INFO(log_, "Left Stick");
            pca_->set_period(8, 1300);
        }

        if (b_button) {
            RCLCPP_INFO(log_, "B");
            pca_->set_period(9, 1900);
            pca_->set_period(10, 1900);
            pca_->set_period(11, 1900);
            pca_->set_period(15, 1900);
        } else if (previous_b_button && !b_button) {
            pca_->set_period(9, 1100);
            pca_->set_period(10, 1100);
            pca_->set_period(11, 1100);
            pca_->set_period(15, 1100);
        }
        // if (y_button) {
        //     RCLCPP_INFO(log_, "Y");
        //     pca_->set_period(9, 1100);
        //     pca_->set_period(10, 1100);
        //     pca_->set_period(11, 1100);
        //     pca_->set_period(15, 1100);
        // }

        if (right_bumper) {
            RCLCPP_INFO(log_, "Right Bumper");
            // pca_->set_period(9, 1800);
            // pca_->set_period(10, 1800);
            // pca_->set_period(11, 1800);
            // pca_->set_period(12, 1800);
            // pca_->set_period(13, 1800);
            pca_->set_period(PWM_GRIPPER_ROLL, 2350);
            // pca_->set_period(15, 1800);

        } else if (left_bumper) {
            RCLCPP_INFO(log_, "Left Bumper");
            // pca_->set_period(9, 1300);
            // pca_->set_period(10, 1300);
            // pca_->set_period(11, 1300);
            // pca_->set_period(12, 1300);
            // pca_->set_period(13, 1300);
            pca_->set_period(PWM_GRIPPER_ROLL, 650);
            // pca_->set_period(15, 1300);
        } else if (a_button) {
            RCLCPP_INFO(log_, "A");
            // pca_->set_period(9, 1500);
            // pca_->set_period(10, 1500);
            // pca_->set_period(11, 1500);
            // pca_->set_period(12, 1500);
            // pca_->set_period(13, 1500);
            pca_->set_period(PWM_GRIPPER_ROLL, 1500);
            // pca_->set_period(15, 1500);
        }


        if (up_dpad) {
            RCLCPP_INFO(log_, "Up DPad");
            pca_->set_period(PWM_FISH_NET, 1700);
        } else if (down_dpad) {
            RCLCPP_INFO(log_, "Down DPad");
            pca_->set_period(PWM_FISH_NET, 650);
        } else if (a_button) {
            RCLCPP_INFO(log_, "A");
            pca_->set_period(PWM_FISH_NET, 1500);
        }

        if (left_dpad) {
            RCLCPP_INFO(log_, "Left DPad");
            pca_->set_period(PWM_POLYP_GRAB, 2350);
        } else if (right_dpad) {
            RCLCPP_INFO(log_, "Right DPad");
            pca_->set_period(PWM_POLYP_GRAB, 650);
        } else if (a_button) {
            RCLCPP_INFO(log_, "A");
            pca_->set_period(PWM_POLYP_GRAB, 1500);
        }
        previous_b_button = b_button;
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

    
    const int32_t PWM_GRIPPER_ROLL = 14;
    const int32_t PWM_POLYP_GRAB = 13;
    const int32_t PWM_FISH_NET = 12;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    const rclcpp::Logger log_;
    std::unique_ptr<Thrusters> thrusters_;
    std::unique_ptr<PCA9685> pca_;
    size_t count_{};
    Thrusters::ThrustVector joy_tvec_;
    int32_t previous_b_button = 0;
};

int main(const int argc, char *argv[]) {
    std::cout << "Thruster node argc = " << argc << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterNode>());
    rclcpp::shutdown();
    return 0;
}
