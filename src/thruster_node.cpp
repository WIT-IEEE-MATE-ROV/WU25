#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "thrusters/thrusters.hpp"
#include "util/controller_map.hpp"
#include "drivers/pca9685.hpp"


using namespace std::chrono_literals;

static constexpr float MAX_THRUST_KGF = 2.4f;

class ThrusterNode final : public rclcpp::Node {
public:
    ThrusterNode() : Node("thruster_node"), log_(this->get_logger()) {
        RCLCPP_INFO(log_, "Starting thruster node");
        timer_ = this->create_wall_timer(20ms, [this] { Loop(); });

        joy_sub_ =
                this->create_subscription<sensor_msgs::msg::Joy>(
                        "joy", 3, std::bind(&ThrusterNode::JoyCallback, this, std::placeholders::_1));

        quat_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "bno/quat", 10, std::bind(&ThrusterNode::QuatCallback, this, std::placeholders::_1));

        thruster_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thrusters/output", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/thrusters/control_mode", 10);

        // PID parameters (runtime-tunable via ros2 param set)
        this->declare_parameter("rot_kp", 1.0);
        this->declare_parameter("rot_ki", 0.0);
        this->declare_parameter("rot_kd", 0.0);
        this->declare_parameter("rot_i_zone", static_cast<double>(std::numeric_limits<float>::max()));
        this->declare_parameter("rot_max_output", static_cast<double>(MAX_THRUST_KGF));
        this->declare_parameter("depth_kp", 0.0);
        this->declare_parameter("depth_ki", 0.0);
        this->declare_parameter("depth_kd", 0.0);

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                bool update_rot = false, update_depth = false;
                for (const auto& p : params) {
                    const auto& name = p.get_name();
                    if      (name == "rot_kp")        { rot_kp_ = static_cast<float>(p.as_double()); update_rot = true; }
                    else if (name == "rot_ki")        { rot_ki_ = static_cast<float>(p.as_double()); update_rot = true; }
                    else if (name == "rot_kd")        { rot_kd_ = static_cast<float>(p.as_double()); update_rot = true; }
                    else if (name == "rot_i_zone")    { rot_i_zone_ = static_cast<float>(p.as_double()); update_rot = true; }
                    else if (name == "rot_max_output"){ rot_max_output_ = static_cast<float>(p.as_double()); update_rot = true; }
                    else if (name == "depth_kp")      { depth_kp_ = static_cast<float>(p.as_double()); update_depth = true; }
                    else if (name == "depth_ki")      { depth_ki_ = static_cast<float>(p.as_double()); update_depth = true; }
                    else if (name == "depth_kd")      { depth_kd_ = static_cast<float>(p.as_double()); update_depth = true; }
                }
                if (update_rot && thrusters_) {
                    thrusters_->SetRotationPIDGains(rot_kp_, rot_ki_, rot_kd_, rot_i_zone_, rot_max_output_);
                    RCLCPP_INFO(log_, "Rotation PID updated: kp=%.3f ki=%.3f kd=%.3f", rot_kp_, rot_ki_, rot_kd_);
                }
                if (update_depth && thrusters_) {
                    thrusters_->SetDepthPIDGains(depth_kp_, depth_ki_, depth_kd_);
                    RCLCPP_INFO(log_, "Depth PID updated: kp=%.3f ki=%.3f kd=%.3f", depth_kp_, depth_ki_, depth_kd_);
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            }
        );

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
    }

    void QuatCallback(const geometry_msgs::msg::QuaternionStamped::UniquePtr &msg) {
        const auto &q = msg->quaternion;
        Eigen::Quaternionf qf(static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z));
        last_quat_ = qf;
        if (thrusters_) {
            thrusters_->SetRotation(qf);
        }
    }

    uint16_t negatePeriod(uint16_t period) {
        uint16_t difference = period - 1500;
        return 1500 - difference;
    }

    void Loop() {
        Thrusters::ThrusterOutputs applied_outputs = thrusters_->Update();

        Thrusters::PCAOutputs pca_outputs = thrusters_->GetPWMOutputs(applied_outputs);
        RCLCPP_INFO(log_, "%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u",
                    pca_outputs[0], pca_outputs[1], pca_outputs[2], pca_outputs[3],
                    pca_outputs[4], pca_outputs[5], pca_outputs[6], pca_outputs[7]);

        const int FLH = 6;
        const int FRH = 1;
        const int BLH = 7;
        const int BRH = 2;

        const int FLV = 4;
        const int FRV = 3;
        const int BLV = 5;
        const int BRV = 0;

        for (auto &period : pca_outputs) {
            if (period > 1490 && period < 1510) {
                period = 1505;
            } else if (period == 1505) {
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

        PCAError ret = pca_->set_period(0, remapped);
        if (ret != PCA_OK) {
            RCLCPP_ERROR(log_, "Failed to set PCA outputs");
        }

        // Publish normalized thruster forces (-1 to 1)
        std_msgs::msg::Float32MultiArray thruster_msg;
        thruster_msg.data.resize(8);
        for (size_t i = 0; i < 8; i++) {
            thruster_msg.data[i] = applied_outputs[i] / MAX_THRUST_KGF;
        }
        thruster_pub_->publish(thruster_msg);

        // Publish control mode state
        PublishControlMode();
    }

    void JoyCallback(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        using namespace XBoxController;

        const float x_translation = Deadband(msg->axes[std::to_underlying(Axis::LEFT_Y)], STICK_DEADBAND);
        const float y_translation = Deadband(-msg->axes[std::to_underlying(Axis::LEFT_X)], STICK_DEADBAND);
        const float z_rotation =    Deadband(-msg->axes[std::to_underlying(Axis::RIGHT_X)], STICK_DEADBAND);
        const float y_rotation =    Deadband(-msg->axes[std::to_underlying(Axis::RIGHT_Y)], STICK_DEADBAND);

        float left_trigger =  Deadband(msg->axes[std::to_underlying(Axis::LEFT_TRIGGER)], TRIGGER_DEADBAND);
        float right_trigger = Deadband(msg->axes[std::to_underlying(Axis::RIGHT_TRIGGER)], TRIGGER_DEADBAND);

        int32_t right_bumper = msg->buttons[std::to_underlying(Button::RIGHT_SHOULDER)];
        int32_t left_bumper = msg->buttons[std::to_underlying(Button::LEFT_SHOULDER)];
        int32_t y_button = msg->buttons[std::to_underlying(Button::Y)];

        // Remap triggers [1:-1] -> [0:1]
        left_trigger = -(left_trigger - 1.f);
        right_trigger = -(right_trigger - 1.f);

        const float z_translation = -(right_trigger - left_trigger) / 2.f;

        RCLCPP_INFO(log_, "X: %.03f\tY: %.03f\t Z: %.03f, Yaw: %.03f\t Pitch: %.03f",
                     x_translation, y_translation, z_translation, z_rotation, y_rotation);

        thrusters_->SetThrustVector({
                                            {x_translation * 3.f, y_translation * 3.f, z_translation * 4.5f},
                                            {0, -y_rotation, z_rotation}});

        // Toggle hold-idle-rotation on Y button rising edge
        if (y_button && !prev_y_button_) {
            hold_idle_rotation_enabled_ = !hold_idle_rotation_enabled_;
            thrusters_->SetHoldIdleRotation(hold_idle_rotation_enabled_);
            if (hold_idle_rotation_enabled_) {
                if (last_quat_.coeffs().norm() != 0.0f) {
                    thrusters_->SetDesiredRotation(last_quat_);
                }
                RCLCPP_INFO(log_, "Y pressed: enabled hold idle rotation");
            } else {
                RCLCPP_INFO(log_, "Y pressed: disabled hold idle rotation");
            }
        }

        if (right_bumper) {
            pca_->set_period(8, 1700);
        } else if (left_bumper) {
            pca_->set_period(8, 1300);
        }

        prev_y_button_ = y_button;
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
    void PublishControlMode() {
        std_msgs::msg::String msg;
        msg.data = std::string("{")
            + "\"hold_rotation\":" + (hold_idle_rotation_enabled_ ? "true" : "false")
            + ",\"hold_depth\":" + (hold_idle_depth_enabled_ ? "true" : "false")
            + ",\"ang_vel_control\":" + (ang_vel_control_enabled_ ? "true" : "false")
            + ",\"depth_lock\":" + (depth_lock_enabled_ ? "true" : "false")
            + "}";
        mode_pub_->publish(msg);
    }

    const float STICK_DEADBAND = 0.1;
    const float TRIGGER_DEADBAND = 0.1;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr quat_sub_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;
    const rclcpp::Logger log_;
    std::unique_ptr<Thrusters> thrusters_;
    std::unique_ptr<PCA9685> pca_;
    size_t count_{};
    Thrusters::ThrustVector joy_tvec_;
    Eigen::Quaternionf last_quat_{1, 0, 0, 0};
    int prev_y_button_{0};

    // Control mode state
    bool hold_idle_rotation_enabled_{false};
    bool hold_idle_depth_enabled_{false};
    bool ang_vel_control_enabled_{false};
    bool depth_lock_enabled_{false};

    // PID gain tracking (mirrors ROS params)
    float rot_kp_{1.0f}, rot_ki_{0.0f}, rot_kd_{0.0f};
    float rot_i_zone_{std::numeric_limits<float>::max()};
    float rot_max_output_{MAX_THRUST_KGF};
    float depth_kp_{0.0f}, depth_ki_{0.0f}, depth_kd_{0.0f};
};

int main(const int argc, char *argv[]) {
    std::cout << "Thruster node argc = " << argc << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterNode>());
    rclcpp::shutdown();
    return 0;
}
