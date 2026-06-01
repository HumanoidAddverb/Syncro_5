#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "addverb_cobot_msgs/msg/joint_jogging_velocity.hpp"

using namespace std::chrono_literals;

class DualArmJogging : public rclcpp::Node
{
public:
    DualArmJogging()
        : Node("dual_arm_joint_jogging")
    {
        // Publishers for both arms
        pub_arm1_ = this->create_publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>(
            "/arm_1_joint_jogging_controller/joint_jogging/command", 10);

        pub_arm2_ = this->create_publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>(
            "/arm_2_joint_jogging_controller/joint_jogging/command", 10);

        this->declare_parameter("num_joints", 6);
        this->declare_parameter("speed_factor", 1.0);

        num_joints_ = this->get_parameter("num_joints").as_int();

        // Start keyboard thread
        keyboard_thread_ = std::thread(&DualArmJogging::keyboardLoop, this);

        RCLCPP_INFO(this->get_logger(), "Press keys 1-%d to move joints in position direction.", num_joints_);
        RCLCPP_INFO(this->get_logger(), "Press keys q, w, e, r, t, y to move joints in negative direction.", num_joints_);

    }

    ~DualArmJogging()
    {
        running_ = false;
        if (keyboard_thread_.joinable())
            keyboard_thread_.join();
    }

private:
    void keyboardLoop()
    {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        fd_set readfds;
        struct timeval tv;

        while (running_)
        {
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);

            tv.tv_sec = 0;
            tv.tv_usec = 10000; // 10 ms

            int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

            if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds))
            {
                char c = getchar();

                if (c == 'x')
                {
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    rclcpp::shutdown();
                    break;
                }

                handleKey(c);

                // ✅ Update timestamp when key received
                last_key_time_ = std::chrono::steady_clock::now();
            }

            // 🔥 HOLD LOGIC
            auto now = std::chrono::steady_clock::now();
            double timeout = 0.1; // 100 ms

            if (active_joint_ != -1 &&
                std::chrono::duration<double>(now - last_key_time_).count() < timeout)
            {
                publishCommand(active_joint_, direction_);
            }
            else
            {
                active_joint_ = -1;
                publishZeroCommand();
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
    
    void handleKey(char c)
    {
        // Positive
        if (c >= '1' && c <= '0' + num_joints_)
        {
            active_joint_ = c - '1';
            direction_ = 1.0;
            return;
        }

        // Negative
        std::string neg_keys = "qwerty";
        for (int i = 0; i < num_joints_; i++)
        {
            if (c == neg_keys[i])
            {
                active_joint_ = i;
                direction_ = -1.0;
                return;
            }
        }

        // Unknown key → stop
        active_joint_ = -1;
    }
        
    void publishZeroCommand()
    {
        // Example: send zero velocity to all joints
        auto msg = addverb_cobot_msgs::msg::JointJoggingVelocity();
        msg.jvel_scaling_factor.resize(num_joints_, 0.0);
        
        pub_arm1_->publish(msg);
        pub_arm2_->publish(msg);
    }

    void publishCommand(int joint_index, double direction)
    {
        auto msg = addverb_cobot_msgs::msg::JointJoggingVelocity();
        msg.jvel_scaling_factor.resize(num_joints_, 0.0);

        double speed = this->get_parameter("speed_factor").as_double();

        msg.jvel_scaling_factor[joint_index] = direction * speed; //reverse the direction for arm 1

        pub_arm1_->publish(msg);
        
        msg.jvel_scaling_factor[joint_index] = direction * speed;
        
        pub_arm2_->publish(msg);

    }

    rclcpp::Publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>::SharedPtr pub_arm1_;
    rclcpp::Publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>::SharedPtr pub_arm2_;

    int num_joints_;
    std::thread keyboard_thread_;
    bool running_ = true;
    std::chrono::steady_clock::time_point last_key_time_;
    int active_joint_ = -1;
    double direction_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualArmJogging>());
    rclcpp::shutdown();
    return 0;
}