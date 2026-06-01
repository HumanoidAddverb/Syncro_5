/**
 * @file demo_joint_jogging.cpp
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Demo on how to use JointJoggingController
 * @version 0.1
 * @date 2025-07-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "addverb_cobot_msgs/msg/joint_jogging_velocity.hpp"

using namespace std::chrono_literals;

class JointJoggingPublisher : public rclcpp::Node
{
public:
    JointJoggingPublisher()
        : Node("demo_joint_jogging")
    {
        // Create publisher for joint jogging commands
        publisher_ = this->create_publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>(
            "/arm_1_joint_jogging_controller/joint_jogging/command", 10);

        // Create timer to publish commands at regular intervals
        timer_ = this->create_wall_timer(
            500ms, std::bind(&JointJoggingPublisher::timer_callback, this));

        this->declare_parameter("num_joints", 6);
        num_joints_ = this->get_parameter("num_joints").as_int();

        // Declare parameter for jogging speed factor 
        this->declare_parameter("speed_factor", 1.0);

        // Initialize joint jogging sequence variables
        current_joint_ = 0;
        going_forward_ = true;

        // way to switch which joint is running
        joint_timer_ = this->create_wall_timer(
            3000ms, std::bind(&JointJoggingPublisher::switch_joint, this));

        RCLCPP_INFO(this->get_logger(), "Starting with joint 0, will switch every 2 seconds");
    }

private:
    // Switch to the next joint in the sequence
    void switch_joint()
    {
    
        if (going_forward_)
        {
            current_joint_++;
            if (current_joint_ > num_joints_ - 1)
            {
                current_joint_ = num_joints_ - 1;
                going_forward_ = false;
            }
        }
        else
        {
            current_joint_--;
            if (current_joint_ < 0)
            {
                current_joint_ = 0;
                joint_timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Demo Complete!");
                rclcpp::shutdown(); // end the demo
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Switching to joint %d", current_joint_);
    }

    void timer_callback()
    {
        auto msg = addverb_cobot_msgs::msg::JointJoggingVelocity();

        // Initialize all joints to zero velocity
        msg.jvel_scaling_factor.resize(num_joints_, 0.0);

        // Get speed factor from parameter
        double speed_factor = this->get_parameter("speed_factor").as_double();

        if (!going_forward_)
        {
            speed_factor = -speed_factor; // Reverse the speed factor if going backward
        }

        // Set velocity for the current joint in the sequence
        msg.jvel_scaling_factor[current_joint_] = speed_factor;

        // RCLCPP_INFO(this->get_logger(),
        //             "Publishing joint jogging command: Joint %d with velocity factor %.2f",
        //             current_joint_, speed_factor);

        // Publish the message
        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr joint_timer_;
    rclcpp::Publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>::SharedPtr publisher_;
    int num_joints_;
    int current_joint_;
    bool going_forward_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointJoggingPublisher>());
    rclcpp::shutdown();
    return 0;
}
