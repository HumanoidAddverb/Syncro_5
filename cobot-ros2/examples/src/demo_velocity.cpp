/**
 * @file demo_velocity.cpp
 * @author N.Aswin Beckham (aswin.beckham@addverb.com)
 * @brief Velocity controller - Demo:  sends different velocity commands to the robot
 * @version 0.1
 * @date 2025-10-13
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node
{
    public:
    VelocityPublisher()
    : Node("demo_velocity")
    {
      velocity_ = 0.0;
      update_rate_ = 30;
      duration_sec_ = 0;  // adjust base duration as needed

      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_1_velocity_controller/commands", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&VelocityPublisher::timer_callback, this));
    }


    void change_velocity()
    {
      rclcpp::Rate rate(update_rate_);
      RCLCPP_INFO(this->get_logger(), "Starting demo...");


      velocity_ = 0.01;
      duration_sec_ = 11;
      RCLCPP_INFO(this->get_logger(), "Velocity = %.2f for a duration of %.2d sec...", velocity_, duration_sec_);
      auto start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }

      velocity_ = 0.0;
      duration_sec_ = 0.1;
      start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }

      velocity_ = -0.03;
      duration_sec_ = 5;
      RCLCPP_INFO(this->get_logger(), "Velocity = %.2f for a duration of %.2d sec...", velocity_, duration_sec_);
      start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }

      velocity_ = 0.0;
      duration_sec_ = 0.5;
      start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }

      velocity_ = 0.06;
      duration_sec_ = 9;
      RCLCPP_INFO(this->get_logger(), "Velocity = %.2f for a duration of %.2d sec...", velocity_, duration_sec_);
      start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }

      velocity_ = 0.0;
      duration_sec_ = 0.8;
      start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }

      velocity_ = -0.1;
      duration_sec_ = 5;
      RCLCPP_INFO(this->get_logger(), "Velocity = %.2f for a duration of %.2d sec...", velocity_, duration_sec_);
      start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }


      velocity_ = 0.0;  //pause
      duration_sec_ = 0.8;
      start = this->now();
      while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
      {
          rclcpp::spin_some(this->get_node_base_interface());
          rate.sleep();
      }

    RCLCPP_INFO(this->get_logger(), "Demo Complete!");
    // rclcpp::shutdown(); // end the demo
    return;
    }


  private:
  

    void timer_callback()
    {
      std_msgs::msg::Float64MultiArray velocity_data_;
      velocity_data_.data.emplace_back(velocity_);
      velocity_data_.data.emplace_back(0);
      velocity_data_.data.emplace_back(0);
      velocity_data_.data.emplace_back(0);
      velocity_data_.data.emplace_back(0);
      velocity_data_.data.emplace_back(0);      

      // RCLCPP_INFO(this->get_logger(), "Publishing joint velocities");
      publisher_->publish(velocity_data_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    double velocity_;
    double update_rate_;
    int duration_sec_ ;  // adjust base duration as needed

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityPublisher>();
  node->change_velocity();
  rclcpp::shutdown();
  return 0;
}