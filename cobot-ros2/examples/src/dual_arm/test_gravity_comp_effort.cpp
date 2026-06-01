#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class EffortPublisher : public rclcpp::Node
{
public:
  EffortPublisher()
  : Node("demo_effort_two_arm")
  {
    // Publisher for arm1
    pub_arm1_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_1_gravity_comp_effort_controller/commands", 10);

    // Publisher for arm2
    pub_arm2_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_2_gravity_comp_effort_controller/commands", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&EffortPublisher::timer_callback, this));
  }

private:

  std_msgs::msg::Float64MultiArray create_effort_msg()
  {
    std_msgs::msg::Float64MultiArray effort;
    effort.data = std::vector<double>(6, 0.0);  // 6 joints
    return effort;
  }

  void timer_callback()
  {
    auto effort_arm1 = create_effort_msg();
    auto effort_arm2 = create_effort_msg();

    // Example: differentiate arms if needed
    // effort_arm2.data[0] = 0.5;

    RCLCPP_INFO(this->get_logger(), "Publishing efforts for arm1 and arm2");

    pub_arm1_->publish(effort_arm1);
    pub_arm2_->publish(effort_arm2);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm1_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EffortPublisher>());
  rclcpp::shutdown();
  return 0;
}