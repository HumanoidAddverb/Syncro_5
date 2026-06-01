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
    : Node("demo_effort")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_1_gravity_comp_effort_controller/commands", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&EffortPublisher::timer_callback, this));
    }

  private:
  
    void timer_callback()
    {
      std_msgs::msg::Float64MultiArray effort;
      effort.data.emplace_back(0.0);
      effort.data.emplace_back(0.0);
      effort.data.emplace_back(0.0);
      effort.data.emplace_back(0.0);
      effort.data.emplace_back(0.0);
      effort.data.emplace_back(0.0);      

      RCLCPP_INFO(this->get_logger(), "Publishing joint efforts");
      publisher_->publish(effort);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EffortPublisher>());
  rclcpp::shutdown();
  return 0;
}