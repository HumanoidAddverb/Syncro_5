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
    : Node("demo_velocity_dual")
    {
        velocity_arm_1_ = 0.0;
        velocity_arm_2_ = 0.0;

        update_rate_ = 30;

        pub_arm_1_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm_1_velocity_controller/commands", 10);

        pub_arm_2_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm_2_velocity_controller/commands", 10);

        timer_ = this->create_wall_timer(
            10ms, std::bind(&VelocityPublisher::timer_callback, this));
    }

    void run_phase(double v1, double v2, double duration)
    {
        velocity_arm_1_ = v1;
        velocity_arm_2_ = v2;

        RCLCPP_INFO(this->get_logger(),
                    "Arm1: %.2f | Arm2: %.2f for %.2f sec",
                    v1, v2, duration);

        rclcpp::Rate rate(update_rate_);
        auto start = this->now();

        while ((this->now() - start).seconds() < duration && rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

    void change_velocity()
    {
        RCLCPP_INFO(this->get_logger(), "Starting dual-arm demo...");

        // Forward slow
        run_phase(0.01, -0.01, 11);

        // Pause
        run_phase(0.0, 0.0, 0.1);

        // Reverse
        run_phase(-0.03, 0.03, 5);

        // Pause
        run_phase(0.0, 0.0, 0.5);

        // Faster forward
        run_phase(0.06, -0.06, 9);

        // Pause
        run_phase(0.0, 0.0, 0.8);

        // Fast reverse
        run_phase(-0.1, 0.1, 5);

        // Final stop
        run_phase(0.0, 0.0, 0.8);

        RCLCPP_INFO(this->get_logger(), "Demo Complete!");
    }

private:
    void publish_velocity(
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
        double vel)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {vel, 0, 0, 0, 0, 0};
        pub->publish(msg);
    }

    void timer_callback()
    {
        publish_velocity(pub_arm_1_, velocity_arm_1_);
        publish_velocity(pub_arm_2_, velocity_arm_2_);
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm_1_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm_2_;

    double velocity_arm_1_;
    double velocity_arm_2_;
    double update_rate_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPublisher>();
    node->change_velocity();
    rclcpp::shutdown();
    return 0;
}