/**
 * @file demo_cartesian_jogging.cpp
 * @author N Aswin Beckham(aswin.beckham@addverb.com)
 * @brief Demo to test CartesianJoggingController
 * @version 0.2
 * @date 2025-10-10
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>

using namespace std::chrono_literals;

class CartesianJoggingPublisher : public rclcpp::Node
{

public:
    CartesianJoggingPublisher() : Node("demo_cartesian_jogging")
    {

        update_rate_ = 30;
        duration_sec_ = 3;  // adjust base duration as needed

        //create publisher for cartesian jogging
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cartesian_jogging_controller/cartesian_jogging/command", 10);
    }

    void perform_demo()
    {

        rclcpp::Rate rate(update_rate_);

        RCLCPP_INFO(this->get_logger(), "Starting linear motions...");
        geometry_msgs::msg::Twist msg;

        // +X
        RCLCPP_INFO(this->get_logger(), "Linear +X for %.2d sec...", duration_sec_);
        msg.linear.x = 1;
        msg.linear.y = msg.linear.z = 0.0;
        msg.angular.x = msg.angular.y = msg.angular.z = 0.0;
        auto start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // -X
        RCLCPP_INFO(this->get_logger(), "Linear -X for %.2d sec...", duration_sec_);
        msg.linear.x = -1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // +Y
        RCLCPP_INFO(this->get_logger(), "Linear +Y for %.2d sec...", duration_sec_);
        msg.linear.x = 0.0;
        msg.linear.y = 1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // -Y
        RCLCPP_INFO(this->get_logger(), "Linear -Y for %.2d sec...", duration_sec_);
        msg.linear.y = -1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // +Z
        RCLCPP_INFO(this->get_logger(), "Linear +Z for %.2d sec...", duration_sec_);
        msg.linear.z = 1;
        msg.linear.y = 0.0;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // -Z
        RCLCPP_INFO(this->get_logger(), "Linear -Z for %.2d sec...", duration_sec_);
        msg.linear.z = -1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Starting angular motions...");
        // Angular motions (+X, -X, +Y, -Y, +Z, -Z)
        // +X
        RCLCPP_INFO(this->get_logger(), "Angular +X for %.2d sec...", duration_sec_);
        msg.angular.x = 1;
        msg.linear.x = msg.linear.y = msg.linear.z = 0.0;
        msg.angular.y = msg.angular.z = 0.0;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // -X
        RCLCPP_INFO(this->get_logger(), "Angular -X for %.2d sec...", duration_sec_);
        msg.angular.x = -1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // +Y
        RCLCPP_INFO(this->get_logger(), "Angular +Y for %.2d sec...", duration_sec_);
        msg.angular.x = 0.0;
        msg.angular.y = 1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // -Y
        RCLCPP_INFO(this->get_logger(), "Angular -Y for %.2d sec...", duration_sec_);
        msg.angular.y = -1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // +Z
        RCLCPP_INFO(this->get_logger(), "Angular +Z for %.2d sec...", duration_sec_);
        msg.angular.z = 1;
        msg.angular.y = 0.0;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // -Z
        RCLCPP_INFO(this->get_logger(), "Angular -Z for %.2d sec...", duration_sec_);
        msg.angular.z = -1;
        start = this->now();
        while ((this->now() - start).seconds() < duration_sec_ && rclcpp::ok())
        {
            publisher_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Motion sequence complete.");
        msg.angular.z = 0;
        publisher_->publish(msg);

        return;
    }
    
private:


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double update_rate_;
    int duration_sec_ ;  // adjust base duration as needed

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianJoggingPublisher>();
    node->perform_demo();
    rclcpp::shutdown();

    return 0;
}