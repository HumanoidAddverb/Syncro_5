/**
 * @file demo_joint_impedance.cpp
 * @author Aditya Pawar
 * @brief Demo to test JointImpedanceController using ROS2 executable
 * @version 0.1
 * @date 2025-07-14
 */

#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class JointImpedanceTestClient : public rclcpp::Node
{
public:
    JointImpedanceTestClient()
        : Node("joint_impedance_test_client"), sent_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Joint Impedance Test Client Node Initialized");

        client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/joint_impedance_controller/follow_joint_trajectory");

        set_controller_parameters();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&JointImpedanceTestClient::send_goal, this));
    }

private:
    void set_controller_parameters()
    {
        std::vector<double> stiffness_values = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
        std::vector<double> damping_values = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

        this->declare_parameter("joint_impedance_controller.stiffness", stiffness_values);
        this->declare_parameter("joint_impedance_controller.damping", damping_values);

        RCLCPP_INFO(this->get_logger(), "Controller parameters declared (local to this node).");
    }

    void send_goal()
    {
        timer_->cancel();

        if (sent_)
            return;

        if (!client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = {0.45, 0.0, 0.0, 0.0, 0.0, 0.1};
        pt.time_from_start = rclcpp::Duration::from_seconds(5.0);

        goal_msg.trajectory.points.push_back(pt);

        pt.positions = {0.9, 0.0, 0.0, 0.0, 0.0, 0.2};
        pt.time_from_start = rclcpp::Duration::from_seconds(10.0);

        goal_msg.trajectory.points.push_back(pt);

        pt.positions = {1.45, 0.0, 0.0, 0.0, 0.0, 0.3};
        pt.time_from_start = rclcpp::Duration::from_seconds(15.0);
        goal_msg.trajectory.points.push_back(pt);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        options.goal_response_callback = [this](std::shared_ptr<GoalHandleFollowJointTrajectory> handle)
        {
            if (!handle)
                RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            else
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
        };
        options.result_callback = [this](const GoalHandleFollowJointTrajectory::WrappedResult &result)
        {
            RCLCPP_INFO(this->get_logger(), "Result received");
            rclcpp::shutdown();
        };
        options.feedback_callback = [this](
                                        std::shared_ptr<GoalHandleFollowJointTrajectory>,
                                        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
        {
            if (feedback)
            {
                std::ostringstream oss;
                oss << "Actual positions: ";
                for (size_t i = 0; i < feedback->actual.positions.size(); ++i)
                {
                    oss << feedback->actual.positions[i];
                    if (i + 1 < feedback->actual.positions.size())
                        oss << ", ";
                }
                RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
            }
        };

        client_->async_send_goal(goal_msg, options);
        sent_ = true;
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool sent_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointImpedanceTestClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
