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
        : Node("joint_impedance_test_client_dual"), sent_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Dual Arm Joint Impedance Test Client Initialized");

        // Action clients for both arms
        client_arm1_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_1_joint_impedance_controller/follow_joint_trajectory");

        client_arm2_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_2_joint_impedance_controller/follow_joint_trajectory");

        set_controller_parameters();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&JointImpedanceTestClient::send_goal, this));
    }

private:
    void set_controller_parameters()
    {
        std::vector<double> stiffness = {200, 200, 200, 200, 200, 200};
        std::vector<double> damping   = {5, 5, 5, 5, 5, 5};

        // Namespaced parameters
        this->declare_parameter("arm_1_joint_impedance_controller.stiffness", stiffness);
        this->declare_parameter("arm_1_joint_impedance_controller.damping", damping);

        this->declare_parameter("arm_2_joint_impedance_controller.stiffness", stiffness);
        this->declare_parameter("arm_2_joint_impedance_controller.damping", damping);

        RCLCPP_INFO(this->get_logger(), "Controller params declared for both arms");
    }

    FollowJointTrajectory::Goal create_goal(const std::vector<std::string>& joint_names)
    {
        FollowJointTrajectory::Goal goal_msg;
        goal_msg.trajectory.joint_names = joint_names;

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

        return goal_msg;
    }

    void send_goal()
    {
        timer_->cancel();

        if (sent_) return;

        // Wait for both servers
        if (!client_arm1_->wait_for_action_server(std::chrono::seconds(5)) ||
            !client_arm2_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "One or both action servers not available");
            rclcpp::shutdown();
            return;
        }

        // Joint names per arm (IMPORTANT: must match your URDF/controller)
        std::vector<std::string> arm1_joints = {
            "arm_1_joint1", "arm_1_joint2", "arm_1_joint3",
            "arm_1_joint4", "arm_1_joint5", "arm_1_joint6"};

        std::vector<std::string> arm2_joints = {
            "arm_2_joint1", "arm_2_joint2", "arm_2_joint3",
            "arm_2_joint4", "arm_2_joint5", "arm_2_joint6"};

        auto goal_arm1 = create_goal(arm1_joints);
        auto goal_arm2 = create_goal(arm2_joints);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        options.goal_response_callback = [this](std::shared_ptr<GoalHandleFollowJointTrajectory> handle)
        {
            if (!handle)
                RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            else
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
        };

        options.result_callback = [this](const GoalHandleFollowJointTrajectory::WrappedResult &)
        {
            RCLCPP_INFO(this->get_logger(), "Result received");
        };

        options.feedback_callback = [this](
            std::shared_ptr<GoalHandleFollowJointTrajectory>,
            const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
        {
            if (feedback)
            {
                std::ostringstream oss;
                oss << "Feedback: ";
                for (double p : feedback->actual.positions)
                    oss << p << ", ";
                RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
            }
        };

        // Send goals to both arms
        client_arm1_->async_send_goal(goal_arm1, options);
        client_arm2_->async_send_goal(goal_arm2, options);

        sent_ = true;
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_arm1_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_arm2_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool sent_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointImpedanceTestClient>());
    rclcpp::shutdown();
    return 0;
}