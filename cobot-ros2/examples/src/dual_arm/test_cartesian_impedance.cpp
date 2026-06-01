#include <memory>
#include <vector>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class CartesianImpedanceTestClient : public rclcpp::Node
{
public:
    CartesianImpedanceTestClient()
        : Node("cartesian_impedance_test_client_dual"), sent_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Dual Arm Cartesian Impedance Client Initialized");

        // Action clients
        client_arm1_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_1_cartesian_impedance_controller/follow_joint_trajectory");

        client_arm2_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_2_cartesian_impedance_controller/follow_joint_trajectory");

        // Optional runtime parameter setting
        set_controller_parameters("arm_1_cartesian_impedance_controller");
        set_controller_parameters("arm_2_cartesian_impedance_controller");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CartesianImpedanceTestClient::send_goal, this));
    }

private:

    void set_controller_parameters(const std::string &controller_ns)
    {
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
            this, "/controller_manager/" + controller_ns);

        if (!param_client->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_WARN(this->get_logger(), "Param service not available for %s", controller_ns.c_str());
            return;
        }

        std::vector<double> stiffness = {1500, 1000, 2000, 250, 250, 250};
        std::vector<double> damping   = {5, 5, 5, 5, 5, 5};
        std::vector<double> mass      = {100, 100, 100, 100, 100, 100};
        std::vector<double> ft_force  = {0, 0, 0, 0, 0, 0};
        std::vector<double> target    = {0, 0, 0, 0, 0, 0};

        param_client->set_parameters({
            rclcpp::Parameter(controller_ns + ".stiffness", stiffness),
            rclcpp::Parameter(controller_ns + ".damping", damping),
            rclcpp::Parameter(controller_ns + ".mass_matrix", mass),
            rclcpp::Parameter(controller_ns + ".ft_force", ft_force),
            rclcpp::Parameter(controller_ns + ".target_force", target)
        });

        RCLCPP_INFO(this->get_logger(), "Params set for %s", controller_ns.c_str());
    }

    FollowJointTrajectory::Goal create_goal(const std::vector<std::string> &joint_names)
    {
        FollowJointTrajectory::Goal goal_msg;
        goal_msg.trajectory.joint_names = joint_names;

        trajectory_msgs::msg::JointTrajectoryPoint pt;

        pt.positions = {0.4, 0.0, 0.0, 0.0, 0.0, 0.1};
        pt.time_from_start = rclcpp::Duration::from_seconds(5.0);
        goal_msg.trajectory.points.push_back(pt);

        return goal_msg;
    }

    void send_goal()
    {
        timer_->cancel();
        if (sent_) return;

        if (!client_arm1_->wait_for_action_server(std::chrono::seconds(5)) ||
            !client_arm2_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            rclcpp::shutdown();
            return;
        }

        // Joint names per arm
        std::vector<std::string> arm1_joints = {
            "arm1_joint1", "arm1_joint2", "arm1_joint3",
            "arm1_joint4", "arm1_joint5", "arm1_joint6"};

        std::vector<std::string> arm2_joints = {
            "arm2_joint1", "arm2_joint2", "arm2_joint3",
            "arm2_joint4", "arm2_joint5", "arm2_joint6"};

        auto goal_arm1 = create_goal(arm1_joints);
        auto goal_arm2 = create_goal(arm2_joints);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        options.goal_response_callback = [this](auto handle)
        {
            if (!handle)
                RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            else
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
        };

        options.feedback_callback = [this](auto, auto feedback)
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

        options.result_callback = [this](auto)
        {
            RCLCPP_INFO(this->get_logger(), "Execution done");
            rclcpp::shutdown();
        };

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
    rclcpp::spin(std::make_shared<CartesianImpedanceTestClient>());
    rclcpp::shutdown();
    return 0;
}