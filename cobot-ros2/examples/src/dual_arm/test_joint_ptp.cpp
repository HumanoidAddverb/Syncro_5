#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <functional>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ptp
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class DualArmPTPClient : public rclcpp::Node
{
public:
    explicit DualArmPTPClient(const rclcpp::NodeOptions &options)
        : Node("dual_arm_ptp_client")
    {
        // --- Create action clients ---
        client_arm1_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_1_ptp_joint_controller/follow_joint_trajectory");

        client_arm2_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_2_ptp_joint_controller/follow_joint_trajectory");

        // --- Direction correction ---
        arm1_sign_ = {1, 1, 1, 1, 1, 1};
        arm2_sign_ = {1, 1, 1, 1, 1, 1};
        
        arm1_name = "arm_1";
        arm2_name = "arm_2";
        

        // --- Timer to send goal ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DualArmPTPClient::sendGoal, this));

        RCLCPP_INFO(this->get_logger(), "Dual arm PTP client started");
    }

private:
    //  Apply sign correction
    std::vector<double> applySign(
        const std::vector<double> &input,
        const std::vector<double> &sign)
    {
        std::vector<double> out = input;
        for (size_t i = 0; i < out.size(); ++i)
            out[i] *= sign[i];
        return out;
    }

    //  Create trajectory goal
    FollowJointTrajectory::Goal createGoal(const std::vector<double> &sign, const std::string& arm_name)
    {
        FollowJointTrajectory::Goal goal_msg;

        goal_msg.trajectory.joint_names = {
            arm_name + "_joint1", arm_name + "_joint2", arm_name + "_joint3",
            arm_name + "_joint4", arm_name + "_joint5", arm_name + "_joint6"};

        trajectory_msgs::msg::JointTrajectoryPoint pt;

        auto add_point = [&](std::vector<double> pos, int t_sec)
        {
            pt.positions = applySign(pos, sign);
            pt.time_from_start.sec = t_sec;
            pt.time_from_start.nanosec = 0;
            goal_msg.trajectory.points.push_back(pt);
        };

        goal_msg.trajectory.points.clear();

        
        add_point({-0.5, -0.3, -0.3, 0, 0, 0}, 10);
        add_point({-0.5, -1.57, -1.57, 0, 0, 0}, 15);
        add_point({-1.57, -1.57, -1.57, 0, 0, 0}, 20);
        



        

        return goal_msg;
    }

    void sendGoal()
    {
        timer_->cancel();

        if (once_)
            return;

        // Wait for both servers
        if (!client_arm1_->wait_for_action_server(std::chrono::seconds(5)) ||
            !client_arm2_->wait_for_action_server(std::chrono::seconds(5)))
        // if (!client_arm2_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "One or both action servers not available");
            rclcpp::shutdown();
            return;
        }

        // Create goals
        auto goal_arm1 = createGoal(arm1_sign_, arm1_name);
        auto goal_arm2 = createGoal(arm2_sign_, arm2_name);

        // Send goals
        sendToClient(client_arm1_, goal_arm1, arm1_name);
        sendToClient(client_arm2_, goal_arm2, arm2_name);

        once_ = true;
    }

    void sendToClient(
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
        const FollowJointTrajectory::Goal &goal,
        const std::string &name)
    {
        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        options.goal_response_callback =
            [this, name](std::shared_ptr<GoalHandleFollowJointTrajectory> handle)
        {
            if (!handle)
                RCLCPP_ERROR(this->get_logger(), "%s goal rejected", name.c_str());
            else
                RCLCPP_INFO(this->get_logger(), "%s goal accepted", name.c_str());
        };

        options.result_callback =
            [this, name](const GoalHandleFollowJointTrajectory::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "%s goal succeeded", name.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "%s goal aborted", name.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "%s goal canceled", name.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "%s unknown result code", name.c_str());
                break;
            }
        };

        options.feedback_callback =
            [this, name](std::shared_ptr<GoalHandleFollowJointTrajectory>,
                         const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
        {
            // if (feedback && !feedback->actual.positions.empty())
            // {
            //     std::ostringstream oss;
            //     oss << name << " feedback: [";
            //     for (size_t i = 0; i < feedback->actual.positions.size(); ++i)
            //     {
            //         oss << feedback->actual.positions[i];
            //         if (i + 1 < feedback->actual.positions.size())
            //             oss << ", ";
            //     }
            //     oss << "]";
            //     RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
            // }
        };

        client->async_send_goal(goal, options);
    }

    // --- Members ---
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_arm1_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_arm2_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> arm1_sign_;
    std::vector<double> arm2_sign_;
    
    std::string arm1_name;
    std::string arm2_name;

    bool once_ = false;
};

} // namespace ptp

RCLCPP_COMPONENTS_REGISTER_NODE(ptp::DualArmPTPClient)