#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "addverb_cobot_msgs/action/follow_cartesian_trajectory.hpp"
#include "addverb_cobot_msgs/msg/cartesian_trajectory_point.hpp"
#include "addverb_cobot_msgs/msg/cartesian_point.hpp"

namespace ptp_tcp
{
using FollowCartesianTrajectory = addverb_cobot_msgs::action::FollowCartesianTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowCartesianTrajectory>;

class DualArmPTPClient : public rclcpp::Node
{
public:
    DualArmPTPClient() : Node("dual_arm_ptp_tcp_client")
    {
        // -------- ACTION CLIENTS --------
        client_arm1_ = rclcpp_action::create_client<FollowCartesianTrajectory>(
            this, "/arm_1_ptp_tcp_controller/follow_cartesian_trajectory");

        client_arm2_ = rclcpp_action::create_client<FollowCartesianTrajectory>(
            this, "/arm_2_ptp_tcp_controller/follow_cartesian_trajectory");

        // -------- SUBSCRIBERS --------
        sub_arm1_ = this->create_subscription<addverb_cobot_msgs::msg::CartesianPoint>(
            "/arm_1_ee_pos_data", 10,
            std::bind(&DualArmPTPClient::poseCallbackArm1, this, std::placeholders::_1));

        sub_arm2_ = this->create_subscription<addverb_cobot_msgs::msg::CartesianPoint>(
            "/arm_2_ee_pos_data", 10,
            std::bind(&DualArmPTPClient::poseCallbackArm2, this, std::placeholders::_1));
    }

private:

    // ================= CALLBACKS =================
    void poseCallbackArm1(const addverb_cobot_msgs::msg::CartesianPoint::SharedPtr msg)
    {
        latest_pose_arm1_ = *msg;
        pose_received_arm1_ = true;

        if (!goal_sent_arm1_)
            sendGoal(client_arm1_, latest_pose_arm1_, final_pose_arm1_, goal_sent_arm1_, "ARM 1");
    }

    void poseCallbackArm2(const addverb_cobot_msgs::msg::CartesianPoint::SharedPtr msg)
    {
        latest_pose_arm2_ = *msg;
        pose_received_arm2_ = true;

        if (!goal_sent_arm2_)
            sendGoal(client_arm2_, latest_pose_arm2_, final_pose_arm2_, goal_sent_arm2_, "ARM 2");
    }

    // ================= GOAL LOGIC =================
    void sendGoal(
        rclcpp_action::Client<FollowCartesianTrajectory>::SharedPtr client,
        const addverb_cobot_msgs::msg::CartesianPoint &start_pose,
        addverb_cobot_msgs::msg::CartesianPoint &final_pose,
        bool &goal_sent,
        const std::string &arm_name)
    {
        if (!client->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "%s action server not available", arm_name.c_str());
            return;
        }

        auto goal_msg = FollowCartesianTrajectory::Goal();
        goal_msg.trajectory.points.resize(3);

        addverb_cobot_msgs::msg::CartesianTrajectoryPoint pt;

        double increment = 0.01;

        for (int i = 0; i < 3; i++)
        {
            pt.point.position.x = start_pose.position.x - (i + 1) * increment;
            pt.point.position.y = start_pose.position.y;
            pt.point.position.z = start_pose.position.z;

            pt.point.orientation = start_pose.orientation;
            pt.time_from_start = 3.0 + i * 1.0;

            goal_msg.trajectory.points[i] = pt;
        }

        final_pose = goal_msg.trajectory.points.back().point;

        RCLCPP_INFO(this->get_logger(), "%s Sending goal...", arm_name.c_str());

        auto options = rclcpp_action::Client<FollowCartesianTrajectory>::SendGoalOptions();

        options.result_callback =
            [this, arm_name, &final_pose](const GoalHandle::WrappedResult &)
        {
            RCLCPP_INFO(this->get_logger(), "%s Result received", arm_name.c_str());
        };

        client->async_send_goal(goal_msg, options);
        goal_sent = true;
    }

    // ================= MEMBERS =================
    rclcpp_action::Client<FollowCartesianTrajectory>::SharedPtr client_arm1_;
    rclcpp_action::Client<FollowCartesianTrajectory>::SharedPtr client_arm2_;

    rclcpp::Subscription<addverb_cobot_msgs::msg::CartesianPoint>::SharedPtr sub_arm1_;
    rclcpp::Subscription<addverb_cobot_msgs::msg::CartesianPoint>::SharedPtr sub_arm2_;

    addverb_cobot_msgs::msg::CartesianPoint latest_pose_arm1_, latest_pose_arm2_;
    addverb_cobot_msgs::msg::CartesianPoint final_pose_arm1_, final_pose_arm2_;

    bool pose_received_arm1_ = false;
    bool pose_received_arm2_ = false;

    bool goal_sent_arm1_ = false;
    bool goal_sent_arm2_ = false;
};

} // namespace ptp_tcp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ptp_tcp::DualArmPTPClient>());
    rclcpp::shutdown();
    return 0;
}