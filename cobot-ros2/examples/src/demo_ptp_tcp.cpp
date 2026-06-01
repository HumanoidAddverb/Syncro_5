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
using GoalHandleFollowCartesianTrajectory =
    rclcpp_action::ClientGoalHandle<FollowCartesianTrajectory>;

class PTPTCPTestClient : public rclcpp::Node
{
public:
    explicit PTPTCPTestClient(const rclcpp::NodeOptions &options)
        : Node("demo_ptp_tcp_client")
    {
        client_ = rclcpp_action::create_client<FollowCartesianTrajectory>(
            this, "/arm_1_ptp_tcp_controller/follow_cartesian_trajectory");

        pose_sub_ = this->create_subscription<addverb_cobot_msgs::msg::CartesianPoint>(
            "/arm_1_ee_pos_data", 10,
            std::bind(&PTPTCPTestClient::pose_callback, this, std::placeholders::_1));
    }

private:
    // ================= SUBSCRIBER =================
    void pose_callback(const addverb_cobot_msgs::msg::CartesianPoint::SharedPtr msg)
    {
        latest_pose_ = *msg;

        if (!pose_received_)
        {
            pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received first pose, sending goal...");
        }

        if (!goal_sent_)
        {
            send_goal();
        }
    }

    // ================= SEND GOAL =================
    void send_goal()
    {
        if (!pose_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Pose not received yet");
            return;
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = FollowCartesianTrajectory::Goal();
        goal_msg.trajectory.points.resize(3);

        addverb_cobot_msgs::msg::CartesianTrajectoryPoint pt;

        double increment = 0.02; // 2 cm

        for (int i = 0; i < 3; i++)
        {
            pt.point.position.x = latest_pose_.position.x - (i + 1) * increment ;
            pt.point.position.y = latest_pose_.position.y + (i + 1) * increment;
            pt.point.position.z = latest_pose_.position.z;

            pt.point.orientation = latest_pose_.orientation;

            pt.time_from_start = 30.0 + i * 20.0; // increasing time

            goal_msg.trajectory.points[i] = pt;
        }

        // Store final target pose
        final_pose_ = goal_msg.trajectory.points.back().point;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        auto options =
            rclcpp_action::Client<FollowCartesianTrajectory>::SendGoalOptions();

        options.goal_response_callback =
            [this](std::shared_ptr<GoalHandleFollowCartesianTrajectory> handle)
        {
            if (!handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
            }
        };

        options.result_callback =
            [this](const GoalHandleFollowCartesianTrajectory::WrappedResult &result)
        {
            RCLCPP_INFO(this->get_logger(), "Result received");

            // Small delay to allow final pose update
            rclcpp::sleep_for(std::chrono::milliseconds(200));

            double dx = latest_pose_.position.x - final_pose_.position.x;
            double dy = latest_pose_.position.y - final_pose_.position.y;
            double dz = latest_pose_.position.z - final_pose_.position.z;

            double error = std::sqrt(dx * dx + dy * dy + dz * dz);

            RCLCPP_INFO(this->get_logger(), "Final position error: %.6f", error);

            if (error < 0.01)
            {
                RCLCPP_INFO(this->get_logger(), "Goal reached ✅");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Goal NOT reached ❌");
            }

            rclcpp::shutdown();
        };

        options.feedback_callback =
            [this](std::shared_ptr<GoalHandleFollowCartesianTrajectory>,
                   const std::shared_ptr<const FollowCartesianTrajectory::Feedback>)
        {
            // Optional: handle feedback
        };

        client_->async_send_goal(goal_msg, options);

        goal_sent_ = true;
    }

    // ================= MEMBERS =================
    rclcpp_action::Client<FollowCartesianTrajectory>::SharedPtr client_;
    rclcpp::Subscription<addverb_cobot_msgs::msg::CartesianPoint>::SharedPtr pose_sub_;

    addverb_cobot_msgs::msg::CartesianPoint latest_pose_;
    addverb_cobot_msgs::msg::CartesianPoint final_pose_;

    bool pose_received_ = false;
    bool goal_sent_ = false;
};

} // namespace ptp_tcp

// ================= MAIN =================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ptp_tcp::PTPTCPTestClient>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}