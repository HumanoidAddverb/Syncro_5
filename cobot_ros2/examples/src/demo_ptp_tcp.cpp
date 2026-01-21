/**
 * @file demo_ptp_tcp.cpp
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Demo for running ptp_tcp_controller
 * @version 0.1
 * @date 2025-10-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "addverb_cobot_msgs/action/follow_cartesian_trajectory.hpp"
#include "addverb_cobot_msgs/msg/cartesian_trajectory_point.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ptp_tcp
{
    using FollowCartesianTrajectory = addverb_cobot_msgs::action::FollowCartesianTrajectory;
    using GoalHandleFollowCartesianTrajectory = rclcpp_action::ClientGoalHandle<FollowCartesianTrajectory>;

    class PTPTCPTestClient : public rclcpp::Node
    {
    public:
        explicit PTPTCPTestClient(const rclcpp::NodeOptions &options)
            : Node("demo_ptp_tcp_client")
        {
            client_ = rclcpp_action::create_client<FollowCartesianTrajectory>(
                this, "/ptp_tcp_controller/follow_cartesian_trajectory");

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&PTPTCPTestClient::send_goal, this));
        }

        void send_goal()
        {
            timer_->cancel();

            if (!once)
            {
                if (!client_->wait_for_action_server(std::chrono::seconds(5)))
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available");
                    rclcpp::shutdown();
                    return;
                }

                auto goal_msg = FollowCartesianTrajectory::Goal();
                goal_msg.trajectory.points.resize(5);

                addverb_cobot_msgs::msg::CartesianTrajectoryPoint pt;
// black cobot
                pt.point.position.x = -0.000911561;
                pt.point.position.y = 0.468131;
                pt.point.position.z = 0.5505;
                pt.point.orientation.x = 0;
                pt.point.orientation.y = 3.14;
                pt.point.orientation.x = 0;
                pt.time_from_start = 0.05;
                goal_msg.trajectory.points[0] = pt;

                pt.point.position.z = 0.4505;
                pt.time_from_start = 15.05;
                goal_msg.trajectory.points[1] = pt;

                pt.point.position.x = -0.4009;
                pt.time_from_start = 30.05;
                goal_msg.trajectory.points[2] = pt;

                pt.point.position.x = -0.0009;
                pt.time_from_start = 45.05;
                goal_msg.trajectory.points[3] = pt;

                pt.point.position.x = -0.4009;
                pt.time_from_start = 60.05;
                goal_msg.trajectory.points[4] = pt;


                auto options = rclcpp_action::Client<FollowCartesianTrajectory>::SendGoalOptions();
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
                    rclcpp::shutdown();
                };
                options.feedback_callback =
                    [this](std::shared_ptr<GoalHandleFollowCartesianTrajectory>,
                           const std::shared_ptr<const FollowCartesianTrajectory::Feedback> feedback)
                {
                    // if (feedback)
                    // {
                    //     std::ostringstream oss;
                    //     oss << "Feedback: ";
                    //     if (!feedback->joint_names.empty() && !feedback->actual.positions.empty())
                    //     {
                    //         oss << "Actual positions: [";
                    //         for (size_t i = 0; i < feedback->actual.positions.size(); ++i)
                    //         {
                    //             oss << feedback->actual.positions[i];
                    //             if (i + 1 < feedback->actual.positions.size())
                    //                 oss << ", ";
                    //         }
                    //         oss << "]";
                    //     }
                    //     RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
                    // }
                };

                client_->async_send_goal(goal_msg, options);
                once = true;
            }
            std::cout << "sent goal once\n";
        }

    private:
        rclcpp_action::Client<FollowCartesianTrajectory>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool once = false;
    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ptp_tcp::PTPTCPTestClient>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}