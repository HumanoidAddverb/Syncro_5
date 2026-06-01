#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ptp
{
    const bool multi_point_target = true;

    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    class DemoClient : public rclcpp::Node
    {
    public:
        explicit DemoClient(const rclcpp::NodeOptions &options)
            : Node("demo_ptp_client")
        {
            client_ = rclcpp_action::create_client<FollowJointTrajectory>(
                this, "/arm_1_ptp_joint_controller/follow_joint_trajectory");

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&DemoClient::send_goal, this));
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

                auto goal_msg = FollowJointTrajectory::Goal();
                goal_msg.trajectory.joint_names = {
                    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

                if (multi_point_target)
                {
                    goal_msg.trajectory.points.clear();

                    trajectory_msgs::msg::JointTrajectoryPoint pt;

                    pt.positions = {0, 0, 0, 0, 0, 0};
                    pt.time_from_start.sec = 4;
                    pt.time_from_start.nanosec = 0;
                    goal_msg.trajectory.points.push_back(pt);

                    pt.positions = {0.5, 0, 0, 0, 0, 0};
                    pt.time_from_start.sec =  8;
                    pt.time_from_start.nanosec = 0;
                    goal_msg.trajectory.points.push_back(pt);

                    pt.positions = {0.5, 0, 0.3, 0, 0, 0};
                    pt.time_from_start.sec =  12;
                    pt.time_from_start.nanosec = 0;
                    goal_msg.trajectory.points.push_back(pt);

                    pt.positions = {-0.5, 0, 0.3, 0, 0, 0};
                    pt.time_from_start.sec =  18;
                    pt.time_from_start.nanosec = 0;
                    goal_msg.trajectory.points.push_back(pt);

                    pt.positions = {-0.5, 0, 0, 0, 0, 0};
                    pt.time_from_start.sec =  22;
                    pt.time_from_start.nanosec = 0;
                    goal_msg.trajectory.points.push_back(pt);

                    pt.positions = {0, 0, 0, 0, 0, 0};
                    pt.time_from_start.sec =  25;
                    pt.time_from_start.nanosec = 0;
                    goal_msg.trajectory.points.push_back(pt);
                }
                else
                {
                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    pt.positions = {0, 0, 0, 0, 0, 0};
                    pt.time_from_start.sec = 5;
                    pt.time_from_start.nanosec = 0;
                    goal_msg.trajectory.points.push_back(pt);
                }

                auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
                options.goal_response_callback =
                    [this](std::shared_ptr<GoalHandleFollowJointTrajectory> handle)
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
                    [this](const GoalHandleFollowJointTrajectory::WrappedResult &result)
                {
                    RCLCPP_INFO(this->get_logger(), "Result received");
                    rclcpp::shutdown();
                };
                options.feedback_callback =
                    [this](std::shared_ptr<GoalHandleFollowJointTrajectory>,
                           const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
                {
                    if (feedback)
                    {
                        std::ostringstream oss;
                        oss << "Feedback: ";
                        if (!feedback->joint_names.empty() && !feedback->actual.positions.empty())
                        {
                            oss << "Actual positions: [";
                            for (size_t i = 0; i < feedback->actual.positions.size(); ++i)
                            {
                                oss << feedback->actual.positions[i];
                                if (i + 1 < feedback->actual.positions.size())
                                    oss << ", ";
                            }
                            oss << "]";
                        }
                        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
                    }
                };

                client_->async_send_goal(goal_msg, options);
                once = true;
            }
            std::cout << "sent goal once\n";
        }

    private:
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool once = false;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(ptp::DemoClient)
