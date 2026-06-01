/**
 * @file ptp_joint_controller.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief PTP Joint Controller - for sending point-to-point joint control commands to the robot
 * @version 0.1
 * @date 2024-06-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef PTP_JOINT_CONTROLLER_H_
#define PTP_JOINT_CONTROLLER_H_

#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "controller_interface/controller_interface.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "utility/controller_defs.h"
#include "utility/data_validator.h"
#include "addverb_cobot_controllers/control_default_params.h"
#include "robot_fsm_enums.h"

namespace addverb_cobot_controllers
{
    class PTPJointController : public controller_interface::ControllerInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PTPJointController);

        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

        /// @brief claim the command interfaces
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /// @brief claim state interfaces
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /// @brief process next command in queue
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /// @brief setup 
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief reset the buffer
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief release resources
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief initialise one-time initialisations
        CallbackReturn on_init() override;

    private:
        /// @brief data validator to validate ptp joint commands
        std::unique_ptr<addverb_cobot::DataValidator> data_validator_;

        /// @brief  subscriber to take incoming commands
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;

        /// @brief command interfaces to claim
        std::vector<std::string> commands_;

        /// @brief state interfaces to claim
        std::vector<std::string> states_;

        /// @brief mutex to prevent disacrd of previous goal in case of new goal
        std::mutex traj_update_mutex_;

        /// @brief mutex to protect goal handle access
        std::mutex goal_handle_mutex_;

        /// @brief goal handle for the current trajectory
        std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle_;

        /// @brief update flag to true to inidcate that the robot is in error state
        std::atomic<bool> robot_in_error_ = false;

        /// @brief flag to indicate if an old trajectory is being processed
        bool ongoing_prev_traj_ = false;

        /// @brief flag to indicate if a new goal is available
        bool has_new_goal_ = false;

        /// @brief index of the current trajectory point being processed
        int cur_index_ = 0;

        /// @brief default time tolerance for trajectory execution
        const double default_time_tolerance = 1.2;

        /// @brief position tolerance for trajectory execution
        double position_tolerance_ = 0.0;

        /// @brief default position tolerance for trajectory execution
        const double default_position_tolerance = 0.03;

        /// @brief target
        double target_ = 0.0;

        /// @brief time tolerance for trajectory execution
        double time_tolerance_ = 0.0;

        /// @brief buffer time for first point in trajectory
        rclcpp::Duration buffer_time_ = rclcpp::Duration::from_seconds(0.02);

        /// @brief queue to hold incoming commands
        FollowJointTrajectory::Goal current_goal_;

        /// @brief start time of trajectory execution
        std::chrono::high_resolution_clock::time_point start_time_;

        /// @brief send the next trajectory command to hardwrae interface
        void sendNextCmd_();

        /// @brief update the feedback message
        void updateFeedback_();

        /// @brief validate the goal before accepting it
        /// @param goal
        /// @return
        bool validateGoal_(const trajectory_msgs::msg::JointTrajectory &goal);

        /// @brief handle goal callback
        rclcpp_action::GoalResponse handleGoal_(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const FollowJointTrajectory::Goal> goal);

        /// @brief handle cancel callback
        rclcpp_action::CancelResponse handleCancel_(const std::shared_ptr<
                                                    GoalHandleFollowJointTrajectory>
                                                        gh);
        /// @brief handle accepted callback
        void handleAccepted_(const std::shared_ptr<
                             GoalHandleFollowJointTrajectory>
                                 gh);
    };

}

#endif
