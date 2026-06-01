/**
 * @file ptp_tcp_controller.h
 * @author Aditya Pawar (aditya.pawar@addverb.com), Siddhi Jain (siddhi.jain@addverb.com)
 * @brief PTP TCP Controller - for sending point-to-point tcp control commands to the robot
 * @version 0.1
 * @date 2025-10-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef PTP_TCP_CONTROLLER_H_
#define PTP_TCP_CONTROLLER_H_

#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "controller_interface/controller_interface.hpp"

#include "addverb_cobot_msgs/msg/cartesian_trajectory.hpp"
#include "addverb_cobot_msgs/action/follow_cartesian_trajectory.hpp"

#include "utility/controller_defs.h"
#include "utility/data_validator.h"

#include "addverb_cobot_controllers/control_default_params.h"
#include "robot_fsm_enums.h"
#include "utility/ros_wrapper_error_codes.h"

namespace addverb_cobot_controllers
{
    class PTPTCPController : public controller_interface::ControllerInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PTPTCPController);

        using FollowCartesianTrajectory = addverb_cobot_msgs::action::FollowCartesianTrajectory;
        using GoalHandleFollowCartesianTrajectory = rclcpp_action::ServerGoalHandle<FollowCartesianTrajectory>;

        /// @brief claim the command interfaces
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /// @brief claim state interfaces
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /// @brief process next command in queue
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /// @brief setup the subscriber
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief reset the buffer
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief release resources
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief initialise one-time initialisations
        CallbackReturn on_init() override;

    private:
        /// @brief data validator to validate ptp Cartesian commands
        std::unique_ptr<addverb_cobot::DataValidator> data_validator_;

        /// @brief  subscriber to take incoming commands
        rclcpp_action::Server<FollowCartesianTrajectory>::SharedPtr server_;

        /// @brief command interfaces to claim
        std::vector<std::string> commands_;

        /// @brief state interfaces to claim
        std::vector<std::string> states_;

        /// @brief mutex to prevent disacrd of previous goal in case of new goal
        std::mutex traj_update_mutex_;

        /// @brief mutex to protect goal handle access
        std::mutex goal_handle_mutex_;

        /// @brief goal handle for the current trajectory
        std::shared_ptr<GoalHandleFollowCartesianTrajectory> goal_handle_;

        /// @brief update flag to true to inidcate that the robot is in error state
        std::atomic<bool> robot_in_error_ = false;

        /// @brief flag to indicate if an old trajectory is being processed
        bool ongoing_prev_traj_ = false;

        /// @brief flag to indicate if a new goal is available
        bool has_new_goal_ = false;

        /// @brief index of the current trajectory point being processed
        int cur_index_ = 0;

        /// @brief target
        double target_time_ = 0.0;

        /// @brief time tolerance for trajectory execution
        double time_tolerance_ = 2.0;

        /// @brief queue to hold incoming commands
        FollowCartesianTrajectory::Goal current_goal_;

        /// @brief start time of trajectory execution
        std::chrono::high_resolution_clock::time_point start_time_;

        /// @brief send the next trajectory command to hardwrae interface
        void sendNextCmd_();

        /// @brief update the feedback message
        void updateFeedback_();

        /// @brief handle goal callback
        rclcpp_action::GoalResponse handleGoal_(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const FollowCartesianTrajectory::Goal> goal);

        /// @brief handle cancel callback
        rclcpp_action::CancelResponse handleCancel_(const std::shared_ptr<
                                                    GoalHandleFollowCartesianTrajectory>
                                                        gh);
        /// @brief handle accepted callback
        void handleAccepted_(const std::shared_ptr<
                             GoalHandleFollowCartesianTrajectory>
                                 gh);
    };

}

#endif
