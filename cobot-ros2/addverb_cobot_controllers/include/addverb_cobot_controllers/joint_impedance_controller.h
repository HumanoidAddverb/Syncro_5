/**
 * @file joint_impedance_controller.h
 * @author Aditya Pawar (aditya.pawar@addverb.com)
 * @brief Joint Impedance Controller - for sending joint impedance control commands to the robot
 * @version 0.1
 * @date 2025-07-11
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef JOINT_IMPEDANCE_CONTROLLER_H_
#define JOINT_IMPEDANCE_CONTROLLER_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "utility/controller_defs.h"
#include "utility/robot_config_info.h"
#include "utility/data_validator.h"
#include "addverb_cobot_controllers/control_default_params.h"
#include "addverb_cobot_controllers/controller_utils.h"

namespace addverb_cobot_controllers
{
  class JointImpedanceController : public controller_interface::ControllerInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(JointImpedanceController);

    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    /// @brief initialise one-time initialisations
    controller_interface::CallbackReturn on_init() override;

    /// @brief setup
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    /// @brief reset the buffer
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    /// @brief release resources
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /// @brief claim the command interfaces
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /// @brief claim state interfaces
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /// @brief process next command in queue
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    /// @brief Data Validator to validate the data received from the robot
    std::unique_ptr<addverb_cobot::DataValidator> data_validator_;

    /// @brief start time of trajectory execution
    std::chrono::high_resolution_clock::time_point start_time_;

    /// @brief command interfaces to claim
    std::vector<std::string> commands_;

    /// @brief state interfaces to claim
    std::vector<std::string> states_;

    // Joint and parameter data
    std::vector<std::string> joints_;
    std::vector<double> stiffness_gains_;
    std::vector<double> damping_gains_;

    /// @brief update flag to true to inidcate that the robot is in error state
    std::atomic<bool> robot_in_error_ = false;

    // Action server
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
    std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle_;
    FollowJointTrajectory::Goal current_goal_;

    /// @brief target
    double target_ = 0.0;

    /// @brief position tolerance for trajectory execution
    double position_tolerance_ = 0.0;

    /// @brief time tolerance for trajectory execution
    double time_tolerance_ = 0.0;

    // State control
    bool has_new_goal_ = false;
    bool ongoing_prev_traj_ = false;
    size_t cur_index_ = 0;

    // Timing and param handling
    std::mutex param_mutex_;
    std::mutex traj_update_mutex_;
    std::mutex goal_handle_mutex_;

    // Dynamic reconfigure callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_parameter_event_(const std::vector<rclcpp::Parameter> &parameters);

    // Action callbacks
    rclcpp_action::GoalResponse handleGoal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handleCancel_(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    void handleAccepted_(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    // Execution logic
    void sendNextCmd_();
    void updateFeedback_();
    bool validateGoal_(const trajectory_msgs::msg::JointTrajectory &goal);
    bool validateImpedanceData_();
  };

} // namespace addverb_cobot_controllers

#endif
