/**
 * @file cartesian_jogging_controller.h
 * @author Aditya Pawar (aditya.pawar@addverb.com)
 * @brief Cartesian Jogging controller - for sending Cartesian Twist velocity commands to the robot
 * @version 0.2
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2025
 */

#ifndef CARTESIAN_JOGGING_CONTROLLER_H_
#define CARTESIAN_JOGGING_CONTROLLER_H_

#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "utility/data_validator.h"
#include "controller_interface/controller_interface.hpp"
#include "addverb_cobot_controllers/control_default_params.h"

namespace addverb_cobot_controllers
{
    class CartesianJoggingController : public controller_interface::ControllerInterface
    {
    public:
        /// @brief Define the list of command interfaces
        /// @return
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /// @brief No state interfaces are required by this controller
        /// @return
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /**
         * @brief Core update function called periodically by the controller manager.
         *        Applies the latest command from the buffer to the robot if valid.
         *
         * @param time Current time stamp
         * @param period Time since last update
         * @return OK or ERROR depending on execution
         */
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /// @brief Called during lifecycle configuration phase; sets up parameters, subscribers, etc.
        /// @return
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief Called when controller is activated
        /// @return
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief Called when controller is deactivated; used to clean up or stop commands
        /// @return
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief Initializes the controller before configuration (e.g., parameter declaration)
        /// @return
        CallbackReturn on_init() override;

    private:
        /// @brief data validator to validate cartesian jogging commands
        std::unique_ptr<addverb_cobot::DataValidator> data_validator_;

        /// @brief Subscriber for cartesian jogging commands (geometry_msgs::msg::Twist)
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;

        /// @brief Real-time buffer to safely pass Twist messages from the ROS subscriber to the controller thread
        realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_buffer_;

        /// @brief List of command interface names
        std::vector<std::string> commands_;

        /// @brief update flag to true to inidcate that the robot is in error state
        std::atomic<bool> robot_in_error_ = false;

        /// @brief state interfaces to claim
        std::vector<std::string> states_;

        /**
         * @brief Resets the command buffer with a default zero-velocity Twist message
         */
        void resetCmdBuffer_();

        /// @brief returns true if given value is within limits
        bool isValid_(const std::shared_ptr<geometry_msgs::msg::Twist> &msg);

        /// @brief set zero command to command interfaces
        void setZeroCmd_();

        /// @brief method to check basic sanity 
        bool sanityChecks_(std::shared_ptr<geometry_msgs::msg::Twist>&);

    };
}

#endif // CARTESIAN_JOGGING_CONTROLLER_H_
