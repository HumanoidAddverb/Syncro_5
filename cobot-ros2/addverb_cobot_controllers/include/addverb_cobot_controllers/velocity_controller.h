/**
 * @file velocity_controller.h
 * @author N.Aswin Beckham (aswin.beckham@addverb.com)
 * @brief Velocity controller - for sending velocity commands to the robot
 * @version 0.1
 * @date 2025-10-10
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef VELOCITY_CONTROLLER_H_
#define VELOCITY_CONTROLLER_H_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

// #include "utility/robot_config_info.h"
#include "utility/data_validator.h"
#include "addverb_cobot_controllers/control_default_params.h"

namespace addverb_cobot_controllers
{
    class VelocityController : public controller_interface::ControllerInterface
    {
    public:
        /// @brief claim the command interfaces
        /// @return
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /// @brief claim state interfaces
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

        /// @brief setup the subscriber
        /// @param previous_state
        /// @return
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief reset the buffer
        /// @param previous_state
        /// @return
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief release resources
        /// @param previous_state
        /// @return
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief initialise one-time initialisations
        /// @return
        CallbackReturn on_init() override;

    private:
        /// @brief data validator to validate velocity commands
        std::unique_ptr<addverb_cobot::DataValidator> data_validator_;

        /// @brief  subscriber to take incoming commands
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_subscriber_;

        /// @brief buffer of commands received from the user
        realtime_tools::RealtimeBuffer<std_msgs::msg::Float64MultiArray> cmd_buffer_;

        /// @brief command interfaces to claim
        std::vector<std::string> commands_;

        /// @brief update flag to true to inidcate that the robot is in error state
        std::atomic<bool> robot_in_error_ = false;

        /// @brief state interfaces to claim
        std::vector<std::string> states_;

        /// @brief latest commanded joint velociy
        std::vector<double> commanded_vel_;

        /// @brief returns true if given value is within limits
        bool isValid_(const std::shared_ptr<std_msgs::msg::Float64MultiArray> &msg);

        /// @brief reset the command buffer to allow non-junk data to be transmitted to the robot
        void resetCmdBuffer_();

        /// @brief set zero command to command interfaces
        void setZeroCmd_();

        /// @brief method to check basic sanity
        bool sanityChecks_(std::shared_ptr<std_msgs::msg::Float64MultiArray> &);
    };
}

#endif
