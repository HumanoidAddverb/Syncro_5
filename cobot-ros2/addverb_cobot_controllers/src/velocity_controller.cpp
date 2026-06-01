/**
 * @file velocity_controller.cpp
 * @author N.Aswin Beckham (aswin.beckham@addverb.com)
 * @brief Velocity controller - for sending velocity commands to the robot
 * @version 0.1
 * @date 2025-10-10
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "addverb_cobot_controllers/velocity_controller.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace addverb_cobot_controllers
{

    /**
     * @brief Initializes the controller by loading parameters
     * @return SUCCESS if initialization completes properly, ERROR otherwise
     */
    controller_interface::CallbackReturn VelocityController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "Initializing velocity controller");

        try
        {
            data_validator_ = std::make_unique<addverb_cobot::DataValidator>(get_node()->get_logger());
            commands_ = get_node()->get_parameter("commands").as_string_array();
            states_ = get_node()->get_parameter("states").as_string_array();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception in initializing velocity_controller: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        catch (...)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Unknown exception in initializing velocity_controller");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Returns the command interfaces that this controller requires
     * @return Configuration specifying the command interfaces
     */
    controller_interface::InterfaceConfiguration VelocityController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = commands_;
        return config;
    }

    /**
     * @brief Returns the state interfaces that this controller requires
     * @return Configuration specifying the state interfaces
     */
    controller_interface::InterfaceConfiguration VelocityController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = states_;
        return config;
    }

    /**
     * @brief Processes the latest velocity command and updates the hardware interface
     * @param time Current time
     * @param period Time since last update call
     * @return Success if command was processed, error code otherwise
     */
    controller_interface::return_type VelocityController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        RobotState robot_state = static_cast<RobotState>(state_interfaces_[0].get_value());

        if (robot_state == RobotState::eError)
        {
            if (!robot_in_error_.load(std::memory_order_acquire))
            {
                robot_in_error_.store(true, std::memory_order_release);

                RCLCPP_ERROR(get_node()->get_logger(), "Robot gone into error. Call ErrorRecoveryService to safely recover from the error and get into the home position.");
            }
        }
        else
        {
            if (robot_in_error_.load(std::memory_order_acquire))
            {
                robot_in_error_.store(false, std::memory_order_release);

                RCLCPP_INFO(get_node()->get_logger(), "Robot recovered from error. you can now start commanding the robot.");
            }
        }

        std::shared_ptr<std_msgs::msg::Float64MultiArray> msg = std::make_shared<std_msgs::msg::Float64MultiArray>(*cmd_buffer_.readFromRT());

        if (!sanityChecks_(msg))
        {
            resetCmdBuffer_();
            setZeroCmd_();
            return controller_interface::return_type::OK;
        }

        if (!isValid_(msg))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Received invalid command. Resetting command buffer.");
            resetCmdBuffer_();
            setZeroCmd_();
            return controller_interface::return_type::OK;
        }

        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            command_interfaces_[i].set_value(msg->data[i]);
        }

        return controller_interface::return_type::OK;
    }

    /**
     * @brief Performs basic sanity checks
     * @return true
     * @return false
     */
    bool VelocityController::sanityChecks_(std::shared_ptr<std_msgs::msg::Float64MultiArray> &msg)
    {
        if (msg == nullptr)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Received null velocity command.");
            return false;
        }

        const int n_pub = cmd_subscriber_->get_publisher_count();

        if (!cmd_subscriber_ || n_pub == 0)
        {
            return false;
        }

        if (n_pub > 1)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Multiple publishers connected to /velocity/command. This may lead to unexpected behavior so stopping the controller.");
            return false;
        }

        return true;
    }

    /**
     * @brief reset command interface commands to zero
     */
    void VelocityController::setZeroCmd_()
    {
        for (auto &interface : command_interfaces_)
        {
            interface.set_value(0.0);
        }
    }

    /**
     * @brief Configures the controller by setting up the command subscriber
     * @param previous_state The state before configuration
     * @return SUCCESS if configuration completes properly, ERROR otherwise
     */
    controller_interface::CallbackReturn VelocityController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Activates the controller and initializes command values to zero
     * @param previous_state The state before activation
     * @return SUCCESS if activation completes properly
     */
    controller_interface::CallbackReturn VelocityController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        try
        {
            // register subscriber
            cmd_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
                "~/commands", 
                rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg)
                {
                    std_msgs::msg::Float64MultiArray trimmed_msg = *msg;
                    trimmed_msg.data.assign(msg->data.begin(), msg->data.begin() + addverb_cobot::n_dof);
                    cmd_buffer_.writeFromNonRT(trimmed_msg);
                });
        }
        catch (...)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        resetCmdBuffer_();

        RCLCPP_INFO(get_node()->get_logger(), "activate successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Deactivates the controller and sets all commands to zero
     * @param previous_state The state before deactivation
     * @return SUCCESS if deactivation completes properly
     */
    controller_interface::CallbackReturn VelocityController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        resetCmdBuffer_();

        try
        {
            // reset subscriber
            cmd_subscriber_.reset();
        }
        catch (...)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Resets the command buffer with a zero-velocity message.
     * Prevents the controller from acting on stale or junk commands.
     */
    void VelocityController::resetCmdBuffer_()
    {
        cmd_buffer_.reset();        
        std_msgs::msg::Float64MultiArray zero_vel;
        zero_vel.data.resize(command_interfaces_.size(), 0.0);
        cmd_buffer_.writeFromNonRT(zero_vel);
    }

    /**
     * @brief Checks if the given command is valid
     * @param msg velocity message to validate
     * @return true if the command is valid, false otherwise
     */
    bool VelocityController::isValid_(const std::shared_ptr<std_msgs::msg::Float64MultiArray> &msg)
    {
        addverb_cobot::hw_interface_defs::Velocity velocity_cmd;

        velocity_cmd.cmd.assign(msg->data.begin(), msg->data.begin() + addverb_cobot::n_dof);

        auto validation_result = data_validator_->validateRequest(velocity_cmd);

        if (validation_result != addverb_cobot::error_codes::NoError)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Invalid velocity command: %d", static_cast<int>(validation_result));
            return false;
        }

        return true;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(addverb_cobot_controllers::VelocityController, controller_interface::ControllerInterface)
