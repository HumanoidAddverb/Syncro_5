// cartesian_jogging_controller.cpp

#include "addverb_cobot_controllers/cartesian_jogging_controller.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace addverb_cobot_controllers
{

    /**
     * @brief Initializes the controller by reading configuration parameters.
     * Loads the list of command interface names from the "commands" parameter.
     */
    controller_interface::CallbackReturn CartesianJoggingController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "Initializing cartesian jogging controller");

        try
        {
            data_validator_ = std::make_unique<addverb_cobot::DataValidator>(get_node()->get_logger());
            commands_ = get_node()->get_parameter("commands").as_string_array();
            states_ = get_node()->get_parameter("states").as_string_array();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception initializing cartesian_jogging_controller: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        catch (...)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Unknown exception in initializing cartesian_jogging_controller");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Configures the controller by setting up the subscriber and latch parameters.
     * Subscribes to the "~/cartesian_jogging/command" topic and reads the permissible latch count.
     */
    controller_interface::CallbackReturn CartesianJoggingController::on_configure(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Called when the controller is activated.
     * Resets the command buffer to avoid using stale data.
     */
    controller_interface::CallbackReturn CartesianJoggingController::on_activate(const rclcpp_lifecycle::State &)
    {
        try
        {
            cmd_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
                "~/cartesian_jogging/command",
                rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg)
                {
                    cmd_buffer_.writeFromNonRT(msg);
                });
        }
        catch (...)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        resetCmdBuffer_();
        RCLCPP_INFO(get_node()->get_logger(), "Cartesian jogging controller activated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Called when the controller is deactivated.
     * Resets the command buffer and unsubscribes from the topic.
     */
    controller_interface::CallbackReturn CartesianJoggingController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        resetCmdBuffer_();
        try
        {
            cmd_subscriber_.reset();
        }
        catch (...)
        {
            return controller_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Cartesian jogging controller deactivated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Returns the list of command interfaces that this controller claims.
     * These interfaces are configured via the "commands" parameter.
     */
    controller_interface::InterfaceConfiguration CartesianJoggingController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = commands_;
        return config;
    }

    /**
     * @brief Returns an empty configuration for state interfaces, as they are not used.
     */
    controller_interface::InterfaceConfiguration CartesianJoggingController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = states_;
        return config;
    }

    /**
     * @brief The main update loop executed periodically.
     * Reads the latest command from the buffer, validates it, and applies the velocities.
     */
    controller_interface::return_type CartesianJoggingController::update(
        const rclcpp::Time &, const rclcpp::Duration &)
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

        std::shared_ptr<geometry_msgs::msg::Twist> msg = *cmd_buffer_.readFromRT();

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

        std::vector<double> velocities = 
        {
            msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z
        };

        //write the velocities onto the hardware
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            command_interfaces_[i].set_value(velocities[i]);
        }
    
        return controller_interface::return_type::OK;
    }

        /**
     * @brief Performs basic sanity checks
     *
     * @return true
     * @return false
     */
    bool CartesianJoggingController::sanityChecks_(std::shared_ptr<geometry_msgs::msg::Twist> &msg)
    {
        if (msg == nullptr)
        {
            return false;
        }

        const int n_pub = cmd_subscriber_->get_publisher_count();

        if (!cmd_subscriber_ || n_pub == 0)
        {
            return false;
        }

        if (n_pub >= 2)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Multiple publishers connected to /cartesian_jogging/command. This may lead to unexpected behavior so stopping the controller.");
            return false;
        }

        return true;
    }

    /**
     * @brief Resets the command buffer with a zero-velocity message.
     * Prevents the controller from acting on stale or junk commands.
     */
    void CartesianJoggingController::resetCmdBuffer_()
    {
        cmd_buffer_.reset();
        auto zero_twist = std::make_shared<geometry_msgs::msg::Twist>();
        zero_twist->linear.x = 0.0;
        zero_twist->linear.y = 0.0;
        zero_twist->linear.z = 0.0;
        zero_twist->angular.x = 0.0;
        zero_twist->angular.y = 0.0;
        zero_twist->angular.z = 0.0;
        cmd_buffer_.writeFromNonRT(zero_twist);
    }

    /**
     * @brief reset command interface commands to zero
     *
     */
    void CartesianJoggingController::setZeroCmd_()
    {
        for (auto &interface : command_interfaces_)
        {
            interface.set_value(0.0);
        }
    }


    /**
     * @brief Validates the incoming Twist message.
     * Checks if the velocities are within the defined limits.
     */
    bool CartesianJoggingController::isValid_(const std::shared_ptr<geometry_msgs::msg::Twist> &msg)
    {
        if (msg == nullptr)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Received null cartesian jogging command.");
            return false;
        }

        addverb_cobot::hw_interface_defs::CartesianJog cartesian_jogging_cmd;

        cartesian_jogging_cmd.jog_cmd.cmd = {msg->linear.x, msg->linear.y, msg->linear.z,
                                             msg->angular.x, msg->angular.y, msg->angular.z};

        auto validation_result = data_validator_->validateRequest(cartesian_jogging_cmd);

        if (validation_result != addverb_cobot::error_codes::NoError)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Invalid cartesian jogging command: %d", static_cast<int>(validation_result));
            return false;
        }

        return true;
    }

} // namespace addverb_cobot_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(addverb_cobot_controllers::CartesianJoggingController, controller_interface::ControllerInterface)
