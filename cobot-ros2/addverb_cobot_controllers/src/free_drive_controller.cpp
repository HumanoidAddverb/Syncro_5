#include "addverb_cobot_controllers/free_drive_controller.h"
#include "rclcpp/rclcpp.hpp"


namespace addverb_cobot_controllers
{
    /**
     * @brief initialise the resources
     *
     * @param 
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn FreeDriveController::on_init()
    {
      try
        {
            commands_ = get_node()->get_parameter("commands").as_string_array();
            states_ = get_node()->get_parameter("states").as_string_array();
        }
        catch (...)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Exception raised in initialising free_drive_controller");
            return controller_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("FreeDriveController"), "Initialised free-drive controller");
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief configure controller 
     *
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn FreeDriveController::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
      RCLCPP_INFO(rclcpp::get_logger("FreeDriveController"), "Configured free-drive controller");
      
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    /**
     * @brief configure controller command interface
     *
     * @param 
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration FreeDriveController::command_interface_configuration() const
    {
      controller_interface::InterfaceConfiguration config;
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      config.names = commands_;
      
      return config;
    }

    /**
     * @brief configure controller state interface
     *
     * @param 
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration FreeDriveController::state_interface_configuration() const
    {
      controller_interface::InterfaceConfiguration config;
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      config.names = states_;
      
      return config;
    }
    
    /**
     * @brief activate controller callback
     *
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn FreeDriveController::on_activate(const rclcpp_lifecycle::State&)
    {
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief deactivate controller callback
     *
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn FreeDriveController::on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger("FreeDriveController"), "Deactivated free-drive controller");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief update controller callback
     *
     * @param time 
     * @param period
     * @return controller_interface::return_type
     */
    controller_interface::return_type FreeDriveController::update(const rclcpp::Time&, const rclcpp::Duration&) 
    {
        // RobotState robot_state = static_cast<RobotState>(state_interfaces_[6].get_value());

        // if (robot_state == RobotState::eError)
        // {
        //     if (robot_in_error_.load(std::memory_order_acquire))
        //     {
        //         return controller_interface::return_type::OK;
        //     }

        //     robot_in_error_.store(true, std::memory_order_release);

        //     RCLCPP_ERROR(get_node()->get_logger(), "Robot gone into error. Call ErrorRecoveryService to safely recover from the error and get into the home position.");
        //     return controller_interface::return_type::OK;
        // }
        // else
        // {
        //     if (robot_in_error_.load(std::memory_order_acquire))
        //     {
        //         robot_in_error_.store(false, std::memory_order_release);

        //         RCLCPP_INFO(get_node()->get_logger(), "Robot recovered from error. you can now start commanding the robot.");
        //     }
        // }

        return controller_interface::return_type::OK;
    }
} /// namespace addverb_cobot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  addverb_cobot_controllers::FreeDriveController, controller_interface::ControllerInterface)





