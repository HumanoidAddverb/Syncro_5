#include "addverb_cobot_controllers/gravity_comp_effort_controller.h"

namespace addverb_cobot_controllers
{
    /**
     * @brief initialise the resources
     *
     * @param
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn GravityCompEffortController::on_init()
    {
      try
      {
        // read joint command interface names from parameter "commands"
        commands_ = get_node()->get_parameter("commands").as_string_array();
        states_ = get_node()->get_parameter("states").as_string_array();
      }
      catch (...)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Exception raised in initialising GravityCompEffortController");
        return controller_interface::CallbackReturn::ERROR;
      }

     

      RCLCPP_INFO(get_node()->get_logger(), "Initialised gravity compensation effort controller");

      return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief configure controller
     *
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn GravityCompEffortController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Configured gravity compensation effort controller");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief configure controller command interface
     *
     * @param
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration GravityCompEffortController::command_interface_configuration() const
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
    controller_interface::InterfaceConfiguration GravityCompEffortController::state_interface_configuration() const
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
    controller_interface::CallbackReturn GravityCompEffortController::on_activate(const rclcpp_lifecycle::State&)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Activating gravity compensation effort controller");

       // create subscriber for effort commands
       effort_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        std::string(get_node()->get_name()) + "/commands", rclcpp::SystemDefaultsQoS(),
        std::bind(&GravityCompEffortController::effort_command_callback_, this, std::placeholders::_1));

      commanded_effort_.resize(addverb_cobot::n_dof, 0.0);

      return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief deactivate controller callback
     *
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn GravityCompEffortController::on_deactivate(const rclcpp_lifecycle::State&)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Deactivated gravity compensation effort controller");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief update controller callback
     *
     * This callback is called in the realtime control loop. It writes the latest
     * commanded efforts (received via topic) into the claimed command interfaces.
     *
     * @param time
     * @param period
     * @return controller_interface::return_type
     */
    controller_interface::return_type GravityCompEffortController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
      RobotState robot_state = static_cast<RobotState>(state_interfaces_[0].get_value());
      if (robot_state == RobotState::eError)
      {
          in_error_ = true;
      
          RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            20000, 
            "Robot gone into error. Call ErrorRecoveryService to safely recover from the error and get into the home position.");
      }
      else
      {
          // Reset flag once robot recovers from error
          in_error_ = false;
      }

      effort_mutex_.lock();
      if (in_error_)
      {
        commanded_effort_.resize(addverb_cobot::n_dof, 0.0);
      }

      for (int i = 0; i < addverb_cobot::n_dof; ++i)
      {
        // set_value() writes into the command interface that the hardware will later read
        command_interfaces_[i].set_value(commanded_effort_[i]);
      }
      effort_mutex_.unlock();
      return controller_interface::return_type::OK;
    }

    /**
     * @brief subscriber callback for effort commands
     *
     * Stores the received efforts in commanded_efforts_ which are consumed in update().
     *
     * @param msg effort command message containing joint efforts
     */
    void GravityCompEffortController::effort_command_callback_(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      // copy received efforts
      effort_mutex_.lock();
      for (int i = 0; i < addverb_cobot::n_dof; ++i)
      {
        commanded_effort_[i] = msg->data[i];

      }
      effort_mutex_.unlock();
    }

} /// namespace addverb_cobot_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    addverb_cobot_controllers::GravityCompEffortController,
    controller_interface::ControllerInterface)
