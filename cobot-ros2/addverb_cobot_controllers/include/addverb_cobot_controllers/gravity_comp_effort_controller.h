/**
 * @file gravity_comp_effort_controller.h
 * @author Yaswanth Gonna (yaswanth.gonna@addverb.com)
 * @brief Gravity compensation effort controller
 * @version 0.1
 * @date 2025-10-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef GRAVITY_COMP_EFFORT_CONTROLLER_H_
#define GRAVITY_COMP_EFFORT_CONTROLLER_H_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "utility/robot_config_info.h"
#include "robot_fsm_enums.h"

namespace addverb_cobot_controllers
{
  class GravityCompEffortController : public controller_interface::ControllerInterface
  {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(GravityCompEffortController)

      /// @brief configure controller command interface
      /// @return command interface configuration
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      /// @brief configure controller state interface
      /// @return state interface configuration
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      /// @brief update controller callback
      /// @param time current time
      /// @param period time since last update
      /// @return controller execution status
      controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

      /// @brief configure controller callback 
      /// @param previous_state lifecycle state before configure
      /// @return success or failure
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

      /// @brief activate controller callback
      /// @param previous_state lifecycle state before activate
      /// @return success or failure
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

      /// @brief deactivate controller callback
      /// @param previous_state lifecycle state before deactivate
      /// @return success or failure
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

      /// @brief controller initialization callback
      /// @return success or failure
      controller_interface::CallbackReturn on_init() override;

    private:
      
      /// @brief subscriber for effort commands
      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_subscriber_;

      /// @brief latest commanded joint efforts
      std::vector<double> commanded_effort_;

      /// @brief command interfaces to claim
      std::vector<std::string> commands_;

      /// @brief staets interface to claim 
      std::vector<std::string> states_;

      /// @brief in error
      bool in_error_;

      /// @brief effort command mutex 
      std::mutex effort_mutex_;

      /// @brief subscriber callback for effort commands
      /// @param msg effort command message containing joint efforts
      void effort_command_callback_(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  };
}

#endif
