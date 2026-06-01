
/**
 * @file free_driver_controller.h
 * @author Yaswanth Gonna (yaswanth.gonna@addverb.com)
 * @brief Free drive controller with recording feature
 * @version 0.1
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef  FREEDRIVE_CONTROLLER_H_
#define  FREEDRIVE_CONTROLLER_H_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "controller_interface/controller_interface.hpp"
#include "data_store.h"

namespace addverb_cobot_controllers
{
  class FreeDriveController: public controller_interface::ControllerInterface
  {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(FreeDriveController);
  
      /// @brief configure controller command interface
      /// @param 
      /// @return
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      
      /// @brief configure controller state interface
      /// @param 
      /// @return
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    
      /// @brief update controller callback
      /// @param time
      /// @param period
      /// @return
      controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    
      /// @brief configure controller callback 
      /// @param previous_state
      /// @return
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    
      /// @brief activate controller callback
      /// @param previous_state
      /// @return
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    
      /// @brief deactivate controller callback
      /// @param previous_state
      /// @return
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
      
      /// @brief controller initialization callback
      /// @param
      /// @return
      controller_interface::CallbackReturn on_init() override;  
      
      
    private:
      /// @brief command interfaces to claim
      std::vector<std::string> commands_;

      /// @brief state interfaces to claim
      std::vector<std::string> states_;

      /// @brief update flag to true to inidcate that the robot is in error state
      std::atomic<bool> robot_in_error_ = false;
  };
}

#endif