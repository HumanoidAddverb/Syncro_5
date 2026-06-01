/**
 * @file gripper_controller.h
 * @author Yaswanth Gonna (yaswanth.gonna@addverb.com)
 * @brief GripperController: Controller for gripper
 * @version 0.1
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef  GRIPPER_CONTROLLER_H_
#define  GRIPPER_CONTROLLER_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "controller_interface/controller_interface.hpp"
#include "addverb_cobot_msgs/srv/gripper.hpp"

#include "utility/controller_defs.h"
#include "utility/hardware_interface_defs.h"
#include "utility/data_validator.h"

namespace addverb_cobot_controllers
{
    class GripperController: public controller_interface::ControllerInterface
    {
        public:
          RCLCPP_SHARED_PTR_DEFINITIONS(GripperController)

          using GripperSrv = addverb_cobot_msgs::srv::Gripper;
    
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
        
          /// @brief gripper command
          addverb_cobot::hw_interface_defs::GripperCmd gripper_cmd_;
          
          /// @brief gripper state
          addverb_cobot::GripperState gripper_state_ = addverb_cobot::GripperState::eNone;
          
          /// @brief data validator object
          std::unique_ptr<addverb_cobot::DataValidator> data_validator_;

          /// @brief gripper service
          rclcpp::Service<GripperSrv>::SharedPtr gripper_srv_;
          
          /// @brief gripper mutex
          std::mutex gripper_mutex_;

          /// @brief gripper service callback 
          void gripperServiceCallback_(const std::shared_ptr<GripperSrv::Request>, std::shared_ptr<GripperSrv::Response>);
    };
}

#endif