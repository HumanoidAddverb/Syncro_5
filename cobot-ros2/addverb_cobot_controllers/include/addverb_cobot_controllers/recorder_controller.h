/**
 * @file recorder_controller.h
 * @author Yaswanth Gonna (yaswanth.gonna@addverb.com)
 * @brief recorder controller with replay feature
 * @version 0.1
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef  RECORDER_CONTROLLER_H_
#define  RECORDER_CONTROLLER_H_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "addverb_cobot_msgs/srv/record.hpp"
#include "addverb_cobot_msgs/action/replay.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "controller_interface/controller_interface.hpp"
#include "data_store.h"
#include "utility/controller_defs.h"
#include "utility/data_validator.h"
#include "utility/hardware_interface_defs.h"
#include "robot_fsm_enums.h"

using namespace std::placeholders;

namespace addverb_cobot_controllers
{
  class RecorderController: public controller_interface::ControllerInterface
  {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(RecorderController);
    
      /// @brief record service data type
      using RecordSrv = addverb_cobot_msgs::srv::Record;
      using ReplayAction = addverb_cobot_msgs::action::Replay;
      using clock = std::chrono::system_clock;
      using GoalHandleReplayAction = rclcpp_action::ServerGoalHandle<ReplayAction>;
    
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
      /// @brief data validator to validate ptp joint commands
      std::unique_ptr<addverb_cobot::DataValidator> data_validator_;

      /// @brief  states required by controller
      std::vector<std::string> states_;

      /// @brief command required by controller
      std::vector<std::string> commands_;
      
      /// @brief  service for recording data
      rclcpp::Service<RecordSrv>::SharedPtr record_srv_;

      /// @brief replay action service
      rclcpp_action::Server<ReplayAction>::SharedPtr replay_action_srv_;
    
      /// @brief recording status 
      bool enable_recording_{false};

      /// @brief status of controller
      std::atomic<bool> is_active_{false};
  
      /// @brief recording rate (hz)
      int recording_rate_{10};
      
      /// @brief latest joint position values
      std::vector<double> recorded_jpos_;

      /// @brief latest joint position trajectory values
      std::vector<std::vector<double>> replay_trajectory_;

      /// @brief latest trajectory time seq 
      std::vector<double> time_seq_;

    /// @brief current index 
      int curr_index_ = 0;

      /// @brief replay iterations
      int curr_iterations_ = 0;

      /// @brief default wait time for record thread ms
      double wait_time_;

      /// @brief start time 
      double start_time_;

      /// @brief current time 
      double current_time_;

      /// @brief max recording rate
      int max_recording_rate_;
      
      /// @brief has new goal
      bool has_new_goal_{false};

      /// @brief has ongoing previous trajectory
      bool ongoing_prev_traj_{false};

      /// @brief data storage helper
      DataStore data_store_;

      /// @brief position tolerance for trajectory execution
      double position_tolerance_;

      /// @brief default time tolerance for trajectory execution
      double time_tolerance_ ;

      /// @brief latest goal
      ReplayAction::Goal current_goal_;
      
      /// @brief latest goal handle
      std::shared_ptr<GoalHandleReplayAction> goal_handle_;

      /// @brief current iteration number 
      int curr_iteration_ = 1;

      /// @brief recorder call back mutex
      std::mutex recorder_mutex_;
      
      /// @brief trajectory start time 
      std::chrono::high_resolution_clock::time_point trajectory_start_time_;

      /// @brief update flag to true to inidcate that the robot is in error state
      std::atomic<bool> robot_in_error_ = false;

      /// @brief recording thread 
      void record_();
      
      /// @brief recoder service callback
      void recorderCallback_(const std::shared_ptr<RecordSrv::Request>,
        std::shared_ptr<RecordSrv::Response>);

      /// @brief send feedback to action client
      void sendFeedback_();

      /// @brief reset time for recording data
      void resetTime_();

      /// @brief get time difference used for keeping count 
      double timeDiff_();

      /// @brief validate request from user
      bool validateReplayRequest_();

      /// @brief validate label 
      bool validateReplayLabel_(const std::string &);

      /// @brief replay accept handle
      void handleReplayAccepted_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayAction>> goal_handle);
      
      /// @brief replay cancel goal handle
      rclcpp_action::CancelResponse handleReplayCancel_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayAction>> goal_handle);

      /// @brief action server feedback callback
      void replayExecute_();

      /// @brief replay goal callback 
      rclcpp_action::GoalResponse handleReplayGoal_(const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ReplayAction::Goal> goal);
  };
}

#endif