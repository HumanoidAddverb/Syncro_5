/**
 * @file cobot_hw_interface.h
 * @author Siddhi Jain (siddhi.jain@addverb.com), Yaswanth Gonna (yaswanth.gonna@addverb.com)
 * @brief Hardware Interface implementation for the cobot
 * @version 0.1
 * @date 2025-05-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef COBOT_HW_INTERFACE_H_
#define COBOT_HW_INTERFACE_H_

// c++ standard headers
#include <functional>
#include <vector>
#include <memory>
#include <string>
#include <map>
#include <array>
#include <atomic>

// ROS headers
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// custom headers
#include "utility/hardware_interface_defs.h"
#include "utility/data_validator.h"
#include "utility/data_converter.h"
#include "utility/data_communicator.h"
#include "utility/robot_config_info.h"
#include "utility/ros_wrapper_error_codes.h"
#include "api_types.h"

/// controller utility
#include "controller_utils.h"

/// cobot services headers
#include "cobot_services.h"
#include "cobot_auxiliary.h"

/// cobot executor header
#include "cobot_executor.h"

/// @brief Hardware interface implementation of the cobot
namespace addverb_cobot
{

    class CobotHWInterface : public hardware_interface::SystemInterface
    {
    public:
        /// @brief declaring smart pointer references
        /// @param
        RCLCPP_SHARED_PTR_DEFINITIONS(CobotHWInterface);

        ~CobotHWInterface() = default;

        /// @brief initialise the variables
        /// @param info
        /// @return
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        /// @brief coneect to robot
        /// @param previous_state
        /// @return
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief power on the robot
        /// @param previous_state
        /// @return
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief power off robot
        /// @param previous_state
        /// @return
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief clean allocated resources and close connection with robot
        /// @param previous_state
        /// @return
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief error callback - handles based on previous state
        /// @param
        /// @return
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief give state interfaces
        /// @return
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /// @brief give command interfaces
        /// @return
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /// @brief switch the controller
        /// @param start_interfaces
        /// @param stop_interfaces
        /// @return
        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

        /// @brief read and update from the state interfaces
        /// @param time
        /// @param period
        /// @return
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /// @brief write to the hw
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        /// @brief Data Validator to validate the data received from the robot
        std::shared_ptr<DataValidator> data_validator_;

        /// @brief cobot services
        std::shared_ptr<CobotServices> cobot_services_node_;

        /// @brief cobot executor
        std::shared_ptr<CobotExecutor> cobot_executor_;

        /// @brief cobot auxiliary
        std::shared_ptr<CobotAuxiliary> cobot_auxiliary_node_;

        std::array<double, 6> vcmd_ = {0, 0, 0, 0, 0, 0};

        std::array<double, 6> effortcmd_ = {0, 0, 0, 0, 0, 0};

        /// @brief replay command received from replay controller
        std::array<double, 3> replaycmd_ = {0, 0, 0};

        /// @brief control mode : default is velocity
        std::string control_mode_ = "";

        /// @brief user selected api
        hw_interface_defs::ControlMode api_;

        /// @brief current state of the robot
        RobotState robot_state_;

        /// @brief flag indicating robot error state
        std::atomic<bool> in_error_{false};

        /// @brief flag indicating robot error state
        std::atomic<bool> error_recovery_failed_{false};

        /// @brief state of the robot
        // hw_interface_defs::RobotFeedback hw_state_;

        // switch addition
        std::vector<double> controller_name_cmd_;

        /// @brief joint position
        std::vector<double> hw_state_jpos_;

        /// @brief gripper position
        double gripper_pos_;

        /// @brief gripper velocity
        double gripper_vel_;

        /// @brief gripper effort
        double gripper_eff_;

        /// @brief joint velocity
        std::vector<double> hw_state_jvel_;

        /// @brief joint effort/torque
        std::vector<double> hw_state_jtor_;

        /// @brief ft feedback
        std::vector<double> hw_ft_feedback_;

        /// @brief ee pos feedback
        std::vector<double> hw_ee_pos_feedback_;

        /// @brief sleep time for error recovery sequence
        int error_recovery_sleep_time_ = 5;

        /// @brief state of the robot to be used by controllers
        double robot_status_;

        /// @brief multi ppint configuration for the robot
        hw_interface_defs::MultiPoint multi_point_;

        /// @brief tcp multi point configuration for the cobot
        hw_interface_defs::TcpMultipoint tcp_multi_point_;

        /// @brief state for the PtP controller
        hw_interface_defs::PtPState ptp_state_;

        /// @brief point to point command
        hw_interface_defs::PtP ptp_cmd_;

        /// @brief state for the TCP PtP controller
        hw_interface_defs::TcpPtPState tcp_ptp_state_;

        /// @brief TCP Point-to-Point command
        hw_interface_defs::TcpPtP tcp_ptp_cmd_;

        /// @brief joint impedance command
        hw_interface_defs::JointImpedance joint_impedance_cmd_;

        /// @brief cartesian impedance command
        hw_interface_defs::CartesianImpedance cartesian_impedance_cmd_;

        /// @brief replay iterations
        double replay_iterations_cmd_;

        std::array<double, 6> tcp_state_ = {0, 0, 0, 0, 0, 0};

        std::array<double, 6> tcp_command_ = {0, 0, 0, 0, 0, 0};

        /// @brief command to send to robot
        /// commanded velocity
        hw_interface_defs::Velocity jvel_cmd_;

        /// commanded effort
        hw_interface_defs::Effort jeffort_cmd_;

        /// @brief gripper configuration
        hw_interface_defs::GripperConfig gripper_config_;

        /// @brief gripper command
        hw_interface_defs::GripperCmd gripper_cmd_;

        /// @brief safety mode
        hw_interface_defs::SafetyMode safety_mode_;

        /// @brief FT configuration
        hw_interface_defs::FTConfig ft_config_;

        /// @brief payload at ee of the robot
        hw_interface_defs::Payload payload_;

        /// @brief joint jogging command
        hw_interface_defs::JointJog joint_jogging_cmd_;

        /// @brief cartesian jogging command
        hw_interface_defs::CartesianJog cartesian_jogging_cmd_;

        /// @brief data processing setup
        std::shared_ptr<DataProcessor> data_processor_;

        /// @brief hold the last of the chain
        std::shared_ptr<DataProcessor> communicator_;

        /// @brief map between control mdoe (string) to API type (API)
        std::map<std::string, API> control_mode_map_;

        /// @brief actual control loop controlling the robot
        std::function<bool()> control_loop_;

        /// @brief effort switch time out
        double time_out_ = 10;

        /// @brief has joint info to make sure that read
        /// is called before sending torque commands
        bool has_jinfo_ = false;

        /// @brief effort controller activity status
        bool effort_inactive_ = true;

        /// @brief future for shutdown async method
        std::future<bool> shutdown_future_;

        /// @brief future for shutdown async method
        std::future<bool> error_recovery_future_;

        /// @brief Variables to hold parameters from cobot_services
        /// @brief is shutdown requested
        bool shutdown_requested_{false};

        /// @brief is shutdown request rejected by hardware
        bool shutdown_request_rejected_{false};

        /// @brief is shutdown request accepted by hardware
        bool shutdown_request_accepted_{false};

        /// @brief is shutdown pre-processed by hardware
        bool shutdown_pre_processed_{false};

        /// @brief is error recovery requested
        bool error_recovery_requested_{false};

        /// @brief is error recovery request rejected by hardware
        bool error_recovery_request_rejected_{false};

        /// @brief is error recovery request accepted by hardware
        bool error_recovery_request_accepted_{false};

        /// @brief is error recovery completed by hardware
        bool error_recovery_success_{false};

        /// @brief is error recovery completed by hardware
        bool error_recovery_failure_{false};

        /// @brief has valid effort command
        bool has_valid_effort_command_;

        /// @brief recorder switched
        bool recorder_switched_ = false;

        /// @brief buffer time (for moveit)
        double buffer_time_ = 1e-2;

        /// @brief effort controller status
        addverb_cobot::effortControllerStatus effort_controller_status_ = addverb_cobot::effortControllerStatus::eInactive;

        /// @brief effort controller event
        addverb_cobot::effortControllerEvents effort_controller_event_ = addverb_cobot::effortControllerEvents::eNone;

        /// @brief setup data processor
        /// @return
        bool setupDataProcessor_();

        /// @brief validate payload
        /// @return
        bool validatePayload_();

        /// @brief validate gripper
        /// @return
        bool validateGripper_();

        /// @brief validate FT
        /// @return
        bool validateFT_();

        /// @brief validate safety
        /// @return
        bool validateSafety_();

        /// @brief validate controller
        /// @return
        bool validateController_(const std::string &);

        /// @brief update controller to given type
        void updateController_(const API &);

        /// @brief switch the controller
        /// @return
        bool switchController_();

        /// @brief set payload
        /// @return
        bool setPayload_();

        /// @brief set gripper
        /// @return
        bool setGripper_();

        /// @brief set safety
        /// @return
        bool setSafety_();

        /// @brief update the controller to run the robot
        /// @return
        bool updateController_();

        /// @brief initialise commands, states and other variables
        void initialise_();

        /// @brief set FT
        /// @return
        bool setFT_();

        /// @brief setup control mode map
        void setControlModeMap_();

        /// @brief setup service required by hardware
        bool setupServices_();

        /// @brief initialise state interface variables
        void initStateVar_();

        /// @brief initialise command interface variables
        void initCmdVar_();

        /// @brief check for robot being in error
        void checkForError_();

        /// @brief print the error onto the console
        void printError_(const error_codes &);

        /// @brief handle forward requests
        bool handleFwdRequest_(const DataProcessorRequest &, DataContainer &);

        /// @brief handle backward requests
        bool handleBwdRequest_(const DataProcessorRequest &, DataContainer &);

        /// @brief setup communication with robot
        bool setupComm_();

        /// @brief connect with robot
        bool connect_();

        /// @brief start the robot
        bool clearErrorState_();

        /// @brief start the robot
        bool powerOnRobot_();

        /// @brief start the robot in error recovery mode
        bool powerOnRobotErrRecovery_();

        /// @brief stop the robot
        bool powerOffRobot_();

        /// @brief shutdown the robot
        bool shutdownRobot_();

        /// @brief automatic error recovery for robot
        bool errorRecovery_();

        /// @brief run the automatic error recovery sequence for robot
        bool executeErrRecovery_();

        /// @brief disconncet with the robot
        bool disconnect_();

        /// @brief check connection with robot
        bool checkConnection_();

        /// @brief validate state interface
        bool validateStateInterface_();

        /// @brief validate command interface
        bool validateCommandInterface_();

        /// @brief switch the control loop based on the controller
        void switchControlLoop_(const API &);

        /// @brief get robot feedback
        bool getFeedback_();

        /// @brief get robot state
        bool getRobotState_();

        /// @brief update FT sensor data
        void updateFTData_(const DataContainer &);

        /// @brief update EE pose data
        void updateEEPosData_(const DataContainer &);

        /// @brief update allied input for joint impedance controller
        /// @return
        bool updateJointImpedance_();

        /// @brief update allied input for cartesian impedance controller
        /// @return
        bool updateCartesianImpedance_();

        /// @brief validate gripper state and command interface
        // bool validateGripperInterface_();

        /// @brief removes conflicting controllers
        void removeConflictingControllers_(const std::vector<std::string> &, std::vector<std::string> &);

        /// @brief go to base
        bool goToBase_();

        /// @brief add buffer time 
        void addBufferTime_();

        /****************      RUN DIFFERENT CONTROLLERS                 ********************* */

        /// @brief run external velocity
        /// @return
        bool extVelocity_();

        /// @brief run external effort
        /// @return
        bool extEffort_();

        /// @brief run ptp controller
        /// @return
        bool jointPtp_();

        /// @brief run replay command
        /// @return
        bool replay_();

        /// @brief run free drive
        /// @return
        bool freeDrive_();

        /// @brief run joint jogging
        /// @return
        bool jointJogging_();

        /// @brief run cartesian jogging
        /// @return
        bool cartesianJogging_();

        /// @brief run joint impedance controller
        /// @return
        bool jointImpedance_();

        /// @brief run cartesian impedance controller
        /// @return
        bool cartesianImpedance_();

        /// @brief run gravity compensation external effort controller
        /// @return
        bool gravityCompExtEffort_();

        /// @brief run gripper
        /// @return
        bool runGripper_();

        /// @brief run tcp ptp controller
        /// @return
        bool tcpPtp_();

        /// @brief change control mode
        /// @param new_mode
        /// @return
        bool changeControlMode_(const std::string &new_mode);
    };
}
#endif