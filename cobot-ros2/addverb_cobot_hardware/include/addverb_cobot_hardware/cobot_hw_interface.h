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
#include <unordered_map>
#include <cstdint>
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
        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

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

        /**
         * @brief struct to hold the intent for mode switch, which will be set by the service callback and used in the control loop to switch the mode
         */
        struct ModeSwitchIntent 
        {
            bool pending = false;
            bool revert_to_default_velocity = false;
            std::string target_control_mode = "";
            int target_robot_index = 0;

            void reset()
            {
                pending = false;
                revert_to_default_velocity = false;
                target_control_mode = "";
                target_robot_index = 0;
            }
        } mode_switch_intent_;

        /// @brief number of grippers in the system
        int num_grippers_ = 0;

        /// @brief number of robot joints
        int num_joint_dof_ = 0;
        
        /// @brief Data Validator to validate the data received from the robot
        std::shared_ptr<DataValidator> data_validator_;

        /// @brief cobot services
        std::shared_ptr<CobotServices> cobot_services_node_;

        /// @brief cobot executor
        std::shared_ptr<CobotExecutor> cobot_executor_;

        /// @brief cobot auxiliary
        std::shared_ptr<CobotAuxiliary> cobot_auxiliary_node_;

        /// @brief Maps for different robot types

        /// @brief map between arm_name and vector indices mapped to interfaces
        std::unordered_map<std::string, std::vector<uint16_t>> arm_groups_;
        /// @brief map between gripper_name and vector indices mapped to interfaces
        std::unordered_map<std::string, std::vector<int>> gripper_groups_;
        /// @brief map between arm_name and robot index
        std::unordered_map<std::string, int> arm_index_;
        /// @brief map between robot index and arm_name
        std::unordered_map<int, std::string> arm_index_reverse_;
        /// @brief map between safety type and robot index
        std::unordered_map<int, uint16_t> safety_map_;
        /// @brief map between payload status and robot index
        std::unordered_map<int, int> payload_status_map_;
        /// @brief map between gripper type and robot index
        std::unordered_map<int, int> gripper_type_map_;
        /// @brief map between force torque sensor type and robot index
        std::unordered_map<int, int> ft_type_map_;
        /// @brief map between force torque sensor rotation matrix and robot index
        std::unordered_map<int, std::vector<std::vector<double>>> ft_rotation_matrix_map_;
        /// @brief map between payload mass and robot index
        std::unordered_map<int, double> payload_mass_map_;
        /// @brief map between payload center of gravity and robot index
        std::unordered_map<int, std::vector<double>> payload_cog_map_;
        /// @brief map between payload inertia and robot index 
        std::unordered_map<int, std::vector<double>> payload_intertia_map_;
        /// @brief map between robot index and vector of gpio entries
        std::unordered_map<int, std::vector<hw_interface_defs::GpioEntry>> gpio_map_;
        /// @brief map between robot index and robot control data struct
        std::unordered_map<int, hw_interface_defs::RobotControlData> robot_control_data_map_;
        /// @brief map between robot index and robot state data struct
        std::unordered_map<int, hw_interface_defs::RobotStateData> robot_state_data_map_;
        /// @brief map between robot index and robot motion command struct for commands
        std::unordered_map<int, hw_interface_defs::RobotMotionStruct> robot_motion_command_data_map_;
        /// @brief map between robot index and robot motion command struct for states
        std::unordered_map<int, hw_interface_defs::RobotMotionStruct> robot_motion_state_data_map_;
        /// @brief map between robot name and gripper index
        std::unordered_map<std::string, int> gripper_map_;

        /// @brief map between robot index and data processor  
        std::unordered_map<int, std::shared_ptr<DataProcessor>> data_processor_map_;
        /// @brief map between robot index and data communicator 
        std::unordered_map<int, std::shared_ptr<DataProcessor>> data_communicator_map_;
        
        /// @brief map between robot index and control mode
        std::unordered_map<int, hw_interface_defs::ControlMode> ros_controller_map_;
        /// @brief map between robot index and effort controller status
        std::unordered_map<int, addverb_cobot::effortControllerStatus> robot_effort_controller_status_map_;
        /// @brief map between robot index and effort controller events
        std::unordered_map<int, addverb_cobot::effortControllerEvents> robot_effort_controller_event_map_;
        /// @brief map between robot index and control loop function
        std::unordered_map<int, std::function<bool()>> control_loop_map_;
        /// @brief map between robot index and multi-point data
        std::unordered_map<int, hw_interface_defs::MultiPoint> multi_point_map_;
        /// @brief map between robot index and TCP multi-point data
        std::unordered_map<int, hw_interface_defs::TcpMultipoint> tcp_multi_point_map_;
        /// @brief map between robot index and Auxiliary objects
        std::unordered_map<int, std::shared_ptr<CobotAuxiliary>> cobot_auxiliary_node_map_;
        
        /// @brief vector of robot names in the system
        std::vector<std::string> arm_names_;
        /// @brief vector of position commands of the system exposed to command interface. This does not include gripper.
        std::vector<double> position_cmd_;
        /// @brief vector of velocity commands of the system exposed to command interface. This does not include gripper.
        std::vector<double> velocity_cmd_;
        /// @brief vector of effort commands of the system exposed to command interface. This does not include gripper.
        std::vector<double> effort_cmd_;

        /// @brief vector of position states of the system exposed to state interface. This does not include gripper.
        std::vector<double> position_state_;
        /// @brief vector of velocity states of the system exposed to state interface. This does not include gripper.
        std::vector<double> velocity_state_;
        /// @brief vector of effort states of the system exposed to state interface. This does not include gripper.
        std::vector<double> effort_state_;

        /// @brief vector of gripper position states of the system exposed to state interface
        std::vector<double> gripper_position_state_;
        /// @brief vector of gripper velocity states of the system exposed to state interface
        std::vector<double> gripper_velocity_state_;
        /// @brief vector of gripper effort states of the system exposed to state interface
        std::vector<double> gripper_effort_state_;

        /// @brief vector of recorder state to hold if controller have been switched
        std::vector<uint8_t> recorder_state_;

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
        bool validatePayload_(const int robot_index);

        /// @brief validate gripper
        /// @return
        bool validateGripper_(const int robot_index);

        /// @brief validate FT
        /// @return
        bool validateFT_(const int robot_index);

        /// @brief validate safety
        /// @return
        bool validateSafety_();

        /// @brief validate controller
        /// @return
        bool validateController_(const std::string &, const int robot_index);

        /// @brief update controller to given type
        void updateController_(const API &, const int robot_index);

        /// @brief switch the controller
        /// @return
        bool switchController_(const int robot_index);

        /// @brief set payload
        /// @return
        bool setPayload_(const int robot_index);

        /// @brief set gripper
        /// @return
        bool setGripper_(const int index);

        /// @brief set safety
        /// @return
        bool setSafety_();
        bool setSafety_(const int robot_index);

        /// @brief update the controller to run the robot
        /// @return
        bool updateController_(const int robot_index);

        /// @brief initialise commands, states and other variables
        void initialise_();

        /// @brief set FT
        /// @return
        bool setFT_(const int robot_index);

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

        /// @brief handle forward requests for specific robot
        bool handleFwdRequest_(const DataProcessorRequest &, DataContainer &, const int robot_index);
        
        /// @brief handle backward requests for specific robot
        bool handleBwdRequest_(const DataProcessorRequest &, DataContainer &, const int robot_index);


        /// @brief setup communication with robot
        bool setupComm_();

        /// @brief connect with robot
        bool connect_();

        /// @brief start the robot
        bool clearErrorState_();
        bool clearErrorState_(const int index);


        /// @brief start the robot
        bool powerOnRobot_();

        /// @brief start the robot in error recovery mode
        bool powerOnRobotErrRecovery_();
        /// @brief start the robot in error recovery mode for specific robot
        bool powerOnRobotErrRecovery_(const int index);

        /// @brief stop the robot
        bool powerOffRobot_();
        /// @brief stop the specific robot
        bool powerOffRobot_(const int index);
        
        /// @brief shutdown the robot
        bool shutdownRobot_();

        /// @brief automatic error recovery for robot
        bool errorRecovery_();

        /// @brief run the automatic error recovery sequence for robots
        bool executeErrRecovery_();
        /// @brief run the automatic error recovery sequence for specific robot
        bool executeErrRecovery_(const int index);

        /// @brief disconncet with the robot
        bool disconnect_();

        /// @brief check connection with robot
        bool checkConnection_();

        /// @brief validate state interface
        bool validateStateInterface_();

        /// @brief validate command interface
        bool validateCommandInterface_();

        /// @brief switch the control loop based on the controller
        void switchControlLoop_(const API &, const int robot_index);

        /// @brief get robot feedback
        bool getFeedback_();

        /// @brief get robot state
        bool getRobotState_();

        /// @brief update FT sensor data
        void updateFTData_(const DataContainer &, const int robot_index);

        /// @brief update EE pose data
        void updateEEPosData_(const DataContainer &, const int robot_index);

        /// @brief update allied input for joint impedance controller
        /// @return
        bool updateJointImpedance_(const int robot_index);

        /// @brief update allied input for cartesian impedance controller
        /// @return
        bool updateCartesianImpedance_(const int robot_index);

        /// @brief validate gripper state and command interface
        // bool validateGripperInterface_();

        /// @brief removes conflicting controllers
        void removeConflictingControllers_(const std::vector<std::string> &, std::vector<std::string> &);

        /// @brief go to base
        bool goToBase_();
        /// @brief go to base for specific robot
        bool goToBase_(const int robot_index);


        /// @brief add buffer time 
        void addBufferTime_(const int robot_index);

        /****************      RUN DIFFERENT CONTROLLERS                 ********************* */

        /// @brief run external velocity
        /// @return
        bool extVelocity_(const int robot_index);

        /// @brief run external effort
        /// @return
        bool extEffort_(const int robot_index);

        /// @brief run ptp controller
        /// @return
        bool jointPtp_(const int robot_index);

        /// @brief run replay command
        /// @return
        bool replay_(const int robot_index);

        /// @brief run free drive
        /// @return
        bool freeDrive_(const int robot_index);

        /// @brief run joint jogging
        /// @return
        bool jointJogging_(const int robot_index);

        /// @brief run cartesian jogging
        /// @return
        bool cartesianJogging_(const int robot_index);

        /// @brief run joint impedance controller
        /// @return
        bool jointImpedance_(const int robot_index);

        /// @brief run cartesian impedance controller
        /// @return
        bool cartesianImpedance_(const int robot_index);

        /// @brief run gravity compensation external effort controller
        /// @return
        bool gravityCompExtEffort_(const int robot_index);

        /// @brief run gripper
        /// @return
        bool runGripper_();

        /// @brief run tcp ptp controller
        /// @return
        bool tcpPtp_(const int robot_index);

        /// @brief change control mode
        /// @param new_mode
        /// @return
        bool changeControlMode_(const std::string &new_mode, const int robot_index);

        /**
         * @brief Initializes all the required mapping for different robots
         */
        void initializeMapping();

        /**
         * @brief Creates map of robot name and number of joint
         */
        void createArmGroupMapping();

        void createArmIndexMap();

        /**
         * @brief create safety mapping
         */
        void createHardwareSafetyMapping();

        /**
         * @brief create payload status mapping
         */
        void createPayloadStatusMapping();

        /**
         * @brief validate the payload parameters
         */
        bool checkPayload();

        /**
         * @brief create gripper type mapping
         */
        void createGripperTypeMapping();
        /**
         * @brief create force torque sensor type mapping
         */
        void createFTMapping();
        /**
         * @brief create force torque sensor rotation matrix mapping
         */
        void createFTRotationMatrixMapping();

        /**
         * @brief create mapping for payload mass
         */
        void createPayloadMassMapping();

        /**
         * @brief create mapping for payload center of gravity
         */
        void createPayloadCOGMapping();

        /**
         * @brief create mapping for payload Inertia
         */
        void createPayloadInertiaMapping();
        
        /**
         * @brief create mapping for gpio entries
         */
        void createGPIORegistry();

        /**
         * @brief create mapping for robot control data
         */
        void createRobotControlDataMapping();

        /**
         * @brief create mapping for robot state data
         */
        void createRobotStateDataMapping();

        /**
         * @brief create mapping for GPIO state interfaces
         */
        void createGPIOStateInterfaces(std::vector<hardware_interface::StateInterface>& state_interfaces);
        
        /**
         * @brief create mapping for GPIO command interfaces
         */
        void createGPIOCommandInterfaces(std::vector<hardware_interface::CommandInterface>& command_interfaces);
        
        /**
         * @brief create mapping for Robot motion mapping
         */
        void createRobotMotionMapping();

        /**
         * @brief create mapping for robot state
         */
        void createRobotStateMapping();
        
        /**
         * @brief create mapping for communication object
         */
        void createCommunicationMapping();

        /**
         * @brief create mapping for control mode
         */
        void createControlModeMapping();

        /**
         * @brief create mapping for effort
         */
        void createEffortControllerMapping();
        
        /**
         * @brief create mapping for control loop
         */
        void createControlLoopMap();

        /**
         * @brief create mapping for multi point loop
         */
        void createMultiPointMap();

        /**
         * @brief get the string from map api
         */
        std::string getStringFromAPI(API api);

        /**
         * @brief update the commands mapping from hardware interface variables to the robot control data struct
         */
        void updateCommandsMappingFromHWInterface();

        /**
         * @brief extract robot name from the input string
         */
        std::string extractRobotName(const std::string& input);

        /**
         * @brief template function to convert string to any type
         */
        template<typename T>
        T convertFromString(const std::string& value);

        /**
         * @brief template function to create mapping for any type of parameter
         */
        template<typename T>
        void createMapping(const std::string& param_suffix,
                                            std::unordered_map<int, T>& target_map);
    };
}
#endif