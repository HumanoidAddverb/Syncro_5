/**
 * @file data_processor_defs.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Definitions and declarations for all the enums, data structures to be used by the Data Processor pipeline
 * @version 0.1
 * @date 2025-05-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef DATA_PROCESSOR_DEFS_H_
#define DATA_PROCESSOR_DEFS_H_

#include <optional>
#include "hardware_interface_defs.h"
#include <ui_data_types.h>
#include <robot_fsm_enums.h>
#include "data_store.h"

namespace addverb_cobot
{

    /// @brief possible requests for data validator
    enum class DataValidatorRequest
    {
        eNone,
        eSetup,
        ePoint,
        eMultiPointt,
        eVelocity,
        eJointJogging,
        eCartesianJogging,
        eJointImpedance,
        eCartesianImpedance,
        eEffort,
        eFreeDrive,
        eReplay,
        eSafety,
        ePayload,
        eGripperConfig,
        eFTSensor,
        eController,
        eGripperCmd
    };

    /// @brief possible requests for data validator
    enum class DataConverterRequest
    {
        eNone,
        eSetup,
        ePoint,
        eMultiPoint,
        eVelocity,
        eJointJogging,
        eCartesianJogging,
        eJointImpedance,
        eCartesianImpedance,
        eEffort,
        eSafety,
        ePayload,
        eGripperConfig,
        eFTSensor,
        eController,
        eReplay,
        eReadFeedback,
        eReadState,
        eTcpMultipoint,
        eFlexPoint,
        eGripperCmd
    };

    /// @brief possible requests for data validator
    enum class DataCommunicatorRequest
    {
        eNone,
        eSetup,
        eVelocity,
        eJointJogging,
        eCartesianJogging,
        eJointImpedance,
        eCartesianImpedance,
        eEffort,
        ePoint,
        eMultiPoint,
        eReplay,
        eSafety,
        ePayload,
        eGripperConfig,
        eFTSensor,
        eController,
        eConnect,
        eDisconnect,
        eCheckConnectivity,
        ePowerOn,
        eSwitchController,
        ePowerOff,
        eReadFeedback,
        eReadState,
        eTcpMultipoint,
        eFlexPoint,
        eClearErrorState,
        ePowerOnErrRecovery,
        eErrRecoverySequence,
        eGripperCmd,
        eShutdownComm
    };

    /// @brief data processor request
    // contains request for each handler
    struct DataProcessorRequest
    {
        /// @brief request for data validation module to execute
        DataValidatorRequest validate = DataValidatorRequest::eNone;

        /// @brief request for data convertor module to execute
        DataConverterRequest convert = DataConverterRequest::eNone;

        /// @brief request for data communicator module to execute
        DataCommunicatorRequest communicate = DataCommunicatorRequest::eNone;
    };

    /// @brief Data to be processed by the validation handler to run sanity checks
    struct DataValidatorContainer
    {
        /// @brief safety mdoe
        std::optional<hw_interface_defs::SafetyMode> safety;

        /// @brief gripper configuration
        std::optional<hw_interface_defs::GripperConfig> gripper;

        /// @brief ft configuration
        std::optional<hw_interface_defs::FTConfig> ft;

        /// @brief payload configuration
        std::optional<hw_interface_defs::Payload> payload;

        /// @brief configuration to reach a target point in given time
        std::optional<hw_interface_defs::Point> point;

        /// @brief configuration to reach multiple target positions with corresponding times
        std::optional<hw_interface_defs::MultiPoint> multi_point;

        /// @brief the feedback from the robot
        std::optional<hw_interface_defs::RobotFeedback> robot_feedback;

        /// @brief velocity commands to robot
        std::optional<hw_interface_defs::Velocity> velocity;

        /// @brief effort commands to robot
        std::optional<hw_interface_defs::Effort> effort;

        /// @brief controller to run the robot
        std::optional<hw_interface_defs::ControlMode> controller;

        /// @brief replay controller mode
        std::optional<hw_interface_defs::ReplayConfig> replay_config;

        /// @brief tcp multi point configuration
        std::optional<hw_interface_defs::TcpMultipoint> tcp_multi_point;

        /// @brief configuration to reach a target point in given time with given flex factor
        std::optional<hw_interface_defs::FlexPoint> flex_point;

        /// @brief joint jogging mode
        std::optional<hw_interface_defs::JointJog> joint_jogging;

        /// @brief cartesian jogging mode
        std::optional<hw_interface_defs::CartesianJog> cartesian_jogging;

        /// @brief joint impedance mode
        std::optional<hw_interface_defs::JointImpedance> joint_impedance;

        /// @brief cartesian impedance mode
        std::optional<hw_interface_defs::CartesianImpedance> cartesian_impedance;
        
        /// @brief gripper command
        std::optional<hw_interface_defs::GripperCmd> gripper_cmd;

        // keep adding controller specific structures here
    };

    /// @brief Data to be actually sent to the robot
    struct DataConverterContainer
    {
        /// @brief safety type
        std::optional<int> safety_type;

        /// @brief controller type
        std::optional<int> controller_type;

        /// @brief gripper configuration
        std::optional<GripperConfig> gripper;

        /// @brief ft sensor configuration
        std::optional<FTConfig> ft;

        /// @brief payload configuration
        std::optional<PayloadConfig> payload;

        /// @brief controller configuration
        std::optional<ControllerConfig> controller;

        /// @brief control interrupt that goes continously
        std::optional<ControlInterrupt> interrupt;

        /// @brief advanced controller config
        std::optional<AdvancedControllerConfig> advanced_config;

        /// @brief ui data type to hw data types
        std::optional<hw_interface_defs::RobotFeedback> hw_robot_feedback;

        /// @brief robot state
        std::optional<RobotState> robot_state;

        /// @brief event config
        std::optional<EventConfig> event_config;
    };

    struct DataCommunicatorContainer
    {
        /// @brief robot feedback
        std::optional<RobotFeedback> robot_feedback;

        /// @brief robot state
        std::optional<RobotState> robot_state;
    };

    /// @brief the overall container that will be in the interface
    struct DataContainer
    {
        /// @brief data to validate
        std::optional<DataValidatorContainer> validation_data;

        /// @brief data to convert
        std::optional<DataConverterContainer> convert_data;

        /// @brief data to commumicate
        std::optional<DataCommunicatorContainer> communication_data;
    };

};

#endif