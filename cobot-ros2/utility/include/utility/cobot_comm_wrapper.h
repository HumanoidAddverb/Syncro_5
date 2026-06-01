/**
 * @file cobot_comm_wrapper.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Wrapper over the lower-level communication handler, to facilitate one-step easy control inetrface to the ros side
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef COBOT_COMM_WRAPPER_H_
#define COBOT_COMM_WRAPPER_H_

#define SAFETY_MODE_ERR_RECOVERY 2

#include <array>
#include <algorithm>
#include <data_handler.h>
#include "ros_wrapper_error_codes.h"
#include <api_types.h>
namespace addverb_cobot
{
    class CobotCommWrapper
    {
    public:
        CobotCommWrapper(const int index) : handler_(index) {};

        ~CobotCommWrapper() = default;

        /** setter methods */

        void setSafety(const int safety);

        void setGripper(const GripperConfig &gripper);

        void setFT(const FTConfig &ft);

        void setPayload(const PayloadConfig &payload);

        error_codes switchController(const int);

        error_codes switchController(const int, const ControlInterrupt&);

        error_codes setCmd(const ControlInterrupt &);

        /// @brief this command is supposed to run the next control action (ptp, multipt, etc) on the robot 
        /// @param  
        /// @return 
        error_codes setCmd(const ControllerConfig &);

        /// @brief this command is supposed to set events (gripper etc) on the robot 
        /// @param  
        /// @return 
        error_codes setCmd(const EventConfig &);

        /// @brief this command is supposed to run the next control action (replay controller) on the robot 
        /// @param  
        /// @return 
        error_codes setCmd(const ControllerConfig &, const AdvancedControllerConfig &);

        /// @brief get current robot joint info 
        /// @param  
        /// @return 
        error_codes getFeedback(RobotFeedback&);

        /// @brief get current robot state
        /// @param  
        /// @return 
        error_codes getState(RobotState &);

        /// @brief shutdown communication with remote
        /// @return 
        error_codes shutdownComm();

        /** executor methods */
        error_codes clearErrorState();

        error_codes powerOnRobotErrorRecovery();

        error_codes executeErrorRecovery();
        
        error_codes powerOnRobot();

        error_codes powerOffRobot();

        error_codes connect();

        error_codes checkConnection();

        error_codes disconnect();

        error_codes read();

        error_codes write();

        error_codes setup();

        /*** getter methods */

    private:
        /// @brief data handler - the communicator with the robot
        DataHandler handler_;

        /// @brief default safety type
        int safety_type_ = 0;

        /// @brief controller type
        int controller_;

        /// @brief array of continous control types
        std::array<API,6> continous_ctrl_api_ = {API::eExternalTorqueAPI,
            API::eExternalVelocityAPI,
            API::eGCompExternalTorqueAPI,
            API::eJogJointAPI,
            API::eJogRPYAPI, 
            API::eFreeDriveAPI};

        std::array<API, 1> critical_controllers_api_ = {API::eExternalTorqueAPI};

        /// @brief gripper config
        std::optional<GripperConfig> gripper_;

        /// @brief ft config
        std::optional<FTConfig> ft_;

        /// @brief payload config
        std::optional<PayloadConfig> payload_;
        
        /// @brief latest control command  to the robot
        ControlInterrupt interrupt_;
    };

};

#endif