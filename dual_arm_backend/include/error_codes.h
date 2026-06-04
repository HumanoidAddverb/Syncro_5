/**
 * @file error_codes.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Error codes for the robot end
 * @version 0.1
 * @date 2024-04-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ERROR_CODES_H_
#define ERROR_CODES_H_

namespace HealErrors
{
    /// @brief Base values for the error codes
    const int robot_base_val = 1;
    const int hardware_base_val = 1000 + robot_base_val;
    const int socket_base_val = 1000 + hardware_base_val;
    const int robot_fsm_base_val = 500 + socket_base_val;

    /**
     * @brief types of error
     *
     */
    enum class Code
    {
        eNone = 0,

        /**********     TCP?IP SOCKET ERRORS     ***************/

        eConnectionLost = socket_base_val,
        eDataLoss,
        eFailedToConnect,
        eFailedToDisconnect,

        /**********     ROBOT FSM ERRORS     ***************/

        eDisconnected = robot_fsm_base_val,
        eFailedToSetupRobot, // temporary
        eFailedToRunRobot,   // temporary
        eFailedToSetupState,
        eFailedToClearResources,
        eInvalidConfigurationData, // temporary
        eInvalidControlInput,
        eUnsupportedAPI,
        eBadJSON, // temporary
        eFailedToStopRobot,
        eFailedToPowerOffRobot,
        eInvalidAction,
        eInvalidEvent,
        eInvalidSafetyType,
        eInvalidGripperType,
        eInvalidFTType,
        eInvalidPayloadMass,
        eInvalidPayloadCOM,
        eInvalidPayloadMOI,

        /**********     DATA MANAGER ERRORS     ***************/

        eFailedToGetAction,
        eFailedToGetEvent,
        eFailedToSetEventStatus,
        eFailedToReadData,
        eFailedToWriteData,
        eFailedToGetRobotData,
        eFailedToGetClampForce,
        eFailedToSetRobotData,
        eFailedToSetUserData,
        eFailedToGetAPI,
        eFailedToSetupNetwork,
        eFailedToShutdownNetwork,

        /**********     DATA HANDLER ERRORS     ***************/

        eEmptyInput,

        /**********     DATA STORE ERRORS     ***************/
        eNonUniqueID,

        /**********     EVENT HANDLER ERRORS     ***************/
        eFailedToResetConfig,
        eFailedToOpenGripper,
        eFailedToCloseGripper

    };

};

#endif
