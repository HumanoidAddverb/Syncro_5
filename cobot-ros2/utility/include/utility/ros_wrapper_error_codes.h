/**
 * @file ros_wrapper_error_codes.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Error codes to be returned by the ROS sytem
 * @version 0.1
 * @date 2025-05-07
 *
 * @copyright Copyright (c) 2025
 *
 */

 #ifndef ROS_WRAPPER_ERROR_CODES_H_
 #define ROS_WRAPPER_ERROR_CODES_H_
 
 namespace addverb_cobot
 {
     enum error_codes
     {
         NoError = 0,
         InvalidRequest,
         DataNotPresent,
         RobotFailedToSetup,
         RobotFailedToPowerOn,
         ConnectionBroken,
         InErrorState,
         FailedToConnect,
         FailedToDisconnnect,
         FailedToWriteData,
         FailedToClearError,
         FailedToReadData,
         InvalidSafetyType, 
         InvalidGripperType,
         InvalidFtType,
         InvalidFTRotationMatrix,
         ReplayError,
         InvalidVelocity,
         InvalidEffort,
         InvalidGripperCmd,
         InvalidState
     };
 
 };
 
 #endif