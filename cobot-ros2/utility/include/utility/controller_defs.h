/**
 * @file controller_defs.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief structures and enumerations used in controllers 
 * @version 0.1
 * @date 2025-05-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #ifndef CONTROLLER_DEFS_H_
 #define CONTROLLER_DEFS_H_

 namespace addverb_cobot
 {
    /// @brief possible states of transfer
    enum class TransferState
    {
        eNone,
        eWaitingForPoint,
        ePublished,
        eRejected,
        eIdling,
        eExecuting
    };

    /// @brief possible commands for transfer
    enum class TransferCommand
    {
        eNone,
        eRcdNewTraj,
        eTransferring,
        ePublish,
        eIdle,
        eExecute
    };

    enum class GripperState
    {
        eNone,
        eClosed,
        eOpen
    };

    enum class GripperTransferCommand
    {
        eNone,
        eHasCommand,
    };

    enum class effortControllerStatus
    {
        eInactive, 
        eReady,
        eActivate
    };

    enum class effortControllerEvents
    {
        eNone,
        eShouldActivate,
        eRequestedToActivate
    };
 };

 #endif