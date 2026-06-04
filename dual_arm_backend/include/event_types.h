/**
 *@file states_actions.h
 * @author Siddhi Jain
 * @brief Event types 
 * @version 0.1
 * @date 2024-03-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef EVENT_TYPES_H_
#define EVENT_TYPES_H_

/// @brief sub-actions for the robot state machine
enum class Event
{
    eNone = 0,
    eResetConfig = 1,
    eOpenGripper = 2,
    eCloseGripper = 3,
    eDisableRobot = 4,
    eMaxRegime = 5
};

/// @brief execution status of the event
enum class EventExecutionStatus
{
    eNone = 0,
    eExecuting = 1,
    eFailed = 2,
    eSucceded = 3
};

#endif