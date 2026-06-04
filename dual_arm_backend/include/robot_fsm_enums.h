/**
 *@file states_actions.h
 * @author Aditya Singh
 * @brief
 * @version 0.1
 * @date 2024-03-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ROBOT_FSM_ENUMS_H_
#define ROBOT_FSM_ENUMS_H_

/// @brief states for the robot state machine
enum class RobotState
{
    eNone = 0,
    eBase = 1,
    eMotion = 2,
    eError = 3,
    eMaxRegime = 4
};

/// @brief actions for the robot state machine
enum class RobotAction
{
    eNone = 0,
    eGoToBaseState = 1,
    eGoToMotionState = 2,
    eGoToErrorState = 3,
    eMaxRegime = 4
};

#endif