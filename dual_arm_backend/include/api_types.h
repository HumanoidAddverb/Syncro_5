/**
 * @file api_types.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief API types to be run on the robot
 * User MUST add each new API to this list to run on the robot
 *
 * @version 0.1
 * @date 2024-04-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef API_TYPES_H_
#define API_TYPES_H_

/// mapping the controller ID with its type
#define JOG_AXIS_API_ID 1
#define FLEX_JOG_AXIS_API_ID 2
#define JOG_RPY_API_ID 3
#define FLEX_JOG_RPY_API_ID 4
#define JOG_JOINT_API_ID 5
#define FLEX_JOG_JOINT_API_ID 6
#define LIN_VEL_API_ID 7
#define FLEX_LIN_VEL_API_ID 8
#define FREE_DRIVE_API_ID 9
#define MULTI_POINT_API_ID 10
#define API1_ID 11 // custom API : number one (for now it is labelled 'home')
#define API2_ID 12
#define API3_ID 13
#define API4_ID 14
#define API5_ID 15
#define API6_ID 16
#define PLAY_REC_API_ID 17

/**
 * @brief API implementations list
 * contains all the possible APIs to run on the robot
 * currently limited to 15 APIs
 */
enum class API
{
    eNone = 0,
    eJogAxisAPI = 1,
    eFlexJogAxisAPI = 2,
    eJogRPYAPI = 3,
    eFlexJogRPYAPI = 4,
    eJogJointAPI = 5,
    eFlexJogJointAPI = 6,
    eLinearVelocityAPI = 7,
    eFlexLinearVelocityAPI = 8,
    eFreeDriveAPI = 9,
    eMultiPointAPI = 10,
    eCustomAPI1 = 11,
    eCustomAPI2 = 12,
    eCustomAPI3 = 13,
    eCustomAPI4 = 14,
    eCustomAPI5 = 15,
    eCustomAPI6 = 16,
    ePlayRecAPI = 17,
    eExternalVelocityAPI = 18,
    eExternalTorqueAPI = 19,
    eJointImpedanceAPI = 20,
    eMultiJointImpedanceAPI = 21,
    eCartesianImpedanceAPI = 22,
    eTcpMultiPointAPI = 23,
    eGCompExternalTorqueAPI = 24,
    eMaxRegime = 25
};
#endif