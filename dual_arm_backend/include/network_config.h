/**
 * @file network_config.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Network Configurations for communication with the robot
 * @version 0.1
 * @date 2024-06-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef NETWORK_CONFIG_H_
#define NETWORK_CONFIG_H_

#include "socket_communication_json.h"
#include "socket_client.h"
#include "communication_types.h"
#include "socket_server.h"

/// @brief configuration and parameters for the communication
namespace CommConfig
{
    /// @brief delimitor at the beginning of a packet
    const std::string start_delim = "#";

    /// @brief delimitor at the end of a packet
    const std::string end_delim = "#";

    /// @brief maximum attempts to read
    const int max_read_attempts = 10;

    /// @brief number of bytes that dictate the limit of data - exceeding which it has to be considered to be a file transfer
    const int quick_transfer_limit = 4096;

    /// @brief size of the static read buffer
    const int read_buffer_size = 4096;

    /// @brief timeout limit for transfer of large files
    const int file_transfer_timeout = 300;

    /// @brief timeout limit for synchronous data transfer operations
    const int data_transfer_timeout = 5;

};

// communication object for Backend
typedef SocketCommunicationJSON<SocketClient> BackendComm;

// communication object for Robot
typedef SocketCommunicationJSON<SocketServer> RobotComm;

// comm type - backend
#define BACKEND_COMM_TYPE CommType::eTCPClient

// comm type - robot
#define ROBOT_COMM_TYPE CommType::eTCPServer

#endif