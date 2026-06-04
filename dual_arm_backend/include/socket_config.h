/**
 * @file comm_configs.h
 * @author Rajesh Kumar (rajesh.kumar01@addverb.com)
 * @brief Comm Configs used to setup the communication
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef COMM_CONFIGS_H_
#define COMM_CONFIGS_H_

/// loop back ip : change this for a different device
#define LOOPBACK_SERVER_IP "127.0.0.1"

#define ETH_IP "192.168.0.12" //cobot

#define WIFI_IP "10.0.2.92"

#define DEFAULT_SERVER_IP ETH_IP

/// hopefully this port isn't blocked :)
#define DEFAULT_SERVER_PORT 15263

/// number of connection requests in queue for server
#define CONNECTION_QUEUE 1

// IP for the haptic server
#define HAPTIC_SERVER_IP "127.0.0.1"

// port of the haptic server
#define HAPTIC_SERVER_PORT 23856

// initial value for socket file descriptor
#define SOCKET_FD_INIT_VAL -1

// maximum expected size for the IP address
#define MAX_SIZE_IP 255

// timeout for generic socket operations
#define READ_TIMEOUT_SEC 5 

// size for the static read buffer
#define READ_BUFFER_SIZE 4096

#endif
