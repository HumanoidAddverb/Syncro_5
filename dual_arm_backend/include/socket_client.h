/**
 * @file socket_client.h
 * @author Rajesh Kumar (rajesh.kumar01@addverb.com)
 * @brief Socket Client : form socket TCP based communication
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef SOCKET_CLIENT_H_
#define SOCKET_CLIENT_H_

#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <netinet/tcp.h>
#include <errno.h>
#include <iosfwd>
#include <sys/time.h>
#include <fstream>
#include <cassert>
#include "socket_config.h"
#include <iostream>

/**
 * @brief Socket Client for the implementation
 * of string communication to the system
 *
 */
class SocketClient
{
public:
    explicit inline SocketClient(const int& idx)
    {
        sock_fd_ = SOCKET_FD_INIT_VAL;
        socket_created_ = false;
        server_idx_ = idx;

        std::cout<<"Socket Client for server "<<server_idx_<<std::endl;
    };

    inline ~SocketClient() {};

    /// @brief create the socket
    void createSocket();

    /// @brief initialisation method specific to client
    /// @return
    bool initConnect(int &);

    /// @brief method to close connection
    void closeConnection();

    /// @brief shutdown the cleint endI
    void terminate();

protected:
    /// @brief server address
    struct sockaddr_in server_addr_;

    /// @brief socket descriptor
    int sock_fd_;

    /// @brief IP of the server to connnect
    char ip_[MAX_SIZE_IP];

    /// @brief port of the server to connect
    int port_;

    /// @brief the index of the server
    int server_idx_;

    /// @brief update the port and IP depending on the server
    virtual void setServerSocket_();

    /// @brief set options for the socket
    virtual void setSocketOptions_();

    /// @brief parse the network config file
    /// @param ip
    /// @param port
    void parseNetworkConfig_(std::string &ip, int &port);

    /// @brief is valid port
    /// @param
    /// @return
    bool isValidPort_(const std::string &, int &);

    /// @brief is valid IP
    /// @param
    /// @return
    bool isValidIP_(const std::string &);

    /// @brief get the filepath based on the server index
    /// @param
    /// @return
    bool getFilepath(std::string &);

private:
    /// @brief socket creation flag
    /// subsequent connect calls to client socket must setup comm on the same socket descriptor
    bool socket_created_;
};

#endif
