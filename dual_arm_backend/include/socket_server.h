/**
 * @file socket_server.h
 * @author Siddhi Jain (siddhi.jainu@addverb.com)
 * @brief Server Implementation for Socket Programming
 * used for sending and receiving data in string format using sockets
 * @version 0.1
 * @date 2023-09-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef SOCKET_SERVER_H_
#define SOCKET_SERVER_H_

#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <iosfwd>
#include <errno.h>
#include <sys/time.h>
#include <poll.h>
#include <fstream>
#include <vector>
#include <cassert>
#include "socket_config.h"

class SocketServer
{
public:
    explicit inline SocketServer(const int &server_idx)
    {
        socket_created_ = false;
        server_idx_ = server_idx;
        std::cout<<"recd server_idx_ "<<server_idx_<<std::endl;
    };

    inline ~SocketServer() {};

    /// @brief method for socket creation
    void createSocket();

    /// @brief initialisation method for server
    /// @return
    bool initConnect(int &);

    /// @brief method to close connection
    void closeConnection();

    /// @brief shutdown the serve end
    void terminate();

protected:
    /// @brief server address
    struct sockaddr_in server_addr_;

    /// @brief client address
    struct sockaddr_in client_addr_;

    /// @brief file descriptor for the server
    int server_fd_;

    /// @brief file descriptor of client
    int client_fd_;

    /// @brief the index of the server
    int server_idx_;

    /// @brief get the server address
    virtual void getServerAddr_();

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
    /// subsequent connect calls to server socket must accept on the same socket
    bool socket_created_;
};

#endif
