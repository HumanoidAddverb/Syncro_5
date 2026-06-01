/**
 * @file socket_communication.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Socket Communication Implementation
 * Templated Communication implementation for Socket
 *  provides methods for
 *  sending and receiving data from the socket
 *  connection is established by server and client implementations
 *  reconnecting
 *
 * @version 0.1
 * @date 2023-09-05
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef SOCKET_COMMUNICATION_H_
#define SOCKET_COMMUNICATION_H_

#include <iosfwd>
#include "socket_config.h"

// templated with the
// transmission data-type = Tx
// receiving data-type = Rx
// underlying object to send/recv data = T (Client/Server) in this case
template <typename Tx, typename Rx, class T>
class SocketCommunication
{
public:
    explicit inline SocketCommunication(const int &idx) : worker_(idx)
    {
        sock_fd_ = SOCKET_FD_INIT_VAL;
    };

    inline ~SocketCommunication() {};

    /// @brief Send the data
    /// @param
    /// @return
    bool sendData(const Tx &out)
    {
        Tx temp;

        temp = out;

        int ret = send(sock_fd_, &temp, sizeof(Tx), 0);

        if (ret > 0)
        {
            return true;
        }

        return false;
    }

    /// @brief Receive the data
    /// @param
    /// @return
    bool receiveData(Rx &out)
    {
        Rx temp;

        int ret = read(sock_fd_, &temp, sizeof(Rx));

        if (ret > 0)
        {
            out = temp;
            return true;
        }

        return false;
    }

    /// @brief create the socket
    void setup()
    {
        worker_.createSocket();
    }

    /// @brief waits for connection to be established
    /// @return false if unable to establish connection
    bool initConnect()
    {
        return worker_.initConnect(sock_fd_);
    }

    /// @brief close socket connection
    void closeConnection()
    {
        worker_.closeConnection();
    }

    /// @brief return if data is available to read on the socket
    /// @return
    virtual bool isDataAvailable()
    {
        return true;
    }

    /// @brief set the read timeout for the socket operations
    /// @param
    /// @return
    virtual bool setReadTimeout(const int)
    {
        return true;
    }

protected:
    // sockfd required to send and receive the data
    int sock_fd_;

    // worker that can either be a client or server
    T worker_;
};

#endif