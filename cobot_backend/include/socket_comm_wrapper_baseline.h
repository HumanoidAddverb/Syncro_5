/**
 * @file socket_comm_wrapper_baseline.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Templated Wrapper class over the communication object
 * NetworkManager will always interact with this wrapper. This wrapper provides the abstraction over the implementation-specific socket processes.
 * Exposes methods to connect to the peer, send/recv data and handles proper termination of the connection
 * This wrapper ensures the verification of established connections and handles the packing/unpacking of data which can vary
 * on a case-to-case basis. Each Derived impelmentation can have its own way of handling the methods.
 * @version 0.1
 * @date 2024-07-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef SOCKET_COMM_WRAPPER_BASELINE_H_
#define SOCKET_COMM_WRAPPER_BASELINE_H_

#include <iosfwd>
#include "network_config.h"

template <typename Tx, typename Rx>
class SocketCommWrapperBaseline
{
public:
    /// @brief ctor
    explicit SocketCommWrapperBaseline(const int& idx) :comm_(idx) {};

    /// @brief dtor
    ~SocketCommWrapperBaseline() {};

    /// @brief connect with the peer
    /// @return :
    // true - if data flow is established between both the ends of the connection
    // false - if failed to connect or data flow could not be established within max_attempts
    bool connect()
    {
        // print_("before socket_comm_wrapper setup");
        // setup socket
        comm_.setup();
        // std::cout<<"setup called again\n";

        bool connection = false;

        // send connection request

        // print_("before init connect socket_comm_wrapper");
        connection = comm_.initConnect();
        // print_("after init connect socket_comm_wrapper");

        if (connection)
        {
            // print_("at leats comm object connected");
            // try to setup data line, and if it fails handle the closing of the connection
            if (!handleDataLineSetup_())
            {
                print_("handle data line fails");   
                return false;
            }
            return true;
        }

        // print_(" comm object not connected");

        return false;
    }

    /// @brief set the initial data to fill the line with before any communication starts
    /// @param
    /// @return
    bool setInitData(const Tx &in_data)
    {
        init_data_ = in_data;
        return true;
    }

    /// @brief send the data to the peer
    /// @param
    /// @return
    virtual bool send(const Tx &) = 0;

    /// @brief read the data from the peer
    /// @param
    /// @return
    virtual bool read(Rx &) = 0;

    /// @brief disconnect from the peer
    /// @return
    bool disconnect()
    {
        comm_.closeConnection();
        return true;
    }

protected:
    /// @brief lower-level communication object
    // for TCP/IP socket communication , this will be either server or client
    BackendComm comm_;

    /// @brief initial data
    Tx init_data_;

    /// @brief handle the establishment of data flow
    /// @return true/false : if data flow is established/not established
    virtual bool handleDataLineSetup_() = 0;

    void print_(const std::string &msg)
    {
        std::cout << msg << std::endl;
    }
};

#endif