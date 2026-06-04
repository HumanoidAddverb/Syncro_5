/**
 * @file socket_client_comm_wrapper.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief TCP/IP Socket Client implementation of the comm_wrapper
 *
 * @version 0.1
 * @date 2024-07-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SOCKET_CLIENT_COMM_WRAPPER_H_
#define SOCKET_CLIENT_COMM_WRAPPER_H_

#include "socket_comm_wrapper.h"

class SocketClientCommWrapper : public SocketCommWrapper
{
public:
    /// @brief ctor
    explicit SocketClientCommWrapper(const int &idx) : SocketCommWrapper(idx) {};

    /// @brief dtor
    ~SocketClientCommWrapper() {};

protected:
    /// @brief handle the setup of the data line
    /// @return true only nad only when the data flow is verified
    bool handleDataLineSetup_() override
    {
        if (!establishDataLine_())
        {
            this->comm_.closeConnection();
            return false;
        }

        return true;
    }

private:
    /// @brief return true only and only if data line establishment with the peer is succesful
    /// @return
    bool establishDataLine_()
    {
        int attempt_count = 0;
        bool line_established = false;

        while (attempt_count < CommConfig::max_read_attempts)
        {
            if (this->comm_.isDataAvailable())
            {
                // std::cout << "got data - client\n";
                line_established = true;
                break;
            }

            send(this->init_data_);

            // std::cout << "init data to send : "
            //   << init_data_ << std::endl;

            attempt_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // std::cout << "attempted to get data\n";
        }

        if (line_established)
        {
            return true;
        }

        return false;
    }
};

#endif