/**
 * @file socket_server_comm_wrapper.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief TCP/IP server implementation over the underlying socket server being used for communication
 * @version 0.1
 * @date 2024-07-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SOCKET_SERVER_COM_WRAPPER_H_
#define SOCKET_SERVER_COM_WRAPPER_H_

#include <iosfwd>
#include "socket_comm_wrapper.h"


class SocketServerCommWrapper : public SocketCommWrapper
{
public:
    /// @brief ctor
    explicit SocketServerCommWrapper(const int& idx) : SocketCommWrapper(idx){};

    /// @brief dtor
    ~SocketServerCommWrapper() {};

protected:
    /// @brief handle the setup of the data line
    /// @return true only nad only when the data flow is verified
    bool handleDataLineSetup_() override
    {
        if (!establishDataLine_())
        {
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
            this->comm_.sendData(this->init_data_);

            if (this->comm_.isDataAvailable())
            {
                std::cout << "got data - server\n";
                line_established = true;
                break;
            }

            attempt_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            std::cout << "attempt \n";
        }

        if (line_established)
        {
            return true;
        }

        return false;
    }
};

#endif