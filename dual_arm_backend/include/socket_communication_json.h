/**
 * @file socket_comm_json.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Socket Communication using std::string for communication
 * @version 0.1
 * @date 2024-06-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SOCKET_COMMUNICATION_JSON_H_
#define SOCKET_COMMUNICATION_JSON_H_

#include <iosfwd>
#include <string>
#include <arpa/inet.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <chrono>
#include <thread>
#include "socket_communication.h"

template <class T>
class SocketCommunicationJSON : public SocketCommunication<std::string, std::string, T>
{
public:
    /// @brief ctor
    explicit inline SocketCommunicationJSON(const int& idx) : SocketCommunication<std::string, std::string, T>(idx)
    {
        fds_[0].events = POLL_IN;
        read_timeout_ = READ_TIMEOUT_SEC;
    };

    /// @brief dtor
    inline ~SocketCommunicationJSON() {};

    /// @brief send the data to peer
    /// @param send_str
    /// @return
    bool sendData(const std::string &send_str)
    {
        std::string temp = send_str;

        int ret = send(this->sock_fd_, send_str.c_str(), send_str.size(), 0);

        return (ret > 0);
    }

    /// @brief recv the data from peer
    /// @param recv_str
    /// @return
    bool receiveData(std::string &recv_str)
    {
        int expected_data_size = 0;

        if (isDataAvailable_(expected_data_size))
        {
            char buffer[READ_BUFFER_SIZE]{};

            int ret = read(this->sock_fd_, buffer, (READ_BUFFER_SIZE-1));

            // std::cout << "recd data size : " << ret << std::endl;

            if (ret > 0)
            {
                recv_str.clear();
                std::string temp(buffer);
                recv_str = temp;
                return true;
            }
        }

        return false;
    }

    /// @brief return true if data is available to be read on the socket
    /// @return
    bool isDataAvailable() override
    {
        int n_bytes = 0;
        if (isDataAvailable_(n_bytes))
        {
            return true;
        }

        return false;
    }

    /// @brief set the read timeout - this method can be called multiple times in the smae program to vary the read_timeout() to handle transfer of large sized data
    /// @param
    /// @return
    bool setReadTimeout(const int timeout) override
    {
        if (timeout <= 0)
        {
            // std::cout << "this timeout value is unsupported\n";
            return false;
        }

        read_timeout_ = timeout;
        return true;
    }

private:

    /// @brief timeout for read() operations
    int read_timeout_;

    /// @brief file descriptor polling structure
    struct pollfd fds_[CONNECTION_QUEUE];

    /// @brief return true if data is available on the socket to read
    /// @return bytes_av : number of bytes available to read on the socket
    bool isDataAvailable_(int &bytes_av)
    {
        fds_[0].fd = (this->sock_fd_);

        bytes_av = 0;

        // read poll to timeout read in specified amount of time
        if ((poll(fds_, CONNECTION_QUEUE, read_timeout_ * 1000)) > 0)
        {
            // if there is some data available to be read
            if (fds_[0].revents & POLL_IN)
            {
                // get the size of data available on socket to read
                int expected_data_size = 0;
                if (ioctl(this->sock_fd_, FIONREAD, &expected_data_size) != -1)
                {
                    if (expected_data_size > 0)
                    {
                        bytes_av = expected_data_size;
                        return true;
                    }
                }
            }
        }

        // std::cout << "no data to read\n";
        return false;
    }

    /// @brief return the time lapsed since the beginning of the stopwatch
    /// @return
    double getLapsedTime_(std::chrono::time_point<std::chrono::high_resolution_clock> &b_time)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> cur_time = std::chrono::high_resolution_clock::now();

        double delta_t = std::chrono::duration_cast<std::chrono::seconds>(cur_time - b_time).count();

        return delta_t;
    }
};

#endif
