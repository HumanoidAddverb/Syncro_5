/**
 * @file socket_comm_wrapper.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief implementation of the socket_comm_wrapper with defined parameters for the template
 * @version 0.1
 * @date 2024-07-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SOCKET_COMM_WRAPPER_H_
#define SOCKET_COMM_WRAPPER_H_

#include <string>
#include <thread>
#include "socket_comm_wrapper_baseline.h"

/// @brief a specific implementation of SocketCommunicationWrapper instantiated with std::string format which will be used for the communication
class SocketCommWrapper : public SocketCommWrapperBaseline<std::string, std::string>
{
public:
    /// @brief ctor
    explicit SocketCommWrapper(const int& idx) : SocketCommWrapperBaseline<std::string, std::string>(idx)
    {
        leftover_exists_ = false;
        file_transfer_alert_ = false;
    };

    /// @brief dtor
    ~SocketCommWrapper() {};

    /// @brief send data over the socket
    /// @param send_data
    /// @return
    bool send(const std::string &send_data)
    {
        std::string data = CommConfig::start_delim + send_data + CommConfig::end_delim;

        if (data.size() > CommConfig::quick_transfer_limit)
        {
            file_transfer_alert_ = true;
            // std::cout << "file transfer alert on\n";
            // std::cout << "expect file size : " << (data.size() - 2) << std::endl;
        }

        return this->comm_.sendData(data);
    }

    /// @brief read the available packet into the provided string
    /// @param out
    /// @return
    bool read(std::string &out)
    {
        out.clear();

        bool done = false;
        bool started = false;
        char delim;
        int pos = 0;
        std::string read_chunk;

        // set delimitor to search for
        delim = CommConfig::start_delim[0];

        if (file_transfer_alert_)
        {
            if (!this->comm_.setReadTimeout(CommConfig::file_transfer_timeout))
            {
                return false;
            }
        }

        while (!done)
        {
            // get the chunk of data to process on
            if (leftover_exists_)
            {
                read_chunk = leftover_packet_;
            }
            else
            {
                // read a chunk
                if (!readChunks_(read_chunk))
                {
                    return false;
                }
            }

            if (!started)
            {
                // if starting delimiter is encountered, find the entire message
                if (containsDelim_(delim, read_chunk, pos))
                {
                    leftover_packet_ = read_chunk.substr((pos + 1), read_chunk.size());
                    leftover_exists_ = true;
                    started = true;
                }
            }
            else
            {
                // update delimiter to the one expected at the end
                delim = CommConfig::end_delim[0];
                if (containsDelim_(delim, read_chunk, pos))
                {
                    read_packet_ = read_packet_ + read_chunk.substr(0, pos);
                    done = true;
                    leftover_exists_ = false;
                    leftover_packet_.clear();
                }
                else
                {
                    read_packet_ = read_packet_ + read_chunk;
                }
            }
        }

        if (done)
        {
            out = read_packet_;

            read_packet_.clear();
        }

        if (file_transfer_alert_)
        {
            // std::cout << out << std::endl;
            file_transfer_alert_ = false;

            if (!this->comm_.setReadTimeout(CommConfig::data_transfer_timeout))
            {
                return false;
            }
        }

        return true;
    }

private:
    /// @brief packet read in this instance
    std::string read_packet_;

    /// @brief data from the last pakcet that can be used
    std::string leftover_packet_;

    /// @brief flag to indicate whether leftover data should be expected or not
    bool leftover_exists_;

    /// @brief flag to indicate transfer of large sized data packets/files
    bool file_transfer_alert_;

    /// @brief read a chunk of data
    /// @param store string to store the data in
    /// @return
    bool readChunks_(std::string &store)
    {
        store.clear();

        if (!this->comm_.receiveData(store))
        {
            // std::cout << "returned false from readData() of socket\n";
            return false;
        }

        return true;
    }

    /// @brief checks the input string for the specified delimiter
    /// @param val
    /// @param found
    /// @param input
    /// @return
    bool containsDelim_(const char &val, const std::string &input, int &pos)
    {
        for (int i = 0; i < input.size(); i++)
        {
            if (input[i] == val)
            {
                pos = i;
                return true;
            }
        }

        return false;
    }
};

#endif