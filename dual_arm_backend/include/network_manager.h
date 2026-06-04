/**
 * @file network_manager.h
 * @author Siddhi Jain , Aditya Singh (siddhi.jain@addverb.com, aditya.singh@addverb.com)
 * @brief Manager over the communication protocol
 * basically runs the communication at a high frequency in a separate thread
 * Templated with :
 *  the object being used for communication
 *  the data type being used for the sending and recieving across the network
 *  the type of communication which happens - ie tcp/ip : this can be further extended to more such ways
 *
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef NETWORK_MANAGER_H_
#define NETWORK_MANAGER_H_

#include <thread>
#include <mutex>
#include <iosfwd>
#include <functional>
#include "communication_types.h"


template <typename T, typename U, typename V, CommType W> // u->tx,v->rx
class NetworkManager
{
public:
    /// @brief ctor
    explicit inline NetworkManager(const int& idx) : net_comm_(idx)
    {
        is_peer_connected_ = false;
        net_flag_ = false;
        comm_type_ = W;
    };

    /// @brief dtor
    inline ~NetworkManager()
    {
        net_flag_ = false;

        if (net_thread_.joinable())
        {
            net_thread_.join();
        }
    };

    /// @brief setup the network manager
    /// @return
    bool setup()
    {
        is_peer_connected_ = false;
        comm_func_ = nullptr;
        net_flag_ = true;

        // will initialize the thread for continuous read/write.

        {
            if (net_thread_.joinable())
            {
                net_thread_.join();
            }

            switch (comm_type_)
            {
            case CommType::eTCPServer:
                comm_func_ = std::bind(&NetworkManager::serverComm_, this);
                std::cout << "server communication setup\n";
                break;

            case CommType::eTCPClient:
                comm_func_ = std::bind(&NetworkManager::clientComm_, this);
                std::cout << "communication pipeline setup - acting as client\n";
                break;

            default:
                std::cout << "No know type of communication\n";
                return false;
            }
            net_thread_ = std::thread(&NetworkManager::startReadWrite_, this);
        }

        return true;
    }

    /// @brief connect with the peer
    /// @return
    bool connect()
    {
        U send;
        mut_.lock();
        send = send_data_;
        mut_.unlock();

        if (!net_comm_.setInitData(send))
        {
            std::cout << "failed to initialize the stream\n";
            return false;
        }

        // print_("set init data network_manager");
        bool connection_status = net_comm_.connect();

        // print_("connected network_manager");
        mut_.lock();
        is_peer_connected_ = connection_status;
        mut_.unlock();

        if (connection_status)
        {
            std::cout << "Connection established with peer.\n";
            return true;
        }

        return false;
    }

    /// @brief close the connection
    /// @return
    bool disconnect()
    {
        mut_.lock();
        is_peer_connected_ = false;
        mut_.unlock();

        net_comm_.disconnect();

        std::cout << "socket connection closed\n";

        return true;
    }

    /// @brief clear the allocated socket descriptor
    void clearResource()
    {
        net_comm_.disconnect();
    }

    /// @brief get data from the peer
    /// @param
    /// @return
    bool getData(V &recv_data)
    {
        mut_.lock();
        recv_data = recv_data_;
        mut_.unlock();
        return true;
    }

    /// @brief send data to the peer
    /// @param
    /// @return
    bool setData(const U &send_data)
    {
        mut_.lock();
        send_data_ = send_data;
        mut_.unlock();
        return true;
    }

    /// @brief return the connectivity status with the peer
    /// @return
    bool isPeerConnected() const
    {
        mut_.lock();
        bool out = is_peer_connected_;
        mut_.unlock();

        return out;
    }

    /// @brief shutdown the pipeline for communicating with the peer
    /// @return
    bool shutdown()
    {
        while (getNetworkFlag_())
        {
            setNetworkFlag_(false);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (getNetworkFlag_())
        {
            std::cout << "failed to toggle flag working the network thread\n";
            return false;
        }

        if (net_thread_.joinable())
        {
            net_thread_.join();
        }

        std::cout << "shutdown network\n";

        return true;
    }

private:
    /// @brief function to be used for communication
    std::function<void()> comm_func_;

    /// @brief thread to run communication with the peer
    std::thread net_thread_;

    /// @brief mutex to prevent datarace
    mutable std::mutex mut_;

    ///@brief socket communication object
    T net_comm_;

    /// @brief variable to send data
    U send_data_;

    /// @brief  variable to recieve data
    V recv_data_;

    /// @brief type of communication
    CommType comm_type_;

    /// @brief  flag to check for socket  connection status.
    bool is_peer_connected_;

    /// @brief network flag
    bool net_flag_;

    /// @brief function to call read and write functions continously in thread
    void startReadWrite_()
    {
        // in case the network flag is not updated
        while (!getNetworkFlag_())
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }

        // once the flag is up
        while (getNetworkFlag_())
        {
            bool connected = false;

            mut_.lock();
            connected = is_peer_connected_;
            mut_.unlock();

            if (connected)
            {
                comm_func_();
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::nanoseconds(10));
            }

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }
    }

    /// @brief communication function when the communicating object is TCP Server
    void serverComm_()
    {
        U send;
        V recv;
        bool flag = true;

        mut_.lock();
        send = send_data_;
        mut_.unlock();

        flag = net_comm_.send(send);

        if (flag)
        {
            flag = net_comm_.read(recv);
        }

        mut_.lock();
        is_peer_connected_ = flag;
        if (flag)
        {
            recv_data_ = recv;
        }
        mut_.unlock();
    }

    /// @brief communication function when the communicating object is TCP Client
    void clientComm_()
    {
        V recv;
        U send;
        bool flag = true;

        // recv data
        flag = net_comm_.read(recv);

        // only if read succeded, attempt to write
        if (flag)
        {
            // acquire data to be sent
            mut_.lock();
            send = send_data_;
            mut_.unlock();
            flag = net_comm_.send(send);
        }

        // update the recd data and connection status
        mut_.lock();
        is_peer_connected_ = flag;
        if (flag)
        {
            recv_data_ = recv;
        }
        mut_.unlock();
    }

    /// @brief return the current value of network flag
    /// @return
    bool getNetworkFlag_()
    {
        bool out = false;

        mut_.lock();
        out = net_flag_;
        mut_.unlock();

        return out;
    }

    /// @brief set network flag to the given value
    /// @param
    void setNetworkFlag_(const bool &flag)
    {
        mut_.lock();
        net_flag_ = flag;
        mut_.unlock();
    }

    void print_(const std::string &msg)
    {
        std::cout << msg << std::endl;
    }
};

#endif /* NETWORK_MANAGER_H_ */
