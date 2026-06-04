/**
 * @file shared_state.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Shared State for maintaining the state of connected clients
 * @version 0.1
 * @date 2024-07-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SHARED_STATE_H_
#define SHARED_STATE_H_

#include <mutex>
#include <iosfwd>

#define MAX_ALLOWED_WSCLIENTS 1

class SharedState
{
public:
    /// ctor
    SharedState()
    {
        n_clients_ = 0;
    };

    /// dtor
    ~SharedState(){};

    /// @brief update status to reflect addition of client
    /// @return false if max clients are already connected - here max possible clients is 1
    bool addClient();

    /// @brief remove client from list of connected clients
    void removeClient();

    /// @brief returns if client is already connected
    /// @return
    bool hasConnectedClient();

private:
    /// @brief maximum number of permissible websocket clients
    int max_clients_ = MAX_ALLOWED_WSCLIENTS;

    /// @brief number of connected clients
    int n_clients_;

    /// @brief mutex to allow multiple classes to access shared state variables concurrently
    std::mutex mutex_;

    /// @brief returns number of connected clients at the current instant
    /// @return
    int getNConnectedClients_();
};
#endif