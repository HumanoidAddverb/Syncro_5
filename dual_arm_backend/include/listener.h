/**
 * @file listener.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Listener implementation for the backend
 * provides a blocking call to socket::accept() method
 * once a connection is established with the client, the server is handed over the entire flow of operations
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef LISTENER_H_
#define LISTENER_H_

#include <memory>
#include <iosfwd>
#include <functional>
#include "backend_decl.h"
#include "http_server.h"

class Listener : public std::enable_shared_from_this<Listener>
{
public:
    Listener(boost::asio::io_context &);

    ~Listener()
    {
        std::cout<<"~Listener()\n";
    };

    /// @brief get the error code
    /// @return
    BoostErrorCode getErrorCode() const
    {
        return ec_;
    }

    /// @brief wait for the client to come
    void waitForClient();

    /// @brief setup the listener with the socket address and port needed
    /// @param  
    /// @return 
    bool setup(TCP::endpoint);

private:
    /// @brief stores the error code in case of failure, can be used to provide access to reason for failure outside this class
    BoostErrorCode ec_;

    /// @brief acceptor
    TCP::acceptor acceptor_;

    /// @brief socket
    TCP::socket socket_;

    /// @brief I/O context object
    boost::asio::io_context ioc_;

    /// @brief shared state of clients
    std::shared_ptr<SharedState> shared_state_;

    /// @brief methods to call once the server accepts the client
    /// @param
    void onAccept_(BoostErrorCode);

    /// @brief check for failure based on error code 
    bool checkFail_(const BoostErrorCode&);

    /// @brief method to call on failure 
    /// @param  
    void onFail_(const BoostErrorCode&,const std::string&);

    /// @brief method to call on failure
    /// @param  
    void onFail_(const std::string&);
};

#endif