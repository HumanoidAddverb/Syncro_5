/**
 * @file websocket_server.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Websocket server
 * @version 0.1
 * @date 2024-06-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef WEBSOCKET_SERVER_H_
#define WEBSOCKET_SERVER_H_

#include <memory>
#include <functional>
#include "backend_decl.h"
#include "error_manager.h"
#include "backend_executor.h"
#include "shared_state.h"

#define MAX_TERMINATE_ATTEMPTS 5

class WebSocketServer : public std::enable_shared_from_this<WebSocketServer>
{
public:
    /// ctor
    WebSocketServer(TCP::socket &&socket, std::shared_ptr<ErrorManagerInterface> &error_man, std::shared_ptr<SharedState> &state) : ws_(std::move(socket)), error_man_(error_man), shared_state_(state) {

                                                                                                                                    };

    /// dtor
    ~WebSocketServer()
    {
        std::cout << "~WebSocketServer()\n";

        // ideally this should be executed only if proper steps to termiante have not been called
        bool terminate = terminateExecutor_();

        if (terminate)
        {
            if (shared_state_->hasConnectedClient())
            {
                shared_state_->removeClient();
            }
        }
    };

    /// @brief the main executor - contains the loop calling async_read/async_write methods
    /// @param  
    void run(HTTP::request<HTTP::string_body>);

private:
    /// @brief read stream buffer
    Beast::multi_buffer read_buffer_;

    /// write buffer
    boost::asio::streambuf write_buffer_;

    /// @brief websocket stream
    WebSocket::stream<Beast::tcp_stream> ws_;

    /// @brief executor for the backend
    BackendExecutor executor_;

    /// @brief error manager
    std::shared_ptr<ErrorManagerInterface> error_man_;

    /// @brief get the shared state of clients
    std::shared_ptr<SharedState> shared_state_;

    /// @brief on handshake success
    /// @param
    void onAccept_(BoostErrorCode);

    /// @brief on reading
    /// @param
    /// @param
    void onRead_(BoostErrorCode, std::size_t);

    /// @brief prepare response
    bool writePrep_(std::string, std::string &);

    /// setup calls
    bool setup_();

    /// method to read
    void read_();

    /// convert to apt format and write to the client
    void write_(std::string);

    /// @brief on write
    /// @param
    /// @param
    void onWrite_(BoostErrorCode, std::size_t);

    /// @brief terminate the execution before shutting down the server
    /// @return
    bool terminateExecutor_();

    /// @brief force termination
    /// @return
    bool forceTerminate_();

};

#endif