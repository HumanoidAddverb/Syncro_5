/**
 * @file http_server.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Server implementation based on HTTP protocol
 * communicates with the front-end to receive user request
 * @version 0.1
 * @date 2024-06-10
 *
 * @copyright Copyright (c) 2024
 *
 *
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include <memory>
#include <functional>
#include <iosfwd>
#include "backend_decl.h"
#include "backend_executor.h"
#include "error_manager.h"
#include "data_parser.h"
#include "websocket_server.h"

/**
 * @brief type of the server for backend
 *
 */
enum class ServerType
{
    eHTTP,
    eWebSocket
};

class HTTPServer : public std::enable_shared_from_this<HTTPServer>
{
public:
    HTTPServer(TCP::socket &socket, std::shared_ptr<SharedState> &state) : socket_(std::move(socket)), shared_state_(state)
    {
        std::cout << "HTTPServer()\n";
    };

    ~HTTPServer()
    {
        std::cout << "~HTTPServer()\n";
    };

    /// @brief configures underlying objects
    /// @return
    bool configure();

    /// @brief run the server
    void run();

private:
    /// @brief Server Type for the backend which by default is WebSocket based Server
    const ServerType server_type = ServerType::eWebSocket;

    /// @brief read stream buffer
    boost::beast::flat_buffer read_buffer_;

    /// @brief HTTP request
    HTTP::request<HTTP::string_body> request_;

    /// @brief HTTP response
    HTTP::response<HTTP::string_body> response_;

    /// @brief socket
    TCP::socket socket_;

    /// @brief executor for the backend
    std::unique_ptr<BackendExecutor> executor_;

    /// @brief parse to and from the request to response structure
    DataParser parser_;

    /// @brief error manager
    std::shared_ptr<ErrorManagerInterface> error_man_;

    /// @brief get the shared state of clients
    std::shared_ptr<SharedState> shared_state_;

    /// @brief method called once asyn_read_until() completes
    /// @param
    /// @param
    void onRead_(BoostErrorCode, std::size_t);

    /// @brief method to call on failure
    void onFail_(const std::string &);

    /// @brief method to call on failure
    /// @param
    /// @param
    void onFail_(const BoostErrorCode &, const std::string &);

    /// @brief sequence of methods to call on reading
    bool writePrep_(std::string, std::string &);

    /// @brief perform the write operation
    /// @param
    void performWrite_();

    /// @brief method to call once write operation has been performed
    /// @param ec
    /// @param
    void onWrite_(const BoostErrorCode &ec, std::size_t);
};

#endif