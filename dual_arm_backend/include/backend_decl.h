/**
 * @file backend_decl.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Declarations for the entire Backend code, for ease of use
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef BACKEND_DECL_H_
#define BACKEND_DECL_H_

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

using TCP = boost::asio::ip::tcp;
using BoostErrorCode = boost::system::error_code;

namespace Beast = boost::beast;
namespace HTTP = boost::beast::http;
namespace WebSocket = boost::beast::websocket;

#define HTTP_N_CLIENT 1
#endif