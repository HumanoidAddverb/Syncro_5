/**
 * @file data_parser.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Data Parser implementation -
 * responsible for parsing the received HTTP request into the corresponding C++ structure
 * and parses the ResponseJSON into HTTP Response
 *
 * @version 0.1
 * @date 2024-06-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef DATA_PARSER_H_
#define DATA_PARSER_H_

#include "backend_decl.h"
#include "error_manager.h"
#include "ui_data_types.h"
class DataParser
{
public:
    DataParser(){};

    ~DataParser(){};

    /// @brief setup the parser with the error managrment
    void setup(std::shared_ptr<ErrorManagerInterface> &);

    /// @brief parse request from HTTP request into std::string
    bool parseRequest(HTTP::request<HTTP::string_body> , std::string &);

    /// @brief formulate HTTP response from inout std::string
    bool formulateResponse(std::string, HTTP::response<HTTP::string_body> &);

private:
    /// @brief error manager
    std::shared_ptr<ErrorManagerInterface> error_man_;

    /// @brief request from the client end
    HTTP::request<HTTP::string_body> request_;
};

#endif