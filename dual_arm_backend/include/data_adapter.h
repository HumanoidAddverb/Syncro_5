/**
 * @file data_adapter.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Data Adapter
 * adapter for two-way conversion of data when reading/writing to robot
 *
 * @version 0.1
 * @date 2024-06-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef DATA_ADAPTER_H_
#define DATA_ADAPTER_H_

#include <sstream>
#include <stdexcept>
#include "ui_data_types.h"
#include "struct_mapping.h"

class DataAdapter
{
public:
    /// @brief ctor
    DataAdapter() {};

    /// @brief dtor
    ~DataAdapter() {};

    /// @brief convert from structure to string
    template <typename T>
    bool structToString(const T &obj, std::string &out)
    {
        std::ostringstream ss;
        T temp = obj;

        try
        {
            struct_mapping::map_struct_to_json(temp, ss, " ", false);
            // throw std::runtime_error("Bad structure format");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            T object;
            ss.clear();
            struct_mapping::map_struct_to_json(object, ss, " ", false);
            std::cout << "modified string : " << ss.str() << std::endl;
        }

        out = ss.str();
        return true;
    };

    /// @brief convert from string to structure
    template <typename T>
    bool stringToStruct(const std::string &str, T &out)
    {
        std::stringstream ss(str);

        try
        {
            struct_mapping::map_json_to_struct(out, ss);
            // throw std::runtime_error("Bad string recd");
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
            T object;
            out = object;
            // std::cout << "string that I recd : " << str << std::endl;
            // std::cout << "modified object \n";
            // out.print();
        }

        return true;
    };
};

#endif