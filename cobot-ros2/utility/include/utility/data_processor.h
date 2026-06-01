/**
 * @file data_processor_baseline.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Data processor baseline - keeps methods common to different handlers
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef DATA_PROCESSOR_H_
#define DATA_PROCESSOR_H_

#include <string>
#include <memory>
#include "ros_wrapper_error_codes.h"
#include "data_processor_defs.h"

//  temp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

namespace addverb_cobot
{
    class DataProcessor
    {
    public:
        DataProcessor() = default;

        ~DataProcessor() = default;

        virtual const std::string getName()
        {
            return "DataProcessor";
        }

        /// @brief set the next and previous handler in the chain
        /// @param handler : the handler to add to the chain
        /// @return
        // to do : add code
        virtual bool setHandlers(std::shared_ptr<DataProcessor> &handler);

        /// @brief execute the given request on the given data
        /// @param request : one/more of validation, conversion and communication
        /// @param container : the data on which the operations are to be performed
        /// @return 0 - no error, any other number is error code
        virtual error_codes handleForward(const DataProcessorRequest &request, DataContainer &container);

        /// @brief execute the given request on the given structure in reverse order
        /// @param request : one/more of validation, conversion and communication
        /// @param container : the data on which the operations are to be performed
        /// @return 0 - no error, any other number is error code
        virtual error_codes handleBackward(const DataProcessorRequest &request, DataContainer &container);

        /// @brief enable access to shared pointer
        /// @return
        virtual std::shared_ptr<DataProcessor> shared_from_this() = 0;

    protected:
        /// @brief pointer to the next handler in the chain
        std::shared_ptr<DataProcessor> next_handler_;

        /// @brief pointer to the previous handler in the chain
        std::weak_ptr<DataProcessor> prev_handler_;

    };

};

#endif