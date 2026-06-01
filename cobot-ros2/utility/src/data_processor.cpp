#include "utility/data_processor.h"
#include <iostream>

namespace addverb_cobot
{
    /**
     * @brief set next and previous handlers for the chain
     *
     * @param handler
     * @return true
     * @return false
     */
    bool DataProcessor::setHandlers(std::shared_ptr<DataProcessor> &handler)
    {
        if (handler == nullptr)
        {
            return false;
        }

        next_handler_ = handler;

        next_handler_->prev_handler_ = shared_from_this();

        return true;
    }

    /**
     * @brief handle the request by the next handler in the chain
     *
     * @param request
     * @param container
     * @return int
     */
    error_codes DataProcessor::handleForward(const DataProcessorRequest &request, DataContainer &container)
    {
        if (next_handler_)
        {
            return next_handler_->handleForward(request, container);
        }
        return error_codes::NoError;
    }

    /**
     * @brief handle the request by the previous handler in the chain
     *
     * @param request
     * @param container
     * @return int
     */
    error_codes DataProcessor::handleBackward(const DataProcessorRequest &request, DataContainer &container)
    {
        if (prev_handler_.lock())
        {
            return (prev_handler_.lock())->handleBackward(request, container);
        }

        return error_codes::NoError;
    }

};
