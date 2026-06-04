/**
 * @file error_manager.h
 * @author Siddhi Jain (siddhi.jain@addverb.com) , Aditya Singh (aditya.singh@addverb.com)
 * @brief Implementation of the Error manager for the stack
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef ERROR_MANAGER_H_
#define ERROR_MANAGER_H_

#include <iosfwd>
#include <vector>
#include <mutex>
#include <thread>
#include "error_codes.h"
#include "error_manager_interface.h"
#include <map>

#define DEFAULT_ERROR_MSG "System Internal Error."
#define ERROR_MSG_APPEND_STR " Please click on 'Clear Error' to proceed further."

class ErrorManager : public ErrorManagerInterface
{
public:
    /// @brief ctor
    inline ErrorManager() { setupErrorMap(); };

    /// @brief dtor
    inline ~ErrorManager(){};

    /// @brief update the error in the error manager
    /// @param type
    void updateError(const HealErrors::Code &error);

    /// @brief clear the error
    void clearError();

    /// @brief get the error code corresponding to the last updated error to the error manager
    /// @return
    int getErrorCode() const;

    /// @brief return if there is any error
    /// @return
    bool isErrorFree() const;

    /// @brief get the error description
    /// @return 
    std::string getErrorMsg();

private:    

    /// @brief sets up the map containing error messages for different error codes
    void setupErrorMap();

    /// @brief error code
    int error_{0};

    /// @brief boolean to check for error state
    bool is_at_error_{false};

    /// @brief  map of error codes to error strings
    std::map<HealErrors::Code, std::string> error_map_;
};

#endif /* ERROR_MANAGER_H_ */
