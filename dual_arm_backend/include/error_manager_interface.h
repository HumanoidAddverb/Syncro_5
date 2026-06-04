/**
 * @file error_manager_interface.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Interface for the error manager
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef ERROR_MANAGER_INTERFACE_H_
#define ERROR_MANAGER_INTERFACE_H_

class ErrorManagerInterface
{
public:
    /// @brief dtor
    inline ~ErrorManagerInterface(){};

    /// @brief update the robot error in the error manager
    /// @param type
    virtual void updateError(const HealErrors::Code &error) = 0;

    /// @brief clear the error
    virtual void clearError() = 0;

    /// @brief get the error code corresponding to the last updated error to the error manager
    /// @return
    virtual int getErrorCode() const = 0;

    /// @brief return if there is any error
    /// @return
    virtual bool isErrorFree() const = 0;

    /// @brief get the error description
    /// @return
    virtual std::string getErrorMsg() = 0;
};

#endif