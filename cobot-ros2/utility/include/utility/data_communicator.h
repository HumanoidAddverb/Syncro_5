/**
 * @file data_communicator.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Data Communicator to the robot ; main functionalities:
 * 1. make/break connection with the robot
 * 2. check connection with the robot
 * 3. read/write data from/to the robot
 *
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef DATA_COMMUNICATOR_H_
#define DATA_COMMUNICATOR_H_

#include <array>
#include "data_processor.h"
#include "cobot_comm_wrapper.h"

namespace addverb_cobot
{
    class DataCommunicator : public DataProcessor, public std::enable_shared_from_this<DataCommunicator>
    {
    public:
        explicit DataCommunicator(const rclcpp::Logger &log, const int index) : logger_(log), comm_(index) {
                                                               };

        ~DataCommunicator() = default;

        const std::string getName() override
        {
            return "DataCommunicator";
        }

        /// @brief execute the given request on the given data
        /// @param request : one/more of validation, conversion and communication
        /// @param container : the data on which the operations are to be performed
        /// @return 0 - no error, any other number is error code
        error_codes handleForward(const DataProcessorRequest &request, DataContainer &container) override;

        /// @brief execute the given request on the given structure in reverse order
        /// @param request : one/more of validation, conversion and communication
        /// @param container : the data on which the operations are to be performed
        /// @return 0 - no error, any other number is error code
        error_codes handleBackward(const DataProcessorRequest &request, DataContainer &container) override;

        /// @brief allow returning shared pointer to this
        /// @return
        std::shared_ptr<DataProcessor> shared_from_this() override;

    private:
        rclcpp::Logger logger_;

        /// @brief the actual communicator object
        CobotCommWrapper comm_;

        /// @brief array of execution requests ; requests that don't need data
        std::array<DataCommunicatorRequest, 10> execution_req_ = {DataCommunicatorRequest::eCheckConnectivity, DataCommunicatorRequest::eConnect, DataCommunicatorRequest::eDisconnect, DataCommunicatorRequest::ePowerOn, DataCommunicatorRequest::ePowerOff, DataCommunicatorRequest::eSetup, DataCommunicatorRequest::eClearErrorState, DataCommunicatorRequest::ePowerOnErrRecovery, DataCommunicatorRequest::eErrRecoverySequence, DataCommunicatorRequest::eShutdownComm};

        /// @brief set safety for robot
        /// @param
        /// @return
        error_codes setSafety_(const DataContainer &);

        /// @brief set gripper config
        /// @param
        /// @return
        error_codes setGripperConfig_(const DataContainer &);

        /// @brief set FT config
        /// @param
        /// @return
        error_codes setFT_(const DataContainer &);

        /// @brief set payload config
        /// @param
        /// @return
        error_codes setPayload_(const DataContainer &);

        /// @brief setup communication
        /// @param
        /// @return
        error_codes setup_();

        /// @brief connect with robot
        /// @param
        /// @return
        error_codes connect_();

        /// @brief connect with robot
        /// @param
        /// @return
        error_codes checkConnection_();

        /// @brief connect with robot
        /// @param
        /// @return
        error_codes disconnect_();

        /// @brief power on the robot
        /// @return
        error_codes powerOn_();

        /// @brief set the next point to point command
        /// @param
        /// @return
        error_codes setPtPCmd_(const DataContainer &);

        /// @brief set gripper 
        /// @param
        /// @return
        error_codes setGripper_(const DataContainer &);

        /// @brief set the next replay command
        /// @param
        /// @return
        error_codes setReplayCmd_(const DataContainer &);

        /// @brief power off the robot
        /// @return
        error_codes powerOff_();

        /// @brief switch controller
        /// @param
        /// @return
        error_codes switchController_(const DataContainer &);

        /// @brief run continous control
        /// @param
        /// @return
        error_codes continousCtrl_(const DataContainer &);

        /// @brief fetch robot joint feedback
        /// @param
        /// @return
        error_codes getFeedback_(DataContainer &);

        /// @brief fetch robot state
        /// @param
        /// @return
        error_codes getState_(DataContainer &);

        /// @brief set the command to go to next target position for tcp multi point
        /// @param container
        /// @return int
        error_codes setTcpCmd_(const DataContainer &);

        /// @brief set the command to go to next target position for flex point
        /// @param container
        /// @return int
        error_codes setFlexPtPCmd_(const DataContainer &);

        /// @brief Clear error state
        /// @param
        /// @return int
        error_codes clearErrorState_();

        /// @brief Power on robot in error recovery mode
        /// @param
        /// @return int
        error_codes powerOnRobotErrRecovery_();

        /// @brief Execute error recovery sequence
        /// @param
        /// @return int
        error_codes executeErrRecovery_();

        /// @brief set gripper command
        /// @param container
        /// @return int
        error_codes setGripperCmd_(const DataContainer &);

        /// @brief shutdown communication with remote
        /// @return 
        error_codes shutdownComm_();
    };

};

#endif