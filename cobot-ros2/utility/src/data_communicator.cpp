#include "utility/data_communicator.h"

namespace addverb_cobot
{
    /**
     * @brief execute the given request on the given data
     *
     * @param request : one/more of validation, conversion and communication
     * @param container : the data on which the operations are to be performed
     * @return int : 0 - no error, any other number is error code
     */
    error_codes DataCommunicator::handleForward(const DataProcessorRequest &request, DataContainer &container)
    {
        error_codes ret = error_codes::DataNotPresent;

        if (std::find(execution_req_.begin(), execution_req_.end(), request.communicate) == execution_req_.end() && request.communicate != DataCommunicatorRequest::eNone)
        {
            if (!container.convert_data.has_value())
            {
                RCLCPP_INFO(logger_, "DataCommunicator:: Data not present bad logic");

                return error_codes::DataNotPresent;
            }
        }

        switch (request.communicate)
        {
        case DataCommunicatorRequest::eNone:
            ret = error_codes::NoError;
            break;

        case DataCommunicatorRequest::eSafety:
            if (container.convert_data->safety_type.has_value())
            {
                ret = setSafety_(container);
            }
            break;

        case DataCommunicatorRequest::eGripperConfig:
            if (container.convert_data->gripper.has_value())
            {
                ret = setGripperConfig_(container);
            }
            break;

        case DataCommunicatorRequest::eFTSensor:
            if (container.convert_data->ft.has_value())
            {
                ret = setFT_(container);
            }
            break;

        case DataCommunicatorRequest::ePayload:
            if (container.convert_data->payload.has_value())
            {
                ret = setPayload_(container);
            }
            break;

        case DataCommunicatorRequest::eController:
            if (container.convert_data->controller_type.has_value())
            {
                ret = switchController_(container);
            }
            break;

        case DataCommunicatorRequest::ePowerOn:
            ret = powerOn_();
            break;

        case DataCommunicatorRequest::ePowerOff:
            ret = powerOff_();
            break;

        case DataCommunicatorRequest::ePoint:
            if (container.convert_data->controller.has_value())
            {
                ret = setPtPCmd_(container);
            }
            break;

        case DataCommunicatorRequest::eMultiPoint:
            if (container.convert_data->controller.has_value())
            {
                ret = setPtPCmd_(container);
            }
            break;

        case DataCommunicatorRequest::eReplay:
            if (container.convert_data->controller.has_value())
            {
                // have to uncomment this one hw read is implemented
                ret = setReplayCmd_(container);
            }
            break;

        case DataCommunicatorRequest::eTcpMultipoint:
            if (container.convert_data->controller.has_value())
            {
                ret = setTcpCmd_(container);
            }
            break;

        case DataCommunicatorRequest::eFlexPoint:
            if (container.convert_data->controller.has_value())
            {
                ret = setFlexPtPCmd_(container);
            }
            break;

        case DataCommunicatorRequest::eGripperCmd:
            if (container.convert_data->event_config.has_value())
            {
                ret = setGripperCmd_(container);
            }
            break;

        case DataCommunicatorRequest::eJointJogging:
            ret = continousCtrl_(container);
            break;

        case DataCommunicatorRequest::eCartesianJogging:
            ret = continousCtrl_(container);
            break;

        case DataCommunicatorRequest::eJointImpedance:
            ret = continousCtrl_(container);
            break;

        case DataCommunicatorRequest::eCartesianImpedance:
            ret = continousCtrl_(container);
            break;

        case DataCommunicatorRequest::eSetup:
            ret = setup_();
            break;

        case DataCommunicatorRequest::eConnect:
            ret = connect_();
            break;

        case DataCommunicatorRequest::eCheckConnectivity:
            ret = checkConnection_();
            break;

        case DataCommunicatorRequest::eDisconnect:
            ret = disconnect_();
            break;

        case DataCommunicatorRequest::eVelocity:
            if (container.convert_data->interrupt.has_value())
            {
                ret = continousCtrl_(container);
            }
            break;

        case DataCommunicatorRequest::eEffort:
            if (container.convert_data->interrupt.has_value())
            {
                ret = continousCtrl_(container);
            }
            break;
        case DataCommunicatorRequest::eClearErrorState:
            ret = clearErrorState_();
            break;
        case DataCommunicatorRequest::ePowerOnErrRecovery:
            ret = powerOnRobotErrRecovery_();
            break;
        case DataCommunicatorRequest::eErrRecoverySequence:
            ret = executeErrRecovery_();
            break;

        case DataCommunicatorRequest::eShutdownComm:
            ret = shutdownComm_();
            break;

        default:
            ret = error_codes::InvalidRequest;
            break;
        }

        if (ret == error_codes::NoError)
        {
            return DataProcessor::handleForward(request, container);
        }

        return ret;
    }

    /**
     * @brief handle backward
     *
     * @param request
     * @param container
     * @return error_codes
     */
    error_codes DataCommunicator::handleBackward(const DataProcessorRequest &request, DataContainer &container)
    {
        error_codes ret = comm_.read();

        if (ret != error_codes::NoError)
        {
            return ret;
        }

        switch (request.communicate)
        {
        case DataCommunicatorRequest::eReadFeedback:
            ret = getFeedback_(container);
            break;

        case DataCommunicatorRequest::eReadState:
            ret = getState_(container);
            break;

        default:
            ret = error_codes::InvalidRequest;
            break;
        }

        if (ret == error_codes::NoError)
        {
            return DataProcessor::handleBackward(request, container);
        }

        return ret;
    }

    /**
     * @brief allow returning shared pointer to this
     *
     * @return std::shared_ptr<DataProcessorInterface>
     */
    std::shared_ptr<DataProcessor> DataCommunicator::shared_from_this()
    {
        return std::enable_shared_from_this<DataCommunicator>::shared_from_this();
    }

    /**
     * @brief setup communication
     *
     * @return int
     */
    error_codes DataCommunicator::setup_()
    {
        return comm_.setup();
    }

    /**
     * @brief connect with robot
     *
     * @return int
     */
    error_codes DataCommunicator::connect_()
    {
        return comm_.connect();
    }

    /**
     * @brief check connection with robot
     *
     * @return int
     */
    error_codes DataCommunicator::checkConnection_()
    {
        return comm_.checkConnection();
    }

    /**
     * @brief disconnect with robot
     *
     * @return int
     */
    error_codes DataCommunicator::disconnect_()
    {
        return comm_.disconnect();
    }

    /**
     * @brief power on the robot
     *
     * @return int
     */
    error_codes DataCommunicator::powerOn_()
    {
        return comm_.powerOnRobot();
    }

    /// @brief Clear error state
    /// @param
    /// @return int
    error_codes DataCommunicator::clearErrorState_()
    {
        return comm_.clearErrorState();
    }

    /// @brief Power on robot in error recovery mode
    /// @param
    /// @return int
    error_codes DataCommunicator::powerOnRobotErrRecovery_()
    {
        return comm_.powerOnRobotErrorRecovery();
    }

    /// @brief Execute error recovery sequence
    /// @param
    /// @return int
    error_codes DataCommunicator::executeErrRecovery_()
    {
        return comm_.executeErrorRecovery();
    }

    /**
     * @brief shutdown communication with remote
     * 
     * @return error_codes 
     */
    error_codes DataCommunicator::shutdownComm_()
    {
        return comm_.shutdownComm();
    }

    /**
     * @brief power off the robot
     *
     * @return int
     */
    error_codes DataCommunicator::powerOff_()
    {
        std::cout << "[Cobot communicator] : trying to power off" << std::endl;
        return comm_.powerOffRobot();
    }

    /**
     * @brief set the command to go to next target position
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setPtPCmd_(const DataContainer &container)
    {
        return comm_.setCmd(*(container.convert_data->controller));
    }

    /**
     * @brief set the command to go to next target position for tcp multi point
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setTcpCmd_(const DataContainer &container)
    {
        return comm_.setCmd(*(container.convert_data->controller));
    }

    /**
     * @brief set the flex ptp command
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setFlexPtPCmd_(const DataContainer &container)
    {
        return comm_.setCmd(*(container.convert_data->controller));
    }

    /**
     * @brief set the replay cmd to robot
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setReplayCmd_(const DataContainer &container)
    {
        return comm_.setCmd(*(container.convert_data->controller),
                            *(container.convert_data->advanced_config));
    }

    /**
     * @brief set the mode of safety
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setSafety_(const DataContainer &container)
    {
        comm_.setSafety(*(container.convert_data->safety_type));
        return error_codes::NoError;
    }

    /**
     * @brief set the gripper
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setGripper_(const DataContainer &container)
    {
        comm_.setGripper(*(container.convert_data->gripper));
        return error_codes::NoError;
    }

    /**
     * @brief set the FT
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setFT_(const DataContainer &container)
    {
        comm_.setFT(*(container.convert_data->ft));
        return error_codes::NoError;
    }

    /**
     * @brief set the payload
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::setPayload_(const DataContainer &container)
    {
        comm_.setPayload(*(container.convert_data->payload));
        return error_codes::NoError;
    }

    /**
     * @brief Set the gripper configuration to the robot
     *
     * @param container: DataContainer containing the gripper configuration
     * @return int: error code
     *
     * This function sets the gripper configuration to the robot using the CommunicationWrapper object.
     */
    error_codes DataCommunicator::setGripperConfig_(const DataContainer &container)
    {
        comm_.setGripper(*(container.convert_data->gripper));

        return error_codes::NoError;
    }

    /**
     * @brief set the gripper command to robot
     *
     * @param container: DataContainer containing the gripper command
     * @return int: error code
     *
     * This function sets the gripper command to robot using the CommunicationWrapper object.
     */
    error_codes DataCommunicator::setGripperCmd_(const DataContainer &container)
    {
        return comm_.setCmd(*(container.convert_data->event_config));
    }

    /**
     * @brief switch controller
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::switchController_(const DataContainer &container)
    {
        int controller_no = (*(container.convert_data->controller_type));
        return comm_.switchController(*(container.convert_data->controller_type));
    }

    /**
     * @brief run continous control
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::continousCtrl_(const DataContainer &container)
    {
        if (comm_.setCmd(*(container.convert_data->interrupt)) != error_codes::NoError)
        {
            RCLCPP_INFO(logger_, "cobot comm wrapper err");
        }

        return error_codes::NoError;
    }

    /**
     * @brief get robot feedback from comm wrapper
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::getFeedback_(DataContainer &container)
    {
        error_codes ret;

        RobotFeedback robot_feedback;
        ret = comm_.getFeedback(robot_feedback);

        DataCommunicatorContainer communication;
        communication.robot_feedback = robot_feedback;

        container.communication_data = communication;

        return ret;
    }

    /**
     * @brief get robot state from comm wrapper
     *
     * @param container
     * @return int
     */
    error_codes DataCommunicator::getState_(DataContainer &container)
    {
        RobotState robot_state;
        comm_.getState(robot_state);

        DataCommunicatorContainer communication;
        communication.robot_state = robot_state;

        container.communication_data = communication;

        return error_codes::NoError;
    }
};