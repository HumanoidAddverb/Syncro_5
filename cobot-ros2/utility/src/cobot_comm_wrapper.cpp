#include "utility/cobot_comm_wrapper.h"

namespace addverb_cobot
{
    /**
     * @brief Set the Safety type
     *
     * @param type
     */
    void CobotCommWrapper::setSafety(const int type)
    {
        safety_type_ = type;
    }

    /**
     * @brief Set the Gripper object
     *
     * @param gripper
     */
    void CobotCommWrapper::setGripper(const GripperConfig &gripper)
    {
        gripper_ = gripper;
    }

    /**
     * @brief set FT config
     *
     * @param ft
     */
    void CobotCommWrapper::setFT(const FTConfig &ft)
    {
        ft_ = ft;
    }

    /**
     * @brief Set the Payload object
     *
     * @param payload
     */
    void CobotCommWrapper::setPayload(const PayloadConfig &payload)
    {
        payload_ = payload;

        if (gripper_)
        {
            payload_->gripper_config = *(gripper_);
        }

        if (ft_)
        {
            payload_->ft_config = *(ft_);
        }
    }

    /**
     * @brief Set the control interrupt command
     *
     * @return int
     */
    error_codes CobotCommWrapper::setCmd(const ControlInterrupt &interrupt)
    {
        interrupt_ = interrupt;
        handler_.setUserInput(interrupt_);

        return write();
    }

    /**
     * @brief Set the controller command
     *
     * @return int
     */
    error_codes CobotCommWrapper::setCmd(const ControllerConfig &config)
    {
        ControllerConfig controller_config = config;
        controller_config.controller = controller_;

        EventConfig event;
        event.event = static_cast<int>(Event::eResetConfig);

        handler_.appendConfig(controller_config);
        handler_.setUserConfig();
        handler_.setEventConfig(event);
        handler_.writeData();

        std::this_thread::sleep_for(std::chrono::nanoseconds(10));

        EventStatus status;
        RobotState state = RobotState::eMotion;
        bool is_connected = true;
        handler_.readData();
        handler_.getEventStatus(status);

        while (status.status != static_cast<int>(EventExecutionStatus::eExecuting) && is_connected)
        {
            handler_.writeData();

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            handler_.readData();
            handler_.getState(state);

            if (state != RobotState::eMotion)
            {
                break;
            }

            handler_.getEventStatus(status);
            is_connected = handler_.isConnected();
        }

        handler_.clearData(UIDataType::eConfigData);
        handler_.clearData(UIDataType::eEventData);
        handler_.writeData();

        if (!is_connected)
        {
            return error_codes::ConnectionBroken;
        }

        if (state == RobotState::eError)
        {
            return error_codes::InErrorState;
        }

        if (state == RobotState::eBase)
        {
            return error_codes::InvalidState;
        }

        return error_codes::NoError;
    }

    /**
     * @brief Set the event configuration command
     *
     * @param event - the event configuration command
     * @return 0 - no error, any other number is error code
     *
     * This function sets the event configuration command for the robot. It first sets the event configuration
     * command and then writes to the robot. After that it waits for the event status to be executing.
     * Once the event status is executing, it clears the data and writes to the robot again.
     */
    error_codes CobotCommWrapper::setCmd(const EventConfig &event)
    {
        handler_.setEventConfig(event);
        handler_.writeData();

        std::this_thread::sleep_for(std::chrono::nanoseconds(10));

        EventStatus status;

        handler_.readData();
        handler_.getEventStatus(status);

        bool is_connected = true;

        // can add timeout condition if we want to
        while (status.status != static_cast<int>(EventExecutionStatus::eExecuting) && is_connected)
        {
            handler_.writeData();

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            handler_.readData();
            handler_.getEventStatus(status);

            is_connected = handler_.isConnected();
        }

        if (!is_connected)
        {
            return error_codes::ConnectionBroken;
        }

        std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        handler_.clearData(UIDataType::eEventData);
        handler_.clearData(UIDataType::eConfigData);

        return write();
    }

    /**
     * @brief Set the controller command for replay controller
     * @param controller
     * @param advanced_config
     * @return int
     */
    error_codes CobotCommWrapper::setCmd(const ControllerConfig &controller,
                                         const AdvancedControllerConfig &advanced_config)
    {

        EventConfig event;
        event.event = static_cast<int>(Event::eResetConfig);

        // append advanced config to play the recorded data
        handler_.appendConfig(advanced_config);
        handler_.appendConfig(*(controller.script_config));
        handler_.appendConfig(controller);

        handler_.setUserConfig();
        handler_.setEventConfig(event);
        handler_.writeData();

        bool is_connected = true;

        EventStatus event_status;
        // add another condition to break
        while (event_status.status != static_cast<int>(EventExecutionStatus::eExecuting) && is_connected)
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
            handler_.readData();
            handler_.getEventStatus(event_status);

            is_connected = handler_.isConnected();
        }

        if (!is_connected)
        {
            return error_codes::ConnectionBroken;
        }

        std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        handler_.clearData(UIDataType::eEventData);
        handler_.clearData(UIDataType::eConfigData);
        handler_.writeData();

        return write();
    }

    /**
     * @brief setup data handler
     * @return int
     */
    error_codes CobotCommWrapper::setup()
    {
        if (!handler_.setup())
        {
            return error_codes::RobotFailedToSetup;
        }

        return error_codes::NoError;
    }

    /**
     * @brief get robot joint feedback
     * @param robot_feedback
     * @return int
     */
    error_codes CobotCommWrapper::getFeedback(RobotFeedback &robot_feedback)
    {
        handler_.getRobotFeedback(robot_feedback);

        return error_codes::NoError;
    }

    /**
     * @brief get robot joint state (base/motion/error)
     * @param robot_state
     * @return int
     */
    error_codes CobotCommWrapper::getState(RobotState &robot_state)
    {
        handler_.getState(robot_state);

        return error_codes::NoError;
    }

    /**
     * @brief Clear error state
     *
     * @return int
     */
    error_codes CobotCommWrapper::clearErrorState()
    {
        RobotState state = RobotState::eError;
        bool is_connected = handler_.isConnected();

        while ((state == RobotState::eError) && is_connected)
        {
            handler_.setAction(RobotAction::eGoToBaseState);
            handler_.writeData();

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            handler_.readData();
            handler_.getState(state);
            is_connected = handler_.isConnected();
        }

        if (!is_connected)
        {
            return error_codes::ConnectionBroken;
        }

        if (state == RobotState::eMotion)
        {
            return error_codes::FailedToClearError;
        }

        if (state == RobotState::eBase)
        {
            handler_.clearData(UIDataType::eConfigData);
            handler_.writeData();
            return error_codes::NoError;
        }

        return error_codes::NoError;
    }

    /**
     * @brief Power on robot in error recovery mode
     *
     * @return int
     */
    error_codes CobotCommWrapper::powerOnRobotErrorRecovery()
    {
        safety_type_ = SAFETY_MODE_ERR_RECOVERY;

        if (powerOnRobot() != error_codes::NoError)
        {
            return error_codes::RobotFailedToPowerOn;
        }

        return error_codes::NoError;
    }

    /**
     * @brief Execute error recovery command
     *
     * @return int
     */
    error_codes CobotCommWrapper::executeErrorRecovery()
    {
        RobotState state = RobotState::eMotion;
        bool is_connected = handler_.isConnected();

        // cusom config to reset to home
        ControllerConfig cconf;

        cconf.controller = static_cast<int>(API::eLinearVelocityAPI);
        cconf.target_pos = std::vector<double>(6, 0);

        // todo - must be dynamic to account for distance from home
        cconf.target_time = 15;

        EventConfig event;
        event.event = static_cast<int>(Event::eResetConfig);

        handler_.appendConfig(cconf);
        handler_.setUserConfig();
        handler_.setEventConfig(event);
        handler_.writeData();

        std::this_thread::sleep_for(std::chrono::nanoseconds(10));

        EventStatus status;
        handler_.readData();
        handler_.getEventStatus(status);

        // can add timeout condition if we want to

        while (status.status != static_cast<int>(EventExecutionStatus::eExecuting) && (state == RobotState::eMotion))
        {
            handler_.writeData();

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            handler_.readData();

            handler_.getState(state);

            handler_.getEventStatus(status);

            is_connected = handler_.isConnected();

            if (!is_connected)
            {
                return error_codes::ConnectionBroken;
            }
        }

        if (state != RobotState::eMotion)
        {
            return error_codes::InErrorState;
        }

        handler_.clearData(UIDataType::eConfigData);
        handler_.clearData(UIDataType::eEventData);
        handler_.writeData();

        while (status.status == static_cast<int>(EventExecutionStatus::eExecuting) && (state == RobotState::eMotion))
        {

            handler_.writeData();
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
            handler_.readData();

            handler_.getState(state);

            handler_.getEventStatus(status);
            is_connected = handler_.isConnected();

            if (!is_connected)
            {
                return error_codes::ConnectionBroken;
            }
        }

        if (state != RobotState::eMotion)
        {
            return error_codes::InErrorState;
        }

        if (status.status == static_cast<int>(EventExecutionStatus::eSucceded))
        {
            std::cout << "Robot reset to home position" << std::endl;
        }

        while (state == RobotState::eMotion)
        {
            handler_.setAction(RobotAction::eGoToBaseState);
            handler_.writeData();
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            handler_.readData();

            handler_.getState(state);
            is_connected = handler_.isConnected();

            if (!is_connected)
            {
                return error_codes::ConnectionBroken;
            }
        }

        if (state == RobotState::eError)
        {
            std::cout << "error recovery -  in error state\n";
            return error_codes::InErrorState;
        }

        return error_codes::NoError;
    }

    /**
     * @brief way to power on robot
     *
     * @return int
     */
    error_codes CobotCommWrapper::powerOnRobot()
    {
        // todo : might have to add a check to see if robot is not already in motion state or error state, basicaly check that robot has to be in base state

        ControllerConfig config;

        config.controller = static_cast<int>(API::eExternalVelocityAPI);
        config.safety_mode = safety_type_;
        // std::cout<<"[CobotCommWrapper::powerOnRobot] : safety type "<<safety_type_<<std::endl<<"\n";

        if (payload_)
        {

            handler_.appendConfig(*payload_);
        }

        handler_.appendConfig(config);
        handler_.setUserConfig();
        handler_.setAction(RobotAction::eGoToMotionState);
        handler_.writeData();

        RobotState state = RobotState::eBase;
        bool is_connected = handler_.isConnected();

        // can add timeout condition if we want to
        while ((state == RobotState::eBase) && is_connected)
        {
            handler_.setAction(RobotAction::eGoToMotionState);
            handler_.writeData();
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
            handler_.readData();
            handler_.getState(state);

            is_connected = handler_.isConnected();
        }

        if (!is_connected)
        {
            std::cout << "Lost connection with robot\n";
            return error_codes::ConnectionBroken;
        }

        if (state == RobotState::eError)
        {
            std::cout << "getting error state\n";
            return error_codes::RobotFailedToPowerOn;
        }

        if (state == RobotState::eMotion)
        {
            handler_.clearData(UIDataType::eConfigData);
            handler_.writeData();

            return error_codes::NoError;
        }

        // no default return as there must be no logic that reaches here
    }

    /**
     * @brief way to power off robot
     *
     * @return int
     */
    error_codes CobotCommWrapper::powerOffRobot()
    {
        RobotState state = RobotState::eMotion;
        bool is_connected = handler_.isConnected();

        while ((state == RobotState::eMotion) && is_connected)
        {
            handler_.setAction(RobotAction::eGoToBaseState);
            handler_.writeData();
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            if (!handler_.readData())
            {
                // handler_.shutdown();
                // return error_codes::FailedToReadData;
                return error_codes::NoError;
            }

            handler_.getState(state);
            is_connected = handler_.isConnected();
        }

        if (!is_connected)
        {
            return error_codes::NoError;

            // handler_.shutdown();
            // return error_codes::ConnectionBroken;
        }

        if (state == RobotState::eError)
        {
            return error_codes::NoError;

            // handler_.shutdown();
            // return error_codes::InErrorState;
        }

        // handler_.shutdown();
        return error_codes::NoError;
    }

    /**
     * @brief shutdown communication with remote
     *
     * @return error_codes
     */
    error_codes CobotCommWrapper::shutdownComm()
    {
        if (handler_.shutdown())
        {
            return error_codes::NoError;
        }

        return error_codes::NoError;

        // this error code is random, just need to tell there is some error
        // return error_codes::ConnectionBroken;
    }

    /**
     * @brief establish connection with the robot
     *
     * @return int
     */
    error_codes CobotCommWrapper::connect()
    {
        if (handler_.connect())
        {
            return error_codes::NoError;
        }

        return error_codes::FailedToConnect;
    }

    /**
     * @brief check if the peer is connected
     *
     * @return int
     */
    error_codes CobotCommWrapper::checkConnection()
    {
        error_codes ret = error_codes::NoError;

        if (!handler_.isConnected())
        {
            ret = error_codes::ConnectionBroken;
        }

        return ret;
    }

    /**
     * @brief disconnect with the robot
     *
     * @return int
     */
    error_codes CobotCommWrapper::disconnect()
    {
        error_codes ret = error_codes::NoError;
        if (!handler_.disconnect())
        {
            ret = error_codes::FailedToDisconnnect;
        }

        return ret;
    }

    /**
     * @brief switch controller to given type
     *
     * @param controller
     */
    error_codes CobotCommWrapper::switchController(const int controller)
    {
        controller_ = controller;

        API ctrl = static_cast<API>(controller_);

        if (std::find(critical_controllers_api_.begin(), critical_controllers_api_.end(), ctrl) != critical_controllers_api_.end())
        {
            return switchController(controller, interrupt_);
        }

        if (std::find(continous_ctrl_api_.begin(), continous_ctrl_api_.end(), ctrl) != continous_ctrl_api_.end())
        {
            // std::cout << "got continuous controller  " << controller_ << std::endl;
            ControllerConfig conf;
            conf.controller = controller_;

            EventConfig event;
            event.event = static_cast<int>(Event::eResetConfig);

            handler_.appendConfig(conf);
            handler_.setUserConfig();
            handler_.setEventConfig(event);
            handler_.writeData();

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            RobotState robot_state = RobotState::eMotion;

            EventStatus status;
            handler_.readData();
            handler_.getEventStatus(status);

            // add another condition to break
            while ((status.status != static_cast<int>(EventExecutionStatus::eExecuting)) && (robot_state == RobotState::eMotion))
            {
                handler_.writeData();

                std::this_thread::sleep_for(std::chrono::nanoseconds(10));

                handler_.readData();
                handler_.getState(robot_state);
                handler_.getEventStatus(status);
                // std::cout << "in the while for event status executing" << std::endl;
            }

            // std::cout << "got event status executing" << std::endl;

            handler_.clearData(UIDataType::eEventData);
            handler_.clearData(UIDataType::eConfigData);
            handler_.writeData();

            if (robot_state != RobotState::eMotion)
            {
                return error_codes::InErrorState;
            }

            // add another condition to break
            while ((status.status != static_cast<int>(EventExecutionStatus::eSucceded)) && (robot_state == RobotState::eMotion))
            {
                handler_.writeData();

                std::this_thread::sleep_for(std::chrono::nanoseconds(10));

                handler_.readData();
                handler_.getState(robot_state);
                handler_.getEventStatus(status);

                // std::cout << "in the while for event status succeded" << std::endl;
            }

            if (robot_state != RobotState::eMotion)
            {
                return error_codes::InErrorState;
            }
            // std::cout << "got event status succeded" << std::endl;
        }

        return error_codes::NoError;
    }

    /**
     * @brief switch controller to given type with interrupt
     *
     * @param controller the type of the controller to switch to
     * @param interrupt the interrupt to send to the controller
     *
     * @return error_codes::NoError if the switch is successful, error_codes::FailedToConnect otherwise
     */
    error_codes CobotCommWrapper::switchController(const int controller, const ControlInterrupt &interrupt)
    {
        controller_ = controller;

        API ctrl = static_cast<API>(controller_);

        if (std::find(continous_ctrl_api_.begin(), continous_ctrl_api_.end(), ctrl) != continous_ctrl_api_.end())
        {
            // std::cout << "got continuous controller  " << controller_ << std::endl;
            ControllerConfig conf;
            conf.controller = controller_;

            EventConfig event;
            event.event = static_cast<int>(Event::eResetConfig);

            handler_.appendConfig(conf);
            handler_.setUserConfig();
            handler_.setEventConfig(event);
            handler_.writeData();

            handler_.setUserInput(interrupt);

            std::this_thread::sleep_for(std::chrono::nanoseconds(10));

            RobotState robot_state = RobotState::eMotion;

            EventStatus status;
            handler_.readData();
            handler_.getEventStatus(status);

            // add another condition to break
            while ((status.status != static_cast<int>(EventExecutionStatus::eExecuting)) && (robot_state == RobotState::eMotion))
            {
                handler_.writeData();

                handler_.setUserInput(interrupt);

                std::this_thread::sleep_for(std::chrono::nanoseconds(10));

                handler_.readData();
                handler_.getState(robot_state);
                handler_.getEventStatus(status);
                // std::cout << "in the while for event status executing" << std::endl;
            }

            // std::cout << "got event status executing" << std::endl;

            handler_.clearData(UIDataType::eEventData);
            handler_.clearData(UIDataType::eConfigData);
            handler_.writeData();

            if (robot_state != RobotState::eMotion)
            {
                return error_codes::InErrorState;
            }

            // add another condition to break
            while ((status.status != static_cast<int>(EventExecutionStatus::eSucceded)) && (robot_state == RobotState::eMotion))
            {
                handler_.writeData();
                handler_.setUserInput(interrupt);

                std::this_thread::sleep_for(std::chrono::nanoseconds(10));

                handler_.readData();
                handler_.getState(robot_state);
                handler_.getEventStatus(status);

                // std::cout << "in the while for event status succeded" << std::endl;
            }

            if (robot_state != RobotState::eMotion)
            {
                return error_codes::InErrorState;
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief read robot state
     *
     * @return int
     */
    error_codes CobotCommWrapper::read()
    {
        if (!handler_.readData())
        {
            return error_codes::FailedToReadData;
        }

        return error_codes::NoError;
    }

    /**
     * @brief write last given command to robot
     *
     * @return int
     */
    error_codes CobotCommWrapper::write()
    {
        error_codes ret = error_codes::NoError;
        if (!handler_.writeData())
        {
            ret = error_codes::FailedToWriteData;
        }

        return ret;
    }

};