#include "addverb_cobot_hardware/cobot_hw_interface.h"

namespace addverb_cobot
{
    /**
     * @brief initialise the resources
     *
     * @param info
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn CobotHWInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        // info is inherted from SystemInterface class
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        num_grippers_ = std::count_if(info_.joints.begin(), info_.joints.end(), [](const hardware_interface::ComponentInfo& joint){
            return joint.name.find("gripper_finger_joint") != std::string::npos;
        });

        num_joint_dof_ = (int)info_.joints.size() - num_grippers_;
        
        if ((int)num_joint_dof_ % n_dof != 0) //! Assumption is there are multiple of 6 joints in the hardware change. can be read from a file if its different.
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"),
                         "Number of joint specified are %zu. %d expected", num_joint_dof_, n_dof);
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "URDF parsed successfully. Found %zu joints.", info.joints.size());

        initializeMapping();
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Created mapping for robots");

        joint_impedance_cmd_.init(n_dof);
        joint_jogging_cmd_.init(n_dof);
        setControlModeMap_();

        if (!setupDataProcessor_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"),
                         "Failed to setup data processor");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!validateSafety_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid safety specified");

            return hardware_interface::CallbackReturn::ERROR;
        }
    
        if (!setSafety_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to upload safety configuration to robot.");

            return hardware_interface::CallbackReturn::FAILURE;
        }

        if (!checkPayload())
        {

            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to check payload for the robot.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // validate the number of command and state joint interfaces
        if (!(validateCommandInterface_() && validateStateInterface_()))
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to validate command interface and state interface for the robot.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!setupComm_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to setup Communication with robot");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!setupServices_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to setup services ...");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "system initialised successfully.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief configure hw interface
     *
     * @param info
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn CobotHWInterface::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "on_configure begin");

        initialise_();

        // setup connection with robot
        if (!connect_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to setup Connection with robot. Re-attempt after making sure server is up and running and there are no loose connections.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Connected with robot");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief power on the robot
     *
     * @param previous_state
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn CobotHWInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // check connection with robot
        if (!checkConnection_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to communicate with robot ...");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // power on robot
        if (!powerOnRobot_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to power on robot ...");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot powered on in default mode");

        for (const auto &[robot_index, api] : ros_controller_map_)
        {
            if (api.controller == API::eNone)
            {
                switchControlLoop_(API::eExternalVelocityAPI, robot_index);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot control mode set to user defined mode");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief power off the robot
     *
     * @param previous_state
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn CobotHWInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        if (!shutdown_pre_processed_)
        {
            if (!shutdownRobot_())
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Failed to power off robot ...");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief power off the robot
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::shutdownRobot_()
    {
        // debug line
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "[CobotHWInterface::shutdownRobot_]  Shut down robot function called...");
        if (checkConnection_())
        {

            for (const auto &[_, index] : arm_index_)
            {
                // debug line
                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "[CobotHWInterface::shutdownRobot_]  Check connection successful");
                if (!powerOffRobot_(index))
                {
                    RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Failed to power off robot");
                    return false;
                }

                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot power off success");
                // return true;
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot already disconnected");
            return true;
        }
        return true;
    }

    /**
     * @brief close connection with the robot and deallocate resources if any
     *
     * @param previous_state
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn CobotHWInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        // setup connection with robot
        if (!checkConnection_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to close connection with the robot");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Clean up successful");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief do this on error
     *
     * @return hardware_interface::CallbackReturn
     */
    hardware_interface::CallbackReturn CobotHWInterface::on_error(const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("CobotHWInterface"),
            "Robot fails to recover from error. Please shutdown the system manually and restart.");

        // std::cout << "previous state : " << prev_state.label() << std::endl;

        // const std::string previous_state = prev_state.label();

        // if(previous_state == "unconfigured")

        return hardware_interface::CallbackReturn::ERROR;
    }

    /**
     * @brief register the state interfaces
     *
     * @return std::vector<hardware_interface::CommandInterface>
     */
    std::vector<hardware_interface::StateInterface>
    CobotHWInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        initStateVar_();
        // register minimum joints states
        /// For gripper there is separate register of interface 
        size_t cmd_idx = 0;
        size_t gripper_idx = 0;

        for (const auto& joint : info_.joints)
        {
            if (joint.name.find("gripper") != std::string::npos)
            {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                        joint.name,
                        hardware_interface::HW_IF_POSITION,
                        &gripper_position_state_[gripper_idx]));

                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                        joint.name,
                        hardware_interface::HW_IF_VELOCITY,
                        &gripper_velocity_state_[gripper_idx]));

                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                        joint.name,
                        hardware_interface::HW_IF_EFFORT,
                        &gripper_effort_state_[gripper_idx]));

                ++gripper_idx;
                continue;
            }


            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name,
                    hardware_interface::HW_IF_POSITION,
                    &position_state_[cmd_idx]));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name,
                    hardware_interface::HW_IF_VELOCITY,
                    &velocity_state_[cmd_idx]));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name,
                    hardware_interface::HW_IF_EFFORT,
                    &effort_state_[cmd_idx]));

            ++cmd_idx;
        }

        createGPIOStateInterfaces(state_interfaces);

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Exported state interfaces");

        return state_interfaces;
    }

    /**
     * @brief register the command interfaces
     *
     * @return std::vector<hardware_interface::CommandInterface>
     */
    std::vector<hardware_interface::CommandInterface>
    CobotHWInterface::export_command_interfaces()
    {
        initCmdVar_();

        // register position, velocity and effort commands
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        size_t cmd_idx = 0;
        /// For gripper there is no command interface. Command interface is through GPIO
        for (const auto& joint : info_.joints)
        {
            if (joint.name.find("gripper") != std::string::npos)
            {
                continue;
            }

            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint.name,
                    hardware_interface::HW_IF_POSITION,
                    &position_cmd_[cmd_idx]));

            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint.name,
                    hardware_interface::HW_IF_VELOCITY,
                    &velocity_cmd_[cmd_idx]));

            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint.name,
                    hardware_interface::HW_IF_EFFORT,
                    &effort_cmd_[cmd_idx]));

            ++cmd_idx;
        }
        createGPIOCommandInterfaces(command_interfaces);

        RCLCPP_INFO(rclcpp::get_logger("CobotHardwareInterface"), "Exported command interfaces");

        return command_interfaces;
    }

    /**
     * @brief read impl
     *
     * @param time
     * @param period
     * @return hardware_interface::return_type
     */
    hardware_interface::return_type CobotHWInterface::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // if in error, avoid acquiring data handler
        if (in_error_.load())
        {
            
            for (const auto &[name, index] : arm_index_)
            {

                robot_state_data_map_.at(index).robot_status_ = 3; //!TODO: Make an Enum
            }
            return hardware_interface::return_type::OK;

        }

        if (!shutdown_request_accepted_)
        {
            // check connection with robot
            if (!checkConnection_())
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Lost connection with the robot. Kindly ensure the robot is powered on, server is running on robot end and the wires are not loose.");
                return hardware_interface::return_type::ERROR;
            }

            if (!getFeedback_())
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Get feedback error");
                return hardware_interface::return_type::ERROR;
            }

            if (!getRobotState_())
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Get robot state error");
                return hardware_interface::return_type::ERROR;
            }

            checkForError_();
        }

        return hardware_interface::return_type::OK;
    }

    /**
     * @brief write impl
     *
     * @param time
     * @param period
     * @return hardware_interface::return_type
     */
    hardware_interface::return_type CobotHWInterface::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        updateCommandsMappingFromHWInterface();

        // Look for shutdown request
        shutdown_requested_ = cobot_services_node_->get_parameter("shutdown_requested").as_bool();

        // if the system is in error, don't send any control commands
        if (in_error_.load())
        {
            // RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "[HW Interface code] : Robot in error");
            if (shutdown_requested_)
            {
                // If robot is in error state, reject shutdown
                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Shutdown request rejected");
                cobot_services_node_->set_parameter(rclcpp::Parameter("shutdown_request_rejected", true));
                // shutdown_requested_ = false;
                return hardware_interface::return_type::OK;
            }

            error_recovery_requested_ = cobot_services_node_->get_parameter("error_recovery_requested").as_bool();

            if (error_recovery_requested_)
            {
                if (error_recovery_request_accepted_)
                {
                    if (error_recovery_future_.wait_for(std::chrono::milliseconds(addverb_cobot::future_wait_time)) != std::future_status::ready)
                    {
                        return hardware_interface::return_type::OK;
                    }
                    else
                    {
                        if (error_recovery_future_.get() == true)
                        {
                            cobot_services_node_->set_parameter(rclcpp::Parameter("error_recovery_success", true));

                            error_recovery_requested_ = false;
                            error_recovery_request_accepted_ = false;

                            while (in_error_.load())
                            {
                                in_error_.store(false);
                                std::this_thread::sleep_for(std::chrono::nanoseconds(10));
                            }

                            error_recovery_future_ = std::future<bool>();

                            return hardware_interface::return_type::OK;
                        }
                        else
                        {
                            cobot_services_node_->set_parameter(rclcpp::Parameter("error_recovery_failure", true));
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                            return hardware_interface::return_type::ERROR;
                        }
                    }
                }
                else
                {
                    cobot_services_node_->set_parameter(rclcpp::Parameter("error_recovery_request_accepted", true));
                    error_recovery_request_accepted_ = true;
                    error_recovery_future_ = std::async(std::launch::async, &CobotHWInterface::errorRecovery_, this);
                    return hardware_interface::return_type::OK;
                }
            }
            else
            {
                return hardware_interface::return_type::OK;
            }
        }

        // shutdown block
        if (shutdown_requested_)
        {
            if (shutdown_request_accepted_)
            {
                if (shutdown_pre_processed_)
                {
                    RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "shutdown pre proc var set to true");
                    return hardware_interface::return_type::OK;
                }
                else if (shutdown_future_.wait_for(std::chrono::milliseconds(addverb_cobot::future_wait_time)) != std::future_status::ready)
                {
                    return hardware_interface::return_type::OK;
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "future is ready");

                    // std::cout<<"apparently the future is ready\n";
                    if (shutdown_future_.get() == true)
                    {
                        cobot_services_node_->set_parameter(rclcpp::Parameter("shutdown_pre_processed", true));
                        shutdown_pre_processed_ = true;

                        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "shutdown pre processed is true");

                        return hardware_interface::return_type::OK;
                    }
                    else
                    {
                        return hardware_interface::return_type::ERROR;
                    }
                }
            }
            else
            {
                // debug line
                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "[CobotHWInterface::write] Shut down request accepted ...");
                cobot_services_node_->set_parameter(rclcpp::Parameter("shutdown_request_accepted", true));
                shutdown_request_accepted_ = true;
                shutdown_future_ = std::async(std::launch::async, &CobotHWInterface::shutdownRobot_, this);
                return hardware_interface::return_type::OK;
            }
        }

        // check connection with robot
        if (!checkConnection_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Lost connection with the robot. Kindly ensure the robot is powered on, server is running on robot end and the wires are not loose.");
            return hardware_interface::return_type::ERROR;
        }

        if (!runGripper_())
        {
            return hardware_interface::return_type::ERROR;
        }

        for (const auto &[name, robot_index] : arm_index_)
        {
            if (!control_loop_map_.at(robot_index)())
            {
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    /**
     * @brief check for any errors on the robot
     *
     */
    void CobotHWInterface::checkForError_()
    {
        for (const auto &[name, robot_index] : arm_index_)
        {
            if (robot_state_data_map_.at(robot_index).robot_status_ == 1.0 * static_cast<int>(RobotState::eError))
            {
                if (in_error_.load() == false)
                {
                    in_error_.store(true);
                    RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"),
                                 "Robot gone into error. Call ErrorRecoveryService to safely recover from the error and get into the home position.");
                }
            }
        }
    }

    /**
     * @brief Recover from error
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::errorRecovery_()
    {
        // try to clear error state

        for (const auto &[_, robot_index] : arm_index_)
        {
            if (!clearErrorState_(robot_index))
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Failed to clear the error state of the robot for the robot, %s", arm_names_.at(robot_index).c_str());
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Clear error state done for the robot %s", arm_names_.at(robot_index).c_str());

             // power on robot in error recovery mode
            if (!powerOnRobotErrRecovery_(robot_index))
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Failed to power on robot in error recovery mode for the robot, %s", arm_names_.at(robot_index).c_str());
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot powered on in error recovery mode for the robot %s", arm_names_.at(robot_index).c_str());


             // execute error recovery sequence
            if (!executeErrRecovery_(robot_index))
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Failed to execute automatic error recovery sequence on the robot. Please attempt manual error recovery.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Error recovery sequence executed for the robot %s", arm_names_.at(robot_index).c_str());

            // Reset safety
            if (!setSafety_(robot_index))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to power off robot and exit from error recovery mode for the robot, %s", arm_names_.at(robot_index).c_str());
                return false;
            }
            std::this_thread::sleep_for(std::chrono::seconds(error_recovery_sleep_time_));

        }
        
       
        // restart the robot in normal OP mode
        if (!powerOnRobot_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to restart the robots in normal OP mode.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robots powered on in normal OP mode");

        // reset controller to user-defined controller
        auto success = true;
        for (const auto &[_, robot_index] : arm_index_)
        {
            success &= switchController_(robot_index);
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robots control mode set to last commanded control mode");

        return success;
    }

    /**
     * @brief setup the data processor pipeline
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setupDataProcessor_()
    {

        createCommunicationMapping();

        data_validator_ = std::make_shared<DataValidator>(rclcpp::get_logger("CobotHWInterface"));

        for (const auto &[_, index] : arm_index_)
        {
            if (!data_processor_map_[index]->setHandlers(data_communicator_map_[index]))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"),
                             "converter could not set handle");
                return false;
            }
        }

        return true;
    }

    /**
     * @brief setup communication with the robot
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setupComm_()
    {
        bool successful = true;

        for (const auto &[_, index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::eSetup;
            successful &= handleFwdRequest_(req, cont, index);
        }

        return successful;
    }

    /**
     * @brief initialise the commands and state
     *
     */
    void CobotHWInterface::initialise_()
    {
        // initialise commands to zero
        initStateVar_();
        initCmdVar_();
    }

    /**
     * @brief setup connection
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::connect_()
    {
        std::vector<std::future<bool>> futures;

        for (const auto &[name, index] : arm_index_)
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            futures.push_back(std::async(std::launch::async, [this, name, index]()
                                         {

                DataProcessorRequest req;
                DataContainer cont;

                req.communicate = DataCommunicatorRequest::eConnect;

                int count = 0;
                const int max_count = max_reattempt_connection_count;

                do
                {
                    if (handleFwdRequest_(req, cont, index))
                    {
                        return true;
                    }

                    count++;

                    RCLCPP_WARN(
                        rclcpp::get_logger("CobotHardwareInterface"),
                        "Failed to connect with robot [%s], retrying...",
                        name.c_str());

                    std::this_thread::sleep_for(std::chrono::seconds(2));

                } while (count <= max_count && rclcpp::ok());

                return false; }));
        }

        bool successful = true;

        for (auto &f : futures)
        {
            successful &= f.get();
        }

        return successful;
    }

    /**
     * @brief close connection with the robot
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::disconnect_()
    {

        bool successful = true;

        for (const auto &[_, index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::eDisconnect;
            successful &= handleFwdRequest_(req, cont, index);
        }
        return successful;
    }

    /**
     * @brief change the state of robot from error
     * @return true
     * @return false
     */
    bool CobotHWInterface::clearErrorState_()
    {
        bool successful = true;
        // std::this_thread::sleep_for(std::chrono::seconds(20));

        // auto x = getRobotState_();

        for (const auto &[_, index] : arm_index_)
        {

            if(robot_state_data_map_.at(index).robot_status_ != 1.0 * static_cast<int>(RobotState::eError))
            {
                RCLCPP_WARN(rclcpp::get_logger("CobotHWInterface"), "Robot not in error state, not clearing error for this robot, %s", arm_names_.at(index).c_str());
                successful &= true;
                continue;
            }
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::eClearErrorState;
            successful &= handleFwdRequest_(req, cont, index);
        }
        return successful;
    }

    /**
     * @brief change the state of robot from error
     * @return true
     * @return false
     */
    bool CobotHWInterface::clearErrorState_(const int index)
    {
        bool successful = true;
       
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::eClearErrorState;
        successful = handleFwdRequest_(req, cont, index);
        return successful;
    }

    /**
     * @brief power on robot
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOnRobot_()
    {
        bool successful = true;

        for (const auto &[_, index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::ePowerOn;
            successful &=  handleFwdRequest_(req, cont, index);
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return successful;
    }

    /**
     * @brief power on robot in errror recovery mode
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOnRobotErrRecovery_()
    {
        bool successful = true;

        for (const auto &[_, index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::ePowerOnErrRecovery;
            successful &= handleFwdRequest_(req, cont, index);
        }
        return successful;
    }

    /**
     * @brief power on robot in errror recovery mode
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOnRobotErrRecovery_(const int robot_index)
    {
        bool successful = true;

        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::ePowerOnErrRecovery;
        successful = handleFwdRequest_(req, cont, robot_index);
    
        return successful;
    }

    /**
     * @brief execute errror recovery sequence
     * @return true
     * @return false
     */
    bool CobotHWInterface::executeErrRecovery_()
    {
        auto success = true;
        for (const auto &[_, robot_index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            // @aravindh - write this impl of eErrRecoverySequence
            req.communicate = DataCommunicatorRequest::eErrRecoverySequence;

            /// reset necessary variables
            robot_effort_controller_status_map_.at(robot_index) = addverb_cobot::effortControllerStatus::eInactive;
            // effortcmd_.fill(0.0);

            recorder_state_.at(robot_index) = 0;

            success &= handleFwdRequest_(req, cont, robot_index);
        }

        return success;
    }

        /**
     * @brief execute errror recovery sequence
     * @return true
     * @return false
     */
    bool CobotHWInterface::executeErrRecovery_(const int robot_index)
    {
        auto success = true;
        
        DataProcessorRequest req;
        DataContainer cont;

        // @aravindh - write this impl of eErrRecoverySequence
        req.communicate = DataCommunicatorRequest::eErrRecoverySequence;

        /// reset necessary variables
        robot_effort_controller_status_map_.at(robot_index) = addverb_cobot::effortControllerStatus::eInactive;
        // effortcmd_.fill(0.0);

        recorder_state_.at(robot_index) = 0;

        success = handleFwdRequest_(req, cont, robot_index);
        

        return success;
    }

    /**
     * @brief power off robot
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOffRobot_()
    {
        bool successful_poweroff = true;

        for (const auto &[_, index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::ePowerOff;
            successful_poweroff &= handleFwdRequest_(req, cont, index);
            
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        bool success_shutdown = true;

        for (const auto &[_, index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::eShutdownComm;
            success_shutdown &= handleFwdRequest_(req, cont, index);
            
        }

        return successful_poweroff & success_shutdown;
    }


        /**
     * @brief power off robot
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOffRobot_(const int index)
    {
        bool successful_poweroff = true;

        
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::ePowerOff;
            successful_poweroff &= handleFwdRequest_(req, cont, index);
            
        
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        bool success_shutdown = true;

            req.communicate = DataCommunicatorRequest::eShutdownComm;
            success_shutdown &= handleFwdRequest_(req, cont, index);
            
        

        return successful_poweroff & success_shutdown;
    }

    /**
     * @brief check connection status with robot after
     * connection request
     * @return true
     * @return false
     */
    bool CobotHWInterface::checkConnection_()
    {
        bool successful = true;

        for (const auto &[_, index] : arm_index_)
        {
            DataProcessorRequest req;
            DataContainer cont;

            req.communicate = DataCommunicatorRequest::eCheckConnectivity;
            successful &= handleFwdRequest_(req, cont, index);
        }
        return successful;
    }

    /**
     * @brief validate the gripper configuration
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateGripper_(const int robot_index)
    {
        gripper_config_.gripper_type = gripper_type_map_[robot_index];

        error_codes validation_result = data_validator_->validateRequest(gripper_config_);
        if (validation_result != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid gripper configuration specified for robot name %s", arm_names_.at(robot_index).c_str());
            return false;
        };

        return true;
    }

    /**
     * @brief validate the FT configuration
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateFT_(const int robot_index)
    {
        int ft_type;
        ft_type = ft_type_map_[robot_index];

        // RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"),"size of ft_rot_mat : %dx%d",ft_rot.size(),ft_rot[2].size());

        ft_config_.ft_type = ft_type;
        ft_config_.rot = ft_rotation_matrix_map_[robot_index];

        error_codes validation_result = data_validator_->validateRequest(ft_config_);
        if (validation_result != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid FTConfig configuration specified for robot %s ", arm_names_.at(robot_index).c_str());
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "FTConfig validated for robot %s ", arm_names_.at(robot_index).c_str());

        return true;
    }

    /**
     * @brief validate the controller given by the user against the list of APIs
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateController_(const std::string &mode, const int robot_index)
    {
        if (control_mode_map_.find(mode) == control_mode_map_.end())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid controller configuration specified for robot name %s", arm_names_.at(robot_index).c_str());
            return false;
        }

        updateController_(control_mode_map_[mode], robot_index);

        // add validator step

        return true;
    }

    /**
     * update local API to given type
     */
    void CobotHWInterface::updateController_(const API &api, const int robot_index)
    {
        ros_controller_map_.at(robot_index).controller = api;
    }

    /**
     * @brief set gripper
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setGripper_(const int robot_index)
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eGripperConfig;
        req.communicate = DataCommunicatorRequest::eGripperConfig;

        DataValidatorContainer container;
        hw_interface_defs::GripperConfig config;
        config.gripper_type = gripper_type_map_[robot_index];

        container.gripper = config;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief validate safety mode
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateSafety_()
    {
        for (const auto &[arm_index, safety_type] : safety_map_)
        {
            safety_mode_.safety_type = safety_type;

            if (!(data_validator_->validateRequest(safety_mode_) == error_codes::NoError))
            {
                RCLCPP_WARN(rclcpp::get_logger("CobotHWInterface"), "Failed to validate safety data for robot %s. Got Safety value %d", arm_names_.at(arm_index).c_str(), safety_type);
                return false;
            }
        }

        return true;
    }

    /**
     * @brief set safety type
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setSafety_()
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eSafety;
        req.communicate = DataCommunicatorRequest::eSafety;

        bool successful = true;

        for (const auto &[name, index] : arm_index_)
        {
            DataValidatorContainer container;
            hw_interface_defs::SafetyMode safety_mode;
            safety_mode.safety_type = safety_map_[index];
            container.safety = safety_mode;
            DataContainer cont;
            cont.validation_data = container;

            successful &= handleFwdRequest_(req, cont, index);

            if (!successful)
                RCLCPP_WARN(rclcpp::get_logger("CobotHWInterface"), "Failed to Set safety data for robot %s, with index %d", name.c_str(), index);
        }
        return successful;
    }

        /**
     * @brief set safety type
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setSafety_(const int robot_index)
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eSafety;
        req.communicate = DataCommunicatorRequest::eSafety;

        bool successful = true;

       
        DataValidatorContainer container;
        hw_interface_defs::SafetyMode safety_mode;
        safety_mode.safety_type = safety_map_[robot_index];
        container.safety = safety_mode;
        DataContainer cont;
        cont.validation_data = container;

        successful &= handleFwdRequest_(req, cont, robot_index);

        if (!successful)
            RCLCPP_WARN(rclcpp::get_logger("CobotHWInterface"), "Failed to Set safety data for robot %s, with index %d", arm_names_.at(robot_index).c_str(), robot_index);
    
        return successful;
    }


    /**
     * @brief set up FT sensor
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setFT_(const int robot_index)
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eFTSensor;
        req.communicate = DataCommunicatorRequest::eFTSensor;

        DataValidatorContainer container;
        hw_interface_defs::FTConfig ft_config;

        ft_config.ft_type = ft_type_map_[robot_index];
        ft_config.rot = ft_rotation_matrix_map_[robot_index];

        container.ft = ft_config;

        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief validate the payload configuration
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validatePayload_(const int robot_index)
    {
        if (!validateGripper_(robot_index))
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid gripper configuration specified for robot %s", arm_names_.at(robot_index).c_str());

            return false;
        }

        if (!validateFT_(robot_index))
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid FT configuration specified for robot %s", arm_names_.at(robot_index).c_str());

            return false;
        }

        DataProcessorRequest req;

        payload_.mass = payload_mass_map_[robot_index];

        payload_.com = payload_cog_map_[robot_index];

        payload_.moi = payload_intertia_map_[robot_index];

        error_codes validation_result = data_validator_->validateRequest(payload_);
        if (validation_result != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid payload configuration specified for robot %s", arm_names_.at(robot_index).c_str());
            return false;
        }

        req.validate = DataValidatorRequest::eNone;

        DataValidatorContainer container;
        container.payload = payload_;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief Set payload configuration
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setPayload_(const int robot_index)
    {
        if (gripper_type_map_[robot_index] != static_cast<int>(GripperTypes::eNone))
        {
            if (!setGripper_(robot_index))
            {
                return false;
            }
        }

        if (ft_config_.ft_type != static_cast<int>(FTTypes::eNone))
        {
            if (!setFT_(robot_index))
            {
                return false;
            }
        }

        DataProcessorRequest req;

        req.convert = DataConverterRequest::ePayload;
        req.communicate = DataCommunicatorRequest::ePayload;

        DataValidatorContainer container;

        hw_interface_defs::Payload payload;
        payload.mass = payload_mass_map_[robot_index];
        payload.com = payload_cog_map_[robot_index];
        payload.moi = payload_intertia_map_[robot_index];

        container.payload = payload;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief handle forward requests
     *
     * @param request
     * @param container
     * @return true
     * @return false
     */
    bool CobotHWInterface::handleFwdRequest_(const DataProcessorRequest &request, DataContainer &container)
    {
        error_codes ec = data_processor_->handleForward(request, container);
        if (ec != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Encountered error in handling the request");

            printError_(ec);
            return false;
        }

        return true;
    }

    /**
     * @brief handle backward requests
     *
     * @param request
     * @param container
     * @return true
     * @return false
     */
    bool CobotHWInterface::handleBwdRequest_(const DataProcessorRequest &request, DataContainer &container)
    {
        error_codes ec = communicator_->handleBackward(request, container);
        if (ec != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "error in handle request");

            printError_(ec);
            return false;
        }

        return true;
    }

    bool CobotHWInterface::handleFwdRequest_(const DataProcessorRequest &request, DataContainer &container, const int robot_index)
    {
        error_codes ec = data_processor_map_[robot_index]->handleForward(request, container);
        if (ec != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Encountered error in handling the request for robot name %s", arm_names_.at(robot_index).c_str());

            printError_(ec);
            return false;
        }

        return true;
    }

    bool CobotHWInterface::handleBwdRequest_(const DataProcessorRequest &request, DataContainer &container, const int robot_index)
    {
        error_codes ec = data_communicator_map_[robot_index]->handleBackward(request, container);
        if (ec != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "error in handle request for robot name %s", arm_names_.at(robot_index).c_str());

            printError_(ec);
            return false;
        }

        return true;
    }

    /**
     * @brief setup the map mapping from sure given control mode(string) to API(API)
     *
     */
    void CobotHWInterface::setControlModeMap_()
    {
        control_mode_map_.clear();

        control_mode_map_.insert({"velocity", API::eExternalVelocityAPI});
        control_mode_map_.insert({"effort", API::eExternalTorqueAPI});
        control_mode_map_.insert({"ptp_joint", API::eLinearVelocityAPI});
        control_mode_map_.insert({"free_drive", API::eFreeDriveAPI});
        control_mode_map_.insert({"recorder", API::ePlayRecAPI});
        control_mode_map_.insert({"ptp_tcp", API::eTcpMultiPointAPI});
        control_mode_map_.insert({"joint_jogging", API::eJogJointAPI});
        control_mode_map_.insert({"cartesian_jogging", API::eJogRPYAPI});
        control_mode_map_.insert({"joint_impedance", API::eJointImpedanceAPI});
        control_mode_map_.insert({"cartesian_impedance", API::eCartesianImpedanceAPI});
        control_mode_map_.insert({"gravity_comp_effort", API::eGCompExternalTorqueAPI});
        control_mode_map_.insert({"", API::eNone});

        // keep adding more
    }

    hardware_interface::return_type CobotHWInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
    {
        // Reset intent state before analyzing the new request
        mode_switch_intent_.reset();

        auto getControllerName = [](const std::string &str) -> std::string
        {
            size_t pos = str.find("controller_name/");
            if (pos != std::string::npos)
            {
                return str.substr(pos + std::string("controller_name/").length());
            }
            return std::string();
        };

        std::vector<std::string> request_to_deactivate;
        std::vector<std::string> request_to_activate;
        std::string robot_name = "";

        // Parse start interfaces
        for (const auto &interface : start_interfaces)
        {
            std::string tmp = getControllerName(interface);
            if (!tmp.empty())
            {
                robot_name = extractRobotName(interface);
                request_to_activate.push_back(tmp);
            }
        }

        // Parse stop interfaces
        for (const auto &interface : stop_interfaces)
        {
            std::string tmp = getControllerName(interface);
            if (!tmp.empty())
            {
                if (robot_name.empty())
                {
                    robot_name = extractRobotName(interface);
                }
                request_to_deactivate.push_back(tmp);
            }
        }

        // If nothing relevant to this hardware is happening, exit cleanly
        if (request_to_activate.empty() && request_to_deactivate.empty())
        {
            return hardware_interface::return_type::OK;
        }

        // Safely get robot index
        int robot_index = 0;
        if (!robot_name.empty() && arm_index_.find(robot_name) != arm_index_.end())
        {
            robot_index = arm_index_.at(robot_name);
        }

        control_mode_ = getStringFromAPI(ros_controller_map_.at(robot_index).controller);

        std::cout << "control Mode " << control_mode_ << std::endl;

        // VALIDATION LOGIC
        if (request_to_activate.size() > 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"),
                         "Cannot activate multiple controllers simultaneously. Aborting.");
            return hardware_interface::return_type::ERROR;
        }

        // Case 1: Deactivating current controller, not starting a new one
        if (request_to_activate.empty() && request_to_deactivate.size() > 0)
        {
            if (std::find(request_to_deactivate.begin(), request_to_deactivate.end(), control_mode_) != request_to_deactivate.end())
            {
                // Setup intent for perform
                mode_switch_intent_.pending = true;
                mode_switch_intent_.revert_to_default_velocity = true;
                mode_switch_intent_.target_robot_index = robot_index;
                return hardware_interface::return_type::OK;
            }
        }

        // Case 2: Activating exactly one controller
        if (request_to_activate.size() == 1)
        {
            std::string requested_mode = request_to_activate[0];

            if (control_mode_.empty())
            {
                // Booting from empty state
                mode_switch_intent_.pending = true;
                mode_switch_intent_.target_control_mode = requested_mode;
                mode_switch_intent_.target_robot_index = robot_index;
                return hardware_interface::return_type::OK;
            }
            else
            {
                // 🚫 Same controller already active on same robot
                if (control_mode_ == requested_mode)
                {
                    RCLCPP_WARN(rclcpp::get_logger("CobotHardwareInterface"),
                                "Controller '%s' already active on robot %d",
                                requested_mode.c_str(), robot_index);
                    return hardware_interface::return_type::OK;
                }
                // Switching controllers. Must ensure current one is being stopped.
                if (std::find(request_to_deactivate.begin(), request_to_deactivate.end(), control_mode_) != request_to_deactivate.end())
                {
                    mode_switch_intent_.pending = true;
                    mode_switch_intent_.target_control_mode = requested_mode;
                    mode_switch_intent_.target_robot_index = robot_index;
                    return hardware_interface::return_type::OK;
                }
                else
                {
                    RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"),
                                 "You must deactivate the current running controller to start another one. Aborting.");
                    return hardware_interface::return_type::ERROR;
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CobotHWInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
    {
        // If prepare didn't flag a switch, do nothing.
        if (!mode_switch_intent_.pending)
        {
            return hardware_interface::return_type::OK;
        }

        // Case 1: Reverting to default (Velocity)
        if (mode_switch_intent_.revert_to_default_velocity)
        {
            if (!changeControlMode_("velocity", mode_switch_intent_.target_robot_index))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"), "Failed to revert to velocity mode on hardware");
                return hardware_interface::return_type::ERROR;
            }
            control_mode_ = ""; // Reset active mode tracker
        }
        // Case 2: Activating a targeted mode
        else
        {
            if (!changeControlMode_(mode_switch_intent_.target_control_mode, mode_switch_intent_.target_robot_index))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"), "Failed to update command mode on hardware");
                return hardware_interface::return_type::ERROR;
            }
            control_mode_ = mode_switch_intent_.target_control_mode;
        }

        // Update map and reset flag
        ros_controller_map_.at(mode_switch_intent_.target_robot_index).controller = control_mode_map_[control_mode_];

        // Safety measure: reset the intent so it can't be accidentally re-triggered
        mode_switch_intent_.reset();

        return hardware_interface::return_type::OK;
    }
    /**
     * @brief checks for controllers which have common interfaces and modifies request to activate list according to it.
     *
     * @return true
     * @return false
     */
    void CobotHWInterface::removeConflictingControllers_(const std::vector<std::string> &start_interfaces, std::vector<std::string> &request_to_activate)
    {
        for (const auto &common_interface_controller_pair : addverb_cobot::common_interface_controllers)
        {
            const std::string &controller = common_interface_controller_pair.first;                            // Effort
            const std::vector<std::string> &conflicting_controllers = common_interface_controller_pair.second; // gravity_comp_effort

            std::cout << "Controller " << controller << std::endl;

            for (const auto &name : conflicting_controllers)
            {
                std::cout << "conflicting controller " << name << std::endl;
            }

            if (std::count(request_to_activate.begin(), request_to_activate.end(), controller) > 1)
            {
                return;
            }

            // If controller is in request_to_activate
            if (std::find(request_to_activate.begin(), request_to_activate.end(), controller) != request_to_activate.end())
            {
                for (const auto &conflicting_controller : conflicting_controllers)
                {
                    std::string controller_interface = "controller_name/" + conflicting_controller;

                    // If conflicting controller is currently in start_interfaces
                    if (std::find(start_interfaces.begin(), start_interfaces.end(), controller_interface) != start_interfaces.end())
                    {
                        // Remove the *controller* (not the conflicting one) from request_to_activate
                        request_to_activate.erase(
                            std::remove(request_to_activate.begin(), request_to_activate.end(), controller),
                            request_to_activate.end());

                        break; // stop checking further conflicts for this controller
                    }
                }
            }
        }
    }

    /**
     * @brief switch controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::switchController_(const int robot_index)
    {

        DataProcessorRequest req;

        req.convert = DataConverterRequest::eController;
        req.communicate = DataCommunicatorRequest::eController;

        DataValidatorContainer container;

        hw_interface_defs::ControlMode api = ros_controller_map_.at(robot_index);

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Switching controller to %d, for robot name %s", static_cast<int>(api.controller), arm_names_.at(robot_index).c_str());

        if (ros_controller_map_.at(robot_index).controller == API::eExternalTorqueAPI)
        {
            if (robot_effort_controller_event_map_.at(robot_index) == addverb_cobot::effortControllerEvents::eShouldActivate) //! TODO: Needs to be updated
            {
                api.controller = API::eExternalTorqueAPI;
                robot_effort_controller_event_map_.at(robot_index) = addverb_cobot::effortControllerEvents::eNone;
            }
            else
            {
                api.controller = API::eExternalVelocityAPI;
                robot_effort_controller_event_map_.at(robot_index) = addverb_cobot::effortControllerEvents::eRequestedToActivate;
            }
        }

        container.controller = api;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief update controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::updateController_(const int robot_index)
    {
        if (switchController_(robot_index))
        {
            switchControlLoop_(ros_controller_map_.at(robot_index).controller, robot_index);
            return true;
        }

        return false;
    }

    /**
     * @brief switch the control loop execution based on the new controller
     *
     */
    void CobotHWInterface::switchControlLoop_(const API &api, const int robot_index)
    {
        switch (api)
        {
        case API::eExternalVelocityAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::extVelocity_, this, robot_index);
            break;

        case API::eExternalTorqueAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::extEffort_, this, robot_index);
            break;

        case API::eLinearVelocityAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::jointPtp_, this, robot_index);
            break;

        case API::ePlayRecAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::replay_, this, robot_index);
            break;

        case API::eFreeDriveAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::freeDrive_, this, robot_index);
            break;

        case API::eJogJointAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::jointJogging_, this, robot_index);
            break;

        case API::eJogRPYAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::cartesianJogging_, this, robot_index);
            break;

        case API::eJointImpedanceAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::jointImpedance_, this, robot_index);
            break;

        case API::eCartesianImpedanceAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::cartesianImpedance_, this, robot_index);
            break;

        case API::eTcpMultiPointAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::tcpPtp_, this, robot_index);
            break;

        case API::eGCompExternalTorqueAPI:
            control_loop_map_.at(robot_index) = std::bind(&CobotHWInterface::gravityCompExtEffort_, this, robot_index);
            break;

        default:
            break;
        }
    }

    /**
     * @brief validate state interface
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateStateInterface_()
    {
        const int n_states = 3;
        const size_t joints_per_robot = n_dof + 1;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            if ((i + 1) % joints_per_robot == 0)
            {
                // gripper joint
                continue;
            }

            // todo : yaswanth.gonna need to replace in with value from header configs
            if (info_.joints[i].state_interfaces.size() != 3) 
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Joint '%s' has %zu state interfaces found. Expected number of states %zu.",
                    info_.joints[i].name.c_str(), info_.joints[i].state_interfaces.size(), n_states);

                return false;
            }
        }
        RCLCPP_INFO(
            rclcpp::get_logger("CobotHWInterface"),
            "state interface successfully validated");

        return true;
    }

    /**
     * @brief initialise state interface variables
     *
     */
    void CobotHWInterface::initStateVar_()
    {
        hw_state_jpos_ = {0, 0, 0, 0, 0, 0};
        hw_state_jvel_ = {0, 0, 0, 0, 0, 0};
        hw_state_jtor_ = {0, 0, 0, 0, 0, 0};
        hw_ft_feedback_ = {0, 0, 0, 0, 0, 0};
        hw_ee_pos_feedback_ = {0, 0, 0, 0, 0, 0};
        ptp_state_.reset();

        position_state_.resize(num_joint_dof_, 0.0);
        velocity_state_.resize(num_joint_dof_, 0.0);
        effort_state_.resize(num_joint_dof_, 0.0);

        gripper_position_state_.resize(num_grippers_, 0.0);
        gripper_velocity_state_.resize(num_grippers_, 0.0);
        gripper_effort_state_.resize(num_grippers_, 0.0);

        recorder_state_.resize(arm_index_.size(), 0);

    }

    /**
     * @brief initialise command interface variables
     *
     */
    void CobotHWInterface::initCmdVar_()
    {
        ptp_cmd_.reset();

        // jvel_cmd_.cmd = std::vector<double>(n_dof, 0);
        jvel_cmd_.prev_cmd = std::vector<double>(n_dof, 0);

        jeffort_cmd_.cmd = std::vector<double>(n_dof, 0);
        jeffort_cmd_.prev_cmd = std::vector<double>(n_dof, 0);
        // effortcmd_.fill(0.0);

        controller_name_cmd_ = std::vector<double>(n_controllers, 0);
        gripper_cmd_.reset();

        position_cmd_.resize(num_joint_dof_, 0.0);
        velocity_cmd_.resize(num_joint_dof_, 0.0);
        effort_cmd_.resize(num_joint_dof_, 0.0);

        // gripper_position_cmd_.resize(num_grippers_, 0.0);
        // gripper_velocity_cmd_.resize(num_grippers_, 0.0);
        // gripper_effort_cmd_.resize(num_grippers_, 0.0);
    }

    /**
     * @brief setup the service node
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setupServices_()
    {

        for (const auto &[name, robot_index] : arm_index_)
        {
            cobot_auxiliary_node_map_[robot_index] = std::make_shared<CobotAuxiliary>(name);
        }
        cobot_executor_ = std::make_shared<CobotExecutor>();

        cobot_services_node_ = std::make_shared<CobotServices>(); // Single service for error and shutdown for all robots
        // cobot_auxiliary_node_ = std::make_shared<CobotAuxiliary>("");
        if (!cobot_executor_->setup())
        {
            return false;
        }

        cobot_executor_->add_node(cobot_services_node_);

        for (const auto &it : cobot_auxiliary_node_map_)
        {
            cobot_executor_->add_node(it.second);
        }

        return true;
    }

    /**
     * @brief validate command interface
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateCommandInterface_()
    {
        const size_t joints_per_robot = n_dof + 1;

        for (uint i = 0; i < info_.joints.size(); i++)
        {
            if((i+1) % joints_per_robot == 0)
            {
                /// Gripper Joints
                continue;
            }

            if (info_.joints[i].command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Joint '%s' has '%s' command interface found. Position expected.",
                    info_.joints[i].name.c_str(), info_.joints[i].command_interfaces[0].name.c_str());
                return false;
            }

            if (info_.joints[i].command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Joint '%s' has '%s' command interface found. Velocity expected.",
                    info_.joints[i].name.c_str(), info_.joints[i].command_interfaces[0].name.c_str());
                return false;
            }

            if (info_.joints[i].command_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CobotHWInterface"),
                    "Joint '%s' has '%s' command interface found. Effort expected.",
                    info_.joints[i].name.c_str(), info_.joints[i].command_interfaces[1].name.c_str());
                return false;
            }
        }
        RCLCPP_INFO(
            rclcpp::get_logger("CobotHWInterface"),
            "command interface successfully validated");
        return true;
    }

    /**
     * @brief print error to console
     *
     */
    void CobotHWInterface::printError_(const error_codes &ec)
    {
        // map ec to string message
        // RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), )
    }

    /****************      RUN DIFFERENT CONTROLLERS                 ********************* */

    /**
     * @brief run external velocity
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::extVelocity_(const int robot_index)
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eVelocity;
        req.communicate = DataCommunicatorRequest::eVelocity;

        DataValidatorContainer container;

        hw_interface_defs::Velocity tmp_vel;

        std::string name = arm_index_reverse_.at(robot_index);

        tmp_vel.cmd.resize(arm_groups_.at(name).size());
        auto &robot_motion_data = robot_motion_command_data_map_.at(robot_index);
        for (int i = 0; i < tmp_vel.cmd.size(); i++)
        {

            tmp_vel.cmd.at(i) = robot_motion_data.velocity_.at(i);
        }
        container.velocity = tmp_vel;
        DataContainer cont;
        cont.validation_data = container;

        if (!container.velocity.has_value())
        {
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "velocity container empty for robot %s", arm_names_.at(robot_index).c_str());
        }

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief run ptp controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::jointPtp_(const int robot_index)
    {

        auto &robot_control_data = robot_control_data_map_.at(robot_index);
        auto &robot_state_data = robot_state_data_map_.at(robot_index);

        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(robot_control_data.ptp_cmd_.transfer_cmd);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            multi_point_map_.at(robot_index).reset();
            robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_map_.at(robot_index).getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = robot_control_data.ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = robot_control_data.ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;
            multi_point_map_.at(robot_index).addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_map_.at(robot_index).getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi point trajectory for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            API api;

            if (multi_point_map_.at(robot_index).getSize() == 1)
            {
                container.point = multi_point_map_.at(robot_index).getPoint(0);
                api = API::eLinearVelocityAPI;
                req.convert = DataConverterRequest::ePoint;
                req.communicate = DataCommunicatorRequest::ePoint;
            }
            else if (multi_point_map_.at(robot_index).getSize() > 1)
            {
                addBufferTime_(robot_index);
                container.multi_point = multi_point_map_.at(robot_index);
                api = API::eMultiPointAPI;
                req.convert = DataConverterRequest::eMultiPoint;
                req.communicate = DataCommunicatorRequest::eMultiPoint;
            }
            std::cout << "size of target : " << multi_point_map_.at(robot_index).getSize() << std::endl;

            updateController_(api, robot_index);

            if (!switchController_(robot_index))
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont, robot_index))
            {
                robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
        }

        return true;
    }

    /**
     * @brief run ptp_tcp controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::tcpPtp_(const int robot_index)
    {
        auto &robot_control_data = robot_control_data_map_.at(robot_index);
        auto &robot_state_data = robot_state_data_map_.at(robot_index);

        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(robot_control_data.tcp_ptp_cmd_.transfer_cmd);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            tcp_multi_point_map_.at(robot_index).reset();
            robot_state_data.tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::TcpPoint pt;
            pt.init();
            std::cout << "TCP point number : " << (tcp_multi_point_map_.at(robot_index).getSize() + 1) << std::endl;
            for (int i = 0; i < 6; i++)
            {
                pt.pose[i] = robot_control_data.tcp_command_[i];
                std::cout << pt.pose[i] << "\t";
            }
            pt.delta_t = robot_control_data.tcp_ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;
            tcp_multi_point_map_.at(robot_index).addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {

            DataProcessorRequest req;
            DataValidatorContainer container;
            req.validate = DataValidatorRequest::eNone;

            if (tcp_multi_point_map_.at(robot_index).getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi-point trajectory for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            API api;
            container.tcp_multi_point = tcp_multi_point_map_.at(robot_index);
            api = API::eTcpMultiPointAPI;
            req.convert = DataConverterRequest::eTcpMultipoint;
            req.communicate = DataCommunicatorRequest::eTcpMultipoint;

            std::cout << "size of target : " << tcp_multi_point_map_.at(robot_index).getSize() << std::endl;
            updateController_(api, robot_index);

            if (!switchController_(robot_index))
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to TCP PTP API for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont, robot_index))
            {
                robot_state_data.tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                robot_state_data.tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            robot_state_data.tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            robot_state_data.tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
        }

        return true;
    }

    /**
     * @brief run play recording
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::replay_(const int robot_index)
    {
        auto &robot_control = robot_control_data_map_.at(robot_index);
        auto &robot_state = robot_state_data_map_.at(robot_index);

        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(robot_control.ptp_cmd_.transfer_cmd);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_map_.at(robot_index).getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = robot_control.ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = robot_control.ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;

            multi_point_map_.at(robot_index).addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_map_.at(robot_index).getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi point trajectory for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            hw_interface_defs::ReplayConfig replay_config;

            replay_config.points = multi_point_map_.at(robot_index);
            replay_config.iterations = static_cast<int>(robot_control.replay_iterations_cmd_);

            container.replay_config = replay_config;

            req.convert = DataConverterRequest::eReplay;
            req.communicate = DataCommunicatorRequest::eReplay;

            std::cout << "size of target : " << multi_point_map_.at(robot_index).getSize() << std::endl;
            API api = API::ePlayRecAPI;

            updateController_(api, robot_index);
            recorder_state_.at(robot_index) = 0;

            if (!switchController_(robot_index))
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Replay API for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;
            if (!handleFwdRequest_(req, cont, robot_index))
            {
                return false;
            }

            robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            multi_point_map_.at(robot_index).reset();
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle || cur_transfer_cmd == TransferCommand::eNone)
        {
            robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);

            if (recorder_state_.at(robot_index) == 0)
            {
                API api = API::eFreeDriveAPI;
                updateController_(api, robot_index);

                if (!switchController_(robot_index))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Free Drive API for robot name %s", arm_names_.at(robot_index).c_str());
                    return false;
                }

                recorder_state_.at(robot_index) = 1;
            }
        }

        return true;
    }

    /**
     * @brief run free drive controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::freeDrive_(const int robot_index)
    {
        // need to do nothing
        return true;
    }

    /**
     * @brief get robot data
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::getFeedback_()
    {
        for (const auto &[name, index] : arm_index_)
        {
            DataProcessorRequest req;

            req.convert = DataConverterRequest::eReadFeedback;
            req.communicate = DataCommunicatorRequest::eReadFeedback;

            DataCommunicatorContainer container;
            DataContainer cont;

            cont.communication_data = container;
            if (!handleBwdRequest_(req, cont, index))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to fetch robot data");
                return false;
            }

            if (!cont.convert_data.has_value())
            {
                return false;
            }

            // read state from robot feedback
            for (int i = 0; i < n_dof; i++)
            {
                position_state_[arm_groups_.at(name).at(i)] = cont.convert_data->hw_robot_feedback->jpos[i];
                velocity_state_[arm_groups_.at(name).at(i)] = cont.convert_data->hw_robot_feedback->jvel[i];
                effort_state_[arm_groups_.at(name).at(i)] = cont.convert_data->hw_robot_feedback->jtor[i];

                robot_motion_state_data_map_.at(index).position_.at(i) = cont.convert_data->hw_robot_feedback->jpos[i];
                robot_motion_state_data_map_.at(index).velocity_.at(i) = cont.convert_data->hw_robot_feedback->jvel[i];
                robot_motion_state_data_map_.at(index).effort_.at(i) = cont.convert_data->hw_robot_feedback->jtor[i];
            }


            updateFTData_(cont, index);
            updateEEPosData_(cont, index);

            has_jinfo_ = true;
        }
        return true;
    }

    /**
     * @brief update FT data
     *
     * @param container
     */
    void CobotHWInterface::updateFTData_(const DataContainer &container, const int robot_index)
    {
        // for (int i = 0; i < 6; i++)
        // {
        //     hw_ft_feedback_[i] = container.convert_data->hw_robot_feedback->ft_data[i];
        // }

        cobot_auxiliary_node_map_.at(robot_index)->updateFTData(container.convert_data->hw_robot_feedback->ft_data);
    }

    /**
     * @brief update EE pos data
     *
     * @param container
     */
    void CobotHWInterface::updateEEPosData_(const DataContainer &container, const int robot_index)
    {
        // for (int i = 0; i < 6; i++)
        // {
        //     hw_ee_pos_feedback_[i] = container.convert_data->hw_robot_feedback->ee_pos_data[i];
        // }

        cobot_auxiliary_node_map_.at(robot_index)->updateEEPosData(container.convert_data->hw_robot_feedback->ee_pos_data);
    }

    /**
     * @brief get current robot state
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::getRobotState_()
    {
        for (const auto &[name, index] : arm_index_)
        {
            DataProcessorRequest req;

            req.convert = DataConverterRequest::eReadState;
            req.communicate = DataCommunicatorRequest::eReadState;

            DataCommunicatorContainer container;
            DataContainer cont;

            cont.communication_data = container;

            if (!handleBwdRequest_(req, cont, index))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to fetch robot data for robot %s", arm_names_.at(index).c_str());
                return false;
            }

            if (!cont.convert_data.has_value())
            {
                return false;
            }

            auto robot_state = cont.convert_data->robot_state.value();

            robot_state_data_map_.at(index).robot_status_ = 1.0 * static_cast<int>(robot_state);

            // Simulate gripper movement to calculate live velocity
            auto &robot_control_data = robot_control_data_map_.at(index);

            int gripper_index = gripper_map_[name];
            
            double target_gripper_pos = (robot_control_data.gripper_cmd_.position == 1) ? 0.0 : 0.725;
            double diff = target_gripper_pos - gripper_position_state_[gripper_index]; // Assuming gripper position is at gripper_index 6 of the state vector for each robot
            
            if (std::abs(diff) > 1e-4) 
            {
                // Moving
                if (diff > 0) {
                    gripper_velocity_state_[gripper_index] = 1.0; // Opening
                    gripper_effort_state_[gripper_index] = 0.5;
                } else {
                    gripper_velocity_state_[gripper_index] = -1.0; // Closing
                    gripper_effort_state_[gripper_index] = -0.5;
                }
                // Update position (assuming 100Hz update rate -> dt = 0.01s)
                gripper_position_state_[gripper_index] += gripper_velocity_state_[gripper_index] * 0.01;
                
                // Prevent overshooting
                if ((gripper_velocity_state_[gripper_index] > 0 && gripper_position_state_[gripper_index] > target_gripper_pos) ||
                    (gripper_velocity_state_[gripper_index] < 0 && gripper_position_state_[gripper_index] < target_gripper_pos)) {
                    gripper_position_state_[gripper_index] = target_gripper_pos;
                }
            } else {
                // At rest
                gripper_position_state_[gripper_index] = target_gripper_pos;
                gripper_velocity_state_[gripper_index] = 0.0;
            }
        }

        return true;
    }

    /**
     * @brief run external effort controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::extEffort_(const int robot_index)
    {

        DataProcessorRequest req;
        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eEffort;
        req.communicate = DataCommunicatorRequest::eEffort;

        DataValidatorContainer container;
        std::vector<double> temp_cmd(addverb_cobot::n_dof, 0.0);

        auto &robot_motion_data = robot_motion_command_data_map_.at(robot_index);
        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            temp_cmd[i] = robot_motion_data.effort_[i];
        }

        jeffort_cmd_.cmd = temp_cmd;

        if (robot_effort_controller_event_map_.at(robot_index) == addverb_cobot::effortControllerEvents::eRequestedToActivate)
        {
            if (robot_effort_controller_status_map_.at(robot_index) != addverb_cobot::effortControllerStatus::eReady)
            {
                if (!goToBase_(robot_index))
                {
                    return false;
                }

                robot_effort_controller_status_map_.at(robot_index) = addverb_cobot::effortControllerStatus::eReady;

                return true;
            }

            error_codes validation_result = data_validator_->validateRequest(jeffort_cmd_);
            if (validation_result == error_codes::NoError)
            {
                API api;
                api = API::eExternalTorqueAPI;

                updateController_(api, robot_index);

                {
                    container.effort = jeffort_cmd_;
                    DataContainer cont;

                    cont.validation_data = container;

                    if (!container.effort.has_value())
                    {
                        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "effort container empty for robot %s", arm_names_.at(robot_index).c_str());
                    }
                    jeffort_cmd_.prev_cmd = temp_cmd;

                    if (!handleFwdRequest_(req, cont, robot_index))
                    {
                        return false;
                    }
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));

                robot_effort_controller_event_map_.at(robot_index) = addverb_cobot::effortControllerEvents::eShouldActivate;

                if (!switchController_(robot_index))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to External Torque API for robot name %s", arm_names_.at(robot_index).c_str());
                    return false;
                }
            }
            else
            {
                return true;
            }

            robot_effort_controller_status_map_.at(robot_index) = addverb_cobot::effortControllerStatus::eActivate; 
        }

        if (robot_effort_controller_status_map_.at(robot_index) == addverb_cobot::effortControllerStatus::eActivate)
        {
            container.effort = jeffort_cmd_;
            DataContainer cont;

            cont.validation_data = container;

            if (!container.effort.has_value())
            {
                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "effort container empty for robot %s", arm_names_.at(robot_index).c_str());
            }

            jeffort_cmd_.prev_cmd = temp_cmd;

            return handleFwdRequest_(req, cont, robot_index);
        }

        return true;
    }

    bool CobotHWInterface::gravityCompExtEffort_(const int robot_index)
    {

        DataProcessorRequest req;
        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eEffort;
        req.communicate = DataCommunicatorRequest::eEffort;

        DataValidatorContainer container;
        std::vector<double> temp_cmd(addverb_cobot::n_dof, 0.0);

        auto &robot_motion_data = robot_motion_command_data_map_.at(robot_index);
        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            temp_cmd[i] = robot_motion_data.effort_[i];
        }

        jeffort_cmd_.cmd = temp_cmd;

        container.effort = jeffort_cmd_;
        DataContainer cont;

        cont.validation_data = container;

        if (!container.effort.has_value())
        {
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "effort container empty for robot %s", arm_names_.at(robot_index).c_str());
        }
        jeffort_cmd_.prev_cmd = temp_cmd;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief run joint jogging controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::jointJogging_(const int robot_index)
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eJointJogging;
        req.communicate = DataCommunicatorRequest::eJointJogging;

        DataValidatorContainer container;

        hw_interface_defs::JointJog tmp_jj(n_dof);

        auto &robot_control_data = robot_control_data_map_.at(robot_index);
        for (int i = 0; i < n_dof; i++)
        {
            tmp_jj.jog_cmd.cmd[i] = robot_control_data.joint_jogging_cmd_.jog_cmd.cmd[i];
        }

        robot_control_data.joint_jogging_cmd_.reset();

        container.joint_jogging = tmp_jj;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief run cartesian jogging controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::cartesianJogging_(const int robot_index)
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eCartesianJogging;
        req.communicate = DataCommunicatorRequest::eCartesianJogging;

        DataValidatorContainer container;

        hw_interface_defs::CartesianJog tmp_jj;

        auto &robot_control_data = robot_control_data_map_.at(robot_index);

        for (int i = 0; i < 6; i++)
        {
            tmp_jj.jog_cmd.cmd[i] = robot_control_data.cartesian_jogging_cmd_.jog_cmd.cmd[i];
        }

        robot_control_data.cartesian_jogging_cmd_.reset();

        container.cartesian_jogging = tmp_jj;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief run joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::jointImpedance_(const int robot_index)
    {
        auto &robot_control_data = robot_control_data_map_.at(robot_index);
        auto &robot_state_data = robot_state_data_map_.at(robot_index);

        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(robot_control_data.ptp_cmd_.transfer_cmd);
        TransferState cur_transfer_state = static_cast<TransferState>(robot_state_data.ptp_state_.transfer_state);

        updateJointImpedance_(robot_index);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            multi_point_map_.at(robot_index).reset();
            robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_map_.at(robot_index).getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = robot_control_data.ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = robot_control_data.ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;

            multi_point_map_.at(robot_index).addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_map_.at(robot_index).getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi point trajectory for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            API api;

            if (multi_point_map_.at(robot_index).getSize() == 1)
            {
                container.point = multi_point_map_.at(robot_index).getPoint(0);
                api = API::eJointImpedanceAPI;
                req.convert = DataConverterRequest::ePoint;
                req.communicate = DataCommunicatorRequest::ePoint;
            }
            else if (multi_point_map_.at(robot_index).getSize() > 1)
            {
                container.multi_point = multi_point_map_.at(robot_index);
                api = API::eMultiJointImpedanceAPI;
                req.convert = DataConverterRequest::eMultiPoint;
                req.communicate = DataCommunicatorRequest::eMultiPoint;
            }

            std::cout << "size of target : " << multi_point_map_.at(robot_index).getSize() << std::endl;

            updateController_(api, robot_index);

            if (!switchController_(robot_index))
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont, robot_index))
            {
                robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            robot_state_data.ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
        }

        return true;
    }

    /**
     * @brief run joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::cartesianImpedance_(const int robot_index)
    {
        auto &robot_control = robot_control_data_map_.at(robot_index);
        auto &robot_state = robot_state_data_map_.at(robot_index);

        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(robot_control.ptp_cmd_.transfer_cmd);
        TransferState cur_transfer_state = static_cast<TransferState>(robot_state.ptp_state_.transfer_state);

        updateCartesianImpedance_(robot_index);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            multi_point_map_.at(robot_index).reset();
            robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_map_.at(robot_index).getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = robot_control.ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = robot_control.ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;

            multi_point_map_.at(robot_index).addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_map_.at(robot_index).getSize() != 1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Wrong number of points were added for the cartesian impedance controller trajectory for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            API api;

            container.point = multi_point_map_.at(robot_index).getPoint(0);
            api = API::eCartesianImpedanceAPI;
            req.convert = DataConverterRequest::ePoint;
            req.communicate = DataCommunicatorRequest::ePoint;

            std::cout << "size of target : " << multi_point_map_.at(robot_index).getSize() << std::endl;

            updateController_(api, robot_index);

            if (!switchController_(robot_index))
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Cartesian Impedance API for robot name %s", arm_names_.at(robot_index).c_str());
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont, robot_index))
            {
                robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            robot_state.ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
        }

        return true;
    }

    /**
     * @brief Move the robot to its base position
     *
     * This function will move the robot to its base position. The base
     * position is the position where the robot is at rest. This
     * function is used to move the robot to a safe position before
     * shutting down the robot.
     */
    bool CobotHWInterface::goToBase_()
    {
        const int robot_index = 0;
        DataProcessorRequest req;
        DataValidatorContainer container;

        hw_interface_defs::Point pt;

        pt.jpos = addverb_cobot::base_config_jpos;
        pt.delta_t = addverb_cobot::base_config_reach_time;

        container.point = pt;
        API api = API::eLinearVelocityAPI;
        req.convert = DataConverterRequest::ePoint;
        req.communicate = DataCommunicatorRequest::ePoint;

        updateController_(api, robot_index);

        if (!switchController_(robot_index))
        {
            RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API for robot name %s", arm_names_.at(robot_index).c_str());
            return false;
        }

        DataContainer cont;
        cont.validation_data = container;
        if (!handleFwdRequest_(req, cont))
        {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(addverb_cobot::base_config_reach_time + addverb_cobot::base_config_reach_buffer));

        return true;
    }


     /**
     * @brief Move the robot to its base position
     *
     * This function will move the robot to its base position. The base
     * position is the position where the robot is at rest. This
     * function is used to move the robot to a safe position before
     * shutting down the robot.
     */
    bool CobotHWInterface::goToBase_(const int robot_index) 
    {
        DataProcessorRequest req;
        DataValidatorContainer container;

        hw_interface_defs::Point pt;

        pt.jpos = addverb_cobot::base_config_jpos;
        pt.delta_t = addverb_cobot::base_config_reach_time;

        container.point = pt;
        API api = API::eLinearVelocityAPI;
        req.convert = DataConverterRequest::ePoint;
        req.communicate = DataCommunicatorRequest::ePoint;

        updateController_(api, robot_index);

        if (!switchController_(robot_index))
        {
            RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API for robot name %s", arm_names_.at(robot_index).c_str());
            return false;
        }

        DataContainer cont;
        cont.validation_data = container;
        if (!handleFwdRequest_(req, cont, robot_index))
        {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(addverb_cobot::base_config_reach_time + addverb_cobot::base_config_reach_buffer));

        return true;
    }

    /**
     * @brief update allied input for joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::updateJointImpedance_(const int robot_index)
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eJointImpedance;
        req.communicate = DataCommunicatorRequest::eJointImpedance;

        DataValidatorContainer container;

        hw_interface_defs::JointImpedance tmp_jimp(n_dof);

        tmp_jimp = robot_control_data_map_.at(robot_index).joint_impedance_cmd_;

        container.joint_impedance->init(n_dof);
        container.joint_impedance = tmp_jimp;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief update allied input for joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::updateCartesianImpedance_(const int robot_index)
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eCartesianImpedance;
        req.communicate = DataCommunicatorRequest::eCartesianImpedance;

        DataValidatorContainer container;

        hw_interface_defs::CartesianImpedance tmp_cimp;

        tmp_cimp = robot_control_data_map_.at(robot_index).cartesian_impedance_cmd_;

        container.cartesian_impedance = tmp_cimp;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont, robot_index);
    }

    /**
     * @brief change control mode
     *
     * @param new_mode
     * @return true
     * @return false
     */
    bool CobotHWInterface::changeControlMode_(const std::string &new_mode, const int robot_index)
    {
        if (!validateController_(new_mode, robot_index))
        {
            std::cout << "invalid mode\n";
            return false;
        }

        if (!updateController_(robot_index))
        {
            std::cout << "failed in update controller\n";
            return false;
        }

        return true;
    }

    /**
     * @brief add buffer time to moveit trajectory
     *
     * @return true
     * @return false
     */
    void CobotHWInterface::addBufferTime_(const int robot_index)
    {
        if (multi_point_map_.at(robot_index).points[0].delta_t == 0.0)
        {
            multi_point_map_.at(robot_index).points[0].delta_t = buffer_time_;
        }
    }

    /**
     * @brief run gripper controller
     *
     * @return true: successful execution
     * @return false: failed execution
     *
     * This function is responsible for sending the gripper command to the
     * gripper controller. It will return true if the command is sent
     * successfully and false otherwise.
     */
    bool CobotHWInterface::runGripper_()
    {
        bool success = true;
        for (const auto &[name, index] : arm_index_)
        {
            auto &robot_control_data = robot_control_data_map_.at(index);
            if (static_cast<GripperTransferCommand>(robot_control_data.gripper_cmd_.transfer_cmd) == GripperTransferCommand::eHasCommand)
            {
                DataProcessorRequest req;

                req.validate = DataValidatorRequest::eNone;
                req.convert = DataConverterRequest::eGripperCmd;
                req.communicate = DataCommunicatorRequest::eGripperCmd;

                DataValidatorContainer container;
                addverb_cobot::hw_interface_defs::GripperCmd tmp_gripper_cmd_ = robot_control_data.gripper_cmd_;
                container.gripper_cmd = tmp_gripper_cmd_;

                DataContainer cont;
                cont.validation_data = container;

                success &= handleFwdRequest_(req, cont, index);
            }
        }
        return success;
    }

    bool CobotHWInterface::checkPayload()
    {
        for (const auto &[robot_index, status] : payload_status_map_)
        {
            RCLCPP_WARN(rclcpp::get_logger("CobotHWInterface"), "robot index %i ", robot_index);

            if (!static_cast<bool>(status))
            {
                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "No payload attached for robot %s ", arm_names_.at(robot_index).c_str());

                continue;
            }
            if (!validatePayload_(robot_index))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid payload configuration specified for robot %s ", arm_names_.at(robot_index).c_str());

                return false;
            }
            if (!setPayload_(robot_index))
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to upload payload configuration to robot %s ", arm_names_.at(robot_index).c_str());

                return false;
            }
        }
        return true;
    }

    void CobotHWInterface::updateCommandsMappingFromHWInterface()
    {
        for (const auto &[name, index] : arm_index_)
        {
            auto &robot_control_data = robot_control_data_map_.at(index);
            auto &robot_motion_data = robot_motion_command_data_map_.at(index);

            auto robot_indices = arm_groups_.at(name);
            for (auto i = 0; i < robot_indices.size(); i++)
            {
                robot_control_data.ptp_cmd_.target.jpos[i] = position_cmd_.at(robot_indices.at(i));
                robot_motion_data.position_.at(i) = position_cmd_.at(robot_indices.at(i));
                robot_motion_data.velocity_.at(i) = velocity_cmd_.at(robot_indices.at(i));
                robot_motion_data.effort_.at(i) = effort_cmd_.at(robot_indices.at(i));
            }
        }
    }
};

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    addverb_cobot::CobotHWInterface, hardware_interface::SystemInterface)
