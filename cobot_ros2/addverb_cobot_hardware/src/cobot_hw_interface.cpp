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

        if ((int)info_.joints.size() != n_dof)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"),
                         "Number of joint specified are %zu. %d expected", info_.joints.size(), n_dof);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "URDF parsed successfully. Found %zu joints.", info.joints.size());

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

        // Set Payload configuration
        bool has_payload = std::stoi(info_.hardware_parameters["payload_status"]);

        if (static_cast<bool>(has_payload))
        {
            if (!validatePayload_())
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid payload configuration specified");

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (!setPayload_())
            {
                RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to upload payload configuration to robot.");

                return hardware_interface::CallbackReturn::FAILURE;
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "No payload attached");
        }

        // validate the number of command and state joint interfaces
        if (!(validateCommandInterface_() && validateStateInterface_()))
        {
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

        if (control_mode_.empty())
        {
            switchControlLoop_(API::eExternalVelocityAPI);
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
            // debug line
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "[CobotHWInterface::shutdownRobot_]  Check connection successful");
            if (!powerOffRobot_())
            {
                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Failed to power off robot");
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot power off success");
            return true;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot already disconnected");
            return true;
        }
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
        for (int i = 0; i < n_dof; i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_jpos_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_jvel_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_jtor_[i]));
        }

        // Register gripper interface
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "finger_joint", hardware_interface::HW_IF_POSITION, &gripper_pos_));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "finger_joint", hardware_interface::HW_IF_VELOCITY, &gripper_vel_));
            
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "finger_joint", hardware_interface::HW_IF_EFFORT, &gripper_eff_));

        // register ptp interface
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios.at(1).name, info_.gpios.at(1).state_interfaces[0].name, &ptp_state_.transfer_state));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios.at(4).name, info_.gpios.at(4).state_interfaces[0].name, &tcp_ptp_state_.transfer_state));

        for (int i = 0; i < n_dof; i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.gpios.at(5).name, info_.gpios.at(5).state_interfaces[i].name, &tcp_state_[i]));
        }

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios.at(11).name, info_.gpios.at(11).state_interfaces[0].name, &robot_status_));

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

        for (int i = 0; i < n_dof; i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &ptp_cmd_.target.jpos[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vcmd_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effortcmd_[i]));
        }

        // register command interface for ptp
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios.at(0).name, info_.gpios.at(0).command_interfaces[0].name, &ptp_cmd_.target.delta_t));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios.at(1).name, info_.gpios.at(1).command_interfaces[0].name, &ptp_cmd_.transfer_cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios.at(2).name, info_.gpios.at(2).command_interfaces[0].name, &replay_iterations_cmd_));

        // register command interface for tcp ptp
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios.at(3).name, info_.gpios.at(3).command_interfaces[0].name, &tcp_ptp_cmd_.target.delta_t));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios.at(4).name, info_.gpios.at(4).command_interfaces[0].name, &tcp_ptp_cmd_.transfer_cmd));

        for (int i = 0; i < n_dof; i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(5).name, info_.gpios.at(5).command_interfaces[i].name, &tcp_command_[i]));
        }

        // register command interface for flex factor
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     info_.gpios.at(6).name, info_.gpios.at(6).command_interfaces[0].name, &flex_point_.flex_factor));

        // register joint jogging command interface
        for (int i = 0; i < joint_jogging_cmd_.jog_cmd.cmd.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(6).name, info_.gpios.at(6).command_interfaces[i].name, &joint_jogging_cmd_.jog_cmd.cmd[i]));
        }

        // register cartesian jogging command interface
        for (int i = 0; i < cartesian_jogging_cmd_.jog_cmd.cmd.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(7).name, info_.gpios.at(7).command_interfaces[i].name, &cartesian_jogging_cmd_.jog_cmd.cmd[i]));
        }

        // register joint impedance command interface
        for (int i = 0; i < joint_impedance_cmd_.stiffness.stiffness.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(8).name, info_.gpios.at(8).command_interfaces[i].name, &joint_impedance_cmd_.stiffness.stiffness[i][i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(8).name, info_.gpios.at(8).command_interfaces[i + 6].name, &joint_impedance_cmd_.damping.damping[i][i]));
        }

        // register cartesian impedance command interface
        for (int i = 0; i < cartesian_impedance_cmd_.stiffness.stiffness.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(9).name, info_.gpios.at(9).command_interfaces[i].name, &cartesian_impedance_cmd_.stiffness.stiffness[i][i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(9).name, info_.gpios.at(9).command_interfaces[i + 6].name, &cartesian_impedance_cmd_.damping.damping[i][i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(9).name, info_.gpios.at(9).command_interfaces[i + 12].name, &cartesian_impedance_cmd_.mass_matrix.mass_matrix[i][i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(9).name, info_.gpios.at(9).command_interfaces[i + 18].name, &cartesian_impedance_cmd_.ft_force.force[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(9).name, info_.gpios.at(9).command_interfaces[i + 24].name, &cartesian_impedance_cmd_.target_force.force[i]));
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHardwareInterface"), "before exorting controller names");

        // register command interface for getting the name of the controller
        for (int i = 0; i < controller_name_cmd_.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(10).name, info_.gpios.at(10).command_interfaces[i].name, &controller_name_cmd_[i]));
        }

        RCLCPP_INFO(rclcpp::get_logger("CobotHardwareInterface"), "Exported command interfaces");

        /// gripper command interface export
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(12).name, info_.gpios.at(12).command_interfaces[0].name, &gripper_cmd_.transfer_cmd));

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(12).name, info_.gpios.at(12).command_interfaces[1].name, &gripper_cmd_.position));

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(12).name, info_.gpios.at(12).command_interfaces[2].name, &gripper_cmd_.grasp_force));
        }
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

        if (!control_loop_())
        {
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    /**
     * @brief check for any errors on the robot
     *
     */
    void CobotHWInterface::checkForError_()
    {
        if (robot_state_ == RobotState::eError)
        {
            if (in_error_.load() == false)
            {
                in_error_.store(true);
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"),
                             "Robot gone into error. Call ErrorRecoveryService to safely recover from the error and get into the home position.");
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
        if (!clearErrorState_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to clear the error state of the robot.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Clear error state done");

        // power on robot in error recovery mode
        if (!powerOnRobotErrRecovery_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to power on robot in error recovery mode.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot powered on in error recovery mode");

        // execute error recovery sequence
        if (!executeErrRecovery_())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CobotHWInterface"),
                "Failed to execute automatic error recovery sequence on the robot. Please attempt manual error recovery.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Error recovery sequence executed");

        // Reset safety
        if (!setSafety_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to power off robot and exit from error recovery mode.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Powering on the robot in normal OP mode");

        std::this_thread::sleep_for(std::chrono::seconds(error_recovery_sleep_time_));

        // restart the robot in normal OP mode
        if (!powerOnRobot_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to restart the robot in normal OP mode.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot powered on in normal OP mode");

        // reset controller to user-defined controller
        if (!switchController_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to reset robot control mode to user defined value.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "Robot control mode set to last commanded control mode");

        return true;
    }

    /**
     * @brief setup the data processor pipeline
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setupDataProcessor_()
    {
        data_processor_ = std::make_shared<DataConverter>(rclcpp::get_logger("CobotHWInterface"));

        communicator_ = std::make_shared<DataCommunicator>(rclcpp::get_logger("CobotHWInterface"));

        data_validator_ = std::make_shared<DataValidator>(rclcpp::get_logger("CobotHWInterface"));

        if (!data_processor_->setHandlers(communicator_))
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"),
                         "converter could not set handle");
            return false;
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
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::eSetup;

        return handleFwdRequest_(req, cont);
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
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::eConnect;

        int count = 0;
        const int max_count = max_reattempt_connection_count;

        do
        {
            if (handleFwdRequest_(req, cont))
            {
                return true;
            }
            count++;
            RCLCPP_WARN(rclcpp::get_logger("CobotHardwareInterface"), "Failed to connect with robot, will re-attempt after 2 seconds. Number of re-attempts remaining : %d . Kindly ensure that the server on robot side is running and the wires are not loose", (max_count - count));
            std::this_thread::sleep_for(std::chrono::seconds(2));
        } while (count <= max_count && rclcpp::ok());

        return false;
    }

    /**
     * @brief close connection with the robot
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::disconnect_()
    {
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::eDisconnect;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief change the state of robot from error
     * @return true
     * @return false
     */
    bool CobotHWInterface::clearErrorState_()
    {
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::eClearErrorState;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief power on robot
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOnRobot_()
    {
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::ePowerOn;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief power on robot in errror recovery mode
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOnRobotErrRecovery_()
    {
        DataProcessorRequest req;
        DataContainer cont;

        // @aravindh - write this impl of ePowerOnErrRecovery
        req.communicate = DataCommunicatorRequest::ePowerOnErrRecovery;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief execute errror recovery sequence
     * @return true
     * @return false
     */
    bool CobotHWInterface::executeErrRecovery_()
    {
        DataProcessorRequest req;
        DataContainer cont;

        // @aravindh - write this impl of eErrRecoverySequence
        req.communicate = DataCommunicatorRequest::eErrRecoverySequence;

        /// reset necessary variables
        {
            effort_controller_status_ = addverb_cobot::effortControllerStatus::eInactive;

            effortcmd_.fill(0.0);

            recorder_switched_ = false;
        }

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief power off robot
     * @return true
     * @return false
     */
    bool CobotHWInterface::powerOffRobot_()
    {
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::ePowerOff;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief check connection status with robot after
     * connection request
     * @return true
     * @return false
     */
    bool CobotHWInterface::checkConnection_()
    {
        DataProcessorRequest req;
        DataContainer cont;

        req.communicate = DataCommunicatorRequest::eCheckConnectivity;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief validate the gripper configuration
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateGripper_()
    {
        int gripper_type;
        gripper_type = std::stoi(info_.hardware_parameters["gripper_type"]);
        gripper_config_.gripper_type = gripper_type;

        error_codes validation_result = data_validator_->validateRequest(gripper_config_);
        if (validation_result != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid gripper configuration specified");
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
    bool CobotHWInterface::validateFT_()
    {
        int ft_type;
        ft_type = std::stoi(info_.hardware_parameters["ft_type"]);

        std::vector<std::vector<double>>
            ft_rot;
        std::vector<double> rot_row;

        for (int i = 1; i < 4; i++)
        {
            for (int j = 1; j < 4; j++)
            {
                std::string entry = "ft_rot_r" + std::to_string(i) +
                                    std::to_string(j);

                rot_row.push_back(std::stod(info_.hardware_parameters[entry]));
                // RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"),"%f",std::stod(info_.hardware_parameters[entry]));
            }

            ft_rot.push_back(rot_row);
            rot_row.clear();
        }

        // RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"),"size of ft_rot_mat : %dx%d",ft_rot.size(),ft_rot[2].size());

        ft_config_.ft_type = ft_type;
        ft_config_.rot = ft_rot;

        error_codes validation_result = data_validator_->validateRequest(ft_config_);
        if (validation_result != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid FTConfig configuration specified");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "FTConfig validated");

        return true;
    }

    /**
     * @brief validate the controller given by the user against the list of APIs
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateController_(const std::string &mode)
    {
        if (control_mode_map_.find(mode) == control_mode_map_.end())
        {
            return false;
        }

        updateController_(control_mode_map_[mode]);

        // add validator step

        return true;
    }

    /**
     * update local API to given type
     */
    void CobotHWInterface::updateController_(const API &api)
    {
        api_.controller = api;
    }

    /**
     * @brief set gripper
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setGripper_()
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eGripperConfig;
        req.communicate = DataCommunicatorRequest::eGripperConfig;

        DataValidatorContainer container;
        container.gripper = gripper_config_;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief validate safety mode
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validateSafety_()
    {
        int safety_type;
        safety_type = std::stoi(info_.hardware_parameters["safety_type"]);
        safety_mode_.safety_type = safety_type;

        if (data_validator_->validateRequest(safety_mode_) == error_codes::NoError)
        {
            return true;
        }

        return false;
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

        DataValidatorContainer container;
        container.safety = safety_mode_;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief set up FT sensor
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setFT_()
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eFTSensor;
        req.communicate = DataCommunicatorRequest::eFTSensor;

        DataValidatorContainer container;
        container.ft = ft_config_;

        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief validate the payload configuration
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::validatePayload_()
    {
        if (!validateGripper_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid gripper configuration specified");

            return false;
        }

        if (!validateFT_())
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid FT configuration specified");

            return false;
        }

        DataProcessorRequest req;

        payload_.mass = std::stod(info_.hardware_parameters["payload_mass"]);

        payload_.com.push_back(std::stod(info_.hardware_parameters["payload_comx"]));
        payload_.com.push_back(std::stod(info_.hardware_parameters["payload_comy"]));
        payload_.com.push_back(std::stod(info_.hardware_parameters["payload_comz"]));

        payload_.moi.push_back(std::stod(info_.hardware_parameters["payload_ixx"]));
        payload_.moi.push_back(std::stod(info_.hardware_parameters["payload_iyy"]));
        payload_.moi.push_back(std::stod(info_.hardware_parameters["payload_izz"]));
        payload_.moi.push_back(std::stod(info_.hardware_parameters["payload_ixy"]));
        payload_.moi.push_back(std::stod(info_.hardware_parameters["payload_ixz"]));
        payload_.moi.push_back(std::stod(info_.hardware_parameters["payload_iyz"]));

        error_codes validation_result = data_validator_->validateRequest(payload_);
        if (validation_result != error_codes::NoError)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Invalid payload configuration specified");
            return false;
        }

        req.validate = DataValidatorRequest::eNone;

        DataValidatorContainer container;
        container.payload = payload_;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief Set payload configuration
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setPayload_()
    {
        if (gripper_config_.gripper_type != static_cast<int>(GripperTypes::eNone))
        {
            if (!setGripper_())
            {
                return false;
            }
        }

        if (ft_config_.ft_type != static_cast<int>(FTTypes::eNone))
        {
            if (!setFT_())
            {
                return false;
            }
        }

        DataProcessorRequest req;

        req.convert = DataConverterRequest::ePayload;
        req.communicate = DataCommunicatorRequest::ePayload;

        DataValidatorContainer container;
        container.payload = payload_;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
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
        // keep adding more
    }

    /**
     * @brief switch to given controllers
     *
     * @param start_interfaces
     * @param stop_interfaces
     * @return hardware_interface::return_type
     */
    hardware_interface::return_type CobotHWInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
    {
        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "mode switch requested");

        auto getControllerName = [](const std::string &str) -> std::string
        {
            std::string input = str;
            if (input.find("controller_name/") != std::string::npos)
            {
                size_t start_pos = std::string("controller_name/").length();
                return input.substr(start_pos);
            }

            else if (input.find("joint1/effort") != std::string::npos)
            {
                return std::string("effort");
            }

            else if (input.find("joint1/velocity") != std::string::npos)
            {
                return std::string("velocity");
            }

            return std::string();
        };

        std::vector<std::string> request_to_deactivate;
        std::vector<std::string> request_to_activate;

        for (int i = 0; i < start_interfaces.size(); i++)
        {
            std::string tmp;
            tmp = getControllerName(start_interfaces[i]);
            if (!tmp.empty())
            {
                request_to_activate.push_back(tmp);
            }
        };

        for (int i = 0; i < stop_interfaces.size(); i++)
        {
            std::string tmp;
            tmp = getControllerName(stop_interfaces[i]);
            if (!tmp.empty())
            {
                request_to_deactivate.push_back(tmp);
            }
        };

        removeConflictingControllers_(start_interfaces, request_to_activate);

        if (request_to_activate.size() > 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"), "You attempted to activate/start more than one controller at the same time. Aborting switch operation. Deactivate all running controllers and then activate only one controller which you wish to operate.");
            return hardware_interface::return_type::ERROR;
        }
        else if (request_to_activate.size() == 0)
        {
            if (request_to_deactivate.size() > 0)
            {
                if (std::find(request_to_deactivate.begin(), request_to_deactivate.end(), control_mode_) != request_to_deactivate.end())
                {
                    // switch to velocity control mode
                    if (!changeControlMode_("velocity"))
                    {
                        RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"), "Failed to update command mode on hardware");
                        return hardware_interface::return_type::ERROR;
                    }

                    RCLCPP_INFO(rclcpp::get_logger("CobotHardwareInterface"), "Deactivating all controllers, including the last running controller on the hardware");
                    control_mode_ = "";
                }
            }
        }
        else if (request_to_activate.size() == 1)
        {
            if (control_mode_ == "")
            {
                // switch to this mode
                if (!changeControlMode_(request_to_activate[0]))
                {
                    RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"), "Failed to update command mode on hardware");
                    return hardware_interface::return_type::ERROR;
                }

                control_mode_ = request_to_activate[0];
                RCLCPP_INFO(rclcpp::get_logger("CobotHardwareInterface"), "Activating the requesting control mode on hardware, mode : %s", control_mode_.c_str());
            }
            else
            {
                if (std::find(request_to_deactivate.begin(), request_to_deactivate.end(), control_mode_) != request_to_deactivate.end())
                {
                    // switch to this mode
                    if (!changeControlMode_(request_to_activate[0]))
                    {
                        RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"), "Failed to update command mode on hardware");
                        return hardware_interface::return_type::ERROR;
                    }

                    control_mode_ = request_to_activate[0];
                    RCLCPP_INFO(rclcpp::get_logger("CobotHardwareInterface"), "Activating the requesting control mode on hardware , mode : %s", control_mode_.c_str());
                }
                else
                {
                    RCLCPP_FATAL(rclcpp::get_logger("CobotHardwareInterface"), "You cannot not deactivate the current running controller and try to start another one. Aborting the attempt to switch control mode on hardware. Deactivate all running controllers and then activate only one controller which you wish to operate.");
                    return hardware_interface::return_type::ERROR;
                }
            }
        }

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
            const std::string &controller = common_interface_controller_pair.first;
            const std::vector<std::string> &conflicting_controllers = common_interface_controller_pair.second;

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
    bool CobotHWInterface::switchController_()
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eController;
        req.communicate = DataCommunicatorRequest::eController;

        DataValidatorContainer container;

        hw_interface_defs::ControlMode api = api_;

        if (api_.controller == API::eExternalTorqueAPI)
        {
            if (effort_controller_event_ == addverb_cobot::effortControllerEvents::eShouldActivate)
            {
                api.controller = API::eExternalTorqueAPI;
                effort_controller_event_ = addverb_cobot::effortControllerEvents::eNone;
            }
            else
            {
                api.controller = API::eExternalVelocityAPI;
                effort_controller_event_ = addverb_cobot::effortControllerEvents::eRequestedToActivate;
            }
        }

        container.controller = api;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief update controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::updateController_()
    {
        if (switchController_())
        {
            switchControlLoop_(api_.controller);
            return true;
        }

        return false;
    }

    /**
     * @brief switch the control loop execution based on the new controller
     *
     */
    void CobotHWInterface::switchControlLoop_(const API &api)
    {
        switch (api)
        {
        case API::eExternalVelocityAPI:
            control_loop_ = std::bind(&CobotHWInterface::extVelocity_, this);
            break;

        case API::eExternalTorqueAPI:
            control_loop_ = std::bind(&CobotHWInterface::extEffort_, this);
            break;

        case API::eLinearVelocityAPI:
            control_loop_ = std::bind(&CobotHWInterface::jointPtp_, this);
            break;

        case API::ePlayRecAPI:
            control_loop_ = std::bind(&CobotHWInterface::replay_, this);
            break;

        case API::eFreeDriveAPI:
            control_loop_ = std::bind(&CobotHWInterface::freeDrive_, this);
            break;

        case API::eJogJointAPI:
            control_loop_ = std::bind(&CobotHWInterface::jointJogging_, this);
            break;

        case API::eJogRPYAPI:
            control_loop_ = std::bind(&CobotHWInterface::cartesianJogging_, this);
            break;

        case API::eJointImpedanceAPI:
            control_loop_ = std::bind(&CobotHWInterface::jointImpedance_, this);
            break;

        case API::eCartesianImpedanceAPI:
            control_loop_ = std::bind(&CobotHWInterface::cartesianImpedance_, this);
            break;

        case API::eTcpMultiPointAPI:
            control_loop_ = std::bind(&CobotHWInterface::tcpPtp_, this);
            break;

        case API::eGCompExternalTorqueAPI:
            control_loop_ = std::bind(&CobotHWInterface::gravityCompExtEffort_, this);
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
        for (uint i = 0; i < info_.joints.size(); i++)
        {
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
        gripper_vel_ = 0.0;
        gripper_eff_ = 0.0;
        ptp_state_.reset();
    }

    /**
     * @brief initialise command interface variables
     *
     */
    void CobotHWInterface::initCmdVar_()
    {
        ptp_cmd_.reset();

        jvel_cmd_.cmd = std::vector<double>(n_dof, 0);
        jvel_cmd_.prev_cmd = std::vector<double>(n_dof, 0);

        jeffort_cmd_.cmd = std::vector<double>(n_dof, 0);
        jeffort_cmd_.prev_cmd = std::vector<double>(n_dof, 0);
        effortcmd_.fill(0.0);

        controller_name_cmd_ = std::vector<double>(n_controllers, 0);
        gripper_cmd_.reset();
    }

    /**
     * @brief setu the service node
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::setupServices_()
    {
        cobot_services_node_ = std::make_shared<CobotServices>();

        cobot_auxiliary_node_ = std::make_shared<CobotAuxiliary>();

        cobot_executor_ = std::make_shared<CobotExecutor>();

        if (!cobot_executor_->setup())
        {
            return false;
        }

        cobot_executor_->add_node(cobot_services_node_);
        cobot_executor_->add_node(cobot_auxiliary_node_);

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
        for (uint i = 0; i < info_.joints.size(); i++)
        {
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
    bool CobotHWInterface::extVelocity_()
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eVelocity;
        req.communicate = DataCommunicatorRequest::eVelocity;

        DataValidatorContainer container;

        hw_interface_defs::Velocity tmp_vel;

        for (int i = 0; i < 6; i++)
        {
            tmp_vel.cmd.push_back(vcmd_[i]);
        }
        container.velocity = tmp_vel;
        DataContainer cont;
        cont.validation_data = container;

        if (!container.velocity.has_value())
        {
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "velocity container empty");
        }

        // RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "about to write velocity");

        // for (int i = 0; i < 6; i++)
        // {
        // RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "expected joint %d - %lf", 0, vcmd_[0]);

        // RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "joint %d - %lf", i, container.velocity->cmd[i]);
        // }

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief run ptp controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::jointPtp_()
    {
        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(ptp_cmd_.transfer_cmd);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            multi_point_.reset();
            ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_.getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;

            multi_point_.addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_.getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi point trajectory");
                return false;
            }

            API api;

            if (multi_point_.getSize() == 1)
            {
                container.point = multi_point_.getPoint(0);
                api = API::eLinearVelocityAPI;
                req.convert = DataConverterRequest::ePoint;
                req.communicate = DataCommunicatorRequest::ePoint;
            }
            else if (multi_point_.getSize() > 1)
            {
                addBufferTime_();
                container.multi_point = multi_point_;
                api = API::eMultiPointAPI;
                req.convert = DataConverterRequest::eMultiPoint;
                req.communicate = DataCommunicatorRequest::eMultiPoint;
            }

            std::cout << "size of target : " << multi_point_.getSize() << std::endl;

            updateController_(api);

            if (!switchController_())
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API");
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont))
            {
                ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
        }

        return true;
    }

    /**
     * @brief run ptp_tcp controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::tcpPtp_()
    {
        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(tcp_ptp_cmd_.transfer_cmd);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            tcp_multi_point_.reset();
            tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::TcpPoint pt;
            pt.init();
            std::cout << "TCP point number : " << (tcp_multi_point_.getSize() + 1) << std::endl;
            for (int i = 0; i < 6; i++)
            {
                pt.pose[i] = tcp_command_[i];
                std::cout << pt.pose[i] << "\t";
            }
            pt.delta_t = tcp_ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;
            tcp_multi_point_.addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {

            DataProcessorRequest req;
            DataValidatorContainer container;
            req.validate = DataValidatorRequest::eNone;

            if (tcp_multi_point_.getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi-point trajectory");
                return false;
            }

            API api;
            container.tcp_multi_point = tcp_multi_point_;
            api = API::eTcpMultiPointAPI;
            req.convert = DataConverterRequest::eTcpMultipoint;
            req.communicate = DataCommunicatorRequest::eTcpMultipoint;

            std::cout << "size of target : " << tcp_multi_point_.getSize() << std::endl;
            updateController_(api);

            if (!switchController_())
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to TCP PTP API");
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont))
            {
                tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            tcp_ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
        }

        return true;
    }

    /**
     * @brief run play recording
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::replay_()
    {
        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(ptp_cmd_.transfer_cmd);

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_.getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;

            multi_point_.addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_.getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi point trajectory");
                return false;
            }

            hw_interface_defs::ReplayConfig replay_config;

            replay_config.points = multi_point_;
            replay_config.iterations = static_cast<int>(replay_iterations_cmd_);

            container.replay_config = replay_config;

            req.convert = DataConverterRequest::eReplay;
            req.communicate = DataCommunicatorRequest::eReplay;

            std::cout << "size of target : " << multi_point_.getSize() << std::endl;
            API api = API::ePlayRecAPI;

            updateController_(api);
            recorder_switched_ = false;

            if (!switchController_())
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Replay API");
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;
            if (!handleFwdRequest_(req, cont))
            {
                return false;
            }

            ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            multi_point_.reset();
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle || cur_transfer_cmd == TransferCommand::eNone)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);

            if (!recorder_switched_)
            {
                API api = API::eFreeDriveAPI;
                updateController_(api);

                if (!switchController_())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Replay API");
                    return false;
                }

                recorder_switched_ = true;
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
    bool CobotHWInterface::freeDrive_()
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
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eReadFeedback;
        req.communicate = DataCommunicatorRequest::eReadFeedback;

        DataCommunicatorContainer container;
        DataContainer cont;

        cont.communication_data = container;

        if (!handleBwdRequest_(req, cont))
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to fetch robot data");
            return false;
        }

        if (!cont.convert_data.has_value())
        {
            return false;
        }

        // read state from robot feedback
        //  todo : put these to robot_state_
        for (int i = 0; i < n_dof; i++)
        {
            hw_state_jpos_[i] = cont.convert_data->hw_robot_feedback->jpos[i];
            hw_state_jvel_[i] = cont.convert_data->hw_robot_feedback->jvel[i];
            hw_state_jtor_[i] = cont.convert_data->hw_robot_feedback->jtor[i];
        }

        updateFTData_(cont);
        updateEEPosData_(cont);

        has_jinfo_ = true;

        return true;
    }

    /**
     * @brief update FT data
     *
     * @param container
     */
    void CobotHWInterface::updateFTData_(const DataContainer &container)
    {
        for (int i = 0; i < 6; i++)
        {
            hw_ft_feedback_[i] = container.convert_data->hw_robot_feedback->ft_data[i];
        }

        cobot_auxiliary_node_->updateFTData(hw_ft_feedback_);
    }

    /**
     * @brief update EE pos data
     *
     * @param container
     */
    void CobotHWInterface::updateEEPosData_(const DataContainer &container)
    {
        for (int i = 0; i < 6; i++)
        {
            hw_ee_pos_feedback_[i] = container.convert_data->hw_robot_feedback->ee_pos_data[i];
        }

        cobot_auxiliary_node_->updateEEPosData(hw_ee_pos_feedback_);
    }

    /**
     * @brief get current robot state
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::getRobotState_()
    {
        DataProcessorRequest req;

        req.convert = DataConverterRequest::eReadState;
        req.communicate = DataCommunicatorRequest::eReadState;

        DataCommunicatorContainer container;
        DataContainer cont;

        cont.communication_data = container;

        if (!handleBwdRequest_(req, cont))
        {
            RCLCPP_FATAL(rclcpp::get_logger("CobotHWInterface"), "Failed to fetch robot data");
            return false;
        }

        if (!cont.convert_data.has_value())
        {
            return false;
        }

        robot_state_ = cont.convert_data->robot_state.value();

        robot_status_ = 1.0 * (static_cast<int>(robot_state_));

        // Simulate gripper movement to calculate live velocity
        double target_gripper_pos = (gripper_cmd_.position == 1) ? 0.0 : 0.725;
        double diff = target_gripper_pos - gripper_pos_;
        
        if (std::abs(diff) > 1e-4) {
            // Moving
            if (diff > 0) {
                gripper_vel_ = 1.0; // Opening
                gripper_eff_ = 0.5;
            } else {
                gripper_vel_ = -1.0; // Closing
                gripper_eff_ = -0.5;
            }
            // Update position (assuming 100Hz update rate -> dt = 0.01s)
            gripper_pos_ += gripper_vel_ * 0.01;
            
            // Prevent overshooting
            if ((gripper_vel_ > 0 && gripper_pos_ > target_gripper_pos) ||
                (gripper_vel_ < 0 && gripper_pos_ < target_gripper_pos)) {
                gripper_pos_ = target_gripper_pos;
            }
        } else {
            // At rest
            gripper_pos_ = target_gripper_pos;
            gripper_vel_ = 0.0;
        }

        return true;
    }

    /**
     * @brief run external effort controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::extEffort_()
    {
        DataProcessorRequest req;
        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eEffort;
        req.communicate = DataCommunicatorRequest::eEffort;

        DataValidatorContainer container;
        std::vector<double> temp_cmd(addverb_cobot::n_dof, 0.0);

        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            temp_cmd[i] = effortcmd_[i];
        }

        jeffort_cmd_.cmd = temp_cmd;

        if (effort_controller_event_ == addverb_cobot::effortControllerEvents::eRequestedToActivate)
        {
            if (effort_controller_status_ != addverb_cobot::effortControllerStatus::eReady)
            {
                if (!goToBase_())
                {
                    return false;
                }

                effort_controller_status_ = addverb_cobot::effortControllerStatus::eReady;

                return true;
            }

            error_codes validation_result = data_validator_->validateRequest(jeffort_cmd_);
            /// TODO: move this logic to controller itself
            if (validation_result == error_codes::NoError)
            {
                API api;
                api = API::eExternalTorqueAPI;

                updateController_(api);

                {
                    container.effort = jeffort_cmd_;
                    DataContainer cont;

                    cont.validation_data = container;

                    if (!container.effort.has_value())
                    {
                        RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "effort container empty");
                    }
                    jeffort_cmd_.prev_cmd = temp_cmd;

                    if (!handleFwdRequest_(req, cont))
                    {
                        return false;
                    }
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));

                effort_controller_event_ = addverb_cobot::effortControllerEvents::eShouldActivate;

                if (!switchController_())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to External Torque API");
                    return false;
                }
            }
            else
            {
                return true;
            }

            effort_controller_status_ = addverb_cobot::effortControllerStatus::eActivate;
        }

        if (effort_controller_status_ == addverb_cobot::effortControllerStatus::eActivate)
        {
            container.effort = jeffort_cmd_;
            DataContainer cont;

            cont.validation_data = container;

            if (!container.effort.has_value())
            {
                RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "effort container empty");
            }

            jeffort_cmd_.prev_cmd = temp_cmd;

            return handleFwdRequest_(req, cont);
        }

        return true;
    }

    bool CobotHWInterface::gravityCompExtEffort_()
    {
        DataProcessorRequest req;
        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eEffort;
        req.communicate = DataCommunicatorRequest::eEffort;

        DataValidatorContainer container;
        std::vector<double> temp_cmd(addverb_cobot::n_dof, 0.0);

        for (int i = 0; i < addverb_cobot::n_dof; i++)
        {
            temp_cmd[i] = effortcmd_[i];
        }

        jeffort_cmd_.cmd = temp_cmd;

        container.effort = jeffort_cmd_;
        DataContainer cont;

        cont.validation_data = container;

        if (!container.effort.has_value())
        {
            RCLCPP_INFO(rclcpp::get_logger("CobotHWInterface"), "effort container empty");
        }
        jeffort_cmd_.prev_cmd = temp_cmd;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief run joint jogging controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::jointJogging_()
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eJointJogging;
        req.communicate = DataCommunicatorRequest::eJointJogging;

        DataValidatorContainer container;

        hw_interface_defs::JointJog tmp_jj;

        for (int i = 0; i < n_dof; i++)
        {
            tmp_jj.jog_cmd.cmd[i] = joint_jogging_cmd_.jog_cmd.cmd[i];
        }

        joint_jogging_cmd_.reset();

        container.joint_jogging = tmp_jj;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief run cartesian jogging controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::cartesianJogging_()
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eCartesianJogging;
        req.communicate = DataCommunicatorRequest::eCartesianJogging;

        DataValidatorContainer container;

        hw_interface_defs::CartesianJog tmp_jj;

        for (int i = 0; i < 6; i++)
        {
            tmp_jj.jog_cmd.cmd[i] = cartesian_jogging_cmd_.jog_cmd.cmd[i];
        }

        cartesian_jogging_cmd_.reset();

        container.cartesian_jogging = tmp_jj;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief run joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::jointImpedance_()
    {
        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(ptp_cmd_.transfer_cmd);
        TransferState cur_transfer_state = static_cast<TransferState>(ptp_state_.transfer_state);

        updateJointImpedance_();

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            multi_point_.reset();
            ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_.getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;

            multi_point_.addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_.getSize() <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "No points were added to the multi point trajectory");
                return false;
            }

            API api;

            if (multi_point_.getSize() == 1)
            {
                container.point = multi_point_.getPoint(0);
                api = API::eJointImpedanceAPI;
                req.convert = DataConverterRequest::ePoint;
                req.communicate = DataCommunicatorRequest::ePoint;
            }
            else if (multi_point_.getSize() > 1)
            {
                container.multi_point = multi_point_;
                api = API::eMultiJointImpedanceAPI;
                req.convert = DataConverterRequest::eMultiPoint;
                req.communicate = DataCommunicatorRequest::eMultiPoint;
            }

            std::cout << "size of target : " << multi_point_.getSize() << std::endl;

            updateController_(api);

            if (!switchController_())
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API");
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont))
            {
                ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
        }

        return true;
    }

    /**
     * @brief run joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::cartesianImpedance_()
    {
        TransferCommand cur_transfer_cmd = static_cast<TransferCommand>(ptp_cmd_.transfer_cmd);
        TransferState cur_transfer_state = static_cast<TransferState>(ptp_state_.transfer_state);

        updateCartesianImpedance_();

        if (cur_transfer_cmd == TransferCommand::eRcdNewTraj)
        {
            multi_point_.reset();
            ptp_state_.transfer_state = static_cast<double>(TransferState::eWaitingForPoint);
        }
        else if (cur_transfer_cmd == TransferCommand::eTransferring)
        {
            hw_interface_defs::Point pt;
            pt.init();

            std::cout << "point number : " << (multi_point_.getSize() + 1) << std::endl;
            for (int i = 0; i < n_dof; i++)
            {
                pt.jpos[i] = ptp_cmd_.target.jpos[i];
                std::cout << pt.jpos[i] << "\t";
            }
            pt.delta_t = ptp_cmd_.target.delta_t;
            std::cout << "in time " << pt.delta_t << std::endl;

            multi_point_.addPoint(pt);
        }
        else if (cur_transfer_cmd == TransferCommand::ePublish)
        {
            DataProcessorRequest req;
            DataValidatorContainer container;

            req.validate = DataValidatorRequest::eNone;

            if (multi_point_.getSize() != 1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Wrong number of points were added for the cartesian impedance controller trajectory.");
                return false;
            }

            API api;

            container.point = multi_point_.getPoint(0);
            api = API::eCartesianImpedanceAPI;
            req.convert = DataConverterRequest::ePoint;
            req.communicate = DataCommunicatorRequest::ePoint;

            std::cout << "size of target : " << multi_point_.getSize() << std::endl;

            updateController_(api);

            if (!switchController_())
            {
                RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API");
                return false;
            }

            DataContainer cont;
            cont.validation_data = container;

            if (!handleFwdRequest_(req, cont))
            {
                ptp_state_.transfer_state = static_cast<double>(TransferState::eRejected);
            }
            else
            {
                ptp_state_.transfer_state = static_cast<double>(TransferState::ePublished);
            }
        }
        else if (cur_transfer_cmd == TransferCommand::eExecute)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eExecuting);
        }
        else if (cur_transfer_cmd == TransferCommand::eIdle)
        {
            ptp_state_.transfer_state = static_cast<double>(TransferState::eIdling);
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
        DataProcessorRequest req;
        DataValidatorContainer container;

        hw_interface_defs::Point pt;

        pt.jpos = addverb_cobot::base_config_jpos;
        pt.delta_t = addverb_cobot::base_config_reach_time;

        container.point = pt;
        API api = API::eLinearVelocityAPI;
        req.convert = DataConverterRequest::ePoint;
        req.communicate = DataCommunicatorRequest::ePoint;

        updateController_(api);

        if (!switchController_())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CobotHWInterface"), "Failed to switch controller to Linear Velocity API");
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
     * @brief update allied input for joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::updateJointImpedance_()
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eJointImpedance;
        req.communicate = DataCommunicatorRequest::eJointImpedance;

        DataValidatorContainer container;

        hw_interface_defs::JointImpedance tmp_jimp;

        tmp_jimp = joint_impedance_cmd_;

        container.joint_impedance = tmp_jimp;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief update allied input for joint impedance controller
     *
     * @return true
     * @return false
     */
    bool CobotHWInterface::updateCartesianImpedance_()
    {
        DataProcessorRequest req;

        req.validate = DataValidatorRequest::eNone;
        req.convert = DataConverterRequest::eCartesianImpedance;
        req.communicate = DataCommunicatorRequest::eCartesianImpedance;

        DataValidatorContainer container;

        hw_interface_defs::CartesianImpedance tmp_cimp;

        tmp_cimp = cartesian_impedance_cmd_;

        container.cartesian_impedance = tmp_cimp;
        DataContainer cont;
        cont.validation_data = container;

        return handleFwdRequest_(req, cont);
    }

    /**
     * @brief change control mode
     *
     * @param new_mode
     * @return true
     * @return false
     */
    bool CobotHWInterface::changeControlMode_(const std::string &new_mode)
    {
        if (!validateController_(new_mode))
        {
            std::cout << "invalid mode\n";
            return false;
        }

        if (!updateController_())
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
    void CobotHWInterface::addBufferTime_()
    {
        if (multi_point_.points[0].delta_t == 0.0)
        {
            multi_point_.points[0].delta_t = buffer_time_;
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
        if (static_cast<GripperTransferCommand>(gripper_cmd_.transfer_cmd) == GripperTransferCommand::eHasCommand)
        {
            DataProcessorRequest req;

            req.validate = DataValidatorRequest::eNone;
            req.convert = DataConverterRequest::eGripperCmd;
            req.communicate = DataCommunicatorRequest::eGripperCmd;

            DataValidatorContainer container;
            addverb_cobot::hw_interface_defs::GripperCmd tmp_gripper_cmd_ = gripper_cmd_;
            container.gripper_cmd = tmp_gripper_cmd_;

            DataContainer cont;
            cont.validation_data = container;

            return handleFwdRequest_(req, cont);
        }

        return true;
    }
};

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    addverb_cobot::CobotHWInterface, hardware_interface::SystemInterface)
