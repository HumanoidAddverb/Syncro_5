#include "addverb_cobot_controllers/recorder_controller.h"

namespace addverb_cobot_controllers
{
    /**
     * @brief initialise the Recorder controller
     *
     * @param 
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn RecorderController::on_init()
    {
        RCLCPP_INFO(rclcpp::get_logger("RecorderController"), "Initialised recorder controller");
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief configure controller 
     * Initialise state and command interface 
     * Set up record and replay service 
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn RecorderController::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
      try
      {
        recorded_jpos_.resize(addverb_cobot::n_dof, 0.0);
        
        data_validator_ = std::make_unique<addverb_cobot::DataValidator>(get_node()->get_logger());
        states_ =  get_node()->get_parameter("states").as_string_array();
        commands_ = get_node()->get_parameter("commands").as_string_array();
      }
      catch(...)
      {
        RCLCPP_WARN(rclcpp::get_logger("RecorderController"), "on configure: Controller input/output param not found. Both expected");
        return LifecycleNodeInterface::CallbackReturn::ERROR;
      }
    
      RCLCPP_INFO(rclcpp::get_logger("RecorderController"), "Configured free-drive controller");
      
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    /**
     * @brief configure controller command interface (play command interface)
     *
     * @param 
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration RecorderController::command_interface_configuration() const
    {
      controller_interface::InterfaceConfiguration config;
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      config.names = commands_;

      return config;
    }

    /**
     * @brief configure controller state interface (recording state interface)
     *
     * @param 
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration RecorderController::state_interface_configuration() const
    {
      controller_interface::InterfaceConfiguration config;
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      config.names = states_;
    
      return config;
    }
    
    /**
     * @brief activate controller callback
     *
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn RecorderController::on_activate(const rclcpp_lifecycle::State&)
    {
        try
        {
            get_node()->get_parameter_or("max_recording_rate", max_recording_rate_, 30);
    
            get_node()->get_parameter_or("position_tolerance", position_tolerance_, 0.001);
            get_node()->get_parameter_or("time_tolerance", time_tolerance_, 15.0);
        
            record_srv_ = get_node()->create_service<RecordSrv>(std::string(get_node()->get_name()) + 
                    "/recorder_controller/record_mode",
                    std::bind(&RecorderController::recorderCallback_, this, _1, _2));
    
            replay_action_srv_ = rclcpp_action::create_server<ReplayAction>(this->get_node(),  std::string(get_node()->get_name()) + 
                    "/recorder_controller/replay_mode", 
                    std::bind(&RecorderController::handleReplayGoal_, this, _1, _2), 
                    std::bind(&RecorderController::handleReplayCancel_, this, _1),
                    std::bind(&RecorderController::handleReplayAccepted_, this, _1));
    
            is_active_.store(true);
        }
        catch(...)
        {
          RCLCPP_WARN(rclcpp::get_logger("RecorderController"), "failed to activate");
          return LifecycleNodeInterface::CallbackReturn::ERROR;
        }


        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief deactivate controller callback
     * resets recording service and stop recording if its running
     * @param previous_state
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn RecorderController::on_deactivate(const rclcpp_lifecycle::State&)
    {
        std::lock_guard<std::mutex> lock(recorder_mutex_);
        try
        {
            record_srv_.reset();
            replay_action_srv_.reset();
    
            if (enable_recording_)
            { 
                enable_recording_ = false;
                data_store_.closeBucket();
            }

            is_active_.store(false);
        }
        catch(...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("RecorderController"), "Deactivated free-drive controller");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief update controller callback
     *
     * @param time 
     * @param period
     * @return controller_interface::return_type
     */
    controller_interface::return_type RecorderController::update(const rclcpp::Time&, const rclcpp::Duration&) 
    {

        RobotState robot_state = static_cast<RobotState>(state_interfaces_[7].get_value());

        if (robot_state == RobotState::eError)
        {
            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eIdle);
            command_interfaces_[7].set_value(cmd);

            if (robot_in_error_.load(std::memory_order_acquire))
            {
                {
                    std::lock_guard<std::mutex> guard(recorder_mutex_);
        
                    ongoing_prev_traj_ = false;
                    has_new_goal_ = false;
                }
                return controller_interface::return_type::OK;
            }

            robot_in_error_.store(true, std::memory_order_release);

            RCLCPP_ERROR(get_node()->get_logger(), "Robot gone into error. Call ErrorRecoveryService to safely recover from the error and get into the home position.");
            return controller_interface::return_type::OK;
        }
        else
        {
            if (robot_in_error_.load(std::memory_order_acquire))
            {
                robot_in_error_.store(false, std::memory_order_release);

                RCLCPP_INFO(get_node()->get_logger(), "Robot recovered from error. you can now start commanding the robot.");
            }
        }

        bool has_new_goal = false;
        bool ongoing_prev_traj = false;
        bool enable_recording = false;

        {
            std::lock_guard<std::mutex> guard(recorder_mutex_);

            ongoing_prev_traj = ongoing_prev_traj_;
            enable_recording = enable_recording_;
            has_new_goal = has_new_goal_;
        }

        if (enable_recording)
        {
            record_();
        }

        if (has_new_goal)
        {
            replayExecute_();
        }

        if (ongoing_prev_traj)
        {
           
            sendFeedback_();
        }

        return controller_interface::return_type::OK;
    }

    /**
     * @brief Recoder service callback 
     * Enables or disables recording of joint states
     * label: name of csv file (string)
     * rate: frequency at which recording to occur (int)
     * enable: to enable or disable recording (true/false)
     * @param request
     * @param response
     * @return 
     */
    void RecorderController::recorderCallback_(const std::shared_ptr<RecordSrv::Request> request,
        std::shared_ptr<RecordSrv::Response> response)
    {
        std::string msg;
        bool status;

        std::string label = request->label;
        int rate = request->rate;

        std::lock_guard<std::mutex> lock(recorder_mutex_);

        if (!request->enable)
        {
            enable_recording_ = false;            
            
            if(!data_store_.closeBucket())
            {
                msg = "Failed to save recording";
                status = false;
            }
            else
            {
                msg = "Stopped and saved recording";
                status = true;
            }
        }
        else if (enable_recording_)
        {
            msg = "Previous recording is still active ...";
            status = false;

            RCLCPP_WARN(rclcpp::get_logger("RecorderController"), 
                        "Please stop previous recording before starting new one ...");
        }
        else if (has_new_goal_ || ongoing_prev_traj_)
        {
            msg = "Previous trajectory is still getting executed ...";
            status = false;
            RCLCPP_WARN(rclcpp::get_logger("RecorderController"), 
                "Previous trajectory is still getting executed ...");
        }
        else
        {
            if(!data_store_.initBucket(label))
            {
                RCLCPP_WARN(rclcpp::get_logger("RecorderController"), 
                        "File with the label already exists ...");

                msg = "Failed to initialse data storage.";
                status = false;
            }
            else
            {
                recording_rate_ = (rate > max_recording_rate_) ? max_recording_rate_ : rate;
            
                resetTime_();

                enable_recording_ = true; 
                msg = "Started recording with file name: " + label;
                status = true;
            }
        }

        response->message = msg;
        response->status = status;
    }

    /**
     * @brief recording method 
     * takes joint position from state interface
     * and records a certain user specifiec frequency
     * @param 
     * @return 
     */
    void RecorderController::record_()
    {
        double duration = timeDiff_();
        if (duration >= wait_time_)
        {
            int entry_no = data_store_.getNumOfEntries() + 1;
    
            for(int i = 0; i < 6; i++)
            {
                recorded_jpos_[i] = state_interfaces_.at(i).get_value();
            }
            
            if(!data_store_.store(recorded_jpos_, 
                    (double(1) / recording_rate_) * entry_no ))
            {
                RCLCPP_INFO(rclcpp::get_logger("RecorderController"), 
                    "Failed to record joint position");
            } 
        }
    }

    /**
     * @brief replay goal 
     * takes replay action as goal
     * @param goal
     * @return rclcpp_action::GoalResponse 
     */
    rclcpp_action::GoalResponse RecorderController::handleReplayGoal_(const rclcpp_action::GoalUUID &uuid, 
                                            std::shared_ptr<const ReplayAction::Goal> goal)
    {
        if (robot_in_error_.load(std::memory_order_acquire))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Robot gone into error. Recover from error before attempting to give new target.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!goal)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Goal handle is null!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(rclcpp::get_logger("RecorderController"), "Received goal request");
        
        {
            std::lock_guard<std::mutex> lock(recorder_mutex_);
            if (enable_recording_)
            {
                enable_recording_ = false;
                data_store_.closeBucket();
                RCLCPP_INFO(rclcpp::get_logger("RecorderController"), "Recording Stopped ...");
            }
    
            if (ongoing_prev_traj_ || has_new_goal_)
            {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        if (!validateReplayLabel_(goal->label))
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief Cancel replay
     * takes replay action as goal
     * @param goal
     * @return rclcpp_action::GoalResponse 
     */
    rclcpp_action::CancelResponse RecorderController::handleReplayCancel_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayAction>> goal_handle)
    {
        // logic for cancelling goal 
        // cancel_replay_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief replay callback accept goal 
     * @param goal
     * @return rclcpp_action::GoalResponse 
     */
    void RecorderController::handleReplayAccepted_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ReplayAction>> goal_handle)
    {
        auto goal = goal_handle->get_goal();

        {
            std::lock_guard<std::mutex> lock(recorder_mutex_);

            data_store_.retreiveData(goal->label, replay_trajectory_, time_seq_);

            /// total requested iterations
            curr_iterations_ = goal->iterations;

            if (!validateReplayRequest_())
            {
                return;
            }

            has_new_goal_ = true;
            ongoing_prev_traj_ = true;

            /// counter to track current executing iteration 
            curr_iteration_ = 1;
            
            /// goal handle
            goal_handle_ = goal_handle; 
        }
    
        RCLCPP_INFO(rclcpp::get_logger("RecorderController"), "Goal accpeted");
    }

    /**
     * @brief replay callback accept goal 
     * @param goal
     * @return rclcpp_action::GoalResponse 
     */
    void RecorderController::replayExecute_()
    {
        addverb_cobot::TransferState transfer_state = static_cast<addverb_cobot::TransferState>(state_interfaces_[6].get_value());
        if (transfer_state == addverb_cobot::TransferState::eNone || transfer_state == addverb_cobot::TransferState::eIdling)
        {
            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eRcdNewTraj);
            command_interfaces_[8].set_value(curr_iterations_);
            command_interfaces_[7].set_value(cmd);
        }
        else if (transfer_state == addverb_cobot::TransferState::eWaitingForPoint)
        {
            if (curr_index_ < time_seq_.size())
            {
                // Send the next trajectory point to the hardware interface
                for (size_t j_idx = 0; j_idx < replay_trajectory_[curr_index_].size(); j_idx++)
                {
                    const auto &jpos = replay_trajectory_[curr_index_][j_idx];
                    command_interfaces_[j_idx].set_value(jpos);
                }

                command_interfaces_[6].set_value(time_seq_[curr_index_]);

                double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eTransferring);
                command_interfaces_[7].set_value(cmd);

                // Update the index for the next point
                curr_index_++;
                
            }
            else
            {
                // All points have been sent, mark the transfer as done
                double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::ePublish);
                command_interfaces_[7].set_value(cmd);
                curr_index_ = 0;
            }
        }
        else if (transfer_state == addverb_cobot::TransferState::ePublished)
        {
            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eExecute);
            command_interfaces_[7].set_value(cmd);

            {
                std::lock_guard<std::mutex> lock(recorder_mutex_);

                has_new_goal_ = false;
                ongoing_prev_traj_ = true;
            }

            trajectory_start_time_ = std::chrono::high_resolution_clock::now();
        }
    } 
    
    /**
     * @brief feedback 
     * @param 
     * @return 
     */
    void RecorderController::sendFeedback_()
    {
        addverb_cobot::TransferState transfer_state =
        static_cast<addverb_cobot::TransferState>(state_interfaces_[6].get_value());
    
        if (transfer_state != addverb_cobot::TransferState::eExecuting)
        {
            // only provide feedback while executing
            return;
        }

        int num_points = replay_trajectory_.size();

        int last_point_idx = num_points - 1;
    
        // Compute elapsed time (seconds, double) since start of execution iteration
        double dt = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - trajectory_start_time_).count();
    
        // total time assigned to current iteration
        double target_time = time_seq_[last_point_idx] * static_cast<double>(curr_iteration_);
    
        // evaluate completion/tolerance
        if (dt >= (target_time + time_tolerance_))
        {
            bool should_allow_reset = false;
            
    
            bool position_within_tolerance = true;
            int exceed_joint_index = -1;
    
            // Check final point positions (use last_point_idx, and iterate joints)
            for (int j = 0; j < addverb_cobot::n_dof; j++)
            {
                double actual_position = state_interfaces_[j].get_value();
                double target_position = replay_trajectory_[last_point_idx][j];
    
                if (std::abs(actual_position - target_position) > position_tolerance_)
                {
                    position_within_tolerance = false;
                    exceed_joint_index = j;
                    break;
                }
            }
    
            if (!position_within_tolerance)
            {
                RCLCPP_WARN_STREAM(get_node()->get_logger(),
                    "Position tolerance exceeded for joint: " << exceed_joint_index << ". Robot will still try to reach the target.");
            }
            else 
            {
                curr_iteration_++;
            }
    
            // If all requested iterations are done, send result and prepare to reset
            if (curr_iteration_ > curr_iterations_)
            {
                should_allow_reset = true;
                auto result = std::make_shared<ReplayAction::Result>();
                result->error_code = position_within_tolerance ? "SUCCESSFUL" : "FAILURE";
    
                if (goal_handle_)
                {
                    goal_handle_->succeed(result);
                    goal_handle_.reset();
                }
                else
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Goal handle not available when trying to send final result");
                }
            }
    
            if (should_allow_reset)
            {
                { // set flags under lock
                    std::lock_guard<std::mutex> guard(recorder_mutex_);
                    ongoing_prev_traj_ = false;
                }
    
                // Reset timing and send idle command to transfer command interface
                double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eIdle);
                command_interfaces_[7].set_value(cmd);

                // Clear stored trajectory
                replay_trajectory_.clear();
                time_seq_.clear();
    
                // Reset indices
                curr_index_ = 0;
                curr_iteration_ = 0;
            }
        }
        else
        {
            if (goal_handle_)
            {
                auto feedback = std::make_shared<ReplayAction::Feedback>();
                feedback->joint_state.resize(addverb_cobot::n_dof);
    
                for (size_t j = 0; j < addverb_cobot::n_dof; j++)
                {
                    feedback->joint_state[j] = state_interfaces_[j].get_value();
                }
                feedback->iteration = curr_iteration_;
    
        
                goal_handle_->publish_feedback(feedback);
    
                RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Published feedback for iteration " << curr_iteration_);
            }
        }
    }

    /**
     * @brief validate replay request
     * @param 
     * @return 
     */    
    bool RecorderController::validateReplayRequest_()
    {
        addverb_cobot::hw_interface_defs::ReplayConfig replay_config;
    
        for (int i = 0; i < time_seq_.size(); i++)
        {
            addverb_cobot::hw_interface_defs::Point pt;

            for (int j = 0; j < N_DOF; j++)
            {
                std::cout << replay_trajectory_[i][j] << std::endl;
                pt.jpos.push_back(replay_trajectory_[i][j]);
            }

            pt.delta_t = time_seq_[i];

            replay_config.points.addPoint(pt);
        }

        std::cout << "number of iterations requested" << curr_iterations_ << std::endl;

        replay_config.iterations = curr_iterations_;

        addverb_cobot::error_codes validation_result = data_validator_->validateRequest(replay_config);

        if (validation_result != addverb_cobot::error_codes::NoError)
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Replay goal validation failed for a trajectory point. Error code: %d",
                    static_cast<int>(validation_result));
                return false; 
            }

        return true;
    }

    bool RecorderController::validateReplayLabel_(const std::string &label)
    {
        if (!data_store_.isIDInStore(label))
        {
            RCLCPP_WARN(rclcpp::get_logger("ReplayController"), "Provided file label does not exists");
            return false;
        }

        return true;
    }
    
    /**
     * @brief recording method 
     * to reset time 
     * @param 
     * @return 
     */
    void RecorderController::resetTime_()
    {
        wait_time_ = std::chrono::seconds(static_cast<int>((1.0/recording_rate_))).count();
        start_time_ = std::chrono::duration_cast<std::chrono::seconds>(
            clock::now().time_since_epoch()).count();

        current_time_ = wait_time_ + start_time_;
    }

    /**
     * @brief recording method for calculating 
     * time difference
     * @param 
     * @return 
     */
    double RecorderController::timeDiff_()
    {

        current_time_ = std::chrono::duration_cast<std::chrono::seconds>(
            clock::now().time_since_epoch()).count();

        double duration = current_time_ - start_time_;

        start_time_ = current_time_;
        return duration;   
    }
} /// namespace addverb_cobot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  addverb_cobot_controllers::RecorderController, controller_interface::ControllerInterface)





