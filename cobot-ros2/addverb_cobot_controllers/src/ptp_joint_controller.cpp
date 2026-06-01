#include "addverb_cobot_controllers/ptp_joint_controller.h"

namespace addverb_cobot_controllers
{

    /**
     * @brief Initializes the PTPJointController.
     *
     * This method is called during the initialization phase of the controller's lifecycle.
     * It attempts to retrieve the "commands" and "states" parameters from the node and stores
     * them for later use in interface configuration. If any exception occurs during parameter
     * retrieval, a warning is logged and the initialization is marked as failed.
     *
     * @return controller_interface::CallbackReturn::SUCCESS if initialization succeeds,
     *         controller_interface::CallbackReturn::ERROR otherwise.
     */

    controller_interface::CallbackReturn PTPJointController::on_init()
    {
        try
        {
            data_validator_ = std::make_unique<addverb_cobot::DataValidator>(get_node()->get_logger());
            commands_ = get_node()->get_parameter("commands").as_string_array();
            states_ = get_node()->get_parameter("states").as_string_array();
        }
        catch (...)
        {
            RCLCPP_WARN(get_node()->get_logger(), "EXCEPTION raised in initialising ptp_joint_controller");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Configures the PTPJointController.
     *
     * This method is called during the configuration phase of the controller's lifecycle.
     * It sets up the action server for handling joint trajectory commands and clears any
     * existing command queue. If any exception occurs during setup, an error is logged and
     * the configuration is marked as failed.
     *
     * @return controller_interface::CallbackReturn::SUCCESS if configuration succeeds,
     *         controller_interface::CallbackReturn::ERROR otherwise.
     */
    controller_interface::CallbackReturn PTPJointController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Activates the PTPJointController.
     *
     * This method is called during the activation phase of the controller's lifecycle.
     * It currently does not perform any specific actions but can be extended in the future.
     *
     * @return controller_interface::CallbackReturn::SUCCESS if activation succeeds,
     *         controller_interface::CallbackReturn::ERROR otherwise.
     */
    controller_interface::CallbackReturn PTPJointController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Activate action server
        try
        {
            server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                this->get_node(),
                std::string(get_node()->get_name()) + "/follow_joint_trajectory",
                std::bind(&PTPJointController::handleGoal_, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&PTPJointController::handleCancel_, this, std::placeholders::_1),
                std::bind(&PTPJointController::handleAccepted_, this, std::placeholders::_1));
        }
        catch (...)
        {
            RCLCPP_INFO(get_node()->get_logger(), "EXCEPTION raised in configuring ptp_joint_controller");

            return controller_interface::CallbackReturn::ERROR;
        }
        // todo : add validation of sequence of command interfaces and state interfaces

        RCLCPP_INFO(get_node()->get_logger(), "activate successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Deactivates the PTPJointController.
     */
    controller_interface::CallbackReturn PTPJointController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            server_.reset();
            goal_handle_ = nullptr;

            {
                std::lock_guard<std::mutex> g(traj_update_mutex_);
                has_new_goal_ = false;
            }
        }
        catch (...)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief specify the command interfaces to claim
     */
    controller_interface::InterfaceConfiguration PTPJointController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = commands_;
        return config;
    }

    /**
     * @brief specify the state interfaces to claim
     */
    controller_interface::InterfaceConfiguration PTPJointController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = states_;
        return config;
    }

    /**
     * @brief update the goal to hardware interface
     *
     */
    controller_interface::return_type PTPJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        RobotState robot_state = static_cast<RobotState>(state_interfaces_[7].get_value());

        if (robot_state == RobotState::eError)
        {
            if (!robot_in_error_.load(std::memory_order_acquire))
            {
                robot_in_error_.store(true, std::memory_order_release);

                RCLCPP_ERROR(get_node()->get_logger(), "Robot gone into error. Call ErrorRecoveryService to safely recover from the error and get into the home position.");
            }
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

        {
            std::lock_guard<std::mutex> guard(traj_update_mutex_);
            has_new_goal = has_new_goal_;
            ongoing_prev_traj = ongoing_prev_traj_;
        }

        if (has_new_goal)
        {
            sendNextCmd_();
        }
        else if (ongoing_prev_traj)
        {
            updateFeedback_();
        }

        return controller_interface::return_type::OK;
    }

    /**
     * @brief handles a new goal
     *
     * @param goal
     * @return rclcpp_action::GoalResponse
     */
    rclcpp_action::GoalResponse PTPJointController::handleGoal_(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
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

        bool ongoing_prev_traj = false;
        bool has_new_goal = false;

        {
            std::lock_guard<std::mutex> guard(traj_update_mutex_);
            ongoing_prev_traj = ongoing_prev_traj_;
            has_new_goal = has_new_goal_;
        }

        if (ongoing_prev_traj || has_new_goal)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Previous trajectory is still being processed, rejecting new goal. Optionally, you can cancel the previous goal, to force the new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!validateGoal_(goal->trajectory))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Invalid PTP goal. Kindly correct the goal and try again.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->trajectory.points.size() == 1)
        {
            target_ = goal->trajectory.points[0].time_from_start.sec + goal->trajectory.points[0].time_from_start.nanosec / 1e9;
        }
        else
        {
            int last_idx = goal->trajectory.points.size() - 1;
            target_ = goal->trajectory.points[last_idx].time_from_start.sec + goal->trajectory.points[last_idx].time_from_start.nanosec / 1e9;
        }

        time_tolerance_ = goal->goal_time_tolerance.sec + goal->goal_time_tolerance.nanosec / 1e9;

        if (time_tolerance_ <= default_params::default_time_tolerance)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Incorrect time tolerance provided, using default value.");
            time_tolerance_ = default_params::default_time_tolerance;
            RCLCPP_INFO(get_node()->get_logger(), "Time tolerance set to: 1 sec 200000000 nanoseconds");
        }

        {
            std::lock_guard<std::mutex> g(traj_update_mutex_);
            has_new_goal_ = true;
        }

        current_goal_ = *goal;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief handle cancelling a goal
     *
     * @return rclcpp_action::CancelResponse
     */
    rclcpp_action::CancelResponse PTPJointController::handleCancel_(
        const std::shared_ptr<GoalHandleFollowJointTrajectory>)
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    /**
     * @brief handle execution of accepted goal
     *
     * @param goal_handle
     */
    void PTPJointController::handleAccepted_(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        std::lock_guard<std::mutex> guard(goal_handle_mutex_);
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Goal handle is null! Cannot accept goal.");
            throw std::runtime_error("Goal handle is null!");
        }

        goal_handle_ = goal_handle;
    }

    /**
     * @brief send the next command in queue to the hardware interface
     *
     */
    void PTPJointController::sendNextCmd_()
    {
        addverb_cobot::TransferState transfer_state = static_cast<addverb_cobot::TransferState>(state_interfaces_[6].get_value());

        if (transfer_state == addverb_cobot::TransferState::eNone || transfer_state == addverb_cobot::TransferState::eIdling)
        {
            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eRcdNewTraj);
            command_interfaces_[7].set_value(cmd);
        }
        else if (transfer_state == addverb_cobot::TransferState::eWaitingForPoint)
        {
            if (robot_in_error_.load(std::memory_order_acquire))
            {
                {
                    std::lock_guard<std::mutex> guard(goal_handle_mutex_);
                    if (goal_handle_)
                    {
                        auto result = std::make_shared<FollowJointTrajectory::Result>();
                        RCLCPP_INFO(get_node()->get_logger(), "Goal rejected as robot in error.");
                        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                        result->error_string = "Goal rejected as robot in error";
                        goal_handle_->abort(result);
                        goal_handle_.reset();
                    }
                }

                {
                    std::lock_guard<std::mutex> guard(traj_update_mutex_);
                    has_new_goal_ = false;
                }

                double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eIdle);
                command_interfaces_[7].set_value(cmd);
                cur_index_ = 0;
            }
            else
            {
                if (cur_index_ < current_goal_.trajectory.points.size())
                {
                    // Send the next trajectory point to the hardware interface
                    const auto &point = current_goal_.trajectory.points[cur_index_];

                    for (size_t i = 0; i < point.positions.size(); ++i)
                    {
                        command_interfaces_[i].set_value(point.positions[i]);
                    }

                    command_interfaces_[6].set_value(point.time_from_start.sec + (point.time_from_start.nanosec) / 1e9);

                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eTransferring);
                    command_interfaces_[7].set_value(cmd);

                    // Update the index for the next point
                    cur_index_++;
                }
                else
                {
                    // All points have been sent, mark the transfer as done
                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::ePublish);
                    command_interfaces_[7].set_value(cmd);
                    cur_index_ = 0;
                }
            }
        }
        else if (transfer_state == addverb_cobot::TransferState::eRejected)
        {
            {
                std::lock_guard<std::mutex> guard(goal_handle_mutex_);
                if (goal_handle_)
                {
                    auto result = std::make_shared<FollowJointTrajectory::Result>();
                    RCLCPP_INFO(get_node()->get_logger(), "Trajectory rejected because of infeasibility. Kindly re-attempt with feasible points.");
                    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                    result->error_string = "Trajectory rejected beacuse of infeasibility.";
                    goal_handle_->abort(result);
                    goal_handle_.reset();
                }
            }

            {
                std::lock_guard<std::mutex> guard(traj_update_mutex_);
                has_new_goal_ = false;
            }

            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eIdle);
            command_interfaces_[7].set_value(cmd);
        }
        else if (transfer_state == addverb_cobot::TransferState::ePublished)
        {
            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eExecute);
            command_interfaces_[7].set_value(cmd);

            {
                std::lock_guard<std::mutex> guard(traj_update_mutex_);
                has_new_goal_ = false;
                ongoing_prev_traj_ = true;
            }

            start_time_ = std::chrono::high_resolution_clock::now();
        }
    }

    /**
     * @brief validate the given goal
     */
    bool PTPJointController::validateGoal_(const trajectory_msgs::msg::JointTrajectory &goal)
    {
        if (goal.joint_names.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Goal validation failed: joint_names is empty.");
            return false;
        }

        if (goal.points.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Goal validation failed: trajectory points is empty.");
            return false;
        }
        double previous_time = 0.0;

        const auto &point = goal.points;

        if (point.size() > 1)
        {
            if (point.size() < 3)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Goal validation failed: number of trajectory points is less than 3. Give a minimum of 3 points for executing multi-point trajectory.");
                return false;
            }
        }

        for (int i = 0; i < point.size(); i++)
        {
            if (goal.joint_names.size() != point[i].positions.size())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Goal validation failed: Mismatch between joint_names size and positions size.");
                return false;
            }

            addverb_cobot::hw_interface_defs::PtP ptp_point_for_validation;
            ptp_point_for_validation.target.jpos = point[i].positions;

            double current_time = rclcpp::Duration(point[i].time_from_start).seconds();
            ptp_point_for_validation.target.delta_t = current_time - previous_time;

            if (i == 0 && ptp_point_for_validation.target.delta_t == 0)
            {
                ptp_point_for_validation.target.delta_t += buffer_time_.seconds();
            }

            addverb_cobot::error_codes validation_result = data_validator_->validateRequest(ptp_point_for_validation);

            if (validation_result != addverb_cobot::error_codes::NoError)
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "PTP goal validation failed for a trajectory point. Error code: %d",
                    static_cast<int>(validation_result));
                return false;
            }

            previous_time = current_time;
        }

        RCLCPP_INFO(get_node()->get_logger(), "PTP Goal validation successful.");
        return true;
    }

    /**
     * @brief update the feedback message based on the current state
     *
     */
    void PTPJointController::updateFeedback_()
    {
        addverb_cobot::TransferState transfer_state = static_cast<addverb_cobot::TransferState>(state_interfaces_[6].get_value());

        if (transfer_state == addverb_cobot::TransferState::eExecuting)
        {
            double dt = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time_).count();

            if (dt >= (target_ + time_tolerance_))
            {
                std::cout << "dt : " << dt << "  target : " << target_ << std::endl;
                std::cout << "time tolerance : " << time_tolerance_ << std::endl;
                bool should_allow_reset = false;
                bool position_within_tolerance = true;
                std::vector<std::string> exceeded_joint_names;

                for (size_t i = 0; i < current_goal_.trajectory.joint_names.size(); ++i)
                {
                    double actual_position = state_interfaces_[i].get_value();
                    double target_position = current_goal_.trajectory.points.back().positions[i];
                    if (std::abs(actual_position - target_position) > default_params::default_position_tolerance)
                    {
                        position_within_tolerance = false;
                        exceeded_joint_names.push_back(current_goal_.trajectory.joint_names[i]);

                        // break;
                    }
                }

                {
                    std::lock_guard<std::mutex> guard(goal_handle_mutex_);

                    if (goal_handle_)
                    {
                        should_allow_reset = true;
                        auto result = std::make_shared<FollowJointTrajectory::Result>();
                        if (position_within_tolerance)
                        {
                            RCLCPP_INFO(get_node()->get_logger(), "Trajectory execution successful.");
                            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                            result->error_string = "Trajectory executed successfully within tolerance.";
                            goal_handle_->succeed(result);
                        }
                        else
                        {
                            result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;

                            result->error_string = "Position tolerance exceeded for joints: ";
                            for (auto &joint_name : exceeded_joint_names)
                            {
                                result->error_string += joint_name + " ";
                            }
                            goal_handle_->succeed(result);
                        }

                        goal_handle_.reset();
                    }
                }

                if (should_allow_reset)
                {
                    {
                        std::lock_guard<std::mutex> guard(traj_update_mutex_);
                        ongoing_prev_traj_ = false;
                    }
                    start_time_ = std::chrono::high_resolution_clock::now();

                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eIdle);
                    command_interfaces_[7].set_value(cmd);
                }
                else
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Waiting to acquire Goal handle, cannot reset ongoing previous trajectory");
                }
            }
            else
            {
                if (robot_in_error_.load(std::memory_order_acquire))
                {
                    {
                        std::lock_guard<std::mutex> guard(goal_handle_mutex_);
                        if (goal_handle_)
                        {
                            auto result = std::make_shared<FollowJointTrajectory::Result>();
                            RCLCPP_INFO(get_node()->get_logger(), "Goal rejected as robot went in error state.");
                            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                            result->error_string = "Goal rejected as robot went in error state";
                            goal_handle_->abort(result);
                            goal_handle_.reset();
                        }
                    }

                    {
                        std::lock_guard<std::mutex> guard(traj_update_mutex_);
                        ongoing_prev_traj_ = false;
                    }

                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eIdle);
                    command_interfaces_[7].set_value(cmd);
                }
                else
                {

                    bool should_publish_feedback = false;

                    {
                        std::lock_guard<std::mutex> guard(goal_handle_mutex_);
                        if (goal_handle_)
                        {
                            should_publish_feedback = true;
                        }
                    }

                    if (should_publish_feedback)
                    {
                        // Publish feedback during execution
                        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
                        feedback->joint_names = current_goal_.trajectory.joint_names;
                        feedback->actual.positions.resize(feedback->joint_names.size());
                        feedback->desired.positions.resize(feedback->joint_names.size());
                        feedback->error.positions.resize(feedback->joint_names.size());
                        for (size_t i = 0; i < feedback->actual.positions.size(); ++i)
                        {
                            feedback->actual.positions[i] = state_interfaces_[i].get_value();
                            feedback->desired.positions[i] = current_goal_.trajectory.points.back().positions[i];
                            feedback->error.positions[i] = feedback->actual.positions[i] - feedback->desired.positions[i];
                        }

                        {
                            std::lock_guard<std::mutex> guard(goal_handle_mutex_);
                            goal_handle_->publish_feedback(feedback);
                        }
                    }
                }
            }
        }
        // publish feedback and when the trajectory is done, ensure ongoing_prev_traj_ is set to false and client is updated to success
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    addverb_cobot_controllers::PTPJointController, controller_interface::ControllerInterface)
