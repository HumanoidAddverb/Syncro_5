#include "addverb_cobot_controllers/ptp_tcp_controller.h"

namespace addverb_cobot_controllers
{
    /**
     * @brief Initializes the PTPTCPController.
     *
     * @return controller_interface::CallbackReturn::SUCCESS if initialization succeeds,
     *         controller_interface::CallbackReturn::ERROR otherwise.
     */

    controller_interface::CallbackReturn PTPTCPController::on_init()
    {
        try
        {
            data_validator_ = std::make_unique<addverb_cobot::DataValidator>(get_node()->get_logger());
            commands_ = get_node()->get_parameter("commands").as_string_array();
            states_ = get_node()->get_parameter("states").as_string_array();
        }
        catch (...)
        {
            RCLCPP_WARN(get_node()->get_logger(), "EXCEPTION raised in initialising ptp_tcp_controller");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Configures the PTPTCPController.
     *
     * @return controller_interface::CallbackReturn::SUCCESS if configuration succeeds,
     *         controller_interface::CallbackReturn::ERROR otherwise.
     */
    controller_interface::CallbackReturn PTPTCPController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Activates the PTPTCPController.
     *     *
     * @return controller_interface::CallbackReturn::SUCCESS if activation succeeds,
     *         controller_interface::CallbackReturn::ERROR otherwise.
     */
    controller_interface::CallbackReturn PTPTCPController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // activate action server
        try
        {
            server_ = rclcpp_action::create_server<FollowCartesianTrajectory>(
                this->get_node(),
                std::string(get_node()->get_name()) + "/follow_cartesian_trajectory",
                std::bind(&PTPTCPController::handleGoal_, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&PTPTCPController::handleCancel_, this, std::placeholders::_1),
                std::bind(&PTPTCPController::handleAccepted_, this, std::placeholders::_1));
        }
        catch (...)
        {
            RCLCPP_INFO(get_node()->get_logger(), "EXCEPTION raised in configuring ptp_tcp_controller");

            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "activate successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Deactivates the PTPTCPController.
     */
    controller_interface::CallbackReturn PTPTCPController::on_deactivate(
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
    controller_interface::InterfaceConfiguration PTPTCPController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = commands_;
        return config;
    }

    /**
     * @brief specify the state interfaces to claim
     */
    controller_interface::InterfaceConfiguration PTPTCPController::state_interface_configuration() const
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
    controller_interface::return_type PTPTCPController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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
    rclcpp_action::GoalResponse PTPTCPController::handleGoal_(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const FollowCartesianTrajectory::Goal> goal)
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
            RCLCPP_WARN(get_node()->get_logger(), "Previous trajectory is still being processed, rejecting new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (data_validator_->validateRequest(*goal) != addverb_cobot::error_codes::NoError)
        {
            return rclcpp_action::GoalResponse::REJECT;
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
    rclcpp_action::CancelResponse PTPTCPController::handleCancel_(
        const std::shared_ptr<GoalHandleFollowCartesianTrajectory>)
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    /**
     * @brief handle execution of accepted goal
     *
     * @param goal_handle
     */
    void PTPTCPController::handleAccepted_(
        const std::shared_ptr<GoalHandleFollowCartesianTrajectory> goal_handle)
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
    void PTPTCPController::sendNextCmd_()
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
                        auto result = std::make_shared<FollowCartesianTrajectory::Result>();
                        RCLCPP_INFO(get_node()->get_logger(), "Goal rejected as robot in error.");
                        result->success = false;
                        result->message = "Goal rejected as robot in error";
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
                    const auto &cartesian_point = current_goal_.trajectory.points[cur_index_].point;

                    command_interfaces_[0].set_value(cartesian_point.position.x);
                    command_interfaces_[1].set_value(cartesian_point.position.y);
                    command_interfaces_[2].set_value(cartesian_point.position.z);
                    command_interfaces_[3].set_value(cartesian_point.orientation.x);
                    command_interfaces_[4].set_value(cartesian_point.orientation.y);
                    command_interfaces_[5].set_value(cartesian_point.orientation.z);

                    const auto &time = current_goal_.trajectory.points[cur_index_].time_from_start;
                    command_interfaces_[6].set_value(time);

                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eTransferring);
                    command_interfaces_[7].set_value(cmd);

                    cur_index_++;
                }
                else
                {
                    // All points have been sent, mark the transfer as done
                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::ePublish);
                    command_interfaces_[7].set_value(cmd);
                    target_time_ = current_goal_.trajectory.points[cur_index_ - 1].time_from_start;
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
                    auto result = std::make_shared<FollowCartesianTrajectory::Result>();
                    RCLCPP_INFO(get_node()->get_logger(), "Trajectory rejected because of infeasibility. Kindly re-attempt with feasible points.");
                    result->success = false;
                    result->message = "Trajectory rejected.";
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
     * @brief update the feedback message based on the current state
     *
     */
    void PTPTCPController::updateFeedback_()
    {
        addverb_cobot::TransferState transfer_state = static_cast<addverb_cobot::TransferState>(state_interfaces_[6].get_value());

        if (transfer_state == addverb_cobot::TransferState::eExecuting)
        {
            double dt = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time_).count();

            if (dt >= (target_time_ + time_tolerance_))
            {
                bool should_allow_reset = false;

                {
                    std::lock_guard<std::mutex> guard(goal_handle_mutex_);
                    if (goal_handle_)
                    {
                        should_allow_reset = true;
                        auto result = std::make_shared<FollowCartesianTrajectory::Result>();
                        RCLCPP_INFO(get_node()->get_logger(), "Trajectory execution successful.");
                        result->success = true;
                        result->message = "Trajectory executed successfully within tolerance.";
                        goal_handle_->succeed(result);
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
                            auto result = std::make_shared<FollowCartesianTrajectory::Result>();
                            RCLCPP_INFO(get_node()->get_logger(), "Goal rejected as robot went in error state.");
                            result->success = false;
                            result->message = "Goal rejected as robot went in error state";
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
            }
        }
    }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    addverb_cobot_controllers::PTPTCPController, controller_interface::ControllerInterface)
