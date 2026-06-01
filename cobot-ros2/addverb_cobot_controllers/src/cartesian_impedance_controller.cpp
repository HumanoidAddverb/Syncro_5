// In joint_impedance_controller.cpp

#include "addverb_cobot_controllers/cartesian_impedance_controller.h"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp/parameter.hpp"

namespace addverb_cobot_controllers
{

    /**
     * @brief Initializes the CartesianImpedanceController
     *
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn CartesianImpedanceController::on_init()
    {
        try
        {
            data_validator_ = std::make_unique<addverb_cobot::DataValidator>(get_node()->get_logger());
            commands_ = get_node()->get_parameter("commands").as_string_array();
            states_ = get_node()->get_parameter("states").as_string_array();

            auto_declare<std::vector<double>>("stiffness", std::vector<double>(addverb_cobot::n_dof, default_params::default_stiffness_gain));
            auto_declare<std::vector<double>>("damping", std::vector<double>(addverb_cobot::n_dof, default_params::default_damping_gain));
            auto_declare<std::vector<double>>("mass_matrix", std::vector<double>(addverb_cobot::n_dof, default_params::default_mass_matrix));
            auto_declare<std::vector<double>>("ft_force", std::vector<double>(addverb_cobot::n_dof, default_params::default_ft_force));
            auto_declare<std::vector<double>>("target_force", std::vector<double>(addverb_cobot::n_dof, default_params::default_target_force));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief setup
     *
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief activate the action server
     *
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State &)
    {
        // Activate action server
        try
        {
            server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                get_node(),
                std::string(get_node()->get_name()) + "/follow_joint_trajectory",
                std::bind(&CartesianImpedanceController::handleGoal_, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&CartesianImpedanceController::handleCancel_, this, std::placeholders::_1),
                std::bind(&CartesianImpedanceController::handleAccepted_, this, std::placeholders::_1));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Action server creation failed: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        auto stiffness = get_node()->get_parameter("stiffness").as_double_array();
        auto damping = get_node()->get_parameter("damping").as_double_array();
        auto mass_matrix = get_node()->get_parameter("mass_matrix").as_double_array();
        auto ft_force = get_node()->get_parameter("ft_force").as_double_array();
        auto target_force = get_node()->get_parameter("target_force").as_double_array();

        if (stiffness.size() != addverb_cobot::n_dof)
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Stiffness param invalid or missing. Using default.");
            stiffness_gains_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_stiffness_gain);
        }
        else
        {
            stiffness_gains_ = stiffness;
        }

        if (damping.size() != addverb_cobot::n_dof)
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Damping param invalid or missing. Using default.");
            damping_gains_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_damping_gain);
        }
        else
        {
            damping_gains_ = damping;
        }

        if (mass_matrix.size() != addverb_cobot::n_dof)
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Mass matrix param invalid or missing. Using default.");
            mass_matrix_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_mass_matrix);
        }
        else
        {
            mass_matrix_ = mass_matrix;
        }

        if (ft_force.size() != addverb_cobot::n_dof)
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "FT force param invalid or missing. Using default.");
            ft_force_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_ft_force);
        }
        else
        {
            ft_force_ = ft_force;
        }

        if (target_force.size() != addverb_cobot::n_dof)
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Target force param invalid or missing. Using default.");
            target_force_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_target_force);
        }
        else
        {
            target_force_ = target_force;
        }

        parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
            std::bind(&CartesianImpedanceController::on_parameter_event_, this, std::placeholders::_1));

        for (size_t i = 0; i < addverb_cobot::n_dof; ++i)
        {
            command_interfaces_[i].set_value(state_interfaces_[i].get_value());
            command_interfaces_[i + addverb_cobot::n_dof].set_value(stiffness_gains_[i]);
            command_interfaces_[i + 2 * addverb_cobot::n_dof].set_value(damping_gains_[i]);
            command_interfaces_[i + 3 * addverb_cobot::n_dof].set_value(mass_matrix_[i]);
            command_interfaces_[i + 4 * addverb_cobot::n_dof].set_value(ft_force_[i]);
            command_interfaces_[i + 5 * addverb_cobot::n_dof].set_value(target_force_[i]);
        }

        RCLCPP_INFO(get_node()->get_logger(), "activate successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief reset and release
     *
     * @return controller_interface::CallbackReturn
     */
    controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        try
        {
            server_.reset();
            parameter_callback_handle_.reset();
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
     * @brief claim command  interfaces
     *
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = commands_;

        return config;
    }

    /**
     * @brief claim state  interfaces
     *
     * @return controller_interface::InterfaceConfiguration
     */
    controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = states_;
        return config;
    }

    /**
     * @brief update goal to hardware
     *
     * @return controller_interface::return_type
     */
    controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time &, const rclcpp::Duration &)
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

        if (!validateImpedanceData_())
        {
            RCLCPP_WARN(get_node()->get_logger(), "Invalid impedance data. Resetting command interfaces and setting stiffness and damping values to default");
            stiffness_gains_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_stiffness_gain);
            damping_gains_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_damping_gain);
            mass_matrix_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_mass_matrix);
            ft_force_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_ft_force);
            target_force_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_target_force);

            for (size_t i = 0; i < addverb_cobot::n_dof; ++i)
            {
                command_interfaces_[i + addverb_cobot::n_dof].set_value(stiffness_gains_[i]);
                command_interfaces_[i + 2 * addverb_cobot::n_dof].set_value(damping_gains_[i]);
                command_interfaces_[i + 3 * addverb_cobot::n_dof].set_value(mass_matrix_[i]);
                command_interfaces_[i + 4 * addverb_cobot::n_dof].set_value(ft_force_[i]);
                command_interfaces_[i + 5 * addverb_cobot::n_dof].set_value(target_force_[i]);
            }
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
    rclcpp_action::GoalResponse CartesianImpedanceController::handleGoal_(
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

        current_goal_ = *goal;

        if (!validateGoal_(current_goal_.trajectory))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Goal validation failed. Rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        target_ = current_goal_.trajectory.points[0].time_from_start.sec + (current_goal_.trajectory.points[0].time_from_start.nanosec) / 1e9;

        time_tolerance_ = current_goal_.goal_time_tolerance.sec + (current_goal_.goal_time_tolerance.nanosec / 1e9);

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

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief handle cancelling a goal
     *
     * @return rclcpp_action::CancelResponse
     */
    rclcpp_action::CancelResponse CartesianImpedanceController::handleCancel_(
        const std::shared_ptr<GoalHandleFollowJointTrajectory>)
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    /**
     * @brief handle execution of accepted goal
     *
     * @param goal_handle
     */
    void CartesianImpedanceController::handleAccepted_(
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
     * @brief send commands to hw interface
     *
     */
    void CartesianImpedanceController::sendNextCmd_()
    {
        addverb_cobot::TransferState transfer_state = static_cast<addverb_cobot::TransferState>(state_interfaces_[6].get_value());

        if (transfer_state == addverb_cobot::TransferState::eNone || transfer_state == addverb_cobot::TransferState::eIdling)
        {
            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eRcdNewTraj);
            command_interfaces_[37].set_value(cmd);
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
                command_interfaces_[37].set_value(cmd);
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

                    command_interfaces_[36].set_value(point.time_from_start.sec + (point.time_from_start.nanosec) / 1e9);

                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eTransferring);
                    command_interfaces_[37].set_value(cmd);

                    // Update the index for the next point
                    cur_index_++;
                }
                else
                {
                    // All points have been sent, mark the transfer as done
                    double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::ePublish);
                    command_interfaces_[37].set_value(cmd);
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
            command_interfaces_[37].set_value(cmd);
        }
        else if (transfer_state == addverb_cobot::TransferState::ePublished)
        {
            double cmd = 1.0 * static_cast<int>(addverb_cobot::TransferCommand::eExecute);
            command_interfaces_[37].set_value(cmd);

            {
                std::lock_guard<std::mutex> guard(traj_update_mutex_);
                has_new_goal_ = false;
                ongoing_prev_traj_ = true;
            }

            start_time_ = std::chrono::high_resolution_clock::now();
        }
    }

    /**
     * @brief reset stiffness and damping
     *
     * @param parameters
     * @return rcl_interfaces::msg::SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult CartesianImpedanceController::on_parameter_event_(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "stiffness")
            {
                auto gains = param.as_double_array();
                if (gains.size() != addverb_cobot::n_dof)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Received invalid stiffness param. Falling back to default.");
                    stiffness_gains_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_stiffness_gain);
                }
                else
                {
                    stiffness_gains_ = gains;
                }
            }
            else if (param.get_name() == "damping")
            {
                auto gains = param.as_double_array();
                if (gains.size() != addverb_cobot::n_dof)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Received invalid damping param. Falling back to default.");
                    damping_gains_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_damping_gain);
                }
                else
                {
                    damping_gains_ = gains;
                }
            }

            else if (param.get_name() == "mass_matrix")
            {
                auto gains = param.as_double_array();
                if (gains.size() != addverb_cobot::n_dof)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Received invalid mass matrix param. Falling back to default.");
                    mass_matrix_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_mass_matrix);
                }
                else
                {
                    mass_matrix_ = gains;
                }
            }

            else if (param.get_name() == "ft_force")
            {
                auto gains = param.as_double_array();
                if (gains.size() != addverb_cobot::n_dof)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Received invalid ft force param. Falling back to default.");
                    ft_force_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_ft_force);
                }
                else
                {
                    ft_force_ = gains;
                }
            }

            else if (param.get_name() == "target_force")
            {
                auto gains = param.as_double_array();
                if (gains.size() != addverb_cobot::n_dof)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Received invalid target force param. Falling back to default.");
                    target_force_ = std::vector<double>(addverb_cobot::n_dof, default_params::default_target_force);
                }
                else
                {
                    target_force_ = gains;
                }
            }
        }

        for (size_t i = 0; i < addverb_cobot::n_dof; ++i)
        {
            command_interfaces_[i + addverb_cobot::n_dof].set_value(stiffness_gains_[i]);
            command_interfaces_[i + 2 * addverb_cobot::n_dof].set_value(damping_gains_[i]);
            command_interfaces_[i + 3 * addverb_cobot::n_dof].set_value(mass_matrix_[i]);
            command_interfaces_[i + 4 * addverb_cobot::n_dof].set_value(ft_force_[i]);
            command_interfaces_[i + 5 * addverb_cobot::n_dof].set_value(target_force_[i]);
        }

        return result;
    }

    /**
     * @brief update the feedback message based on the current state
     *
     */
    void CartesianImpedanceController::updateFeedback_()
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
                    command_interfaces_[37].set_value(cmd);
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
                    command_interfaces_[37].set_value(cmd);
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

    /**
     * @brief validate the given goal
     * @param goal
     * @return true if the goal is valid, false otherwise
     */

    bool CartesianImpedanceController::validateGoal_(const trajectory_msgs::msg::JointTrajectory &goal)
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

        if (goal.points.size() > 1)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Goal validation failed: number of trajectory points is greater than 1.");
            return false;
        }

        double previous_time = 0.0;

        for (const auto &point : goal.points)
        {
            if (goal.joint_names.size() != point.positions.size())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Goal validation failed: joint_names size does not match point positions size.");
                return false;
            }

            addverb_cobot::hw_interface_defs::PtP cartesian_impedance_point;
            cartesian_impedance_point.target.jpos = point.positions;

            double current_time = rclcpp::Duration(point.time_from_start).seconds();
            cartesian_impedance_point.target.delta_t = current_time - previous_time;

            addverb_cobot::error_codes point_validation_result = data_validator_->validateRequest(cartesian_impedance_point);

            if (point_validation_result != addverb_cobot::error_codes::NoError)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Cartesian Impedance goal validation failed for a trajectory point. Error code: %d", static_cast<int>(point_validation_result));
                return false;
            }

            previous_time = current_time;
        }

        return true;
    }

    /**
     * @brief Validate the impedance data
     */
    bool CartesianImpedanceController::validateImpedanceData_()
    {
        addverb_cobot::hw_interface_defs::CartesianImpedance cartesian_impedance_data;
        for (size_t i = 0; i < addverb_cobot::n_dof; ++i)
        {
            cartesian_impedance_data.stiffness.stiffness[i][i] = stiffness_gains_[i];
            cartesian_impedance_data.damping.damping[i][i] = damping_gains_[i];
            cartesian_impedance_data.mass_matrix.mass_matrix[i][i] = mass_matrix_[i];
            cartesian_impedance_data.ft_force.force[i] = ft_force_[i];
            cartesian_impedance_data.target_force.force[i] = target_force_[i];
        }

        addverb_cobot::error_codes impedance_validation_result = data_validator_->validateRequest(cartesian_impedance_data);

        if (impedance_validation_result != addverb_cobot::error_codes::NoError)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Cartesian Impedance data validation failed. Error code: %d", static_cast<int>(impedance_validation_result));
            return false;
        }

        return true;
    }

} // namespace addverb_cobot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    addverb_cobot_controllers::CartesianImpedanceController, controller_interface::ControllerInterface);
