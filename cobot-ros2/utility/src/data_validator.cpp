#include "utility/data_validator.h"

namespace addverb_cobot
{

    /**
     * @brief validate safety type
     *
     * @param mode
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::SafetyMode &mode)
    {
        int safety_min = static_cast<int>(SafetyContextEnums::eNothing);
        int safety_max = static_cast<int>(SafetyContextEnums::eMaxRegime);

        if ((mode.safety_type < safety_min) || (mode.safety_type >= safety_max))
        {
            return error_codes::InvalidSafetyType;
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate gripper type
     *
     * @param gripper
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::GripperConfig &gripper)
    {
        int gripper_type = gripper.gripper_type;

        int gripper_type_min = static_cast<int>(GripperTypes::eNone);
        int gripper_type_max = static_cast<int>(GripperTypes::eMaxRegime);

        // if ((gripper_type < gripper_type_min) || (gripper_type >= gripper_type_max))
        // {
        //     return error_codes::InvalidGripperType;
        // }

        if (gripper_type != static_cast<int>(GripperTypes::eFeetechGripper))
        {
            return error_codes::InvalidGripperType;
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate ft configuration
     *
     * @param ft
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::FTConfig &ft)
    {
        int ft_type = ft.ft_type;

        int ft_type_min = static_cast<int>(FTTypes::eNone);
        int ft_type_max = static_cast<int>(FTTypes::eMaxRegime);

        // check type
        if ((ft_type < ft_type_min) || (ft_type >= ft_type_max))
        {
            return error_codes::InvalidFtType;
        }

        // check size
        if (ft.rot.size() != 3)
        {
            return error_codes::InvalidFTRotationMatrix;
        }

        for (int i = 0; i < 3; i++)
        {
            if (ft.rot[i].size() != 3)
            {
                return error_codes::InvalidFTRotationMatrix;
            }

            for (int j = 0; j < 3; j++)
            {
                if (std::isnan(ft.rot[i][j]))
                {
                    return error_codes::InvalidFTRotationMatrix;
                }
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate controller
     *
     * @param controller
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::ControlMode &controller)
    {
        // temporary
        return error_codes::NoError;
    }

    /**
     * @brief check velocity values
     *
     * @param vel
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::Velocity &vel)
    {
        for (int i = 0; i < n_dof; i++)
        {
            double jvel = vel.cmd[i];

            if ((jvel > joint::vel_maxabs) ||
                (jvel < joint::vel_minabs))
            {
                RCLCPP_INFO(logger_, "Invalid velocity. Kindly ensure velocity commands are within valid range of -0.5 to +0.5 rad/s");
                return error_codes::InvalidVelocity;
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate payload
     *
     * @param payload
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::Payload &payload)
    {
        if (payload.mass <= 0)
        {
            RCLCPP_INFO(logger_, "Payload validation failed: Invalid mass");
            return error_codes::InvalidRequest;
        }

        if (payload.com.size() != 3)
        {
            RCLCPP_INFO(logger_, "Payload validation failed: Invalid center of mass size");
            return error_codes::InvalidRequest;
        }

        if (payload.moi.size() != 6)
        {
            RCLCPP_INFO(logger_, "Payload validation failed: Invalid moment of inertia size");
            return error_codes::InvalidRequest;
        }

        for (const auto &moi_value : payload.moi)
        {
            if (moi_value < 0)
            {
                RCLCPP_INFO(logger_, "Payload validation failed: Moment of inertia value is negative");
                return error_codes::InvalidRequest;
            }
        }

        RCLCPP_INFO(logger_, "Payload validation successful");
        return error_codes::NoError;
    }

    /**
     * @brief check effort values
     *
     * @param effort
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::Effort &effort)
    {
        for (int i = 0; i < n_dof; i++)
        {
            double jeffort = effort.cmd[i];
            double prev_jeffort = effort.prev_cmd[i];

            if (std::abs(jeffort) > 1e-4)
            {
                return error_codes::NoError;
            }
        }

        RCLCPP_WARN(logger_, "Commanded total effort is near 0 value, robot might start to fall.");

        return error_codes::InvalidEffort;
    }

    /**
     * @brief validate replay config
     *
     * @param replay_config
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::ReplayConfig &replay_config)
    {
        if (replay_config.iterations < 1)
        {
            RCLCPP_WARN(logger_, "Validation failed: Replay iterations cannot be less than 1.");
            return error_codes::ReplayError;
        }

        hw_interface_defs::MultiPoint points = replay_config.points;

        if (points.getSize() < 3)
        {
            RCLCPP_WARN(logger_, "Validation failed: Replay trajectories can't have less 3 points.");
            return error_codes::InvalidRequest;
        }

        for (int i = 0; i < points.getSize(); i++)
        {
            hw_interface_defs::PtP ptp;

            ptp.target = points.getPoint(i);

            if (validateRequest(ptp) != error_codes::NoError)
            {
                return error_codes::InvalidRequest;
            };
        }
        return error_codes::NoError;
    }

    /**
     * @brief validate PTP command
     *
     * @param ptp
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::PtP &ptp)
    {
        if (ptp.target.jpos.size() != n_dof)
        {
            RCLCPP_ERROR(logger_, "PTP command validation failed: Mismatch between joint names and positions size.");
            return error_codes::InvalidRequest;
        }

        for (int i = 0; i < n_dof; i++)
        {
            if (std::isnan(ptp.target.jpos[i]) || std::isinf(ptp.target.jpos[i]))
            {
                RCLCPP_ERROR(logger_, "PTP command validation failed: Invalid joint position at index %d.", i);
                return error_codes::InvalidRequest;
            }
        }

        // check delta time
        if (ptp.target.delta_t <= 0.0)
        {
            RCLCPP_ERROR(logger_, "PTP command validation failed: Invalid time step %d.", ptp.target.delta_t);
            return error_codes::InvalidRequest;
        }

        return error_codes::NoError;
    }

    error_codes DataValidator::validateRequest(const hw_interface_defs::TcpMultipoint &tcp_multi_point)
    {
        if (tcp_multi_point.points.empty())
        {
            RCLCPP_ERROR(logger_, "TCP MultiPoint validation failed: No points provided.");
            return error_codes::InvalidRequest;
        }

        for (const auto &point : tcp_multi_point.points)
        {
            // Validate linear velocity
            if (point.pose.size() < 6) // Ensure the pose vector has at least 6 elements (x, y, z, roll, pitch, yaw)
            {
                RCLCPP_ERROR(logger_, "TCP MultiPoint validation failed: Pose vector size is less than 6.");
                return error_codes::InvalidRequest;
            }

            for (size_t i = 0; i < point.pose.size(); ++i)
            {
                if (std::isnan(point.pose[i]) || std::isinf(point.pose[i]))
                {
                    RCLCPP_ERROR(logger_, "TCP MultiPoint validation failed: Invalid pose value at index %zu.", i);
                    return error_codes::InvalidRequest;
                }
            }
        }

        RCLCPP_INFO(logger_, "TCP MultiPoint validation successful.");
        return error_codes::NoError;
    }

    /**
     * @brief validate joint jogging command
     *
     * @param joint_jog
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::JointJog &joint_jog)
    {
        if (joint_jog.jog_cmd.cmd.size() != n_dof)
        {
            RCLCPP_ERROR(logger_, "Joint jogging command validation failed: Mismatch between joint names and velocities size.");
            return error_codes::InvalidRequest;
        }

        for (int i = 0; i < n_dof; i++)
        {
            double jvel = joint_jog.jog_cmd.cmd[i];

            if ((jvel > 3) ||
                (jvel < -3))
            {
                RCLCPP_ERROR(logger_, "Joint jogging command validation failed. Too high magnitude of command. Max limit is 3 in positive and negative direction.");
                return error_codes::InvalidVelocity;
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate cartesian jogging command
     *
     * @param cartesian_jog
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::CartesianJog &cartesian_jog)
    {
        if (cartesian_jog.jog_cmd.cmd.size() != n_dof)
        {
            RCLCPP_ERROR(logger_, "Cartesian jogging command validation failed: Invalid command size.");
            return error_codes::InvalidRequest;
        }

        for (int i = 0; i < 6; i++)
        {
            double jvel = cartesian_jog.jog_cmd.cmd[i];

            if ((jvel > 3) ||
                (jvel < -3))
            {
                RCLCPP_ERROR(logger_, "Cartesian jogging command validation failed. Too high magnitude of command. Max limit is 3 in positive and negative direction.");
                return error_codes::InvalidVelocity;
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate joint impedance command
     *
     * @param joint_impedance
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::JointImpedance &joint_impedance)
    {
        if (joint_impedance.stiffness.stiffness.size() != n_dof || joint_impedance.damping.damping.size() != n_dof)
        {
            RCLCPP_ERROR(logger_, "Joint impedance command validation failed: Invalid stiffness or damping size.");
            return error_codes::InvalidRequest;
        }

        for (int i = 0; i < n_dof; i++)
        {
            if (joint_impedance.stiffness.stiffness[i][i] < 0 || joint_impedance.damping.damping[i][i] < 0)
            {
                RCLCPP_ERROR(logger_, "Joint impedance command validation failed: Negative stiffness or damping value.");
                return error_codes::InvalidRequest;
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate cartesian impedance command
     *
     * @param cartesian_impedance
     * @return error_codes
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::CartesianImpedance &cartesian_impedance)
    {
        if (cartesian_impedance.stiffness.stiffness.size() != n_dof || cartesian_impedance.damping.damping.size() != n_dof)
        {
            RCLCPP_ERROR(logger_, "Joint impedance command validation failed: Invalid stiffness or damping size.");
            return error_codes::InvalidRequest;
        }

        for (int i = 0; i < n_dof; i++)
        {
            if (cartesian_impedance.stiffness.stiffness[i][i] < 0 || cartesian_impedance.damping.damping[i][i] < 0)
            {
                RCLCPP_ERROR(logger_, "Joint impedance command validation failed: Negative stiffness or damping value.");
                return error_codes::InvalidRequest;
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate gripper command
     *
     * @param gripper_command
     * @return error_codes
     *
     * Check if gripper command is valid. The command should be either 0 or 1.
     */
    error_codes DataValidator::validateRequest(const hw_interface_defs::GripperCmd &gripper_command)
    {
        if (static_cast<int>(gripper_command.position) != 0 && static_cast<int>(gripper_command.position) != 1)
        {
            RCLCPP_ERROR(logger_, "Gripper command validation failed: Invalid command value.");
            return error_codes::InvalidGripperCmd;
        }

        if (static_cast<int>(gripper_command.position) == 0)
        {
            if (gripper_command.grasp_force < gripper::lower_limit::force || gripper_command.grasp_force > gripper::upper_limit::force)
            {
                RCLCPP_ERROR(logger_, "Gripper command validation failed: Invalid force value.");
                return error_codes::InvalidGripperCmd;
            }
        }

        return error_codes::NoError;
    }

    /**
     * @brief validate tcp ptp
     *
     * @return error_codes
     *
     */
    error_codes DataValidator::validateRequest(const addverb_cobot_msgs::action::FollowCartesianTrajectory::Goal &goal)
    {
        const int n_points = goal.trajectory.points.size();

        if (n_points < 3)
        {
            RCLCPP_ERROR(logger_, "PTP TCP command validation failed: Invalid trajectory size. Please give a minimun of three points in the trajectory.");
            return error_codes::InvalidRequest;
        }

        for (int i = 0; i < n_points; i++)
        {
            double cur_time, prev_time, diff_time;

            if (i == 0)
            {
                prev_time = 0;
            }
            else
            {
                prev_time = goal.trajectory.points[i - 1].time_from_start;
            }

            cur_time = goal.trajectory.points[i].time_from_start;
            diff_time = cur_time - prev_time;

            if (diff_time <= 0)
            {
                RCLCPP_ERROR(logger_, "PTP TCP command validation failed: Invalid target time . Please ensure that consecutive target time are in increasing order.");
                return error_codes::InvalidRequest;
            }
        }

        return error_codes::NoError;
    }

};