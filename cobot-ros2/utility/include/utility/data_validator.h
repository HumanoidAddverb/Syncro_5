/**
 * @file data_validator.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Data Validator : performs basic sanity checks on the data to be published to the robot
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef DATA_VALIDATOR_H_
#define DATA_VALIDATOR_H_

#include <cmath>
#include "data_processor.h"
#include "robot_config_info.h"

#include "addverb_cobot_msgs/msg/cartesian_trajectory.hpp"
#include "addverb_cobot_msgs/action/follow_cartesian_trajectory.hpp"

namespace addverb_cobot
{
    class DataValidator
    {
    public:
        explicit DataValidator(const rclcpp::Logger &log) : logger_(log) {

                                                            };

        ~DataValidator() = default;

        /// @brief validate the given data
        /// @param ptp
        /// @return
        error_codes validateRequest(const hw_interface_defs::PtP &ptp);

        /// @brief validate the given data
        /// @param SafetyMode
        /// @return
        error_codes validateRequest(const hw_interface_defs::SafetyMode &mode);

        /// @brief validate the given data
        /// @param GripperConfig
        /// @return
        error_codes validateRequest(const hw_interface_defs::GripperConfig &gripper);

        /// @brief validate the given data
        /// @param FTConfig
        /// @return
        error_codes validateRequest(const hw_interface_defs::FTConfig &ft);

        /// @brief validate the given data
        /// @param Payload
        /// @return
        error_codes validateRequest(const hw_interface_defs::Payload &payload);

        /// @brief validate the given data
        /// @param vel
        /// @return
        error_codes validateRequest(const hw_interface_defs::Velocity &vel);

        /// @brief validate the given data
        /// @param effort
        /// @return
        error_codes validateRequest(const hw_interface_defs::Effort &effort);

        /// @brief validate the given data
        /// @param control_mode
        /// @return
        error_codes validateRequest(const hw_interface_defs::ControlMode &control_mode);

        /// @brief validate the given data
        /// @param replay_config
        /// @return
        error_codes validateRequest(const hw_interface_defs::ReplayConfig &replay_config);

        /// @brief validate the given data
        /// @param tcp_multi_point
        /// @return
        error_codes validateRequest(const hw_interface_defs::TcpMultipoint &tcp_multi_point);

        /// @brief validate the given data
        /// @param joint_jog
        /// @return
        error_codes validateRequest(const hw_interface_defs::JointJog &joint_jog);

        /// @brief validate the given data
        /// @param cartesian_jog
        /// @return
        error_codes validateRequest(const hw_interface_defs::CartesianJog &cartesian_jog);

        /// @brief validate the given data
        /// @param joint_impedance
        /// @return
        error_codes validateRequest(const hw_interface_defs::JointImpedance &joint_impedance);

        /// @brief validate the given data
        /// @param cartesian_impedance
        /// @return
        error_codes validateRequest(const hw_interface_defs::CartesianImpedance &cartesian_impedance);

        /// @brief validate the given data
        /// @param gripper_command
        /// @return
        error_codes validateRequest(const hw_interface_defs::GripperCmd &gripper_command);

        /// @brief validate the target for TCP PTP
        error_codes validateRequest(const addverb_cobot_msgs::action::FollowCartesianTrajectory::Goal &goal);

    private:
    /// @brief logger
    rclcpp::Logger logger_;

    };

};

#endif