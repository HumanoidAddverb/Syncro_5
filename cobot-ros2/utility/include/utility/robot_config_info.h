/**
 * @file robot_config_info.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Robot configuration information - copied from robot end - must make sure manually to update this one
 * @version 0.1
 * @date 2025-05-09
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef ROBOT_CONFIG_INFO_H_
#define ROBOT_CONFIG_INFO_H_

#include <array>
#include <map>
#include <string>

namespace addverb_cobot
{
    /// @brief safety types
    enum class SafetyContextEnums
    {
        eNothing = 0,
        eNoHumanSafety = 1,
        eNoPosSafety = 2,
        eMaxRegime = 3
    };

    // enum to hold the types of grippers
    enum class GripperTypes
    {
        eNone,
        eDynamixelGripper,
        eDhGripper,
        eSuctionGripper,
        eFeetechGripper,
        eMaxRegime
    };

    /// @brief ft types
    enum class FTTypes
    {
        eNone,
        eRobotiq,
        eRobotous,
        eMaxRegime
    };

    /// @brief dof of robot
    const int n_dof = 6;

    /// @brief number of predefined controllers
    const int n_controllers = 11;

    /// @brief max reattempt connection count
    const int max_reattempt_connection_count = 100;

    /// @brief wait time for any future calls
    const int future_wait_time = 10;

    /// @brief base configuration reach time
    const int base_config_reach_time = 10;

    /// @brief base configuration reach time buffer
    const int base_config_reach_buffer = 1;

    /// @brief base configuration jpos
    const std::vector<double> base_config_jpos(n_dof, 0.0);

    /// @brief maintains a list of controllers which have common interfaces (eg: joint/effort)
    const std::map<std::string, std::vector<std::string>> common_interface_controllers = {{"effort", {"gravity_comp_effort"}}};

    namespace joint
    {
        const double vel_maxabs = 0.5;
        const double vel_minabs = -0.5;
        const double vel_maxrel = 0.2;
        const double vel_minrel = 0.2;

        const double effort_maxabs = 60.0;
        const double effort_minabs = -60.0;
        const double effort_maxrel = 40.0;
        const double effort_minrel = -40.0;

        constexpr std::array<double, 12> default_joint_limits = {
            -3.1, 3.1,  // Joint 1 limits
            -1.5, 1.7,  // Joint 2 limits
            -1.5, 3.3,  // Joint 3 limits
            -2.9, 2.9,  // Joint 4 limits
            -1.5, 1.5,  // Joint 5 limits
            -6.26, 6.26 // Joint 6 limits
        };

    }

    namespace gripper
    {
        namespace lower_limit
        {
            const double force = 0;
        }

        namespace upper_limit
        {
            const double force = 150;
        }
    };

};

#endif