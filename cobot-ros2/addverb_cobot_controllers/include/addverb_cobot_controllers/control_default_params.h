/**
 * @file control_default_params.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Default parameters for controllers when not specified by the user
 * @version 0.1
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONTROL_DEFAULT_PARAMS_H_
#define CONTROL_DEFAULT_PARAMS_H_

#include <array>
namespace addverb_cobot_controllers
{
    namespace default_params
    {
        // Default position tolerance for trajectory execution
        constexpr double default_position_tolerance = 0.03;

        // Default time tolerance for trajectory execution
        constexpr double default_time_tolerance = 1.5; // seconds

        // Default stiffness gain for joint impedance control
        constexpr double default_stiffness_gain = 200.0; // N/m

        // Default damping gain for joint impedance control
        constexpr double default_damping_gain = 5.0; // Ns/m

        // Default mass matrix for Cartesian impedance control
        constexpr double default_mass_matrix = 100.0; // kg

        // Default force applied by the force-torque sensor
        constexpr double default_ft_force = 0.0; // N

        // Default target force for Cartesian impedance control
        constexpr double default_target_force = 0.0; // N

        constexpr std::array<double, 6> default_bounding_box = {
            -0.3, 0.28,
            -0.4, 0.35,
            0.48, 0.76};

        constexpr std::array<double, 12> default_joint_limits = {
            -3.1, 3.1,  // Joint 1 limits
            -1.5, 1.7,  // Joint 2 limits
            -1.5, 3.3,  // Joint 3 limits
            -2.9, 2.9,  // Joint 4 limits
            -1.5, 1.5,  // Joint 5 limits
            -6.26, 6.26 // Joint 6 limits
        };

        constexpr double rpy_safety_limit = 1.48;

    }
}

#endif // CONTROL_DEFAULT_PARAMS_H_
