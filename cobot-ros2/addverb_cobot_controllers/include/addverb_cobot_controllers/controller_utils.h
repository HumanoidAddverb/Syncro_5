/**
 * @file controller_utils.h
 * @author Aditya Pawar (aditya.pawar@addverb.com)
 * @brief Utility class for transferring trajectories and managing controller states
 * @version 0.1
 * @date 2025-07-20
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef CONTROLLER_UTILS_H
#define CONTROLLER_UTILS_H

 // c++ standard headers
#include <vector>
#include <string>

/// ROS headers
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

/// Utility headers
#include "utility/robot_config_info.h"
#include "utility/controller_defs.h"

/// KDL headers
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp> 
#include <kdl/jntarray.hpp>

namespace controller_utils
{
    class TrajectoryTransferUtil
    {
        public:
            using trajectory = trajectory_msgs::msg::JointTrajectory;
        
            TrajectoryTransferUtil(
                 std::vector<hardware_interface::LoanedCommandInterface> &cmds,
                 std::vector<hardware_interface::LoanedStateInterface> &states);
        
            ~TrajectoryTransferUtil() {}
        
            void setTrajectory(const trajectory &);
            void setTrajectory(const std::vector<std::string> &,
                               const std::vector<std::vector<double>> &,
                               const std::vector<double> &);
        
            void sendTrajectory();
            void updateControllerStatus();
            void execute();
        
            trajectory convertToTrajectory(const std::vector<std::string> &joint_names,
                                           const std::vector<std::vector<double>> &trajectory_points,
                                           const std::vector<double> &time_seq);
        
        private:
            std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces_;
            std::vector<hardware_interface::LoanedStateInterface> &state_interfaces_;
        
            trajectory_msgs::msg::JointTrajectory trajectory_;
        
            addverb_cobot::TransferState transfer_state_;
            int cur_index_ = 0;
    };

    class KdlUtil
    {
        public:
            KdlUtil(std::string& urdf_path, std::string& base_link, std::string &tip_link, std::vector<double> 
            &gravity_dir, const rclcpp::Logger& logger): urdf_path_(urdf_path), base_link_(base_link), tip_link_(tip_link), logger_(logger)
            {
                for (int i = 0; i < 3; i++)
                {
                    gravity_(i) = gravity_dir[i];
                }
            };
    
            ~KdlUtil(){};
            
            /// @brief initialise KDL 
            bool init();
            
            /// @brief compute gravity torque
            void computeGravityComp(std::vector<double>&);
            
            /// @brief compute coriolis torque 
            void computeCoriolisTorque(std::vector<double>&);
            
            /// @brief set joint position
            bool setJpos(std::vector<double>&);
            
            /// @brief set joint velocity
            bool setJvel(std::vector<double>&);
    
        private:
            /// @brief URDF path
            std::string urdf_path_;
        
            /// @brief root link
            std::string base_link_;
        
            /// @brief tip link
            std::string tip_link_;
    
            /// @brief KDL tree 
            KDL::Tree kdl_tree_;
            
            /// @brief KDL chain 
            KDL::Chain kdl_chain_;
            
            /// @brief Gravity vector 
            KDL::Vector gravity_ = {0.0, 0.0, -9.81};
    
            /// @brief Dynamic param solver
            std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
    
            /// @brief number of joints
            int num_joints_;
            
            /// @brief KDL variables
            KDL::JntArray q_, q_dot_, gravity_torque_, coriolis_torque_;
            
            /// @brief logger to print info or errors
            rclcpp::Logger logger_;

            /// @brief initialise KDL tree 
            bool initKDL_();

            /// @brief create KDL variables
            void initKDLVar_();

            /// @brief create KDL dynamics solver
            void initDynSolver_();
    };
}

#endif // CONTROLLER_UTILS_H
