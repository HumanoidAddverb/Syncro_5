/**
 * @file hardware_interface_defs.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Definitions and declarations for the hardware interface - just to make life simpler
 * @version 0.1
 * @date 2025-05-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef HARDWARE_INTERFACE_DEFS_H_
#define HARDWARE_INTERFACE_DEFS_H_

#include <vector>
#include "api_types.h"
#include "robot_config_info.h"
#include "controller_defs.h"
#include <stdexcept>
#include <iostream>

namespace addverb_cobot
{
    /// @brief will contain the definition of the structures for hardware interface, and also constants
    namespace hw_interface_defs
    {

        /// @brief structure for holding configuration for Point to Point motion (Joint Space)
        /// where the initial point is the current position and the final point is specified by the user
        /// along with the time in which user wishes the robot to attain the target position
        struct Point
        {
            /// @brief vector of target joint positions
            std::vector<double> jpos;

            /// @brief time in which to attain the target position
            double delta_t;

            /// @brief initialisation
            void init()
            {
                jpos.clear();
                jpos.resize(n_dof, 0.0);
                delta_t = 0.0;
            }
        };

        /// @brief flex point
        struct FlexPoint
        {
            Point point;
            double flex_factor;

            void init(const int n = 0)
            {
                point.init();
                flex_factor = 1;
            }
        };

        /// @brief flex multi-point
        struct FlexMultiPoint
        {
            std::vector<Point> points;
            double flex_factor;

            void init(const int n = 0)
            {
                points.clear();
                points.resize(n);

                for (int i = 0; i < n; i++)
                {
                    points[i].init();
                }

                flex_factor = 0;
            }
        };

        /// @brief specify details of multi point
        struct MultiPoint
        {
            /// @brief vector of points
            std::vector<Point> points;

            /// @brief initialise
            /// @param n
            void init(const int n = 0)
            {
                points.clear();
                points.resize(n);

                for (int i = 0; i < n; i++)
                {
                    points[i].init();
                }
            }

            /// @brief reset the points
            void reset()
            {
                points.clear();
            }

            /// @brief add point
            /// @param p
            void addPoint(const Point &p)
            {
                points.push_back(p);
            }

            /// @brief get point at given index
            Point getPoint(const int id) const
            {
                if (id >= points.size())
                {
                    return Point();
                }

                return points[id];
            }

            /// @brief get number of points
            /// @return
            const int getSize() const
            {
                return static_cast<int>(points.size());
            }
        };

        /// @brief structure for holding configuration for Point to Point motion (Joint Space)
        struct PtP
        {
            /// @brief target joint position
            Point target;

            /// @brief current transfer cmd
            double transfer_cmd;

            /// @brief reset to zero
            void reset()
            {
                target.init();
                transfer_cmd = static_cast<double>(TransferCommand::eNone);
            }
        };

        /// @brief current state of the robot
        struct RobotFeedback
        {
            /// @brief read joint position
            std::vector<double> jpos;

            /// @brief read joint velocity
            std::vector<double> jvel;

            /// @brief read joint acceleration
            std::vector<double> jacc;

            /// @brief read joint effort
            std::vector<double> jtor;

            /// @brief read FT data
            std::vector<double> ft_data;

            /// @brief read EE pose data
            std::vector<double> ee_pos_data;

        };

        /// @brief state for the PtP
        struct PtPState
        {
            /// @brief state of the transfer
            double transfer_state;

            /// @brief reset the transfer state
            void reset()
            {
                transfer_state = static_cast<double>(TransferState::eNone);
            }
        };

        /**
         * @brief joint velocity commands
         *
         */
        struct Velocity
        {
            /// @brief current commanded velocity
            std::vector<double> cmd;

            /// @brief previously commanded velocity
            std::vector<double> prev_cmd;
        };

        /**
         * @brief joint effort commands
         *
         */
        struct Effort
        {
            /// @brief current commanded effort
            std::vector<double> cmd;

            /// @brief previously commanded effort
            std::vector<double> prev_cmd;
        };

        /**
         * @brief Force commands
         *
         */
        struct Force
        {
            std::vector<double> force;

            /// @brief initialise the force vector
            /// @param size
            void init(int size)
            {
                force.clear();
                force.resize(size, 0.0);
            }

            void reset()
            {
                std::fill(force.begin(), force.end(), 0.0);
            }
        };

        /**
         * @brief baseline structure for any jogging commands
         *
         */
        struct JogCommand
        {
            /// @brief current command
            std::vector<double> cmd = std::vector<double>(6, 0.0);

            /// @brief initialise the command vector
            /// @param size
            void init(int size)
            {
                cmd.clear();
                cmd.resize(size, 0.0);
            }

            /// @brief reset the command vector to zero
            void reset()
            {
                std::fill(cmd.begin(), cmd.end(), 0.0);
            }
        };

        /**
         * @brief jogging command for jogging in joint space
         */
        struct JointJog
        {
            /// @brief jogging command for joint jogging
            JogCommand jog_cmd;

            JointJog() = default;

            /// @brief initialise with n_joints
            JointJog(const int n_dof)
            {
                jog_cmd.init(n_dof);
            }

            void init(const int n_dof)
            {
                jog_cmd.init(n_dof);
            }

            /// @brief reset to zero
            void reset()
            {
                jog_cmd.reset();
            }
        };

        /**
         * @brief jogging command for jogging in cartesian space
         */
        struct CartesianJog
        {
            /// @brief jogging command for cartesian jogging
            JogCommand jog_cmd;

            /// @brief initialise with 6
            CartesianJog()
            {
                jog_cmd.init(6);
            }

            /// @brief reset to zero
            void reset()
            {
                jog_cmd.reset();
            }
        };

        /**
         * @brief stiffness matrix
         */
        struct Stiffness
        {
            /// @brief stiffness matrix
            std::vector<std::vector<double>> stiffness;

            /// @brief initialise with given size
            void init(const int dim)
            {
                stiffness.clear();
                stiffness.resize(dim, std::vector<double>(dim, 0.0));
            }

            /// @brief reset the command vector to zero
            void reset()
            {
                for (auto &row : stiffness)
                {
                    std::fill(row.begin(), row.end(), 0.0);
                }
            }
        };

        /**
         * @brief damping matrix
         */
        struct Damping
        {
            /// @brief damping matrix
            std::vector<std::vector<double>> damping;

            /// @brief initialise with given size
            void init(int dim)
            {
                damping.clear();
                damping.resize(dim, std::vector<double>(dim, 0.0));
            }

            /// @brief reset the command vector to zero
            void reset()
            {
                for (auto &row : damping)
                {
                    std::fill(row.begin(), row.end(), 0.0);
                }
            }
        };

        /**
         * @brief mass matrix
         */
        struct MassMatrix
        {
            /// @brief mass matrix
            std::vector<std::vector<double>> mass_matrix;

            /// @brief initialise with given size
            /// @param size
            void init(int dim)
            {
                mass_matrix.clear();
                mass_matrix.resize(dim, std::vector<double>(dim, 0.0));
            }

            /// @brief reset the command vector to zero
            void reset()
            {
                for (auto &row : mass_matrix)
                {
                    std::fill(row.begin(), row.end(), 0.0);
                }
            }
        };

        /**
         * @brief joint impedance configuration
         *
         */
        struct JointImpedance
        {
            /// @brief stiffness matrix
            Stiffness stiffness;

            /// @brief damping matrix
            Damping damping;

            JointImpedance() = default;

            /// @brief initialise with n_dof
            JointImpedance(const int n_dof)
            {
                stiffness.init(n_dof);
                damping.init(n_dof);
            }

            void init(const int n_dof)
            {
                stiffness.init(n_dof);
                damping.init(n_dof);
            }

            /// @brief reset the command vector to zero
            void reset()
            {
                stiffness.reset();
                damping.reset();
            }
        };

        /**
         * @brief cartesian impedance configuration
         *
         */
        struct CartesianImpedance
        {
            /// @brief stiffness matrix
            Stiffness stiffness;

            /// @brief damping matrix
            Damping damping;

            /// @brief mass matrix
            MassMatrix mass_matrix;

            /// @brief FT force
            Force ft_force;

            /// @brief Target Force
            Force target_force;

            /// @brief initialise with 6
            CartesianImpedance()
            {
                mass_matrix.init(6);
                stiffness.init(6);
                damping.init(6);
                ft_force.init(6);
                target_force.init(6);
            }

            /// @brief reset the command vector to zero
            void reset()
            {
                mass_matrix.reset();
                stiffness.reset();
                damping.reset();
                ft_force.reset();
            }
        };

        /**
         * @brief gripper configuration
         *
         */
        struct GripperConfig
        {
            int gripper_type;
        };

        /**
         * @brief FT configuration
         *
         */
        struct FTConfig
        {
            int ft_type;
            std::vector<std::vector<double>> rot;
        };

        /**
         * @brief Replay controller configuration
         *
         */
        struct ReplayConfig
        {
            MultiPoint points;
            int iterations;
        };

        /**
         * @brief safety mdoe
         *
         */
        struct SafetyMode
        {
            int safety_type;
        };

        /**
         * @brief payload configuration
         *
         */
        struct Payload
        {
            double mass;
            std::vector<double> com;
            std::vector<double> moi;
        };

        /**
         * @brief control mode
         *
         */
        struct ControlMode
        {
            API controller;
        };

        struct TcpPoint
        {
            /// @brief vector representing the pose
            std::vector<double> pose;

            /// @brief time in which to attain the target position
            double delta_t;

            /// @brief initialize the TcpPoint
            void init(const int size = 6)
            {
                pose.clear();
                pose.resize(size, 0.0); // Default size is 6 (e.g., x, y, z, roll, pitch, yaw)
                delta_t = 0.0;          // Default time to reach the target position
            }

            /// @brief clear the TcpPoint
            void clear()
            {
                pose.clear();
            }

            /// @brief resize the TcpPoint
            /// @param size New size for the pose vector
            void resize(const int size)
            {
                pose.resize(size, 0.0);
            }
        };

        struct TcpMultipoint
        {
            /// @brief vector of TcpPoint objects
            std::vector<TcpPoint> points;

            /// @brief initialize the TcpMultiPoint
            /// @param num_points Number of points to initialize
            /// @param point_size Size of each TcpPoint
            void init(const int num_points, const int point_size = 6)
            {
                points.clear();
                points.resize(num_points);
                for (auto &point : points)
                {
                    point.init(point_size);
                }
            }

            /// @brief clear all points in TcpMultiPoint
            void clear()
            {
                points.clear();
            }

            /// @brief resize the TcpMultiPoint
            /// @param num_points New number of points
            /// @param point_size Size of each TcpPoint
            void resize(const int num_points, const int point_size = 6)
            {
                points.resize(num_points);
                for (auto &point : points)
                {
                    point.resize(point_size);
                }
            }

            /// @brief add a new TcpPoint to the TcpMultiPoint
            /// @param point TcpPoint to add
            void addPoint(const TcpPoint &point)
            {
                points.push_back(point);
            }

            /// @brief get the size of the TcpMultiPoint
            /// @return Number of points
            int getSize() const
            {
                return points.size();
            }

            /// @brief get a specific TcpPoint
            /// @param index Index of the point
            /// @return Reference to the TcpPoint
            const TcpPoint &getPoint(const int index) const
            {
                if (index < 0 || index >= static_cast<int>(points.size()))
                {
                    throw std::out_of_range("Index out of range in TcpMultiPoint::getPoint");
                }
                return points[index];
            }

            /// @brief reset to zero
            void reset()
            {
                points.clear();
            }
        };
        struct TcpPtP
        {
            /// @brief target joint position
            TcpPoint target;

            /// @brief current transfer cmd
            double transfer_cmd;

            /// @brief reset to zero
            void reset()
            {
                target.init();
                transfer_cmd = static_cast<double>(TransferCommand::eNone);
            }
        };

        /// @brief state for the TCP PtP
        struct TcpPtPState
        {
            /// @brief state of the transfer
            double transfer_state;

            /// @brief reset the transfer state
            void reset()
            {
                transfer_state = static_cast<double>(TransferState::eNone);
            }
        };

        struct GripperCmd
        {
            /// @brief gripper transfer cmd
            double transfer_cmd;
            /// @brief gripper position
            double position;
            /// @brief gripper force
            double grasp_force;
            /// @brief Get the size of the GripperCmd structure
            /// @return The size of the structure, which is currently 2 (transfer_state and position)
            int size()
            {
                return 3;
            }

            void print()
            {
                std::cout << "gripper_position: " << position << std::endl;
                std::cout << "gripper force: " << grasp_force << std::endl;
            }
            void reset()
            {
                transfer_cmd = 0.0;
                position = 0.0;
                grasp_force = 0.0;
            }
        };

        enum class GpioType
        {
            TimeCmd,
            PtpTransfer,
            RecordCmd,
            TcpTimeCmd,
            PtpTcpTransfer,
            Tcp,
            JointJogging,
            CartesianJogging,
            JointImpedance,
            CartesianImpedance,
            ControllerName,
            RobotState,
            Gripper,
        };


        std::string gpioTypeToString(GpioType type);
        std::ostream& operator<<(std::ostream& os, GpioType type);


        struct GpioEntry
        {
            int robot_id;
            GpioType type;
            int gpio_index;

            friend std::ostream& operator<<(std::ostream& os, GpioEntry entry);
            static const std::vector<std::pair<std::string, GpioType>> gpio_map;
        };

        struct RobotControlData
        {
            double replay_iterations_cmd_;
            PtP ptp_cmd_;
            TcpPtP tcp_ptp_cmd_;
            JointImpedance joint_impedance_cmd_;
            CartesianImpedance cartesian_impedance_cmd_;
            GripperCmd gripper_cmd_;
            JointJog joint_jogging_cmd_;
            CartesianJog cartesian_jogging_cmd_;
            std::array<double, 6> tcp_command_;
            std::vector<double> controller_name_cmd_;

            RobotControlData() = default;

            RobotControlData(int n_dof, int n_controllers) : joint_jogging_cmd_(n_dof), joint_impedance_cmd_(n_dof) 
            {
                ptp_cmd_.reset();
                tcp_ptp_cmd_.reset();
                controller_name_cmd_.resize(n_controllers);
                cartesian_jogging_cmd_.reset();
            }
            
        };

        struct RobotStateData
        {
            double robot_status_;
            PtPState ptp_state_;
            TcpPtPState tcp_ptp_state_;
            std::array<double, 6> tcp_state_;

            RobotStateData() = default;

        };

        struct RobotMotionStruct
        {
            std::vector<double> position_;
            std::vector<double> velocity_;
            std::vector<double> effort_;

            RobotMotionStruct() = default;

            RobotMotionStruct(const int n_dof) : position_(n_dof, 0.0), velocity_(n_dof, 0.0), effort_(n_dof, 0.0) {}
        };

    }

};

#endif