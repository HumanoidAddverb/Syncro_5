/**
 * @file cobot_auxiliary.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Auxiliary functions for cobot hardware
 * @version 0.1
 * @date 2025-10-18
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef COBOT_AUXILIARY_H_
#define COBOT_AUXILIARY_H_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "addverb_cobot_msgs/msg/cartesian_point.hpp"

namespace addverb_cobot
{
    class CobotAuxiliary : public rclcpp::Node
    {
    public:
        /// @brief constructor
        CobotAuxiliary(const std::string& robot_name);

        ~CobotAuxiliary();

        /// @brief set FT data
        void updateFTData(const std::vector<double> &);

        /// @brief set EE position data
        void updateEEPosData(const std::vector<double> &);

    private:
        /// @brief mutex for ft data
        std::mutex ft_mutex_;

        /// @brief mutex for ee pos data
        std::mutex ee_pos_mutex_;

        /// @brief timer for FT data
        rclcpp::TimerBase::SharedPtr timer_;

        /// @brief publisher for FT data
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr ft_pub_;

        /// @brief publisher for EE pos data
        rclcpp::Publisher<addverb_cobot_msgs::msg::CartesianPoint>::SharedPtr ee_pos_pub_;

        /// @brief last updated value of ft data
        std::vector<double> ft_data_;

        /// @brief last updated value of ee_pos data
        std::vector<double> ee_pos_data_;

        /// @brief setup FT sensor data publisher
        void setupFTPub_();

        /// @brief setup EE pose vector data publisher
        void setupEEPosPub_();

        /// @brief setup callback for publishing data
        void setupCallback_();

        /// @brief callback for FT sensor
        void pubCallback_();

        /// @brief publish FT data
        void publishFTData_();

        /// @brief publish EE pos data
        void publishEEPosData_();
        
        std::string robot_name_;
    };
}

#endif