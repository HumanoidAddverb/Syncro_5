/**
 * @file cobot_services.h
 * @author Yaswanth Gonna (yaswanth.gonna@addverb.com), Siddhi jain (siddhi.jain@addverb.com), Aravindh Ganesan (aravindh.ganesan@addverb.com)
 * @brief This class defines all the services used by cobot hardware
 * eg: shutdown, error_recovery
 * @version 0.1
 * @date 2025-8-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef COBOT_SERVICES_H_
#define COBOT_SERVICES_H_

/// std headers
#include <atomic>
#include <chrono>

/// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace addverb_cobot
{
    class CobotServices : public rclcpp::Node
    {
    public:
        /// @brief constructor
        CobotServices();

        ~CobotServices() {};

    private:
        /// @brief  pointer to shutdown service.
        // @aravindh - can you change the service type to std_srvs::Trigger?
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_srv_;

        /// @brief pointer to error recovery action server
        // @aravindh - can you change the service type to std_srvs::Trigger?
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr error_recovery_srv_;

        /// @brief failure code
        // @aravindh - can you try implementing this?
        int failure_code_{0};

        /// @brief is shutdown requested
        bool shutdown_requested_{false};

        /// @brief is shutdown request rejected by hardware
        bool shutdown_request_rejected_{false};

        /// @brief is shutdown request accepted by hardware
        bool shutdown_request_accepted_{false};

        /// @brief is shutdown pre-processed by hardware
        bool shutdown_pre_processed_{false};

        /// @brief is error recovery requested
        bool error_recovery_requested_{false};

        /// @brief is error recovery request rejected by hardware
        bool error_recovery_request_rejected_{false};

        /// @brief is error recovery request accepted by hardware
        bool error_recovery_request_accepted_{false};

        /// @brief is error recovery completed by hardware
        bool error_recovery_success_{false};

        /// @brief is error recovery completed by hardware
        bool error_recovery_failure_{false};

        /// @brief shutdown callback
        void shutdownCallback_(std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response>);

        /// @brief error recovery callback
        void errorRecoveryCallback_(std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response>);

        /// @brief setup the shutdown service
        void setupShutdownSrv_();

        /// @brief setup the error recovery service
        void setupErrorRecoverySrv_();

    };
}

#endif