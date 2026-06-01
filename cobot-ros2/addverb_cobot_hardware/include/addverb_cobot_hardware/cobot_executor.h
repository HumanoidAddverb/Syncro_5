/**
 * @file cobot_hw_interface.h
 * @author Yaswanth Gonna (yaswanth.gonna@addverb.com)
 * @brief This class defines cobot executor which runs spin 
 * continously in a thread to process callbacks
 * @version 0.1
 * @date 2025-8-25 
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef COBOT_EXECUTOR_H
#define COBOT_EXECUTOR_H


#include "rclcpp/rclcpp.hpp"
#include "thread"
namespace  addverb_cobot
{
    // @yaswanth : single-threaded executor?
    class CobotExecutor: public rclcpp::executors::MultiThreadedExecutor 
    {
        public:
            /// @brief constructor
            CobotExecutor() {};
            
            /// @brief destructor
            ~CobotExecutor();

            /// @brief setup executor
            bool setup();
    
        private:
            /// @brief thread to run spin for calling callbacks
            std::thread spin_thread_;

            /// @brief timeout for spinning (ideally should start immediately)
            int count_limit_ = 50;

            /// @brief call spin()
            void run_();

            /// @brief stop spinning the node
            void shutdown_();
    };
}

#endif


