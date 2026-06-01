#include "addverb_cobot_hardware/cobot_services.h"

namespace addverb_cobot
{
    /**
     * @brief Contructor to launch services
     */
    CobotServices::CobotServices() : rclcpp::Node("cobot_services")
    {
        setupShutdownSrv_();
        setupErrorRecoverySrv_();
    }

    /**
     * @brief setup the shutdown service
     *
     */
    void CobotServices::setupShutdownSrv_()
    {
        this->declare_parameter<bool>("shutdown_requested", false);
        this->declare_parameter<bool>("shutdown_request_accepted", false);
        this->declare_parameter<bool>("shutdown_request_rejected", false);
        this->declare_parameter<bool>("shutdown_pre_processed", false);

        shutdown_srv_ = this->create_service<std_srvs::srv::Trigger>(std::string(this->get_name()) + "/shutdown_srv",
                                                                     std::bind(&CobotServices::shutdownCallback_, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief setup the error_recovery service
     *
     */
    void CobotServices::setupErrorRecoverySrv_()
    {
        // @aravindh - please add params
        this->declare_parameter<bool>("error_recovery_requested", false);
        this->declare_parameter<bool>("error_recovery_request_rejected", false);
        this->declare_parameter<bool>("error_recovery_request_accepted", false);
        this->declare_parameter<bool>("error_recovery_success", false);
        this->declare_parameter<bool>("error_recovery_failure", false);
        error_recovery_srv_ = this->create_service<std_srvs::srv::Trigger>(std::string(this->get_name()) + "/error_recovery_srv",
                                                                           std::bind(&CobotServices::errorRecoveryCallback_, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief shutdown callback
     * this method is callback for shutdown service
     * which enables flag for shutdown of hardware
     *
     *
     * @param request
     * @param response
     * @return
     */
    void CobotServices::shutdownCallback_(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Shutdown requested via shutdown service!");

        this->set_parameter(rclcpp::Parameter("shutdown_requested", true));
        shutdown_requested_ = true;

        shutdown_request_rejected_ = false;
        shutdown_request_accepted_ = false;
        shutdown_pre_processed_ = false;

        uint32_t counter_ = 0;
        while (!shutdown_request_accepted_ && !shutdown_request_rejected_)
        {

            rclcpp::sleep_for(std::chrono::milliseconds(1));
            shutdown_request_accepted_ = this->get_parameter("shutdown_request_accepted").as_bool();
            shutdown_request_rejected_ = this->get_parameter("shutdown_request_rejected").as_bool();
            if (counter_ > 5000)
            {
                shutdown_request_rejected_ = true;
                shutdown_request_accepted_ = false;
                break;
            }
            counter_++;
        }

        if (shutdown_request_rejected_)
        {
            RCLCPP_INFO(this->get_logger(), "Shutdown request rejected!");

            // Reset all params and internal variables to false
            this->set_parameter(rclcpp::Parameter("shutdown_requested", false));
            shutdown_requested_ = false;

            this->set_parameter(rclcpp::Parameter("shutdown_request_rejected", false));
            shutdown_request_rejected_ = false;

            this->set_parameter(rclcpp::Parameter("shutdown_request_accepted", false));
            shutdown_request_accepted_ = false;

            this->set_parameter(rclcpp::Parameter("shutdown_pre_processed", false));
            shutdown_pre_processed_ = false;

            // //Response for trigger service
            response->success = false;
            response->message = "Shutdown request rejected!";
            return;
        }

        while (!shutdown_pre_processed_)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(1));
            shutdown_pre_processed_ = this->get_parameter("shutdown_pre_processed").as_bool();
        }

        response->success = true;
        response->message = "Shutting down the system...";
        rclcpp::sleep_for(std::chrono::milliseconds(1));

        rclcpp::shutdown();
    }

    /**
     * @brief error recovery callback - to recover automatically from error state of the robot
     *
     * @param request
     * @param response
     */
    void CobotServices::errorRecoveryCallback_(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Error Recovery requested via error_recovery service!");

        this->set_parameter(rclcpp::Parameter("error_recovery_requested", true));
        error_recovery_requested_ = true;

        uint32_t counter_ = 0;
        while (!error_recovery_request_accepted_ && !error_recovery_request_rejected_)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(1));
            error_recovery_request_accepted_ = this->get_parameter("error_recovery_request_accepted").as_bool();
            error_recovery_request_rejected_ = this->get_parameter("error_recovery_request_rejected").as_bool();
            // Error recovery timeout condition
            if (counter_ > 5000)
            {
                error_recovery_request_rejected_ = true;
                error_recovery_request_accepted_ = false;
                break;
            }
            counter_++;
        }

        if (error_recovery_request_rejected_)
        {
            RCLCPP_INFO(this->get_logger(), "Error recovery request rejected!");

            // Reset all params and internal variables to false
            this->set_parameter(rclcpp::Parameter("error_recovery_requested", false));
            error_recovery_requested_ = false;

            this->set_parameter(rclcpp::Parameter("error_recovery_request_accepted", false));
            error_recovery_request_accepted_ = false;

            this->set_parameter(rclcpp::Parameter("error_recovery_request_rejected", false));
            error_recovery_request_rejected_ = false;

            response->success = true;
            response->message = "Error recovery request rejected!";
            return;
        }

        // if (error_recovery_request_accepted_)
        // {
        RCLCPP_INFO(this->get_logger(), "Error recovery request accepted. Please wait till the robot recovers");
        error_recovery_request_accepted_ = false;
        // }

        while (!error_recovery_success_ && !error_recovery_failure_)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(1));
            error_recovery_success_ = this->get_parameter("error_recovery_success").as_bool();
            error_recovery_failure_ = this->get_parameter("error_recovery_failure").as_bool();
        }

        if(error_recovery_success_)
        {
            RCLCPP_INFO(this->get_logger(), "Error recovery done!!!");
            response->success = true;
            response->message = "Error recovery done!!!";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Error recovery failed!!!");
            response->success = false;
            response->message = "Error recovery failed!!!";
        }

        // Reset all params and internal variables to false
        this->set_parameter(rclcpp::Parameter("error_recovery_requested", false));
        error_recovery_requested_ = false;

        this->set_parameter(rclcpp::Parameter("error_recovery_request_accepted", false));
        error_recovery_request_accepted_ = false;

        this->set_parameter(rclcpp::Parameter("error_recovery_request_rejected", false));
        error_recovery_request_rejected_ = false;

        this->set_parameter(rclcpp::Parameter("error_recovery_success", false));
        error_recovery_success_ = false;

        this->set_parameter(rclcpp::Parameter("error_recovery_failure", false));
        error_recovery_failure_ = false;
    }
}