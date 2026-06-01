#include "addverb_cobot_hardware/cobot_auxiliary.h"

namespace addverb_cobot
{

    /**
     * @brief Contructor, robot name is the namespace
     */
    CobotAuxiliary::CobotAuxiliary(const std::string& robot_name) : rclcpp::Node(robot_name + "_cobot_auxiliary" ), robot_name_(robot_name)
    {
        setupFTPub_();
        setupEEPosPub_();
        setupCallback_();
    }

    /**
     * @brief Destructor
     */
    CobotAuxiliary::~CobotAuxiliary()
    {
    }

    /**
     * @brief setup the publisher for FT data
     *
     */
    void CobotAuxiliary::setupFTPub_()
    {
        ft_data_ = std::vector<double>(6, 0);

        // Create publisher for ft data
        ft_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
            "/" + robot_name_ + "_ft_data", 10);
    }

    /**
     * @brief setup the publisher for end effector data
     *
     */
    void CobotAuxiliary::setupEEPosPub_()
    {
        ee_pos_data_ = std::vector<double>(6, 0);

        // Create publisher for ee_pos data
        ee_pos_pub_ = this->create_publisher<addverb_cobot_msgs::msg::CartesianPoint>(
            "/" + robot_name_ + "_ee_pos_data", 10);
    }

    /**
     * @brief setup the publisher callback
     *
     */
    void CobotAuxiliary::setupCallback_()
    {
        // Create timer to publish commands at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&CobotAuxiliary::pubCallback_, this));
    }

    /**
     * @brief callback for publishing data
     *
     */
    void CobotAuxiliary::pubCallback_()
    {
        publishFTData_();
        publishEEPosData_();
    }

    /**
     * @brief publish ee pos data
     *
     */
    void CobotAuxiliary::publishEEPosData_()
    {
        std::vector<double> ee_pos_data;
        {
            std::lock_guard<std::mutex> lock(ee_pos_mutex_);
            ee_pos_data = ee_pos_data_;
        }
        auto msg = addverb_cobot_msgs::msg::CartesianPoint();

        msg.position.x = ee_pos_data[0];
        msg.position.y = ee_pos_data[1];
        msg.position.z = ee_pos_data[2];

        msg.orientation.x = ee_pos_data[3];
        msg.orientation.y = ee_pos_data[4];
        msg.orientation.z = ee_pos_data[5];

        ee_pos_pub_->publish(msg);
    }

    /**
     * @brief publish FT data
     *
     */
    void CobotAuxiliary::publishFTData_()
    {
        std::vector<double> ft_data;

        {
            std::lock_guard<std::mutex> lock(ft_mutex_);
            ft_data = ft_data_;
        }

        auto msg = geometry_msgs::msg::Wrench();

        msg.force.x = ft_data[0];
        msg.force.y = ft_data[1];
        msg.force.z = ft_data[2];
        msg.torque.x = ft_data[3];
        msg.torque.y = ft_data[4];
        msg.torque.z = ft_data[5];

        ft_pub_->publish(msg);
    }

    /**
     * @brief set FT data
     *
     * @param ft_data
     */
    void CobotAuxiliary::updateFTData(const std::vector<double> &ft_data)
    {
        std::lock_guard<std::mutex> lock(ft_mutex_);
        ft_data_ = ft_data;
    }

    /**
     * @brief set EE pos data
     *
     * @param ee_pos_data
     */
    void CobotAuxiliary::updateEEPosData(const std::vector<double> &ee_pos_data)
    {
        std::lock_guard<std::mutex> lock(ee_pos_mutex_);
        ee_pos_data_ = ee_pos_data;
    }

};