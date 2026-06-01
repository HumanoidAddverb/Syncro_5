#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class WaypointRecorder : public rclcpp::Node
{
public:
    WaypointRecorder() : Node("waypoint_recorder")
    {
        // Subscribe to the joint states topic
        // If your dual arms publish to separate topics (e.g., /arm_1/joint_states), update this accordingly.
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&WaypointRecorder::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waypoint Recorder Started.");
        RCLCPP_INFO(this->get_logger(), "Press 's' to save the current joint state.");
        RCLCPP_INFO(this->get_logger(), "Press 'q' to save to file and quit.");

        // Start the keyboard listener in a separate thread so it doesn't block ROS callbacks
        keyboard_thread_ = std::thread(&WaypointRecorder::keyboardLoop, this);
    }

    ~WaypointRecorder()
    {
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_joint_state_ = *msg;
        has_received_data_ = true;
    }

    // Helper function to read a single character from the terminal without pressing Enter
    char getch()
    {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
            perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
        return buf;
    }

    void keyboardLoop()
    {
        bool running = true;
        while (running && rclcpp::ok())
        {
            char c = getch();

            if (c == 's' || c == 'S')
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (has_received_data_)
                {
                    saved_waypoints_.push_back(latest_joint_state_);
                    RCLCPP_INFO(this->get_logger(), "Waypoint %zu saved!", saved_waypoints_.size());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "No joint state data received yet. Cannot save.");
                }
            }
            else if (c == 'q' || c == 'Q')
            {
                RCLCPP_INFO(this->get_logger(), "Quitting and writing to file...");
                writeToFile("waypoints.txt");
                running = false;
                rclcpp::shutdown();
            }
        }
    }

    void writeToFile(const std::string& filename)
    {
        std::ofstream outfile(filename);
        if (!outfile.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing.", filename.c_str());
            return;
        }

        outfile << "--- Recorded Waypoints ---\n\n";

        for (size_t i = 0; i < saved_waypoints_.size(); ++i)
        {
            outfile << "Waypoint " << (i + 1) << ":\n";
            const auto& state = saved_waypoints_[i];
            
            // Loop through the joints and print their names and positions
            for (size_t j = 0; j < state.name.size(); ++j)
            {
                outfile << "  " << state.name[j] << ": " << state.position[j] << "\n";
            }
            outfile << "\n";
        }

        outfile.close();
        RCLCPP_INFO(this->get_logger(), "Successfully wrote %zu waypoints to %s", saved_waypoints_.size(), filename.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::thread keyboard_thread_;
    std::mutex mutex_;
    
    sensor_msgs::msg::JointState latest_joint_state_;
    bool has_received_data_ = false;
    std::vector<sensor_msgs::msg::JointState> saved_waypoints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointRecorder>();
    
    // Spin in the main thread (handles the joint state callbacks)
    // The keyboard listener runs concurrently in its own thread.
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}