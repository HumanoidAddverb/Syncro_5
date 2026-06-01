#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "addverb_cobot_msgs/srv/gripper.hpp"

using namespace std::chrono_literals;
using GripperSrv = addverb_cobot_msgs::srv::Gripper;

class GripperClient : public rclcpp::Node
{
    public:
        GripperClient()
            : Node("demo_gripper")
        {
            client_ = this->create_client<GripperSrv>("/arm_2_gripper_controller/command");
    
            RCLCPP_INFO(this->get_logger(), "Waiting for gripper service...");
            client_->wait_for_service();
    
            RCLCPP_INFO(this->get_logger(), "Connected to gripper service. Starting loop...");
    
            // Run the loop
            run_loop();
        }
    
    private:
        void run_loop()
        {
            bool open = false;
            const double force = 50.0;
    
            while (rclcpp::ok())
            {
                auto request = std::make_shared<GripperSrv::Request>();
                request->position = open ? 1.0 : 0.0;
                request->grasp_force = force;
    
                std::string action = open ? "Opening" : "Closing";
                RCLCPP_INFO(this->get_logger(), "%s gripper (pos=%.2f, force=%.2f)...",
                            action.c_str(), request->position, request->grasp_force);
    
                // Send the request and wait for result
                auto future = client_->async_send_request(request);
    
                // Wait for response
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto response = future.get();
                    if (response->success)
                        RCLCPP_INFO(this->get_logger(), "Gripper %s successful: %s", action.c_str(), response->message.c_str());
                    else
                        RCLCPP_WARN(this->get_logger(), "Gripper %s failed: %s", action.c_str(), response->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call gripper service");
                }
    
                // Toggle state for next iteration
                open = !open;
    
                // Wait 3 seconds before next action
                rclcpp::sleep_for(3s);
            }
        }
        
        /// service client 
        rclcpp::Client<GripperSrv>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
