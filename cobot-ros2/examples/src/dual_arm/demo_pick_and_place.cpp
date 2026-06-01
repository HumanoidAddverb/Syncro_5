#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Joint Trajectory includes
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// Cartesian Trajectory includes
#include "addverb_cobot_msgs/action/follow_cartesian_trajectory.hpp"
#include "addverb_cobot_msgs/msg/cartesian_trajectory_point.hpp"
#include "addverb_cobot_msgs/msg/cartesian_point.hpp"

// Gripper includes
#include "addverb_cobot_msgs/srv/gripper.hpp"

using namespace std::chrono_literals;

namespace dual_arm_demo
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FollowCartesianTrajectory = addverb_cobot_msgs::action::FollowCartesianTrajectory;
using GripperSrv = addverb_cobot_msgs::srv::Gripper;

class MultiControllerDemo : public rclcpp::Node
{
public:
    MultiControllerDemo() : Node("dual_arm_multi_controller_demo")
    {
        // ==========================================
        // 1. INITIALIZE CLIENTS
        // ==========================================
        
        // --- Joint PTP Clients ---
        arm1_joint_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_1_ptp_joint_controller/follow_joint_trajectory");
        // arm2_joint_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        //     this, "/arm_2_ptp_joint_controller/follow_joint_trajectory");

        // // --- Cartesian TCP Clients ---
        // arm1_cartesian_client_ = rclcpp_action::create_client<FollowCartesianTrajectory>(
        //     this, "/arm_1_ptp_tcp_controller/follow_cartesian_trajectory");
        // arm2_cartesian_client_ = rclcpp_action::create_client<FollowCartesianTrajectory>(
        //     this, "/arm_2_ptp_tcp_controller/follow_cartesian_trajectory");

        // // --- Joint Impedance Clients ---
        // arm1_impedance_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        //     this, "/arm_1_joint_impedance_controller/follow_joint_trajectory");
        arm2_impedance_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_2_joint_impedance_controller/follow_joint_trajectory");

        // --- Gripper Service Clients ---
        arm1_gripper_client_ = this->create_client<GripperSrv>("/arm_1_gripper_controller/command");
        arm2_gripper_client_ = this->create_client<GripperSrv>("/arm_2_gripper_controller/command");

        // --- Declare Impedance Parameters ---
        std::vector<double> stiffness = {100, 100, 100, 100, 100, 100};
        std::vector<double> damping   = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5};
        // this->declare_parameter("arm_1_joint_impedance_controller.stiffness", stiffness);
        // this->declare_parameter("arm_1_joint_impedance_controller.damping", damping);
        this->declare_parameter("arm_2_joint_impedance_controller.stiffness", stiffness);
        this->declare_parameter("arm_2_joint_impedance_controller.damping", damping);
        RCLCPP_INFO(this->get_logger(), "Impedance controller parameters declared.");

        // ==========================================
        // 2. WAIT FOR SERVERS
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Waiting for all action and service servers...");
        arm1_joint_client_->wait_for_action_server();
        // arm2_joint_client_->wait_for_action_server();
        // arm1_cartesian_client_->wait_for_action_server();
        // arm2_cartesian_client_->wait_for_action_server();
        // arm1_impedance_client_->wait_for_action_server();
        arm2_impedance_client_->wait_for_action_server();
        arm1_gripper_client_->wait_for_service();
        arm2_gripper_client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "All servers connected!");

        // Start the execution sequence in a separate thread
        demo_thread_ = std::thread(&MultiControllerDemo::executeSequence, this);
    }

    ~MultiControllerDemo()
    {
        if (demo_thread_.joinable()) {
            demo_thread_.join();
        }
    }

private:
    // ==========================================
    // 3. REUSABLE MOVEMENT WRAPPERS
    // ==========================================

    // Generalized Joint Movement (Multi-Point)
    bool sendJointTrajectory(
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
        const std::string& arm_name, 
        const std::vector<std::vector<double>>& waypoints, 
        const std::vector<double>& segment_durations_sec,
        const std::string& move_type)
    {
        // Safety check: Ensure we have a duration for every waypoint
        if (waypoints.size() != segment_durations_sec.size()) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Mismatch between waypoints and durations count!", arm_name.c_str());
            return false;
        }

        FollowJointTrajectory::Goal goal_msg;
        goal_msg.trajectory.joint_names = {
            arm_name + "_joint1", arm_name + "_joint2", arm_name + "_joint3",
            arm_name + "_joint4", arm_name + "_joint5", arm_name + "_joint6"
        };

        double cumulative_time = 0.0;

        // Loop through all points and add them to the trajectory
        for (size_t i = 0; i < waypoints.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = waypoints[i];
            
            // ROS 2 expects the time from the START of the trajectory, so we accumulate the times
            cumulative_time += segment_durations_sec[i];
            pt.time_from_start = rclcpp::Duration::from_seconds(cumulative_time);
            
            goal_msg.trajectory.points.push_back(pt);
        }

        RCLCPP_INFO(this->get_logger(), "[%s] Sending %s joint goal with %zu points...", arm_name.c_str(), move_type.c_str(), waypoints.size());

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);

        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "[%s] %s goal rejected/timed out.", arm_name.c_str(), move_type.c_str());
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return false;

        auto result_future = client->async_get_result(goal_handle);
        
        // Wait for the total cumulative time + a 5 second buffer
        if (result_future.wait_for(std::chrono::seconds(static_cast<int>(cumulative_time) + 5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "[%s] %s execution timed out.", arm_name.c_str(), move_type.c_str());
            return false;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "[%s] %s movement succeeded.", arm_name.c_str(), move_type.c_str());
            return true;
        }
        return false;
    }

    // Move using standard PTP Joint Trajectory (Multi-Point)
    bool moveJoints(const std::string& arm_name, const std::vector<std::vector<double>>& waypoints, const std::vector<double>& segment_durations_sec)
    {
        auto client = (arm_name == "arm_1") ? arm1_joint_client_ : arm2_joint_client_;
        return sendJointTrajectory(client, arm_name, waypoints, segment_durations_sec, "PTP");
    }

    // Move using Impedance Joint Trajectory (Multi-Point)
    bool moveImpedance(const std::string& arm_name, const std::vector<std::vector<double>>& waypoints, const std::vector<double>& segment_durations_sec)
    {
        auto client = (arm_name == "arm_1") ? arm1_impedance_client_ : arm2_impedance_client_;
        return sendJointTrajectory(client, arm_name, waypoints, segment_durations_sec, "Impedance");
    }

    // // Move using Cartesian TCP Trajectory
    // bool moveCartesian(const std::string& arm_name, const addverb_cobot_msgs::msg::CartesianPoint& target_pose, double duration_sec)
    // {
    //     auto client = (arm_name == "arm_1") ? arm1_cartesian_client_ : arm2_cartesian_client_;
        
    //     FollowCartesianTrajectory::Goal goal_msg;
    //     addverb_cobot_msgs::msg::CartesianTrajectoryPoint pt;
        
    //     pt.point = target_pose;
    //     pt.time_from_start = duration_sec;
    //     goal_msg.trajectory.points.push_back(pt);

    //     RCLCPP_INFO(this->get_logger(), "[%s] Sending Cartesian TCP goal...", arm_name.c_str());

    //     auto send_goal_options = rclcpp_action::Client<FollowCartesianTrajectory>::SendGoalOptions();
    //     auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);

    //     if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
    //         RCLCPP_ERROR(this->get_logger(), "[%s] Cartesian goal rejected.", arm_name.c_str());
    //         return false;
    //     }

    //     auto goal_handle = goal_handle_future.get();
    //     if (!goal_handle) return false;

    //     auto result_future = client->async_get_result(goal_handle);
    //     if (result_future.wait_for(std::chrono::seconds(static_cast<int>(duration_sec) + 5)) != std::future_status::ready) {
    //         RCLCPP_ERROR(this->get_logger(), "[%s] Cartesian execution timed out.", arm_name.c_str());
    //         return false;
    //     }

    //     auto result = result_future.get();
    //     if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    //         RCLCPP_INFO(this->get_logger(), "[%s] Cartesian movement succeeded.", arm_name.c_str());
    //         return true;
    //     }
    //     return false;
    // }

    // Synchronous function to operate the gripper
    bool controlGripper(const std::string& arm_name, bool open, double force = 50.0)
    {
        auto client = (arm_name == "arm_1") ? arm1_gripper_client_ : arm2_gripper_client_;
        auto request = std::make_shared<GripperSrv::Request>();
        
        request->position = open ? 1.0 : 0.0;
        request->grasp_force = force;

        std::string action = open ? "Opening" : "Closing";
        RCLCPP_INFO(this->get_logger(), "[%s] %s gripper...", arm_name.c_str(), action.c_str());

        auto future = client->async_send_request(request);
        
        if (future.wait_for(3s) == std::future_status::ready) {
            if (future.get()->success) {
                RCLCPP_INFO(this->get_logger(), "[%s] Gripper action successful.", arm_name.c_str());
                return true;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "[%s] Gripper action failed.", arm_name.c_str());
        return false;
    }

    // ==========================================
    // 4. SCRIPT SEQUENCE
    // ==========================================
    
    void executeSequence()
    {
        std::this_thread::sleep_for(1s);
        RCLCPP_INFO(this->get_logger(), "--- STARTING SEQUENTIAL WAYPOINT DEMO ---");

        // ==========================================
        // STAGE 1: ARM 1 MOVES TOWARDS PICK
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Stage 1: Arm 1 approaching pick location...");
        
        // Open Arm 1 gripper before approaching
        controlGripper("arm_1", true); 

        // Pack all approach waypoints into a single vector of vectors
        std::vector<std::vector<double>> a1_approach_wps = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {-0.886538, 0.000155716, 0.0, 0.0, 0.0, 0.0},
            {-1.45275, -0.793888, -0.821186, -0.00982208, 0.0, 0.0},
            {-1.45276, -1.3712, -1.35778, -0.00984604, 0.0, 0.0} // Final Pick Pose
        };

        // Define the time to reach each waypoint from the previous one
        std::vector<double> a1_approach_times = {10.0, 10.0, 10.0, 10.0};

        // Send the entire continuous trajectory
        moveJoints("arm_1", a1_approach_wps, a1_approach_times);

        // Close Arm 1 gripper to grab the object
        RCLCPP_INFO(this->get_logger(), "Stage 1.5: Grasping object...");
        controlGripper("arm_1", false, 50.0); 
        std::this_thread::sleep_for(5s); // Give it a second to secure the grip

        // ==========================================
        // STAGE 2: ARM 1 LIFTS AND MOVES TO HANDOVER
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Stage 2: Arm 1 moving to handover pose...");
        
        // Skipping WP1 here as it's almost identical to the pick pose
        std::vector<std::vector<double>> a1_lift_wps = {
            {-1.45275, -0.831331, -1.02307, -0.78851, -0.447252, 0.0},
            {-0.564637, -0.506987, -0.53516, -1.18118, -0.468226, 0.0042762},
            {-0.564613, -0.171419, 0.110139, -1.21185, -0.490673, 0.626481},
            {-0.572462, -0.627152, 0.0256324, -1.32623, -0.829294, -0.161644} // Handover Pose

        };
        std::vector<double> a1_lift_times = {10.0, 10.0, 10.0, 10.0};

        moveJoints("arm_1", a1_lift_wps, a1_lift_times);

        // ==========================================
        // STAGE 3: ARM 2 MOVES TO GRAB FROM ARM 1
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Stage 3: Arm 2 moving to grab object...");
        
        // Open Arm 2 gripper before it gets close
        controlGripper("arm_2", true);
        std::this_thread::sleep_for(5s); // Give it a second to secure the grip


        std::vector<std::vector<double>> a2_grab_wps = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.334886, -0.610753, -0.215607, 2.2043, -0.611461, 0.630781},
            {0.465902, -0.533578, -0.107396, 2.82354, -1.04, 1.46849} // Grab Pose
        };
        std::vector<double> a2_grab_times = {10.0, 10.0, 10.0};

        // moveJoints("arm_2", a2_grab_wps, a2_grab_times);
        moveImpedance("arm_2", a2_grab_wps, a2_grab_times);
        // Close Arm 2 gripper to take the object
        RCLCPP_INFO(this->get_logger(), "Stage 3.5: Transferring object to Arm 2...");
        controlGripper("arm_2", false, 50.0);
        std::this_thread::sleep_for(7s);

        // Open Arm 1 to release the object
        controlGripper("arm_1", true);
        std::this_thread::sleep_for(5s);

        // ==========================================
        // STAGE 4: RETURN HOME (Optional Cleanup)
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Stage 4: Both arms returning home...");
        
        std::vector<std::vector<double>> home = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };
        
        // Move Arm 1 out of the way first
        moveJoints("arm_1", home, std::vector<double>{10.0});
        // Move Arm 2 away
        // moveJoints("arm_2", home, std::vector<double>{10.0});
        moveImpedance("arm_2", home, std::vector<double>{10.0});
        controlGripper("arm_2", true, 50.0);

        RCLCPP_INFO(this->get_logger(), "--- DEMO COMPLETE ---");
    }

    // --- Members ---
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm1_joint_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm2_joint_client_;
    // rclcpp_action::Client<FollowCartesianTrajectory>::SharedPtr arm1_cartesian_client_;
    // rclcpp_action::Client<FollowCartesianTrajectory>::SharedPtr arm2_cartesian_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm1_impedance_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm2_impedance_client_;
    
    rclcpp::Client<GripperSrv>::SharedPtr arm1_gripper_client_;
    rclcpp::Client<GripperSrv>::SharedPtr arm2_gripper_client_;
    
    std::thread demo_thread_;
};

} // namespace dual_arm_demo

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<dual_arm_demo::MultiControllerDemo>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}