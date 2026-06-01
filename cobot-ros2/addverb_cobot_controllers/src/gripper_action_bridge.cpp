#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "addverb_cobot_msgs/srv/gripper.hpp"

using namespace std::chrono_literals;

using GripperCommand = control_msgs::action::GripperCommand;
using GripperSrv = addverb_cobot_msgs::srv::Gripper;

class GripperActionServer : public rclcpp::Node
{
public:
  GripperActionServer()
  : Node("gripper_action_server")
  {
     // 1. Declare the parameter with a default fallback
    this->declare_parameter<std::string>("arm_prefix", "arm_1");
    
    // 2. Read the parameter provided by the launch file
    std::string arm_prefix;
    this->get_parameter("arm_prefix", arm_prefix);

    // 3. Construct your strings dynamically
    std::string srv_name = "/" + arm_prefix + "gripper_controller/command";
    std::string action_name = "/" + arm_prefix + "gripper_controller/gripper_cmd";

    srv_client_ = this->create_client<GripperSrv>(srv_name);


    RCLCPP_INFO(this->get_logger(), "Starting Gripper Action Server for: %s", arm_prefix.c_str());

    // EXACT action name MoveIt expects
    action_server_ = rclcpp_action::create_server<GripperCommand>(
      this,
      action_name,
      std::bind(&GripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&GripperActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Gripper Action Server READY (Humble)");
  }

private:
  rclcpp::Client<GripperSrv>::SharedPtr srv_client_;
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;

  /* ---------- Goal handling ---------- */

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperCommand::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> goal_handle)
  {
    // NO threads, NO spinning
    execute(goal_handle);
  }

  /* ---------- Execution ---------- */

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    if (srv_client_->wait_for_service(100ms)) {
      auto req = std::make_shared<GripperSrv::Request>();

      bool want_open = goal->command.position < 0.5;

      if (want_open) {
        req->position = 1;       // OPEN
      } else {
        req->position = 0;   // CLOSE
      }

      req->grasp_force =
        (goal->command.max_effort > 0.0) ? goal->command.max_effort : 50.0;

      srv_client_->async_send_request(req);
    }

    auto result = std::make_shared<GripperCommand::Result>();
    result->position = goal->command.position;  // semantic
    result->effort = goal->command.max_effort;
    result->stalled = false;
    result->reached_goal = true;

    goal_handle->succeed(result);
  }


};

/* ---------- main ---------- */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperActionServer>();

  // REQUIRED
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
