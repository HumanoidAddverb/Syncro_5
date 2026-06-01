#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <addverb_cobot_msgs/srv/record.hpp>
#include <addverb_cobot_msgs/action/replay.hpp>

using namespace std::chrono_literals;

class RecorderClient : public rclcpp::Node
{
    public:
      using RecordSrv = addverb_cobot_msgs::srv::Record;
      using ReplayAction = addverb_cobot_msgs::action::Replay;
      using GoalHandleReplay = rclcpp_action::ClientGoalHandle<ReplayAction>;
    
      RecorderClient() : Node("demo_recorder")
      {
        record_client_ = this->create_client<RecordSrv>("/arm_1_recorder_controller/record_mode");
        replay_client_ = rclcpp_action::create_client<ReplayAction>(
            this,
            "/recorder_controller/replay_mode");
      }
    
      void run()
      {
        if (!record_client_->wait_for_service(5s)) {
          RCLCPP_ERROR(this->get_logger(), "Record service not available.");
          return;
        }
    
        if (!replay_client_->wait_for_action_server(5s)) {
          RCLCPP_ERROR(this->get_logger(), "Replay action server not available.");
          return;
        }
    
        auto start_req = std::make_shared<RecordSrv::Request>();
        start_req->enable = true;
        start_req->label = "demo_motion";
        start_req->rate = 20;
    
        RCLCPP_INFO(this->get_logger(), "Starting recording...");
        auto start_future = record_client_->async_send_request(start_req);
        auto start_result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), start_future);
    
        if (start_result == rclcpp::FutureReturnCode::SUCCESS)
          RCLCPP_INFO(this->get_logger(), "Record start response: %s", start_future.get()->message.c_str());
        else
          RCLCPP_ERROR(this->get_logger(), "Failed to call start recording service.");
    
        RCLCPP_INFO(this->get_logger(), "Recording for 60 seconds...");
        rclcpp::sleep_for(60s);
    
        auto stop_req = std::make_shared<RecordSrv::Request>();
        stop_req->enable = false;
        stop_req->label = "demo_motion";  // same label to save
        stop_req->rate = 0;
    
        RCLCPP_INFO(this->get_logger(), "Stopping recording...");
        auto stop_future = record_client_->async_send_request(stop_req);
        auto stop_result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), stop_future);
    
        if (stop_result == rclcpp::FutureReturnCode::SUCCESS)
          RCLCPP_INFO(this->get_logger(), "Record stop response: %s", stop_future.get()->message.c_str());
        else
          RCLCPP_ERROR(this->get_logger(), "Failed to call stop recording service.");
    
        RCLCPP_INFO(this->get_logger(), "Sending replay goal...");
        ReplayAction::Goal goal;
        goal.label = "demo_motion";
        goal.iterations = 1;
    
        auto send_goal_options = rclcpp_action::Client<ReplayAction>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](GoalHandleReplay::SharedPtr, const std::shared_ptr<const ReplayAction::Feedback> feedback)
            {
              RCLCPP_INFO(this->get_logger(), "Feedback iteration: %d", feedback->iteration);
            };
    
        send_goal_options.result_callback =
            [this](const GoalHandleReplay::WrappedResult &result)
            {
              if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(this->get_logger(), "Replay completed successfully: %s", result.result->error_code.c_str());
              else
                RCLCPP_WARN(this->get_logger(), "Replay failed or was canceled.");
            };
    
        replay_client_->async_send_goal(goal, send_goal_options);
      }
    
    private:
      rclcpp::Client<RecordSrv>::SharedPtr record_client_;
      rclcpp_action::Client<ReplayAction>::SharedPtr replay_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecorderClient>();
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
