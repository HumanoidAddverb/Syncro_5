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

  RecorderClient() : Node("demo_recorder_dual")
  {
    // Service clients
    record_client_arm1_ = this->create_client<RecordSrv>("/arm_1_recorder_controller/record_mode");
    record_client_arm2_ = this->create_client<RecordSrv>("/arm_2_recorder_controller/record_mode");

    // Action clients
    replay_client_arm1_ = rclcpp_action::create_client<ReplayAction>(
        this, "/arm_1_recorder_controller/replay_mode");

    replay_client_arm2_ = rclcpp_action::create_client<ReplayAction>(
        this, "/arm_2_recorder_controller/replay_mode");
  }

  void run()
  {
    // Wait for services
    if (!record_client_arm1_->wait_for_service(5s) ||
        !record_client_arm2_->wait_for_service(5s))
    {
      RCLCPP_ERROR(this->get_logger(), "Record services not available.");
      return;
    }

    if (!replay_client_arm1_->wait_for_action_server(5s) ||
        !replay_client_arm2_->wait_for_action_server(5s))
    {
      RCLCPP_ERROR(this->get_logger(), "Replay action servers not available.");
      return;
    }

    // ------------------ START RECORDING ------------------
    auto start_record = [&](auto client, const std::string &label)
    {
      auto req = std::make_shared<RecordSrv::Request>();
      req->enable = true;
      req->label = label;
      req->rate = 20;

      auto future = client->async_send_request(req);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    };

    RCLCPP_INFO(this->get_logger(), "Starting recording for both arms...");
    start_record(record_client_arm1_, "arm1_demo_motion");
    start_record(record_client_arm2_, "arm2_demo_motion");

    RCLCPP_INFO(this->get_logger(), "Recording for 60 seconds...");
    rclcpp::sleep_for(60s);

    // ------------------ STOP RECORDING ------------------
    auto stop_record = [&](auto client, const std::string &label)
    {
      auto req = std::make_shared<RecordSrv::Request>();
      req->enable = false;
      req->label = label;
      req->rate = 0;

      auto future = client->async_send_request(req);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    };

    RCLCPP_INFO(this->get_logger(), "Stopping recording...");
    stop_record(record_client_arm1_, "arm1_demo_motion");
    stop_record(record_client_arm2_, "arm2_demo_motion");

    // ------------------ REPLAY ------------------
    auto send_replay = [&](auto client, const std::string &label)
    {
      ReplayAction::Goal goal;
      goal.label = label;
      goal.iterations = 1;

      auto options = rclcpp_action::Client<ReplayAction>::SendGoalOptions();

      options.feedback_callback =
          [this, label](GoalHandleReplay::SharedPtr,
                        const std::shared_ptr<const ReplayAction::Feedback> feedback)
      {
        RCLCPP_INFO(this->get_logger(), "[%s] Iteration: %d",
                    label.c_str(), feedback->iteration);
      };

      options.result_callback =
          [this, label](const GoalHandleReplay::WrappedResult &result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
          RCLCPP_INFO(this->get_logger(), "[%s] Replay success", label.c_str());
        else
          RCLCPP_WARN(this->get_logger(), "[%s] Replay failed", label.c_str());
      };

      client->async_send_goal(goal, options);
    };

    RCLCPP_INFO(this->get_logger(), "Replaying both arms...");
    send_replay(replay_client_arm1_, "arm1_demo_motion");
    send_replay(replay_client_arm2_, "arm2_demo_motion");
  }

private:
  rclcpp::Client<RecordSrv>::SharedPtr record_client_arm1_;
  rclcpp::Client<RecordSrv>::SharedPtr record_client_arm2_;

  rclcpp_action::Client<ReplayAction>::SharedPtr replay_client_arm1_;
  rclcpp_action::Client<ReplayAction>::SharedPtr replay_client_arm2_;
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