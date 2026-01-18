/**
 * Node: pick_action_client
 * 
 * type: action client node
 * 
 * purpose:
 * Sends requests to the pick action server to pick up cubes.
 * Communicates the cube ID to be picked and waits for action completion.
 * Handles feedback and result messages from the pick action server.
 * 
 * 
 * actions:
 *  Sends goals to the "pick" action server with cube ID to trigger pick operations.
 */

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "group_6_assignment_2/action/pick.hpp"

using namespace std::chrono_literals;

class PickActionClient : public rclcpp::Node
{
public:
  using Pick = group_6_assignment_2::action::Pick;
  using GoalHandlePick = rclcpp_action::ClientGoalHandle<Pick>;

  explicit PickActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("pick_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Pick>(this, "pick");
  }

  void send_goal(uint8_t cube_id)
  {
    if (!this->client_ptr_->wait_for_action_server(20s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Pick::Goal();
    goal_msg.cube_id = cube_id;

    RCLCPP_INFO(this->get_logger(), "Sending goal for cube_id: %d", cube_id);

    auto send_goal_options = rclcpp_action::Client<Pick>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&PickActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&PickActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&PickActionClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Pick>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandlePick::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandlePick::SharedPtr,
    const std::shared_ptr<const Pick::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());
  }

  void result_callback(const GoalHandlePick::WrappedResult& result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded with result: %s", result.result->success ? "true" : "false");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickActionClient>();
  
  // Parse cube_id from command-line argument (default: 1)
  uint8_t cube_id = 1;
  if (argc > 1) {
    cube_id = static_cast<uint8_t>(std::stoi(argv[1]));
  }
  
  RCLCPP_INFO(node->get_logger(), "Sending pick goal for cube_id: %d", cube_id);
  node->send_goal(cube_id);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
