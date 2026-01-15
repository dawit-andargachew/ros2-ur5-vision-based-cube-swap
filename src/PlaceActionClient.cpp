#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "group_6_assignment_2/action/place.hpp"

using namespace std::chrono_literals;

class PlaceActionClient : public rclcpp::Node
{
public:
  using Place = group_6_assignment_2::action::Place;
  using GoalHandlePlace = rclcpp_action::ClientGoalHandle<Place>;

  PlaceActionClient()
  : Node("place_action_client")
  {
    client_ = rclcpp_action::create_client<Place>(this, "place");
  }

  void send_goal(uint8_t reference_cube_id)
  {
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "Place action server not available");
      return;
    }

    Place::Goal goal;
    goal.reference_cube_id = reference_cube_id;

    auto options = rclcpp_action::Client<Place>::SendGoalOptions();
    options.feedback_callback =
      std::bind(&PlaceActionClient::feedback_cb, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
      std::bind(&PlaceActionClient::result_cb, this, std::placeholders::_1);

    client_->async_send_goal(goal, options);
  }

private:
  rclcpp_action::Client<Place>::SharedPtr client_;

  void feedback_cb(
    GoalHandlePlace::SharedPtr,
    const std::shared_ptr<const Place::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Feedback: %s", feedback->status.c_str());
  }

  void result_cb(const GoalHandlePlace::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Place succeeded");
    } else {
      RCLCPP_ERROR(get_logger(), "Place failed");
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlaceActionClient>();
  node->send_goal(10);  // place relative to cube 2
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
