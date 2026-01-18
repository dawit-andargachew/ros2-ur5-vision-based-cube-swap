#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "group_6_assignment_2/action/home.hpp"

using namespace std::chrono_literals;

class HomeActionClient : public rclcpp::Node
{
public:
  using Home = group_6_assignment_2::action::Home;
  using GoalHandleHome = rclcpp_action::ClientGoalHandle<Home>;

  HomeActionClient()
  : Node("home_action_client")
  {
    client_ = rclcpp_action::create_client<Home>(this, "home");
    send_goal();
  }

private:
  rclcpp_action::Client<Home>::SharedPtr client_;

  void send_goal()
  {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Home action server not available");
      return;
    }

    Home::Goal goal;

    auto options = rclcpp_action::Client<Home>::SendGoalOptions();
    options.feedback_callback =
      std::bind(&HomeActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
      std::bind(&HomeActionClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal, options);
  }

  void feedback_callback(
    GoalHandleHome::SharedPtr,
    const std::shared_ptr<const Home::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Feedback: %s", feedback->status.c_str());
  }

  void result_callback(const GoalHandleHome::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Home action succeeded: %s", result.result->message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Home action failed");
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomeActionClient>());
  return 0;
}
