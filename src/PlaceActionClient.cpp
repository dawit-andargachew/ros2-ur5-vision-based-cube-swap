#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>

#include "group_6_assignment_2/action/place.hpp"

using namespace std::chrono_literals;

class PlaceActionClient : public rclcpp::Node
{
public:
  using Place = group_6_assignment_2::action::Place;

  PlaceActionClient()
  : Node("place_action_client")
  {
    client_ = rclcpp_action::create_client<Place>(this, "place");
  }

  void send_goal(uint8_t cube_id)
  {
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "Place action server not available");
      return;
    }

    Place::Goal goal;
    goal.cube_id = cube_id;

    client_->async_send_goal(goal);
  }

private:
  rclcpp_action::Client<Place>::SharedPtr client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlaceActionClient>();
  node->send_goal(1);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
