/**
 * Node: home_action_server
 * 
 * type: action server node
 * 
 * purpose:
 * Handles home position requests for the robotic arm.
 * Provides an action server that moves the arm to a predefined home configuration.
 * The home position is loaded from ROS parameters and executed using MoveIt.
 * 
 * 
 * actions:
 *  Provides an action server "home" that moves the robot to the home position.
 */

#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "group_6_assignment_2/action/home.hpp"
#include "group_6_assignment_2/motion_util.hpp"

using namespace std::chrono_literals;

namespace home_action_cpp
{

class HomeActionServer : public rclcpp::Node
{
public:
  using Home = group_6_assignment_2::action::Home;
  using GoalHandleHome = rclcpp_action::ServerGoalHandle<Home>;

  explicit HomeActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("home_action_server", options),
    arm_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), motion_util::kArmGroupName)
  {
    declare_home_parameters();
    load_home_joint_configuration();

    action_server_ = rclcpp_action::create_server<Home>(
      this,
      "home",
      std::bind(&HomeActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&HomeActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&HomeActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Home Action Server ready");
  }

private:

  // Members
  rclcpp_action::Server<Home>::SharedPtr action_server_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  std::map<std::string, double> home_joint_config_;

  // Parameters
  void declare_home_parameters()
  {
    this->declare_parameter("home_position.shoulder_pan_joint", 0.0);
    this->declare_parameter("home_position.shoulder_lift_joint", -1.5708);
    this->declare_parameter("home_position.elbow_joint", 0.0);
    this->declare_parameter("home_position.wrist_1_joint", -1.5708);
    this->declare_parameter("home_position.wrist_2_joint", 0.0);
    this->declare_parameter("home_position.wrist_3_joint", 0.0);
  }

  void load_home_joint_configuration()
  {
    home_joint_config_["shoulder_pan_joint"] =
      this->get_parameter("home_position.shoulder_pan_joint").as_double();
    home_joint_config_["shoulder_lift_joint"] =
      this->get_parameter("home_position.shoulder_lift_joint").as_double();
    home_joint_config_["elbow_joint"] =
      this->get_parameter("home_position.elbow_joint").as_double();
    home_joint_config_["wrist_1_joint"] =
      this->get_parameter("home_position.wrist_1_joint").as_double();
    home_joint_config_["wrist_2_joint"] =
      this->get_parameter("home_position.wrist_2_joint").as_double();
    home_joint_config_["wrist_3_joint"] =
      this->get_parameter("home_position.wrist_3_joint").as_double();

    RCLCPP_INFO(get_logger(), "Loaded home joint configuration from parameters");
  }

  // Action callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Home::Goal>)
  {
    RCLCPP_INFO(get_logger(), "Received request to move robot to home position");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleHome>)
  {
    RCLCPP_WARN(get_logger(), "Home action cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleHome> goal_handle)
  {
    std::thread{
      std::bind(&HomeActionServer::execute, this, std::placeholders::_1),
      goal_handle
    }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleHome> goal_handle)
  {
    auto feedback = std::make_shared<Home::Feedback>();
    auto result   = std::make_shared<Home::Result>();

    feedback->status = "Planning motion to home position";
    goal_handle->publish_feedback(feedback);

    if (!arm_group_.getCurrentState(1.0)) {
      result->success = false;
      result->message = "Robot state unavailable";
      goal_handle->abort(result);
      return;
    }

    feedback->status = "Executing motion to home position";
    goal_handle->publish_feedback(feedback);

    bool success = motion_util::move_to_joint_configuration(
      home_joint_config_,
      arm_group_,
      "home",
      get_logger()
    );

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Home action canceled";
      goal_handle->canceled(result);
      return;
    }

    result->success = success;
    result->message = success ? "Robot reached home position"
                              : "Failed to reach home position";

    if (success) {
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Robot successfully moved to home position");
    } else {
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "Failed to move robot to home position");
    }
  }
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<home_action_cpp::HomeActionServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
