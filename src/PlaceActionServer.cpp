#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "group_6_assignment_2/action/place.hpp"
#include "group_6_assignment_2/srv/get_cube_poses.hpp"
#include "group_6_assignment_2/motion_util.hpp"

using namespace std::chrono_literals;
using namespace motion_util;

namespace place_action_cpp
{

class PlaceActionServer : public rclcpp::Node
{
public:
  using Place = group_6_assignment_2::action::Place;
  using GoalHandlePlace = rclcpp_action::ServerGoalHandle<Place>;

  explicit PlaceActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("place_action_server", options),
    arm_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kArmGroupName),
    gripper_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kGripperGroupName)
  {
    configure_moveit(arm_group_, gripper_group_);

    cube_pose_client_ =
      this->create_client<group_6_assignment_2::srv::GetCubePoses>("get_cube_poses");

    action_server_ = rclcpp_action::create_server<Place>(
      this,
      "place",
      std::bind(&PlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PlaceActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  // ======================
  // Action server
  // ======================
  rclcpp_action::Server<Place>::SharedPtr action_server_;

  // ======================
  // MoveIt
  // ======================
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  // ======================
  // Perception
  // ======================
  rclcpp::Client<group_6_assignment_2::srv::GetCubePoses>::SharedPtr cube_pose_client_;
  geometry_msgs::msg::Pose reference_cube_pose_;

  // ======================
  // Parameters
  // ======================
  const double place_offset_y_ = 0.60;
  const double pre_place_offset_z_ = 0.20;
  const double retreat_offset_z_ = 0.20;

  // ======================
  // Action callbacks
  // ======================
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const Place::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "Received place goal relative to cube_id %d",
      goal->reference_cube_id);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePlace>)
  {
    RCLCPP_WARN(get_logger(), "Place goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    std::thread{
      std::bind(&PlaceActionServer::execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    auto result = std::make_shared<Place::Result>();
    bool success = run_place_sequence(goal_handle->get_goal()->reference_cube_id, goal_handle);
    result->success = success;

    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ======================
  // Place sequence
  // ======================
  bool run_place_sequence(
    uint8_t reference_cube_id,
    const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    auto feedback = std::make_shared<Place::Feedback>();

    feedback->status = "Sensing reference cube";
    goal_handle->publish_feedback(feedback);
    if (!sense_reference_cube(reference_cube_id)) return false;

    if (goal_handle->is_canceling()) return false;

    geometry_msgs::msg::Pose place_pose = compute_place_pose();

    feedback->status = "Moving to pre-place";
    goal_handle->publish_feedback(feedback);
    if (!move_to_pre_place(place_pose)) return false;

    feedback->status = "Descending to place";
    goal_handle->publish_feedback(feedback);
    if (!move_to_place_cartesian(place_pose)) return false;

    feedback->status = "Opening gripper";
    goal_handle->publish_feedback(feedback);
    if (!setGripper(0.0, gripper_group_, get_logger())) return false;

    feedback->status = "Detaching object";
    goal_handle->publish_feedback(feedback);
    if (!detachObject(kCubeId, arm_group_, get_logger())) return false;

    feedback->status = "Retreating";
    goal_handle->publish_feedback(feedback);
    if (!retreat_from_place(place_pose)) return false;

    feedback->status = "Place complete";
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(get_logger(), "Place action completed successfully");
    return true;
  }

  // ======================
  // Helpers
  // ======================
  bool sense_reference_cube(uint8_t cube_id)
  {
    if (!cube_pose_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "GetCubePoses service unavailable");
      return false;
    }

    auto req = std::make_shared<group_6_assignment_2::srv::GetCubePoses::Request>();
    req->target_ids.push_back(static_cast<int32_t>(cube_id));

    auto future = cube_pose_client_->async_send_request(req);
    if (future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Cube pose service timeout");
      return false;
    }

    auto res = future.get();
    if (!res->success || res->poses.empty()) return false;

    reference_cube_pose_ = res->poses[0];
    return true;
  }

  geometry_msgs::msg::Pose compute_place_pose()
  {
    geometry_msgs::msg::Pose pose = reference_cube_pose_;
    pose.position.y += place_offset_y_;
    return pose;
  }

  bool move_to_pre_place(const geometry_msgs::msg::Pose& place_pose)
  {
    geometry_msgs::msg::Pose pre = place_pose;
    pre.position.z += pre_place_offset_z_;
    return plan_and_execute_to_pose(pre, arm_group_, get_logger());
  }

  bool move_to_place_cartesian(const geometry_msgs::msg::Pose& place_pose)
  {
    return executeCartesianPath(
      {place_pose}, 0.01, 0.15, arm_group_, get_logger());
  }

  bool retreat_from_place(const geometry_msgs::msg::Pose& place_pose)
  {
    geometry_msgs::msg::Pose retreat = place_pose;
    retreat.position.z += retreat_offset_z_;
    return executeCartesianPath(
      {retreat}, 0.01, 0.15, arm_group_, get_logger());
  }
};

}  // namespace place_action_cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<place_action_cpp::PlaceActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
