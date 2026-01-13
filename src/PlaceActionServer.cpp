#include <chrono>
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
    action_server_ = rclcpp_action::create_server<Place>(
      this,
      "place",
      std::bind(&PlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PlaceActionServer::handle_accepted, this, std::placeholders::_1));

    configure_moveit(arm_group_, gripper_group_);
  }

private:
  // ======================================================
  // Action server
  // ======================================================
  rclcpp_action::Server<Place>::SharedPtr action_server_;

  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  geometry_msgs::msg::Pose cube_pose_;
  geometry_msgs::msg::Pose grasp_pose_;

  geometry_msgs::msg::Pose compute_grasp_pose(const geometry_msgs::msg::Pose& cube_pose) const
  {
    geometry_msgs::msg::Pose grasp_pose = cube_pose;
    grasp_pose.position.z = cube_pose.position.z + kGraspCenterZOffset;
    grasp_pose.position.y = cube_pose.position.y + kGripperLengthOffset;
    // Orientation for top grasp: gripper fingers pointing down, aligned with table
    tf2::Quaternion q;
    const double roll = 1.5707;  
    const double pitch = 0.0;
    const double yaw = 0.0;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    grasp_pose.orientation.x = q.x();
    grasp_pose.orientation.y = q.y();
    grasp_pose.orientation.z = q.z();
    grasp_pose.orientation.w = q.w();

    return grasp_pose;
  }

  // ======================================================
  // Action callbacks
  // ======================================================
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const Place::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received place goal for cube_id %d", goal->cube_id);
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
    std::thread{std::bind(&PlaceActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    auto feedback = std::make_shared<Place::Feedback>();
    auto result = std::make_shared<Place::Result>();

    feedback->status = "Starting place sequence";
    goal_handle->publish_feedback(feedback);

    if (!wait_for_robot_state()) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    if (!setup_scene()) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    if (!run_place_sequence(goal_handle)) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  // ======================================================
  // Place pipeline
  // ======================================================
  bool run_place_sequence(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    auto feedback = std::make_shared<Place::Feedback>();

    feedback->status = "Lowering object";
    goal_handle->publish_feedback(feedback);
    if (!lower_down()) return false;

    feedback->status = "Detaching object";
    goal_handle->publish_feedback(feedback);
    if (!detachObject(kCubeId1, arm_group_, get_logger())) return false;

    rclcpp::sleep_for(500ms);

    feedback->status = "Opening gripper";
    goal_handle->publish_feedback(feedback);
    if (!setGripper(kOpenVal, gripper_group_, get_logger())) return false;

    feedback->status = "Retreating from object";
    goal_handle->publish_feedback(feedback);
    if (!retreat()) return false;

    return true;
  }

  // ======================================================
  // Motion helpers
  // ======================================================
  bool lower_down()
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose_);
    return executeCartesianPath(waypoints, 0.01, 0.3, arm_group_, get_logger());
  }

  bool retreat()
  {
    geometry_msgs::msg::Pose retreat_pose = grasp_pose_;
    retreat_pose.position.y += kPreGraspApproachDist;
    return plan_and_execute_to_pose(retreat_pose, arm_group_, get_logger());
  }

  // ======================================================
  // Scene + state
  // ======================================================
  bool wait_for_robot_state()
  {
    return arm_group_.getCurrentState(2.0) != nullptr;
  }

  bool setup_scene()
  {
    if (!motion_util::setup_scene(planning_scene_)) return false;

    constexpr double kOffsetXLateral = +0.035;
    cube_pose_.position.x = 3.97 + kOffsetXLateral;
    cube_pose_.position.y = -1.10;
    cube_pose_.position.z = 0.4;
    cube_pose_.orientation.w = 1.0;

    grasp_pose_ = compute_grasp_pose(cube_pose_);
    return true;
  }
};

} // namespace place_action_cpp

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({ rclcpp::Parameter("use_sim_time", true) });

  auto node = std::make_shared<place_action_cpp::PlaceActionServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
