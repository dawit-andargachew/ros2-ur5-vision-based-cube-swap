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
#include "group_6_assignment_2/motion_util.hpp"
#include <tf2/LinearMath/Quaternion.h>

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
    const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    place_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/place_pose", pose_qos);

    MotionConfig config;  // Use default configuration
    configure_moveit(arm_group_, gripper_group_, config);
    apply_path_constraints(arm_group_, get_logger());  // ADDED: Apply path constraints

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
  // Debug visualization
  // ======================
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr place_pose_pub_;

  // ======================
  // Action callbacks
  // ======================
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const Place::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received place goal request for grabbed_cube_id %d onto target_table_id %d",
                goal->grabbed_cube_id, goal->target_table_id);
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
    uint8_t grabbed_cube_id = goal_handle->get_goal()->grabbed_cube_id;
    uint8_t target_table_id = goal_handle->get_goal()->target_table_id;
    RCLCPP_INFO(get_logger(), "Executing place for grabbed_cube_id: %d onto target_table_id: %d",
                grabbed_cube_id, target_table_id);

    bool success = run_place_sequence(grabbed_cube_id, target_table_id, goal_handle);
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
    uint8_t grabbed_cube_id,
    uint8_t target_table_id,
    const std::shared_ptr<GoalHandlePlace> goal_handle)
  {

    auto feedback = std::make_shared<Place::Feedback>();

    feedback->status = "Starting";
    goal_handle->publish_feedback(feedback);
    const std::string current_cube_id = "cube_" + std::to_string(grabbed_cube_id);
    
    //STEP 1 compute goal place pose
    RCLCPP_INFO(get_logger(), "=== PLACE SEQUENCE START: cube_%d -> table_%d ===", grabbed_cube_id, target_table_id);
    feedback->status = "Computing place pose";
    goal_handle->publish_feedback(feedback);
    
    std::optional<geometry_msgs::msg::Pose> place_pose_opt = compute_goal_place_pose(target_table_id);
    if (!place_pose_opt) {
      RCLCPP_ERROR(get_logger(), "Failed to compute goal place pose. Check target_table_id validity");
      return false;
    }
    geometry_msgs::msg::Pose place_pose = *place_pose_opt;
    publish_debug_pose(place_pose, place_pose_pub_);
    RCLCPP_INFO(get_logger(), "Target place pose: x=%.3f, y=%.3f, z=%.3f | orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                place_pose.position.x, place_pose.position.y, place_pose.position.z,
                place_pose.orientation.x, place_pose.orientation.y, place_pose.orientation.z, place_pose.orientation.w);

  
    //STEP 2 move to pre-place pose
    RCLCPP_INFO(get_logger(), "STEP 2: Moving to pre-place pose with orientation constraint");
    feedback->status = "Moving to pre-place pose";
    goal_handle->publish_feedback(feedback);

    // // Add orientation constraint to keep gripper pointing down during motion
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = "world";
    oc.link_name = arm_group_.getEndEffectorLink();
    oc.orientation = place_pose.orientation;  // Keep target orientation (downward)
    oc.absolute_x_axis_tolerance = 3.14159;  // Allow ~180 degrees deviation, we just want to prevent a full 360 flip
    oc.absolute_y_axis_tolerance = 3.14159;
    oc.absolute_z_axis_tolerance = 3.14159;
    oc.weight = 1.0; // we just want to suggest a little bit, not enforce strictly

    // new path constraints
    moveit_msgs::msg::Constraints old_path_constraints = arm_group_.getPathConstraints();
    moveit_msgs::msg::Constraints path_constraints = old_path_constraints; // start from existing constraints
  

    path_constraints.orientation_constraints.push_back(oc);
    arm_group_.setPathConstraints(path_constraints);
    RCLCPP_INFO(get_logger(), "Applied orientation constraint: link=%s, tolerance=±180°", oc.link_name.c_str());

    bool place_success = motion_util::plan_and_execute_to_pose(place_pose, arm_group_, get_logger());
    //back to old constraints
    arm_group_.setPathConstraints(old_path_constraints);
    RCLCPP_INFO(get_logger(), "Removed orientation constraint, joint constraints remain active");

    if(!place_success) {
      RCLCPP_ERROR(get_logger(), "FAILED: Could not reach pre-place pose");
      return false;
    }
    RCLCPP_INFO(get_logger(), "SUCCESS: Reached pre-place pose");
    if (goal_handle->is_canceling()) return false;

    //STEP 3 OPEN THE ARM
    RCLCPP_INFO(get_logger(), "STEP 3: Opening gripper to release cube");
    feedback->status = "Opening gripper to release object";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::setGripper(0.0, gripper_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "FAILED: Could not open gripper");
      return false;
    }
    RCLCPP_INFO(get_logger(), "SUCCESS: Gripper opened, cube released");
    if (goal_handle->is_canceling()) return false;

    // STEP 4 DETACH OBJECT
    RCLCPP_INFO(get_logger(), "STEP 4: Detaching cube from planning scene");
    feedback->status = "Detaching object from gripper";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::detachObject(current_cube_id, arm_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "FAILED: Could not detach cube from gripper");
      return false;
    }
    RCLCPP_INFO(get_logger(), "SUCCESS: Cube detached from planning scene");
    if (goal_handle->is_canceling()) return false;
    rclcpp::sleep_for(1s); // Brief pause to let physics settle


    //remove the object from the planning scene to avoid collision issues
    RCLCPP_INFO(get_logger(), "Removing cube from planning scene collision objects");
    planning_scene_.removeCollisionObjects({current_cube_id});
    rclcpp::sleep_for(500ms); // Brief pause to let planning scene update


    // STEP FINISH
    RCLCPP_INFO(get_logger(), "=== PLACE SEQUENCE COMPLETE: cube_%s placed on table_%s ===", 
                current_cube_id.c_str(), std::to_string(target_table_id).c_str());
    RCLCPP_INFO(get_logger(), "Place action completed successfully");
    return true;
  
  }

  // compute goal pose function returns a pose
  // simple offset depending on the table id
  std::optional<geometry_msgs::msg::Pose> compute_goal_place_pose(
    uint8_t target_table_id)
  {  
    geometry_msgs::msg::Pose place_pose;
    if (target_table_id == 1) {
      place_pose.position.x = 3.97 + 0.10;
      place_pose.position.y = -1.10 - 0.05;
      place_pose.position.z = 0.446 + 0.18;
    } else if (target_table_id == 2) {
      place_pose.position.x = 4.56 - 0.05;
      place_pose.position.y = -0.50 - 0.10;
      place_pose.position.z = 0.446 + 0.18;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid target_table_id: %d", target_table_id);
      return std::nullopt;
    }

    // set quaternion via RPY
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI, 0.0);
    q.normalize();

    place_pose.orientation.x = q.x();
    place_pose.orientation.y = q.y();
    place_pose.orientation.z = q.z();
    place_pose.orientation.w = q.w();

    return place_pose;

  }

  void publish_debug_pose(
    const geometry_msgs::msg::Pose& pose,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
    const std::string& frame_id = "world")
  {
    if (!publisher) {
      return;
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    msg.pose = pose;
    publisher->publish(msg);
  }
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<place_action_cpp::PlaceActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
