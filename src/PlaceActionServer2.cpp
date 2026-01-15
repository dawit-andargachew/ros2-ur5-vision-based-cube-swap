#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Include your custom interfaces and utils
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
    // Publishers for debugging
    const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/place_target_pose", pose_qos);

    // Initialize MoveIt via your util function
    configure_moveit(arm_group_, gripper_group_);

    // Create Action Server
    this->action_server_ = rclcpp_action::create_server<Place>(
      this,
      "place",
      std::bind(&PlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PlaceActionServer::handle_accepted, this, std::placeholders::_1));

    // Load Parameters (Home/Active positions)
    declare_and_get_parameters();
  }

private:
  rclcpp_action::Server<Place>::SharedPtr action_server_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

  // Parameters to hold joint configurations
  std::map<std::string, double> home_joint_config_;
  std::map<std::string, double> active_joint_config_;
  double place_approach_z_offset_;
  double gripper_open_value_;

  // --- Action Lifecycle Methods ---

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Place::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received PLACE request for cube ID: %d", goal->cube_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePlace>)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel Place action");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    // Spin up a new thread so we don't block the executor
    std::thread{std::bind(&PlaceActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // --- Main Execution Logic ---

  void execute(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Place::Feedback>();
    auto result = std::make_shared<Place::Result>();
    
    /*
    // 1. Move to "Active" position (Safe travel height)
    feedback->status = "Moving to Active Position";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::move_to_joint_configuration(active_joint_config_, arm_group_, "active", get_logger())) {
      result->success = false;
      goal_handle->abort(result);
      return;
    }
    */

    // 2. Calculate "Pre-Place" Pose (Target + Z offset)
    geometry_msgs::msg::Pose pre_place_pose = goal->place_pose;
    pre_place_pose.position.z += place_approach_z_offset_; // e.g., +0.15m

    // Debug: Publish where we are trying to go
    publish_debug_pose(goal->place_pose);

    // 3. Move to Pre-Place Pose (using standard planning)
    feedback->status = "Moving to Pre-Place approach point";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::plan_and_execute_to_pose(pre_place_pose, arm_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to reach pre-place pose");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    // 4. Cartesian Approach (Linear Move Down)
    feedback->status = "Descending to Place Surface";
    goal_handle->publish_feedback(feedback);
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    approach_waypoints.push_back(goal->place_pose); // The actual target on the table
    
    // Use motion_util Cartesian helper
    // 0.01 = eef_step, 0.5 = jump_threshold
    if (!motion_util::executeCartesianPath(approach_waypoints, 0.01, 0.0, arm_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Cartesian approach failed");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    // 5. Open Gripper
    feedback->status = "Releasing Object";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::setGripper(gripper_open_value_, gripper_group_, get_logger())) {
      RCLCPP_WARN(get_logger(), "Gripper failed to open fully, proceeding anyway...");
    }

    // 6. Detach Object (CRITICAL UPDATE)
    // We must tell MoveIt the cube is no longer attached to the hand.
    // Note: PickActionServer attached it as 'kCubeId'. We must detach 'kCubeId'.
    feedback->status = "Detaching collision object";
    goal_handle->publish_feedback(feedback);
    motion_util::detachObject(kCubeId, arm_group_, get_logger());
    
    rclcpp::sleep_for(1s); // Brief pause to let physics settle

    // 7. Retreat (Move back up to Pre-Place)
    feedback->status = "Retreating";
    goal_handle->publish_feedback(feedback);
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    retreat_waypoints.push_back(pre_place_pose);
    
    if (!motion_util::executeCartesianPath(retreat_waypoints, 0.01, 0.0, arm_group_, get_logger())) {
      // If Cartesian fails, try standard planning to retreat
      motion_util::plan_and_execute_to_pose(pre_place_pose, arm_group_, get_logger());
    }

    // 8. Return to Home
    feedback->status = "Returning Home";
    goal_handle->publish_feedback(feedback);
    motion_util::move_to_joint_configuration(active_joint_config_, arm_group_, "active", get_logger());
    motion_util::move_to_joint_configuration(home_joint_config_, arm_group_, "home", get_logger());

    RCLCPP_INFO(get_logger(), "Place Action Complete");
    result->success = true;
    goal_handle->succeed(result);
  }

  // --- Helper Functions ---

  void publish_debug_pose(const geometry_msgs::msg::Pose& pose)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = kWorldFrame;
    msg.pose = pose;
    target_pose_pub_->publish(msg);
  }

  void declare_and_get_parameters()
  {
    // Define offset for approach (default 15cm)
    place_approach_z_offset_ = this->declare_parameter("place.approach_z_offset", 0.15);
    gripper_open_value_ = this->declare_parameter("gripper.open", 0.0);

    // Hardcoded defaults matching your PickServer for consistency
    // (Ideally, load these from the same YAML file)
    active_joint_config_["shoulder_pan_joint"] = this->declare_parameter("active_position.shoulder_pan_joint", -1.64);
    active_joint_config_["shoulder_lift_joint"] = this->declare_parameter("active_position.shoulder_lift_joint", -1.46);
    active_joint_config_["elbow_joint"] = this->declare_parameter("active_position.elbow_joint", 1.12);
    active_joint_config_["wrist_1_joint"] = this->declare_parameter("active_position.wrist_1_joint", -1.48);
    active_joint_config_["wrist_2_joint"] = this->declare_parameter("active_position.wrist_2_joint", -1.37);
    active_joint_config_["wrist_3_joint"] = this->declare_parameter("active_position.wrist_3_joint", -0.38);

    home_joint_config_["shoulder_pan_joint"] = this->declare_parameter("home_position.shoulder_pan_joint", 0.0);
    home_joint_config_["shoulder_lift_joint"] = this->declare_parameter("home_position.shoulder_lift_joint", -1.5708);
    home_joint_config_["elbow_joint"] = this->declare_parameter("home_position.elbow_joint", 0.0);
    home_joint_config_["wrist_1_joint"] = this->declare_parameter("home_position.wrist_1_joint", -1.5708);
    home_joint_config_["wrist_2_joint"] = this->declare_parameter("home_position.wrist_2_joint", 0.0);
    home_joint_config_["wrist_3_joint"] = this->declare_parameter("home_position.wrist_3_joint", 0.0);
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