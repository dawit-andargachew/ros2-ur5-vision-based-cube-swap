#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "group_6_assignment_2/motion_util.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "group_6_assignment_2/action/pick.hpp"
#include "group_6_assignment_2/srv/get_cube_poses.hpp"

using namespace std::chrono_literals;
using namespace motion_util;

namespace pick_action_cpp
{
class PickActionServer : public rclcpp::Node
{
public:
  using Pick = group_6_assignment_2::action::Pick;
  using GoalHandlePick = rclcpp_action::ServerGoalHandle<Pick>;
  
  explicit PickActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("pick_action_server", options),
    arm_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kArmGroupName),
    gripper_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kGripperGroupName)
  { 
    const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    grasp_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/grasp_pose", pose_qos);
    pregrasp_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/pregrasp_pose", pose_qos);

    // Create action server
    this-> action_server_ = rclcpp_action::create_server<Pick>(
      this,
      "pick",
      std::bind(&PickActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickActionServer::handle_accepted, this, std::placeholders::_1));

    configure_moveit(arm_group_, gripper_group_);  // Updated: Use motion_util::configure_moveit
    //apply_path_constraints(arm_group_, get_logger());  // ADDED: Apply path constraints

    // Create service client for cube pose detection
    cube_pose_client_ = this->create_client<group_6_assignment_2::srv::GetCubePoses>("get_cube_poses");

    declare_and_get_parameters();
  }

  bool run_pick_sequence(uint8_t cube_id, const std::shared_ptr<GoalHandlePick> goal_handle)
  {

    auto feedback = std::make_shared<Pick::Feedback>();
    feedback->status = "Starting pick sequence";
    goal_handle->publish_feedback(feedback);


    // Wait for robot state before starting
    feedback->status = "Waiting for robot state";
    goal_handle->publish_feedback(feedback);
    if (!wait_for_robot_state()) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot state. Check if /joint_states is being published.");
      return false;
    }
    
    //we check cancelling after each major step of the pipeline
    if (goal_handle->is_canceling()) return false;

    // Sense cube position FIRST (before moving, to get latest position)
    feedback->status = "Sensing cube position";
    goal_handle->publish_feedback(feedback);
    if (!sense_cube(cube_id)) return false;
    if (goal_handle->is_canceling()) return false;

    // Move to home position (safe position before cleanup)
    feedback->status = "Moving to home position";
    goal_handle->publish_feedback(feedback);
    if (!move_to_home()) return false;
    if (goal_handle->is_canceling()) return false;

    // Clean up scene state from any previous attempts (at home position, safe to detach)
    feedback->status = "Cleaning up scene state";
    goal_handle->publish_feedback(feedback);
    if (!cleanup_scene()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Setting up planning scene";
    goal_handle->publish_feedback(feedback);
    if (!setup_scene()) return false;
    if (goal_handle->is_canceling()) return false;
    
    //go to active position
    feedback->status = "Moving to active position";
    goal_handle->publish_feedback(feedback);
    if (!move_to_active()) return false;
    if (goal_handle->is_canceling()) return false;
    
    //we check cancelling after each major step of the pipeline
    if (goal_handle->is_canceling()) return false;

    // Wait for planning scene to update with collision objects
    feedback->status = "Waiting for scene synchronization";
    goal_handle->publish_feedback(feedback);
    std::vector<std::string> expected_objects = {kTableId, kCubeId};
    if (!wait_for_scene_update(expected_objects)) {
      RCLCPP_ERROR(get_logger(), "Planning scene did not update in time");
      return false;
    }

    feedback->status = "Moving to pre-grasp position";
    goal_handle->publish_feedback(feedback);
    if (!move_to_pre_grasp()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Executing grasp";
    goal_handle->publish_feedback(feedback);
    if (!open_gripper()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Moving to grasp position";
    goal_handle->publish_feedback(feedback);
    if (!move_to_grasp_non_cartesian()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Closing gripper and attaching object";
    goal_handle->publish_feedback(feedback);
    if (!close_gripper()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Attaching object to gripper";
    goal_handle->publish_feedback(feedback);
    if (!attach_object()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Lifting object";
    goal_handle->publish_feedback(feedback);
    if (!lift_up_non_cartesian()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Pick sequence complete";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Pick complete!");
    return true;
  }

private:

  rclcpp_action::Server<Pick>::SharedPtr action_server_;
  rclcpp::Client<group_6_assignment_2::srv::GetCubePoses>::SharedPtr cube_pose_client_;
  
  // =========================
  // Members (own the state)
  // =========================
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  geometry_msgs::msg::Pose cube_pose_{};
  geometry_msgs::msg::Pose grasp_pose_{};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pregrasp_pose_pub_;

  void publish_debug_pose(
    const geometry_msgs::msg::Pose& pose,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
    const std::string& frame_id)
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

  struct PickParams
  {
    // Scene
    double table_x = 3.97;
    double table_y = -1.10;
    double table_z = 0.322;

    double cube_base_x = 3.97;
    double cube_base_y = -1.10;
    double cube_base_z = 0.4;
    double cube_offset_x = 0.035;

    // Cube and AprilTag dimensions
    double apriltag_size = 0.050;  // AprilTag is 0.050x0.050m
    double cube_width = 0.06;      // Cube is 0.06x0.06x0.1m
    double cube_depth = 0.06;
    double cube_height = 0.1;
    // Offset from AprilTag center to cube center (tag is on one face)
    double tag_to_cube_center_offset = 0.03;  // Half of cube depth (0.06/2)
    double lateral_x_offset = 0.035;          // Lateral X correction

    // Grasp
    double grasp_roll = 0.0;
    double grasp_pitch = -1.5707;
    double grasp_yaw = 0.0;
    double grasp_center_z_offset = -0.02;
    double tool_length_offset = 0.18;

    // Motions
    double pregrasp_approach_dist_z = 0.15;
    double lift_dist_z = 0.20;

    // Gripper
    double gripper_open = 0.0;
    double gripper_closed = 0.10;
  } params_;

  void declare_and_get_parameters()
  {
    // Scene
    this->declare_parameter<double>("scene.table.x", params_.table_x);
    this->declare_parameter<double>("scene.table.y", params_.table_y);
    this->declare_parameter<double>("scene.table.z", params_.table_z);

    this->declare_parameter<double>("scene.cube.base_x", params_.cube_base_x);
    this->declare_parameter<double>("scene.cube.base_y", params_.cube_base_y);
    this->declare_parameter<double>("scene.cube.base_z", params_.cube_base_z);
    this->declare_parameter<double>("scene.cube.offset_x", params_.cube_offset_x);

    // Cube and AprilTag dimensions
    this->declare_parameter<double>("dimensions.apriltag_size", params_.apriltag_size);
    this->declare_parameter<double>("dimensions.cube_width", params_.cube_width);
    this->declare_parameter<double>("dimensions.cube_depth", params_.cube_depth);
    this->declare_parameter<double>("dimensions.cube_height", params_.cube_height);
    this->declare_parameter<double>("apriltag.tag_to_cube_center_offset", params_.tag_to_cube_center_offset);
    this->declare_parameter<double>("apriltag.lateral_x_offset", params_.lateral_x_offset);

    // Grasp
    this->declare_parameter<double>("grasp.rpy.roll", params_.grasp_roll);
    this->declare_parameter<double>("grasp.rpy.pitch", params_.grasp_pitch);
    this->declare_parameter<double>("grasp.rpy.yaw", params_.grasp_yaw);
    this->declare_parameter<double>("grasp.center_z_offset", params_.grasp_center_z_offset);
    this->declare_parameter<double>("grasp.tool_length_offset", params_.tool_length_offset);

    // Motions
    this->declare_parameter<double>("motion.pregrasp_approach_dist_z", params_.pregrasp_approach_dist_z);
    this->declare_parameter<double>("motion.lift_dist_z", params_.lift_dist_z);

    // Gripper
    this->declare_parameter<double>("gripper.open", params_.gripper_open);
    this->declare_parameter<double>("gripper.closed", params_.gripper_closed);

    // Fetch values (including YAML overrides)
    this->get_parameter("scene.table.x", params_.table_x);
    this->get_parameter("scene.table.y", params_.table_y);
    this->get_parameter("scene.table.z", params_.table_z);

    this->get_parameter("scene.cube.base_x", params_.cube_base_x);
    this->get_parameter("scene.cube.base_y", params_.cube_base_y);
    this->get_parameter("scene.cube.base_z", params_.cube_base_z);
    this->get_parameter("scene.cube.offset_x", params_.cube_offset_x);

    this->get_parameter("dimensions.apriltag_size", params_.apriltag_size);
    this->get_parameter("dimensions.cube_width", params_.cube_width);
    this->get_parameter("dimensions.cube_depth", params_.cube_depth);
    this->get_parameter("dimensions.cube_height", params_.cube_height);
    this->get_parameter("apriltag.tag_to_cube_center_offset", params_.tag_to_cube_center_offset);
    this->get_parameter("apriltag.lateral_x_offset", params_.lateral_x_offset);

    this->get_parameter("grasp.rpy.roll", params_.grasp_roll);
    this->get_parameter("grasp.rpy.pitch", params_.grasp_pitch);
    this->get_parameter("grasp.rpy.yaw", params_.grasp_yaw);
    this->get_parameter("grasp.center_z_offset", params_.grasp_center_z_offset);
    this->get_parameter("grasp.tool_length_offset", params_.tool_length_offset);

    this->get_parameter("motion.pregrasp_approach_dist_z", params_.pregrasp_approach_dist_z);
    this->get_parameter("motion.lift_dist_z", params_.lift_dist_z);

    this->get_parameter("gripper.open", params_.gripper_open);
    this->get_parameter("gripper.closed", params_.gripper_closed);

    RCLCPP_INFO(
      get_logger(),
      "Pick params: cube(base)=%.3f,%.3f,%.3f offset_x=%.3f | grasp rpy=%.3f,%.3f,%.3f | z_off=%.3f tool=%.3f | pre_z=%.3f lift_z=%.3f",
      params_.cube_base_x, params_.cube_base_y, params_.cube_base_z, params_.cube_offset_x,
      params_.grasp_roll, params_.grasp_pitch, params_.grasp_yaw,
      params_.grasp_center_z_offset, params_.tool_length_offset,
      params_.pregrasp_approach_dist_z, params_.lift_dist_z);
  }

//action callbacks
private:
  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Pick::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received pick goal request for cubeid %d", goal->cube_id);
    (void)uuid; //what is this? 
    // Accept all goals
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PickActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    uint8_t cube_id = goal->cube_id;
    RCLCPP_INFO(get_logger(), "Executing pick for cube_id: %d", cube_id);

    bool success = run_pick_sequence(cube_id, goal_handle);  // Pass cube_id and goal_handle for feedback/cancel
    auto result = std::make_shared<Pick::Result>();
    result->success = success;
    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }


private:

  bool sense_cube(uint8_t cube_id)
  {
    // Wait for service to be available
    if (!cube_pose_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "GetCubePoses service not available after waiting");
      return false;
    }

    // Create request
    auto request = std::make_shared<group_6_assignment_2::srv::GetCubePoses::Request>();
    request->target_ids.push_back(static_cast<int32_t>(cube_id));

    // Call service asynchronously and wait without spinning (node is already spinning in main thread)
    auto future = cube_pose_client_->async_send_request(request);
    
    // Wait for response with timeout (don't spin - main thread is already spinning)
    auto status = future.wait_for(10s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "GetCubePoses service call timed out");
      return false;
    }

    auto response = future.get();
    if (!response->success || response->detected_ids.empty()) {
      RCLCPP_ERROR(get_logger(), "Cube ID %d not detected: %s", cube_id, response->message.c_str());
      return false;
    }

    // Store the detected AprilTag pose
    geometry_msgs::msg::Pose apriltag_pose = response->poses[0];
    RCLCPP_INFO(get_logger(), "Sensed AprilTag %d at: x=%.3f, y=%.3f, z=%.3f",
                cube_id, apriltag_pose.position.x, apriltag_pose.position.y, apriltag_pose.position.z);
    
    // Transform AprilTag pose to cube center pose
    // AprilTag is mounted on one face of the cube, need to offset to cube center
    cube_pose_ = transform_apriltag_to_cube_center(apriltag_pose);
    
    RCLCPP_INFO(get_logger(), "Transformed to cube center at: x=%.3f, y=%.3f, z=%.3f",
                cube_pose_.position.x, cube_pose_.position.y, cube_pose_.position.z);
    
    return true;
  }

  geometry_msgs::msg::Pose transform_apriltag_to_cube_center(const geometry_msgs::msg::Pose& apriltag_pose) const
  {
    // AprilTag pose is already at cube center, but needs lateral X correction
    geometry_msgs::msg::Pose cube_center_pose = apriltag_pose;
    
    // Apply lateral X offset (empirical correction from trial and error)
    cube_center_pose.position.x = apriltag_pose.position.x + params_.lateral_x_offset;
    
    // Orientation remains the same as the AprilTag
    cube_center_pose.orientation = apriltag_pose.orientation;
    
    return cube_center_pose;
  }

  bool wait_for_robot_state()
  {
    int attempts = 0;
    const int max_attempts = 10;
    
    while (rclcpp::ok() && attempts < max_attempts) {
      auto current_state = arm_group_.getCurrentState(2.0);
      if (current_state) {
        RCLCPP_INFO(get_logger(), "Robot state received successfully");
        return true;
      }
      
      RCLCPP_WARN(get_logger(), "Waiting for robot state (attempt %d/%d)...", attempts + 1, max_attempts);
      rclcpp::sleep_for(1s);
      attempts++;
    }
    
    RCLCPP_ERROR(get_logger(), "Failed to get robot state after %d attempts", max_attempts);
    return false;
  }

  bool verify_detached(const std::string& object_id, int max_attempts = 10)
  {
    RCLCPP_INFO(get_logger(), "Verifying '%s' is detached from planning scene...", object_id.c_str());
    for (int i = 0; i < max_attempts; ++i) {
      auto attached_objects = planning_scene_.getAttachedObjects();
      if (attached_objects.find(object_id) == attached_objects.end()) {
        RCLCPP_INFO(get_logger(), "Confirmed: '%s' is detached", object_id.c_str());
        return true;
      }
      RCLCPP_WARN(get_logger(), "Waiting for '%s' to detach (attempt %d/%d)...", object_id.c_str(), i + 1, max_attempts);
      rclcpp::sleep_for(200ms);
    }
    RCLCPP_ERROR(get_logger(), "Scene still shows '%s' as attached after %d attempts", object_id.c_str(), max_attempts);
    return false;
  }

  bool wait_for_scene_update(const std::vector<std::string>& expected_objects, int max_attempts = 20)
  {
    RCLCPP_INFO(get_logger(), "Waiting for planning scene to update with collision objects...");
    for (int i = 0; i < max_attempts; ++i) {
      auto known_objects = planning_scene_.getKnownObjectNames();
      bool all_present = true;
      for (const auto& obj : expected_objects) {
        if (std::find(known_objects.begin(), known_objects.end(), obj) == known_objects.end()) {
          all_present = false;
          break;
        }
      }
      if (all_present) {
        RCLCPP_INFO(get_logger(), "Planning scene updated successfully");
        return true;
      }
      rclcpp::sleep_for(200ms);
    }
    RCLCPP_ERROR(get_logger(), "Planning scene did not update after %d attempts", max_attempts);
    return false;
  }

  bool move_to_active() {
    RCLCPP_INFO(get_logger(), "Moving to active position...");

    // Hardcoded active joint configuration (in radians)
    std::map<std::string, double> active_joints;
    // 171° (No change needed, within +/- 180)
    active_joints["shoulder_pan_joint"] = 2.98;
    active_joints["shoulder_lift_joint"] = -2.39; 
    active_joints["elbow_joint"] = 1.87;
    active_joints["wrist_1_joint"] = 0.66;
    active_joints["wrist_2_joint"] = -0.33;
    active_joints["wrist_3_joint"] = 2.98;

    arm_group_.setJointValueTarget(active_joints);

    // Try direct move (bypasses explicit planning)
    auto result = arm_group_.move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Direct move to active position failed");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Successfully moved to active position");
    return true;
  }

  bool move_to_home()
  {
    RCLCPP_INFO(get_logger(), "Moving to home position...");
    
    // Hardcoded home joint configuration (in radians)
    std::map<std::string, double> home_joints;
    home_joints["shoulder_pan_joint"] = 0.0;           // -0°
    home_joints["shoulder_lift_joint"] = -1.5708;      // -90°
    home_joints["elbow_joint"] = 0.0;                  // 0°
    home_joints["wrist_1_joint"] = -1.5708;            // -90°
    home_joints["wrist_2_joint"] = 0.0;                // 0°
    home_joints["wrist_3_joint"] = 0.0;                // -0°
    
    arm_group_.setJointValueTarget(home_joints);
    
    // Try direct move (bypasses explicit planning)
    auto result = arm_group_.move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Direct move to home position failed");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "Successfully moved to home position");
    return true;
  }

  bool cleanup_scene()
  {
    RCLCPP_INFO(get_logger(), "Cleaning up scene state from previous attempts...");
    
    // Check if the target cube is attached from a previous attempt
    auto attached_objects = planning_scene_.getAttachedObjects();
    if (attached_objects.find(kCubeId) != attached_objects.end()) {
      RCLCPP_WARN(get_logger(), "Cube '%s' is attached from previous attempt, detaching...", kCubeId);
      if (!motion_util::detachObject(kCubeId, arm_group_, get_logger())) {
        RCLCPP_ERROR(get_logger(), "Failed to detach cube during cleanup");
        return false;
      }
      // Verify planning scene updated (scene synchronization)
      if (!verify_detached(kCubeId)) {
        RCLCPP_ERROR(get_logger(), "Planning scene still shows cube as attached after detach");
        return false;
      }
    }
    
    // Ensure gripper is open (in case it was closed from previous attempt)
    RCLCPP_INFO(get_logger(), "Ensuring gripper is open...");
    if (!open_gripper()) {
      RCLCPP_ERROR(get_logger(), "Failed to open gripper during cleanup");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "Scene cleanup complete");
    return true;
  }

  bool setup_scene()
  {
    // Defensive check: ensure cube is detached before updating scene
    // (should have been done in cleanup_scene, but double-check for safety)
    auto attached_objects = planning_scene_.getAttachedObjects();
    if (attached_objects.find(kCubeId) != attached_objects.end()) {
      RCLCPP_WARN(get_logger(), "Cube '%s' still attached, detaching before scene update...", kCubeId);
      motion_util::detachObject(kCubeId, arm_group_, get_logger());
      rclcpp::sleep_for(500ms);
    }
    
    // Check if scene objects already exist (for repeated calls)
    auto existing_objects = planning_scene_.getKnownObjectNames();
    bool table_exists = std::find(existing_objects.begin(), existing_objects.end(), kTableId) != existing_objects.end();
    bool cube_exists = std::find(existing_objects.begin(), existing_objects.end(), kCubeId) != existing_objects.end();

    if (!table_exists || !cube_exists) {
      // First time: add collision objects to scene
      if (!motion_util::setup_scene(planning_scene_)) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "Initial scene setup completed");
    } else {
      RCLCPP_INFO(get_logger(), "Scene objects already exist, updating poses");
    }

    // Update table pose from parameters
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = params_.table_x;
    table_pose.position.y = params_.table_y;
    table_pose.position.z = params_.table_z;
    table_pose.orientation.w = 1.0;
    motion_util::moveCollisionObject(kTableId, table_pose, kWorldFrame, planning_scene_);

    // Update cube pose using sensed position (cube_pose_ is set by sense_cube())
    motion_util::moveCollisionObject(kCubeId, cube_pose_, kWorldFrame, planning_scene_);

    // Precompute grasp pose to keep locality: used by pre-grasp/grasp/lift)
    grasp_pose_ = compute_grasp_pose(cube_pose_);
    publish_debug_pose(grasp_pose_, grasp_pose_pub_, kWorldFrame);
    
    RCLCPP_INFO(get_logger(), "Cube pose: x=%.3f, y=%.3f, z=%.3f", 
                cube_pose_.position.x, cube_pose_.position.y, cube_pose_.position.z);
    RCLCPP_INFO(get_logger(), "Grasp pose: x=%.3f, y=%.3f, z=%.3f", 
                grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z);
    
    return true;
  }

  // geometry_msgs::msg::Pose compute_grasp_pose(const geometry_msgs::msg::Pose& cube_pose) const
  // {
  //   geometry_msgs::msg::Pose grasp_pose = cube_pose;
  //   // Top-down pick:
  //   // - Keep x/y aligned with the cube center
  //   // - Offset along +Z by the gripper/tool length so the end-effector link (typically at the wrist)
  //   //   ends up above the grasp point while the fingers can close around the cube.
  //   grasp_pose.position.y = cube_pose.position.y;
  //   grasp_pose.position.z = cube_pose.position.z + params_.grasp_center_z_offset + params_.tool_length_offset;

  //   // Orientation for top grasp: gripper approaching downwards
  //   tf2::Quaternion q;
  //   const double roll = params_.grasp_roll;
  //   const double pitch = params_.grasp_pitch;
  //   const double yaw = params_.grasp_yaw;
  //   q.setRPY(roll, pitch, yaw);
  //   q.normalize();

  //   grasp_pose.orientation.x = q.x();
  //   grasp_pose.orientation.y = q.y();
  //   grasp_pose.orientation.z = q.z();
  //   grasp_pose.orientation.w = q.w();

  //   return grasp_pose;
  // }
  geometry_msgs::msg::Pose compute_grasp_pose(const geometry_msgs::msg::Pose& cube_pose) const
  {
    geometry_msgs::msg::Pose grasp_pose = cube_pose;
    grasp_pose.position.z = cube_pose.position.z + params_.grasp_center_z_offset;
    grasp_pose.position.y = cube_pose.position.y + params_.tool_length_offset;
    // Orientation for side grasp: gripper fingers pointing down, aligned with table
    tf2::Quaternion q;
    const double roll = params_.grasp_roll;
    const double pitch = params_.grasp_pitch;
    const double yaw = params_.grasp_yaw;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    grasp_pose.orientation.x = q.x();
    grasp_pose.orientation.y = q.y();
    grasp_pose.orientation.z = q.z();
    grasp_pose.orientation.w = q.w();

    return grasp_pose;
  }

  bool move_to_pre_grasp()
  {
    // Ensure we have current state
    if (!arm_group_.getCurrentState(2.0)) {
      RCLCPP_ERROR(get_logger(), "No robot state available for planning");
      return false;
    }
    
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose_;
    // Approach from the side (increase Y), then move inward to grasp_pose_.
    pre_grasp_pose.position.y += params_.pregrasp_approach_dist_z;

    publish_debug_pose(pre_grasp_pose, pregrasp_pose_pub_, kWorldFrame);

    RCLCPP_INFO(get_logger(), "Planning to pre-grasp pose: x=%.3f, y=%.3f, z=%.3f",
                pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
    
    return plan_and_execute_to_pose(pre_grasp_pose, arm_group_, get_logger());
  }


  bool open_gripper()
  {
    RCLCPP_INFO(get_logger(), "Opening gripper...");
    return setGripper(params_.gripper_open, gripper_group_, get_logger());
  }

  bool move_to_grasp_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Approaching object (Cartesian)...");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose_);
    const double eef_step = 0.01;
    return executeCartesianPath(waypoints, eef_step, 0.9, arm_group_, get_logger());
  }

  bool move_to_grasp_non_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Approaching object (Non-Cartesian)...");
    return plan_and_execute_to_pose(grasp_pose_, arm_group_, get_logger());
  }

  bool close_gripper()
  {
    RCLCPP_INFO(get_logger(), "Closing gripper...");
    bool result = setGripper(params_.gripper_closed, gripper_group_, get_logger());
    
    // Add a brief pause after closing to ensure grip
    if (result) {
      rclcpp::sleep_for(1s);
    }
    
    return result;
  }

  bool attach_object()
  {
    RCLCPP_INFO(get_logger(), "Attaching '%s' to end effector...", kCubeId);
    return attachObject(kCubeId, arm_group_, get_logger());
  }

  bool lift_up_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Lifting object (Cartesian)...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose_;
    // Lift up in Z direction after side grasp
    lift_pose.position.z += params_.lift_dist_z;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(lift_pose);
    return executeCartesianPath(waypoints, 0.01, 0.5, arm_group_, get_logger());
  }

  bool lift_up_non_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Lifting object (Non-Cartesian)...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose_;
    // Lift up in Z direction after side grasp
    lift_pose.position.z += params_.lift_dist_z;

    return plan_and_execute_to_pose(lift_pose, arm_group_, get_logger());
  }
};
} // namespace pick_action_cpp
// Updated main
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({ rclcpp::Parameter("use_sim_time", true) });
  auto node = std::make_shared<pick_action_cpp::PickActionServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
