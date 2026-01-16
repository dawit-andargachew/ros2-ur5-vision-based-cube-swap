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

    MotionConfig config;  // Use default configuration
    configure_moveit(arm_group_, gripper_group_, config);
    //apply_path_constraints(arm_group_, get_logger());  // ADDED: Apply path constraints

    // Create service client for cube pose detection
    cube_pose_client_ = this->create_client<group_6_assignment_2::srv::GetCubePoses>("get_cube_poses");

    declare_and_get_parameters();
    
    // Load joint configurations from parameters
    home_joint_config_["shoulder_pan_joint"] = this->get_parameter("home_position.shoulder_pan_joint").as_double();
    home_joint_config_["shoulder_lift_joint"] = this->get_parameter("home_position.shoulder_lift_joint").as_double();
    home_joint_config_["elbow_joint"] = this->get_parameter("home_position.elbow_joint").as_double();
    home_joint_config_["wrist_1_joint"] = this->get_parameter("home_position.wrist_1_joint").as_double();
    home_joint_config_["wrist_2_joint"] = this->get_parameter("home_position.wrist_2_joint").as_double();
    home_joint_config_["wrist_3_joint"] = this->get_parameter("home_position.wrist_3_joint").as_double();
    
    active_joint_config_["shoulder_pan_joint"] = this->get_parameter("active_position.shoulder_pan_joint").as_double();
    active_joint_config_["shoulder_lift_joint"] = this->get_parameter("active_position.shoulder_lift_joint").as_double();
    active_joint_config_["elbow_joint"] = this->get_parameter("active_position.elbow_joint").as_double();
    active_joint_config_["wrist_1_joint"] = this->get_parameter("active_position.wrist_1_joint").as_double();
    active_joint_config_["wrist_2_joint"] = this->get_parameter("active_position.wrist_2_joint").as_double();
    active_joint_config_["wrist_3_joint"] = this->get_parameter("active_position.wrist_3_joint").as_double();
  }

  bool run_pick_sequence(uint8_t cube_id, const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    // Set the current cube ID for this pick sequence
    current_cube_id_ = "cube_" + std::to_string(cube_id);

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

    // // Move to home position (safe position before cleanup)
    // feedback->status = "Moving to home position";
    // goal_handle->publish_feedback(feedback);
    // if (!motion_util::move_to_joint_configuration(home_joint_config_, arm_group_, "home", get_logger())) return false;
    // if (goal_handle->is_canceling()) return false;

    //go to active position
    feedback->status = "Moving to active position";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::move_to_joint_configuration(active_joint_config_, arm_group_, "active", get_logger())) return false;
    if (goal_handle->is_canceling()) return false;

    // Clean up scene state from any previous attempts (at home position, safe to detach)
    feedback->status = "Cleaning up scene state";
    goal_handle->publish_feedback(feedback);
    if (!cleanup_scene()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Setting up planning scene";
    goal_handle->publish_feedback(feedback);
    if (!PickActionServer::setup_scene()) return false;
    if (goal_handle->is_canceling()) return false;
    
    // Wait for planning scene to update with collision objects
    feedback->status = "Waiting for scene synchronization";
    goal_handle->publish_feedback(feedback);
    std::vector<std::string> expected_objects = {"table_1", "table_2", current_cube_id_};
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
    if (!move_to_grasp_cartesian()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Attaching object to gripper";
    goal_handle->publish_feedback(feedback);
    if (!attach_object()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Closing gripper after attaching object";
    goal_handle->publish_feedback(feedback);
    if (!close_gripper()) return false;
    if (goal_handle->is_canceling()) return false;

    feedback->status = "Lifting object";
    goal_handle->publish_feedback(feedback);
    if (!lift_up_cartesian()) return false;
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
  std::string current_cube_id_;  // Dynamic cube ID based on action request

  std::map<std::string, double> home_joint_config_;
  std::map<std::string, double> active_joint_config_;

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

    // Cube and AprilTag dimensions
    double apriltag_size = 0.050;  // AprilTag is 0.050x0.050m
    double cube_width = 0.06;      // Cube is 0.06x0.06x0.1m
    double cube_depth = 0.06;
    double cube_height = 0.1;
    // Offset from AprilTag center to cube center (tag is on one face)
    double tag_to_cube_center_offset = 0.03;  // Half of cube depth (0.06/2)
    double lateral_x_offset = 0.035;          // Lateral X correction

    // Grasp orientation (RPY, radians)
    double grasp_orientation_roll = 0.0;
    double grasp_orientation_pitch = -1.5707;
    double grasp_orientation_yaw = 0.0;

    // Grasp position offsets from cube center (meters)
    // For side grasp: x=0, y=tool_length(~0.18), z=slight_offset(~0.15)
    // For top grasp:  x=0, y=0, z=tool_length(~0.18)
    double grasp_offset_x = 0.0;
    double grasp_offset_y = 0.18;
    double grasp_offset_z = 0.15;

    // Pre-grasp approach offsets from grasp pose (meters)
    // For side grasp: approach from above (z=0.20), then descend
    // For top grasp:  approach from above (z=0.20), then descend
    double pregrasp_offset_x = 0.0;
    double pregrasp_offset_y = 0.0;
    double pregrasp_offset_z = 0.20;

    // Lift motion offsets from grasp pose (meters)
    double lift_offset_x = 0.0;
    double lift_offset_y = 0.0;
    double lift_offset_z = 0.20;

    // Gripper
    double gripper_open = 0.0;
    double gripper_closed = 0.10;
  } params_;

  void declare_and_get_parameters()
  {
    RCLCPP_INFO(get_logger(), "=== Loading Parameters from YAML ===");
    
    // Scene - Use declare_parameter return value (YAML overrides C++ default)
    params_.table_x = this->declare_parameter("scene.table.x", params_.table_x);
    params_.table_y = this->declare_parameter("scene.table.y", params_.table_y);
    params_.table_z = this->declare_parameter("scene.table.z", params_.table_z);
    RCLCPP_INFO(get_logger(), "Scene: table=[%.3f, %.3f, %.3f]", params_.table_x, params_.table_y, params_.table_z);

    // Cube and AprilTag dimensions
    params_.apriltag_size = this->declare_parameter("dimensions.apriltag_size", params_.apriltag_size);
    params_.cube_width = this->declare_parameter("dimensions.cube_width", params_.cube_width);
    params_.cube_depth = this->declare_parameter("dimensions.cube_depth", params_.cube_depth);
    params_.cube_height = this->declare_parameter("dimensions.cube_height", params_.cube_height);
    params_.tag_to_cube_center_offset = this->declare_parameter("apriltag.tag_to_cube_center_offset", params_.tag_to_cube_center_offset);
    params_.lateral_x_offset = this->declare_parameter("apriltag.lateral_x_offset", params_.lateral_x_offset);
    RCLCPP_INFO(get_logger(), "Cube dimensions: [%.3f x %.3f x %.3f]", params_.cube_width, params_.cube_depth, params_.cube_height);

    // Grasp orientation
    params_.grasp_orientation_roll = this->declare_parameter("grasp.orientation.roll", params_.grasp_orientation_roll);
    params_.grasp_orientation_pitch = this->declare_parameter("grasp.orientation.pitch", params_.grasp_orientation_pitch);
    params_.grasp_orientation_yaw = this->declare_parameter("grasp.orientation.yaw", params_.grasp_orientation_yaw);
    RCLCPP_INFO(get_logger(), "Grasp orientation (RPY): [%.4f, %.4f, %.4f] rad", 
                params_.grasp_orientation_roll, params_.grasp_orientation_pitch, params_.grasp_orientation_yaw);

    // Grasp position offsets
    params_.grasp_offset_x = this->declare_parameter("grasp.offset.x", params_.grasp_offset_x);
    params_.grasp_offset_y = this->declare_parameter("grasp.offset.y", params_.grasp_offset_y);
    params_.grasp_offset_z = this->declare_parameter("grasp.offset.z", params_.grasp_offset_z);
    RCLCPP_WARN(get_logger(), ">>> GRASP OFFSETS: x=%.3f, y=%.3f, z=%.3f <<<", 
                params_.grasp_offset_x, params_.grasp_offset_y, params_.grasp_offset_z);

    // Pre-grasp approach offsets
    params_.pregrasp_offset_x = this->declare_parameter("grasp.pregrasp_offset.x", params_.pregrasp_offset_x);
    params_.pregrasp_offset_y = this->declare_parameter("grasp.pregrasp_offset.y", params_.pregrasp_offset_y);
    params_.pregrasp_offset_z = this->declare_parameter("grasp.pregrasp_offset.z", params_.pregrasp_offset_z);
    RCLCPP_INFO(get_logger(), "Pre-grasp offsets: [%.3f, %.3f, %.3f]", 
                params_.pregrasp_offset_x, params_.pregrasp_offset_y, params_.pregrasp_offset_z);

    // Lift motion offsets
    params_.lift_offset_x = this->declare_parameter("grasp.lift_offset.x", params_.lift_offset_x);
    params_.lift_offset_y = this->declare_parameter("grasp.lift_offset.y", params_.lift_offset_y);
    params_.lift_offset_z = this->declare_parameter("grasp.lift_offset.z", params_.lift_offset_z);
    RCLCPP_INFO(get_logger(), "Lift offsets: [%.3f, %.3f, %.3f]", 
                params_.lift_offset_x, params_.lift_offset_y, params_.lift_offset_z);

    // Gripper
    params_.gripper_open = this->declare_parameter("gripper.open", params_.gripper_open);
    params_.gripper_closed = this->declare_parameter("gripper.closed", params_.gripper_closed);

    // Robot target positions (home and active) - Declare here so they're available for constructor
    this->declare_parameter("home_position.shoulder_pan_joint", 0.0);
    this->declare_parameter("home_position.shoulder_lift_joint", -1.5708);
    this->declare_parameter("home_position.elbow_joint", 0.0);
    this->declare_parameter("home_position.wrist_1_joint", -1.5708);
    this->declare_parameter("home_position.wrist_2_joint", 0.0);
    this->declare_parameter("home_position.wrist_3_joint", 0.0);

    this->declare_parameter("active_position.shoulder_pan_joint", -1.64);
    this->declare_parameter("active_position.shoulder_lift_joint", -1.46);
    this->declare_parameter("active_position.elbow_joint", 1.12);
    this->declare_parameter("active_position.wrist_1_joint", -1.48);
    this->declare_parameter("active_position.wrist_2_joint", -1.37);
    this->declare_parameter("active_position.wrist_3_joint", -0.38);

    RCLCPP_WARN(get_logger(), "=== FINAL PARAMETER SUMMARY ===");
    RCLCPP_WARN(
      get_logger(),
      "GRASP: rpy=[%.3f,%.3f,%.3f] xyz=[%.3f,%.3f,%.3f]",
      params_.grasp_orientation_roll, params_.grasp_orientation_pitch, params_.grasp_orientation_yaw,
      params_.grasp_offset_x, params_.grasp_offset_y, params_.grasp_offset_z);
    RCLCPP_WARN(
      get_logger(),
      "PREGRASP: xyz=[%.3f,%.3f,%.3f] | LIFT: xyz=[%.3f,%.3f,%.3f]",
      params_.pregrasp_offset_x, params_.pregrasp_offset_y, params_.pregrasp_offset_z,
      params_.lift_offset_x, params_.lift_offset_y, params_.lift_offset_z);
    RCLCPP_INFO(get_logger(), "=== Parameters loaded successfully ===");
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

    // Store the cube center pose (already transformed by CubeDetector service)
    cube_pose_ = response->poses[0];
    RCLCPP_INFO(get_logger(), "Sensed cube %d center at: x=%.3f, y=%.3f, z=%.3f",
                cube_id, cube_pose_.position.x, cube_pose_.position.y, cube_pose_.position.z);
    
    return true;
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

  bool cleanup_scene()
  {
    RCLCPP_INFO(get_logger(), "Cleaning up scene state from previous attempts...");
    
    // Check if the target cube is attached from a previous attempt
    auto attached_objects = planning_scene_.getAttachedObjects();
    if (attached_objects.find(current_cube_id_) != attached_objects.end()) {
      RCLCPP_WARN(get_logger(), "Cube '%s' is attached from previous attempt, detaching...", current_cube_id_.c_str());
      if (!motion_util::detachObject(current_cube_id_, arm_group_, get_logger())) {
        RCLCPP_ERROR(get_logger(), "Failed to detach cube during cleanup");
        return false;
      }
      // Verify planning scene updated (scene synchronization)
      if (!verify_detached(current_cube_id_)) {
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
    auto attached_objects = planning_scene_.getAttachedObjects();
    if (attached_objects.find(current_cube_id_) != attached_objects.end()) {
      RCLCPP_WARN(get_logger(), "Cube '%s' still attached, detaching before scene update...", current_cube_id_.c_str());
      motion_util::detachObject(current_cube_id_, arm_group_, get_logger());
      rclcpp::sleep_for(500ms);
    }
    
    // Check if scene objects already exist (for repeated calls)
    auto existing_objects = planning_scene_.getKnownObjectNames();
    bool tables_exist = std::find(existing_objects.begin(), existing_objects.end(), "table_1") != existing_objects.end();
    bool cube_exists = std::find(existing_objects.begin(), existing_objects.end(), current_cube_id_) != existing_objects.end();

    // Add static tables (only needs to be added once)
    if (!tables_exist) {
      if (!motion_util::setup_scene(planning_scene_)) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "Tables added to scene");
    }

    // Add or update cube using ONLY sensed position (sense->plan->act)
    if (!cube_exists) {
      // First time: add cube collision object with sensed pose
      moveit_msgs::msg::CollisionObject cube;
      cube.header.frame_id = kWorldFrame;
      cube.id = current_cube_id_;

      shape_msgs::msg::SolidPrimitive cube_prim;
      cube_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      cube_prim.dimensions = {params_.cube_width, params_.cube_depth, params_.cube_height};

      cube.primitives.push_back(cube_prim);
      cube.primitive_poses.push_back(cube_pose_);  // Use sensed pose
      cube.operation = moveit_msgs::msg::CollisionObject::ADD;

      planning_scene_.applyCollisionObject(cube);
      RCLCPP_INFO(get_logger(), "Cube added to scene at sensed position");
    } else {
      // Subsequent times: move cube to updated sensed position
      motion_util::moveCollisionObject(current_cube_id_, cube_pose_, kWorldFrame, planning_scene_);
      RCLCPP_INFO(get_logger(), "Cube position updated to sensed position");
    }

    // Precompute grasp pose (uses sensed cube_pose_)
    grasp_pose_ = compute_grasp_pose(cube_pose_);
    publish_debug_pose(grasp_pose_, grasp_pose_pub_, kWorldFrame);
    
    RCLCPP_INFO(get_logger(), "Cube pose (sensed): x=%.3f, y=%.3f, z=%.3f", 
                cube_pose_.position.x, cube_pose_.position.y, cube_pose_.position.z);
    RCLCPP_INFO(get_logger(), "Grasp pose: x=%.3f, y=%.3f, z=%.3f", 
                grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z);
    
    return true;
  }

  geometry_msgs::msg::Pose compute_grasp_pose(const geometry_msgs::msg::Pose& cube_pose) const
  {
    geometry_msgs::msg::Pose grasp_pose = cube_pose;
    
    // Apply XYZ offsets from cube center to grasp position
    grasp_pose.position.x = cube_pose.position.x + params_.grasp_offset_x;
    grasp_pose.position.y = cube_pose.position.y + params_.grasp_offset_y;
    grasp_pose.position.z = cube_pose.position.z + params_.grasp_offset_z;
    
    // Apply grasp orientation
    tf2::Quaternion q;
    q.setRPY(params_.grasp_orientation_roll, 
             params_.grasp_orientation_pitch, 
             params_.grasp_orientation_yaw);
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
    
    // Apply pre-grasp offsets (defines approach direction)
    pre_grasp_pose.position.x += params_.pregrasp_offset_x;
    pre_grasp_pose.position.y += params_.pregrasp_offset_y;
    pre_grasp_pose.position.z += params_.pregrasp_offset_z;

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
    MotionConfig config;
    config.cartesian_step_sizes = {0.01};
    config.min_cartesian_fraction = 0.5;
    return executeCartesianPath(waypoints, config, arm_group_, get_logger());
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
    RCLCPP_INFO(get_logger(), "Attaching '%s' to end effector...", current_cube_id_.c_str());
    return attachObject(current_cube_id_, arm_group_, get_logger());
  }

  bool lift_up_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Lifting object (Cartesian)...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose_;
    
    // Apply lift offsets from grasp position
    lift_pose.position.x += params_.lift_offset_x;
    lift_pose.position.y += params_.lift_offset_y;
    lift_pose.position.z += params_.lift_offset_z;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(lift_pose);
    MotionConfig config;
    config.cartesian_step_sizes = {0.01};
    config.min_cartesian_fraction = 0.5;
    return executeCartesianPath(waypoints, config, arm_group_, get_logger());
  }

  bool lift_up_non_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Lifting object (Non-Cartesian)...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose_;
    
    // Apply lift offsets from grasp position
    lift_pose.position.x += params_.lift_offset_x;
    lift_pose.position.y += params_.lift_offset_y;
    lift_pose.position.z += params_.lift_offset_z;

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
