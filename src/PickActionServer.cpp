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

    feedback->status = "Setting up planning scene";
    goal_handle->publish_feedback(feedback);
    if (!setup_scene()) return false;
    
    if (goal_handle->is_canceling()) return false;
   
    
    // Give time for planning scene to update
    rclcpp::sleep_for(2s);

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

  bool setup_scene()
  {
    // Updated: Call motion_util::setup_scene to add collision objects
    if (!motion_util::setup_scene(planning_scene_)) {
      return false;
    }

    // Override default scene object poses using parameters so collision + grasp stay consistent.
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = params_.table_x;
    table_pose.position.y = params_.table_y;
    table_pose.position.z = params_.table_z;
    table_pose.orientation.w = 1.0;
    motion_util::moveCollisionObject(kTableId, table_pose, kWorldFrame, planning_scene_);

    cube_pose_ = geometry_msgs::msg::Pose{};
    cube_pose_.position.x = params_.cube_base_x + params_.cube_offset_x;
    cube_pose_.position.y = params_.cube_base_y;
    cube_pose_.position.z = params_.cube_base_z;
    cube_pose_.orientation.w = 1.0;
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
