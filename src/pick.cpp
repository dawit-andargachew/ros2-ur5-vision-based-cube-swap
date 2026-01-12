#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "group_6_assignment_2/motion_util.hpp"

using namespace std::chrono_literals;
using namespace motion_util;

class PickNode : public rclcpp::Node
{
public:
  explicit PickNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("pick_monitored", options),
    arm_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kArmGroupName),
    gripper_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kGripperGroupName)
  {
    configure_moveit(arm_group_, gripper_group_);  // Updated: Use motion_util::configure_moveit
    //apply_path_constraints(arm_group_, get_logger());  // ADDED: Apply path constraints
  }

  bool run_pick_sequence()
  {
    // Wait for robot state before starting
    RCLCPP_INFO(get_logger(), "Waiting for robot state to be available...");
    if (!wait_for_robot_state()) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot state. Check if /joint_states is being published.");
      return false;
    }
    
    if (!setup_scene()) return false;
    
    // Give time for planning scene to update
    rclcpp::sleep_for(2s);
    
    if (!move_to_pre_grasp()) return false;
    if (!open_gripper()) return false;
    if (!move_to_grasp_non_cartesian()) return false;
    if (!close_gripper()) return false;
    if (!attach_object()) return false;
    if (!lift_up_non_cartesian()) return false;

    RCLCPP_INFO(get_logger(), "Pick complete!");
    return true;
  }

private:
  // =========================
  // Members (own the state)
  // =========================
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  geometry_msgs::msg::Pose cube_pose_{};
  geometry_msgs::msg::Pose grasp_pose_{};

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

    // Keep local cube pose computation (consistent with motion_util::setup_scene)
    constexpr double kOffsetXLateral = +0.035; //trial and error
    cube_pose_ = geometry_msgs::msg::Pose{};
    cube_pose_.position.x = 3.97 + kOffsetXLateral;
    cube_pose_.position.y = -1.10;
    cube_pose_.position.z = 0.4;
    cube_pose_.orientation.w = 1.0;

    // Precompute grasp pose to keep locality: used by pre-grasp/grasp/lift)
    grasp_pose_ = compute_grasp_pose(cube_pose_);
    
    RCLCPP_INFO(get_logger(), "Cube pose: x=%.3f, y=%.3f, z=%.3f", 
                cube_pose_.position.x, cube_pose_.position.y, cube_pose_.position.z);
    RCLCPP_INFO(get_logger(), "Grasp pose: x=%.3f, y=%.3f, z=%.3f", 
                grasp_pose_.position.x, grasp_pose_.position.y, grasp_pose_.position.z);
    
    return true;
  }

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

  bool move_to_pre_grasp()
  {
    // Ensure we have current state
    if (!arm_group_.getCurrentState(2.0)) {
      RCLCPP_ERROR(get_logger(), "No robot state available for planning");
      return false;
    }
    
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose_;
    pre_grasp_pose.position.y += kPreGraspApproachDist;

    RCLCPP_INFO(get_logger(), "Planning to pre-grasp pose: x=%.3f, y=%.3f, z=%.3f",
                pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
    
    return plan_and_execute_to_pose(pre_grasp_pose, arm_group_, get_logger());
  }


  bool open_gripper()
  {
    RCLCPP_INFO(get_logger(), "Opening gripper...");
    return setGripper(kOpenVal, gripper_group_, get_logger());
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
    bool result = setGripper(kClosedVal, gripper_group_, get_logger());
    
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
    lift_pose.position.z += 0.20;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(lift_pose);
    return executeCartesianPath(waypoints, 0.01, 0.5, arm_group_, get_logger());
  }

  bool lift_up_non_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Lifting object (Non-Cartesian)...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose_;
    lift_pose.position.z += 0.20;

    return plan_and_execute_to_pose(lift_pose, arm_group_, get_logger());
  }
};

// pick main
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.parameter_overrides({ rclcpp::Parameter("use_sim_time", true) });

  auto node = std::make_shared<PickNode>(options);

  // Best practice: explicit executor instead of detached spin
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::thread spin_thread([&exec]() { exec.spin(); });
  
  const bool ok = node->run_pick_sequence();

  exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();

  rclcpp::shutdown();
  return ok ? 0 : 1;
}