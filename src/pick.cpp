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

using namespace std::chrono_literals;

class PickNode : public rclcpp::Node
{
public:
  explicit PickNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("pick_monitored", options),
    arm_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kArmGroupName),
    gripper_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kGripperGroupName)
  {
    configure_moveit();
  }

  bool run_pick_sequence()
  {
    if (!setup_scene()) return false;
    if (!move_to_pre_grasp()) return false;
    if (!open_gripper()) return false;
    if (!move_to_grasp_cartesian()) return false;
    if (!close_gripper()) return false;
    if (!attach_object()) return false;
    if (!lift_up_cartesian()) return false;

    RCLCPP_INFO(get_logger(), "Pick complete!");
    return true;
  }

private:
  // =========================
  // Constants (stable config)
  // =========================
  static constexpr const char* kArmGroupName = "ir_arm";
  static constexpr const char* kGripperGroupName = "ir_gripper";
  static constexpr const char* kActiveJointName = "robotiq_85_left_knuckle_joint";
  static constexpr const char* kWorldFrame = "world";
  static constexpr const char* kCubeId = "target_cube";
  static constexpr const char* kTableId = "surface_collision";

  static constexpr double kOpenVal = 0.0;
  static constexpr double kClosedVal = 0.10;  // tune for collisions

  static constexpr double kGripperLengthOffset = 0.18;
  static constexpr double kGraspCenterZOffset = -0.02;
  static constexpr double kPreGraspApproachDist = 0.15;

  // =========================
  // Members (own the state)
  // =========================
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  geometry_msgs::msg::Pose cube_pose_{};
  geometry_msgs::msg::Pose grasp_pose_{};

private:
  void configure_moveit()
  {
    arm_group_.setMaxVelocityScalingFactor(0.5);
    arm_group_.setMaxAccelerationScalingFactor(0.5);
    arm_group_.setPlanningTime(20.0);

    // Optional: reduce flakiness waiting for current state
    // (MoveGroupInterface may need a moment to receive robot state)
    (void)arm_group_.getCurrentState(2.0);
    (void)gripper_group_.getCurrentState(2.0);

    apply_path_constraints();
  }


  //stop from getting caught under table top
  void apply_path_constraints()
  {
    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::JointConstraint lift_constraint;

    lift_constraint.joint_name = "shoulder_lift_joint";
    lift_constraint.position = -1.57;
    lift_constraint.tolerance_above = 1.0;
    lift_constraint.tolerance_below = 1.5;
    lift_constraint.weight = 1.0;

    path_constraints.joint_constraints.push_back(lift_constraint);
    arm_group_.setPathConstraints(path_constraints);

    RCLCPP_INFO(get_logger(), "Elbow-up joint constraint applied.");
  }

  static void log_result(rclcpp::Logger logger, const std::string& action,
                         const moveit::core::MoveItErrorCode& code)
  {
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, ">> %s: SUCCESS", action.c_str());
    } else {
      RCLCPP_ERROR(logger, ">> %s: FAILED (MoveItErrorCode=%d)", action.c_str(), code.val);
    }
  }

  bool setup_scene()
  {
    // ---- 1) Table collision object
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = kWorldFrame;
    table.id = kTableId;

    shape_msgs::msg::SolidPrimitive table_prim;
    table_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_prim.dimensions = {0.3652, 0.3652, 0.016};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 3.97;
    table_pose.position.y = -1.10;
    table_pose.position.z = 0.322;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(table_prim);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    // ---- 2) Cube collision object (also store pose for later planning)
    moveit_msgs::msg::CollisionObject cube;
    cube.header.frame_id = kWorldFrame;
    cube.id = kCubeId;

    shape_msgs::msg::SolidPrimitive cube_prim;
    cube_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    cube_prim.dimensions = {0.06, 0.06, 0.1};

    constexpr double kOffsetXLateral = +0.035;
    cube_pose_ = geometry_msgs::msg::Pose{};
    cube_pose_.position.x = 3.97 + kOffsetXLateral;
    cube_pose_.position.y = -1.10;
    cube_pose_.position.z = 0.4;
    cube_pose_.orientation.w = 1.0;

    cube.primitives.push_back(cube_prim);
    cube.primitive_poses.push_back(cube_pose_);
    cube.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_.applyCollisionObjects({table, cube});

    // Give PlanningSceneInterface a beat to help sim
    rclcpp::sleep_for(500ms);

    // Precompute grasp pose to keep locality: used by pre-grasp/grasp/lift)
    grasp_pose_ = compute_grasp_pose(cube_pose_);
    return true;
  }

  geometry_msgs::msg::Pose compute_grasp_pose(const geometry_msgs::msg::Pose& cube_pose) const
  {
    geometry_msgs::msg::Pose grasp_pose = cube_pose;
    grasp_pose.position.z = cube_pose.position.z + kGraspCenterZOffset;
    grasp_pose.position.y = cube_pose.position.y + kGripperLengthOffset;

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
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose_;
    pre_grasp_pose.position.y += kPreGraspApproachDist;

    RCLCPP_INFO(get_logger(), "Planning to pre-grasp pose...");
    arm_group_.setPoseTarget(pre_grasp_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_group_.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      log_result(get_logger(), "Plan pre-grasp", plan_result);
      return false;
    }

    const auto exec_result = arm_group_.execute(plan);
    log_result(get_logger(), "Execute pre-grasp", exec_result);
    return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool open_gripper()
  {
    RCLCPP_INFO(get_logger(), "Opening gripper...");
    gripper_group_.setStartStateToCurrentState();

    std::map<std::string, double> joint_values;
    joint_values[kActiveJointName] = kOpenVal;

    gripper_group_.setJointValueTarget(joint_values);
    const auto result = gripper_group_.move();
    log_result(get_logger(), "Open gripper", result);
    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool move_to_grasp_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Approaching object (Cartesian)...");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose_);

    moveit_msgs::msg::RobotTrajectory traj;
    const double eef_step = 0.01;
    const double jump_thresh = 0.0; // consider setting non-zero if needed
    const double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

    RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.1f%%", fraction * 100.0);
    if (fraction < 0.9) {
      RCLCPP_ERROR(get_logger(), "Approach failed (only %.1f%% computed).", fraction * 100.0);
      return false;
    }

    const auto exec_result = arm_group_.execute(traj);
    log_result(get_logger(), "Execute approach", exec_result);
    return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool close_gripper()
  {
    RCLCPP_INFO(get_logger(), "Closing gripper...");
    gripper_group_.setStartStateToCurrentState();

    std::map<std::string, double> joint_values;
    joint_values[kActiveJointName] = kClosedVal;

    gripper_group_.setJointValueTarget(joint_values);
    const auto result = gripper_group_.move();
    log_result(get_logger(), "Close gripper", result);

    rclcpp::sleep_for(2s);
    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool attach_object()
  {
    RCLCPP_INFO(get_logger(), "Attaching '%s' to end effector...", kCubeId);

    // Prefer explicit end-effector link if you know it; getEndEffectorLink() is okay if configured.
    const auto ee_link = arm_group_.getEndEffectorLink();
    if (ee_link.empty()) {
      RCLCPP_ERROR(get_logger(), "End-effector link is empty. Check MoveIt SRDF/end-effector config.");
      return false;
    }

    const bool ok = arm_group_.attachObject(kCubeId, ee_link);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "attachObject() returned false (check MoveIt warnings).");
    } else {
      RCLCPP_INFO(get_logger(), ">> Attach object: SUCCESS");
    }
    return ok;
  }

  bool lift_up_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Lifting object (Cartesian)...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose_;
    lift_pose.position.z += 0.20;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(lift_pose);

    moveit_msgs::msg::RobotTrajectory traj;
    const double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj);

    RCLCPP_INFO(get_logger(), "Lift path fraction: %.1f%%", fraction * 100.0);
    if (fraction < 0.9) {
      RCLCPP_ERROR(get_logger(), "Lift failed (only %.1f%% computed).", fraction * 100.0);
      return false;
    }

    const auto exec_result = arm_group_.execute(traj);
    log_result(get_logger(), "Execute lift", exec_result);
    return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
  }
};

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
