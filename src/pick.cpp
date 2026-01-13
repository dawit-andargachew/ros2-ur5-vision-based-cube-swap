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
    // 1. Setup the world (Table + Cube)
    if (!setup_scene()) return false;

    // 2. Move to "Ready Pose" (High Elbow)
    if (!move_to_ready_pose()) return false;

    // 3. Move to "Pre-Grasp" (Hover above cube)
    if (!move_to_pre_grasp()) return false;

    // 4. Open Gripper (Explicitly called here)
    if (!open_gripper()) return false;

    // 5. Move Down to Grasp
    if (!move_to_grasp_cartesian()) return false;

    // 6. Close Gripper (Grab cube)
    if (!close_gripper()) return false;

    // 7. Attach object to robot logic
    if (!attach_object()) return false;

    // 8. Lift Up
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
  static constexpr double kClosedVal = 0.8;  // Changed to 0.8 to ensure tight grip (0.1 might be too loose)

  // --- HEIGHT ADJUSTMENT (UP/DOWN) ---
  // 0.30 meters from cube center to wrist. 
  // If gripper hits the cube, INCREASE this (e.g., 0.32).
  // If gripper grasps air above cube, DECREASE this (e.g., 0.28).
  static constexpr double kGripperLengthOffset = 0.05; // it was 0.30 before 
  
  // --- SIDE ADJUSTMENT (LEFT/RIGHT) ---
  // Positive = Left (in standard UR frame)
  // Negative = Right
  // 0.02 = 2cm Left. Adjust this if it's not centered!
  static constexpr double kLateralOffset = 0.015;  // it was 0.02 before

  static constexpr double kGraspCenterZOffset = 0.05; 
  static constexpr double kPreGraspApproachDist = 0.15;

  // =========================
  // Members
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

    // Initial check of state
    (void)arm_group_.getCurrentState(2.0);
    (void)gripper_group_.getCurrentState(2.0);
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

    moveit_msgs::msg::CollisionObject cube;
    cube.header.frame_id = kWorldFrame;
    cube.id = kCubeId;

    shape_msgs::msg::SolidPrimitive cube_prim;
    cube_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    cube_prim.dimensions = {0.06, 0.06, 0.1};

    cube_pose_ = geometry_msgs::msg::Pose{};
    cube_pose_.position.x = 3.97;
    cube_pose_.position.y = -1.10;
    cube_pose_.position.z = 0.4;
    cube_pose_.orientation.w = 1.0;

    cube.primitives.push_back(cube_prim);
    cube.primitive_poses.push_back(cube_pose_);
    cube.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_.applyCollisionObjects({table, cube});
    rclcpp::sleep_for(500ms);

    // Compute the grasp pose
    grasp_pose_ = compute_grasp_pose(cube_pose_);
    return true;
  }

  bool move_to_ready_pose()
  {
    RCLCPP_INFO(get_logger(), "Moving to 'Elbow Up' Ready Pose...");
    std::map<std::string, double> target_joints;
    target_joints["shoulder_pan_joint"] = -1.73; // -1.64;
    target_joints["shoulder_lift_joint"] = -1.46; 
    target_joints["elbow_joint"]         =  0.85; //1.12; 
    target_joints["wrist_1_joint"]       = -1.48;
    target_joints["wrist_2_joint"]       = -1.37;
    target_joints["wrist_3_joint"]       = -0.38;
   
    arm_group_.setJointValueTarget(target_joints);
    arm_group_.setMaxVelocityScalingFactor(0.5);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    return arm_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  geometry_msgs::msg::Pose compute_grasp_pose(const geometry_msgs::msg::Pose& cube_pose) const
  {
    geometry_msgs::msg::Pose grasp_pose = cube_pose;
   
    // === FIX 1: APPLY HEIGHT OFFSET (UP) ===
    grasp_pose.position.z = cube_pose.position.z + kGripperLengthOffset + kGraspCenterZOffset;
    
    // === FIX 2: APPLY LATERAL OFFSET (LEFT) ===
    // This was missing in your code! Now it adds the 0.02m (2cm) shift.
    grasp_pose.position.y = cube_pose.position.y + kLateralOffset;
    
    grasp_pose.position.x = cube_pose.position.x;
   
    tf2::Quaternion q;
    // Orientation: 3.14 (180 deg) points down. 
    q.setRPY(0.0, 3.1415, 0.0);
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
    // Approach from TOP (Z-axis)
    pre_grasp_pose.position.z += kPreGraspApproachDist;

    RCLCPP_INFO(get_logger(), "Planning to pre-grasp pose (Top approach)...");
    arm_group_.setPoseTarget(pre_grasp_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_group_.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      log_result(get_logger(), "Plan pre-grasp", plan_result);
      return false;
    }

    const auto exec_result = arm_group_.execute(plan);
    return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool open_gripper() 
  {
      RCLCPP_INFO(get_logger(), "Opening gripper...");
      gripper_group_.setStartStateToCurrentState();
  
      // Use the explicit joint value (0.0) instead of the name "open"
      std::map<std::string, double> joint_values;
      joint_values[kActiveJointName] = kOpenVal; 
  
      gripper_group_.setJointValueTarget(joint_values);
      
      // Using move() is simpler and often more reliable for single joints
      const auto result = gripper_group_.move(); 
      
      log_result(get_logger(), "Open gripper", result);
      return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool move_to_grasp_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Approaching object (Cartesian Down)...");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose_);

    moveit_msgs::msg::RobotTrajectory traj;
    // Lower velocity for the final approach
    const double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj);

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

    // Wait for the gripper to physically close in Sim
    rclcpp::sleep_for(2s);
    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool attach_object()
  {
    RCLCPP_INFO(get_logger(), "Attaching '%s'...", kCubeId);
    const auto ee_link = arm_group_.getEndEffectorLink();
    return arm_group_.attachObject(kCubeId, ee_link);
  }

  bool lift_up_cartesian()
  {
    RCLCPP_INFO(get_logger(), "Lifting object...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose_;
    lift_pose.position.z += 0.20;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(lift_pose);

    moveit_msgs::msg::RobotTrajectory traj;
    const double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj);

    if (fraction < 0.9) return false;

    const auto exec_result = arm_group_.execute(traj);
    return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({ rclcpp::Parameter("use_sim_time", true) });

  auto node = std::make_shared<PickNode>(options);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  node->run_pick_sequence();

  exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return 0;
}