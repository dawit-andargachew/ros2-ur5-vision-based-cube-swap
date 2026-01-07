#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
// 1. Include Constraint Messages
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pick");
    
    std::thread([&node]() { rclcpp::spin(node); }).detach();

    moveit::planning_interface::MoveGroupInterface arm_group(node, "ir_arm");
    moveit::planning_interface::MoveGroupInterface gripper_group(node, "ir_gripper");  
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm_group.setMaxVelocityScalingFactor(0.5);
    arm_group.setMaxAccelerationScalingFactor(0.5);
    arm_group.setPlanningTime(20.0);

    // 2. Add Target Cube (Same as before)
    moveit_msgs::msg::CollisionObject cube;
    cube.header.frame_id = "world"; 
    cube.id = "target_cube";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.06, 0.06, 0.1}; 
    
       // 2. SIDEWAYS (X-Axis): Controls Left/Right alignment.
    // If gripper is too far LEFT, make this POSITIVE (e.g., 0.02)
    // If gripper is too far RIGHT, make this NEGATIVE (e.g., -0.02)
    const double OFFSET_X_LATERAL = +0.035;
    geometry_msgs::msg::Pose cube_pose;
    cube_pose.position.x = 3.97 + OFFSET_X_LATERAL; // Fix "Sideways"
    cube_pose.position.y = -1.10;
    cube_pose.position.z = 0.4; 
    cube_pose.orientation.w = 1.0;

    cube.primitives.push_back(primitive);
    cube.primitive_poses.push_back(cube_pose);
    cube.operation = cube.ADD;
    planning_scene_interface.applyCollisionObject(cube);

    // ==========================================
    // CRITICAL SECTION: JOINT CONSTRAINT
    // ==========================================
    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::JointConstraint lift_constraint;

    // The joint connecting the base to the upper arm is usually "shoulder_lift_joint".
    // This controls the "elevation" of the arm.
    lift_constraint.joint_name = "shoulder_lift_joint"; 

    // TARGET: -1.57 radians (-90 degrees) is usually "Straight Up" on UR robots.
    // By setting the target here, we bias the solver to look for solutions where the arm is high.
    lift_constraint.position = -1.57; 

    // TOLERANCE: Define the "Safe Zone".
    // If we allow it to go to 0.0, the arm is horizontal. If positive, it hits the table.
    // By allowing 1.0 deviation, the range is [-2.57, -0.57].
    // -0.57 is roughly -30 degrees (still pointing up).
    // This mathematically guarantees the arm cannot dip below -30 degrees.
    lift_constraint.tolerance_above = 1.0; 
    lift_constraint.tolerance_below = 1.5; 
    lift_constraint.weight = 1.0; // High priority: Planner must respect this.

    path_constraints.joint_constraints.push_back(lift_constraint);
    
    // Apply the constraint
    arm_group.setPathConstraints(path_constraints);
    
    RCLCPP_INFO(node->get_logger(), "Elbow-Up Constraint Applied.");

    // ... existing includes ...
#include <tf2/LinearMath/Quaternion.h>

// ... inside main ...

    // ==========================================
    // CONFIGURATION CONSTANTS (No Magic Numbers)
    // ==========================================
    // 1. Gripper Geometry
    // Distance from the robot wrist (tool0) to the center of the fingertips.
    // Adjust this if you change grippers.
    const double GRIPPER_LENGTH_OFFSET = 0.18; 

    // 2. Grasp Strategy
    // How far below the object center to grasp? (e.g., -0.02 to grab slightly lower)
    const double GRASP_CENTER_Z_OFFSET = -0.02; 
    
    // How far back to stop before the final slow approach?
    const double PRE_GRASP_APPROACH_DIST = 0.15; 

    // 3. Move to PRE-GRASP
    geometry_msgs::msg::Pose grasp_pose = cube_pose;

    
    grasp_pose.position.x = cube_pose.position.x; 
    // Apply Z offset relative to the cube's dynamic center
    grasp_pose.position.z = cube_pose.position.z + GRASP_CENTER_Z_OFFSET;
    
    // Apply Y offset (Robot needs to be at Cube Y + Gripper Length to wrap fingers around it)
    grasp_pose.position.y = cube_pose.position.y + GRIPPER_LENGTH_OFFSET;
    
 


    // Calculate Orientation (Roll +90 degrees around X-axis)
    // This points the gripper sideways towards the object (assuming -Y approach)
    tf2::Quaternion q_rot;
    double roll = 1.5707; // 90 degrees (M_PI / 2.0)
    double pitch = 0.0;
    double yaw = 0.0;
    q_rot.setRPY(roll, pitch, yaw);
    q_rot.normalize(); // Good practice to ensure valid rotation

    grasp_pose.orientation.x = q_rot.x();
    grasp_pose.orientation.y = q_rot.y();
    grasp_pose.orientation.z = q_rot.z();
    grasp_pose.orientation.w = q_rot.w();

    // Create Pre-Grasp Pose
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
    
    // Back up along the Y axis for the approach
    pre_grasp_pose.position.y += PRE_GRASP_APPROACH_DIST;

    arm_group.setPoseTarget(pre_grasp_pose);
    
    // We must use plan() + execute() to ensure constraints are respected during planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        arm_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed with constraints! The goal might be out of reach with this constraint.");
        return 1;
    }

    // 4. Open Gripper
    RCLCPP_INFO(node->get_logger(), "Opening Gripper to MAX (0.0)...");
    
    std::vector<double> gripper_open_values;
    gripper_open_values.push_back(0.0); // 0.0 = Fully Open
    gripper_group.setJointValueTarget(gripper_open_values);
    gripper_group.move();

    // 5. Approach (Cartesian)
    // Note: computeCartesianPath tries to follow a straight line. 
    // If the start state is "safe" (due to step 3), a short straight line will likely stay safe.
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm_group.computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction < 0.9) {
        RCLCPP_ERROR(node->get_logger(), "Approach path failed");
        return 1;
    }
    arm_group.execute(trajectory);

    // 7. Attach Object
    arm_group.attachObject(cube.id);

    // 6. Close Gripper
    std::vector<double> gripper_joint_values;
    gripper_joint_values.push_back(0.7); 
    gripper_group.setJointValueTarget(gripper_joint_values);
    gripper_group.move();
    
    std::this_thread::sleep_for(std::chrono::seconds(1));

    

    // IMPORTANT: Clear constraints for the lift?
    // You might want to KEEP them if you are lifting high to avoid hitting the table on the way up.
    // Or clear them if you need to rotate somewhere else. 
    // Usually safe to clear once you are up in the air.
    // arm_group.clearPathConstraints(); 

    // 8. Lift Up
    waypoints.clear();
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.20; 
    waypoints.push_back(lift_pose);

    fraction = arm_group.computeCartesianPath(waypoints, 0.01, trajectory);
    arm_group.execute(trajectory);

    RCLCPP_INFO(node->get_logger(), "Pick complete!");
    rclcpp::shutdown();
    return 0;
}