#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pick");
    
    std::thread([&node]() { rclcpp::spin(node); }).detach();

    // 1. Setup MoveIt Interfaces
    // Make sure "ir_arm" and "ir_gripper" match your SRDF file exactly
    moveit::planning_interface::MoveGroupInterface arm_group(node, "ir_arm");
    moveit::planning_interface::MoveGroupInterface gripper_group(node, "ir_gripper");  

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Set scaling factors for safer movement
    arm_group.setMaxVelocityScalingFactor(0.5);
    arm_group.setMaxAccelerationScalingFactor(0.5);
    arm_group.setPlanningTime(50.0); // Increase planning time

    // 2. Add the Cube to the Scene
    moveit_msgs::msg::CollisionObject cube;
    
    // --- CRITICAL CHANGE: Frame ID ---
    // Because X=3.97, these are global map coordinates. 
    // We MUST use "world" (defined in SRDF) so MoveIt knows to transform them relative to the robot.
    cube.header.frame_id = "world"; 
    
    cube.id = "target_cube";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.06, 0.06, 0.1}; 
    
    geometry_msgs::msg::Pose cube_pose;
    // --- UPDATED HARDCODED CUBE LOCATION (From Service) ---
    cube_pose.position.x = 3.97; 
    cube_pose.position.y = -1.10;
    // Updated Z to 0.36 to match real cube on table
    cube_pose.position.z = 0.4; 
    cube_pose.orientation.w = 1.0;
    
    RCLCPP_INFO(node->get_logger(), "Adding collision object at: [%.2f, %.2f, %.2f]", 
        cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);

    cube.primitives.push_back(primitive);
    cube.primitive_poses.push_back(cube_pose);
    cube.operation = cube.ADD;
    planning_scene_interface.applyCollisionObject(cube);

    // Wait for the object to appear in the scene
    RCLCPP_INFO(node->get_logger(), "Waiting for object to appear in scene...");
    std::vector<std::string> object_ids = {cube.id};
    bool object_found = false;
    int retries = 0;
    while (!object_found && retries < 20) {
        std::map<std::string, moveit_msgs::msg::CollisionObject> objects = 
            planning_scene_interface.getObjects(object_ids);
        if (objects.find(cube.id) != objects.end()) {
            object_found = true;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            retries++;
        }
    }
    if (!object_found) {
        RCLCPP_ERROR(node->get_logger(), "Object not found in scene!");
        return 1;
    }

    // 3. Move to PRE-GRASP Position (Side Approach)
    // Cube Y = -1.1. Robot Y = -0.5.
    // Gripper length ~0.16m.
    // We want the Wrist (tool0) to be at Y = CubeY + 0.13 (Deep grasp, fingers surround cube).
    
    geometry_msgs::msg::Pose grasp_pose = cube_pose;
    // Z=0.38 (Center of cube is 0.38 if table is 0.33 + 0.05).
    // This is safe and reachable.
    grasp_pose.position.z = 0.38; 
    
    // Y = CubeY + 0.18. Gripper is ~0.16. Fingers will be at edge of cube.
    // This reduces the "reach" depth required.
    grasp_pose.position.y = cube_pose.position.y + 0.18; 
    
    // Orientation: Point -Y.
    // TF: Base(X+,Y+,Z+). We want ToolZ to be -Y.
    // Rotate -90 around X. (X -> X, Y -> -Z, Z -> Y) -> Wrong.
    // Rotate +90 around X. (X -> X, Y -> Z, Z -> -Y). CORRECT.
    // Quaternion for +90 deg X: x=0.707, w=0.707.
    grasp_pose.orientation.x = 0.7071068; 
    grasp_pose.orientation.y = 0.0;
    grasp_pose.orientation.z = 0.0;
    grasp_pose.orientation.w = 0.7071068;

    // Pre-grasp is 15cm back from grasp
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
    pre_grasp_pose.position.y += 0.15;

    RCLCPP_INFO(node->get_logger(), "Moving to Pre-Grasp: [%.2f, %.2f, %.2f]", 
        pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);

    arm_group.setPoseTarget(pre_grasp_pose);
    if (arm_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to pre-grasp!");
        return 1;
    }

    // 4. Open Gripper
    gripper_group.setNamedTarget("open"); 
    gripper_group.move();

    // 5. Move to GRASP Position (Linear Approach)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose); // Move to defined deep grasp pose

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    // jump_threshold is deprecated/removed in newer MoveIt versions
    double fraction = arm_group.computeCartesianPath(waypoints, eef_step, trajectory);

    if (fraction < 0.9) {
        RCLCPP_ERROR(node->get_logger(), "Cartesian path failed (%.2f%%)", fraction * 100);
        return 1;
    }
    arm_group.execute(trajectory);

    // 6. Close Gripper (Pick)
    // Cube width is 6cm. Fully open is 8.5cm (0.0). Fully closed is 0cm (0.8).
    // Target ~6cm is ~0.25 rad. We command 0.7 to ensure MAX force/squeeze.
    // gripper_group.setNamedTarget("close"); 
    std::vector<double> gripper_joint_values;
    gripper_joint_values.push_back(0.7); // Tighten grasp
    gripper_group.setJointValueTarget(gripper_joint_values);
    
    // Allow collision between gripper and cube during grasp
    // (MoveIt often handles this automatically during 'pick', but manual move needs care)
    // For now, we rely on the controller handling the stall.
    
    if (gripper_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node->get_logger(), "Gripper close reported failure (likely hit object), proceeding to attach...");
    }
    
    // Give time for the grasp to settle physically
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 7. Attach Object
    arm_group.attachObject(cube.id);

    // 8. Lift Up (Purely Vertical first)
    waypoints.clear();
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.20; // Lift straight up 20cm
    waypoints.push_back(lift_pose);

    fraction = arm_group.computeCartesianPath(waypoints, eef_step, trajectory);
    if (fraction < 0.9) {
        RCLCPP_WARN(node->get_logger(), "Lift path incomplete, executing partial");
    }
    arm_group.execute(trajectory);

    RCLCPP_INFO(node->get_logger(), "Pick complete!");
    rclcpp::shutdown();
    return 0;
}