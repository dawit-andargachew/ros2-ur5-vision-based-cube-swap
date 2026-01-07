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
    // Updated Z to 0.45 (approx resting height after falling to table). 
    // 0.81 was spawn height but unreachable.
    cube_pose.position.z = 0.45; 
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
    // The cube is at Y = -1.1. The robot is at Y = -0.5.
    // Approaching from the side (Y-axis) is much closer than reaching down from top.
    
    geometry_msgs::msg::Pose target_pose = cube_pose;
    // Align wrist height with cube center
    target_pose.position.z = cube_pose.position.z; 
    // Move wrist back (in +Y direction) to account for gripper length + approach distance
    // Gripper ~0.16m. Pre-grasp offset 0.10m. Total 0.26m.
    target_pose.position.y = cube_pose.position.y + 0.26; 
    
    // Orientation: Point gripper towards -Y (Rotation around X by 90 degrees)
    // Quaternion for 90 deg rotation around X-axis: 0.707, 0, 0, 0.707
    target_pose.orientation.x = 0.7071068; 
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.7071068;

    RCLCPP_INFO(node->get_logger(), "Moving to Pre-Grasp: [%.2f, %.2f, %.2f]", 
        target_pose.position.x, target_pose.position.y, target_pose.position.z);

    arm_group.setPoseTarget(target_pose);
    if (arm_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to pre-grasp!");
        return 1;
    }

    // 4. Open Gripper
    gripper_group.setNamedTarget("open"); 
    gripper_group.move();

    // 5. Move to GRASP Position (Linear Approach along Y)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose grasp_pose = target_pose;
    // Move forward in -Y direction by 10cm (leaving ~16cm for gripper length)
    grasp_pose.position.y -= 0.10; 
    waypoints.push_back(grasp_pose);

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
    gripper_group.setNamedTarget("close"); 
    gripper_group.move();

    // 7. Attach Object
    arm_group.attachObject(cube.id);

    // 8. Lift Up (Retreat back +Y and Up +Z)
    waypoints.clear();
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.y += 0.10; // Back out
    lift_pose.position.z += 0.20; // Lift up
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