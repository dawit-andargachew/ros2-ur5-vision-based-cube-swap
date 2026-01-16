#include "group_6_assignment_2/motion_util.hpp"
#include <chrono>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace motion_util {

    void configure_moveit(moveit::planning_interface::MoveGroupInterface& arm_mgi,
                    moveit::planning_interface::MoveGroupInterface& gripper_mgi,
                    const MotionConfig& config) {
        // Consistent arm settings - optimized for reliability
        arm_mgi.setMaxVelocityScalingFactor(config.max_velocity_scaling_factor);  // Moderate speed
        arm_mgi.setMaxAccelerationScalingFactor(config.max_acceleration_scaling_factor);  // Smooth acceleration
        arm_mgi.setPlanningTime(config.planning_time);  // More time for complex plans
        arm_mgi.setNumPlanningAttempts(config.num_planning_attempts);  // More attempts
        
        // Relaxed tolerances for higher success rate
        arm_mgi.setGoalPositionTolerance(config.goal_position_tolerance);  // 2cm
        arm_mgi.setGoalOrientationTolerance(config.goal_orientation_tolerance);  // ~8.6 degrees
        arm_mgi.setGoalJointTolerance(config.goal_joint_tolerance);
        
        // Use reliable planner
        arm_mgi.setPlanningPipelineId("ompl");
        arm_mgi.setPlannerId("RRTConnect");
        
        // Set pose reference frame explicitly (Page 36, 65 documentation)
        arm_mgi.setPoseReferenceFrame(kWorldFrame);
        
        // Gripper settings
        gripper_mgi.setMaxVelocityScalingFactor(config.gripper_max_velocity_scaling_factor);
        gripper_mgi.setMaxAccelerationScalingFactor(config.gripper_max_acceleration_scaling_factor);
        gripper_mgi.setPlanningTime(config.gripper_planning_time);
        gripper_mgi.setNumPlanningAttempts(config.gripper_num_planning_attempts);
        gripper_mgi.setGoalJointTolerance(config.gripper_goal_joint_tolerance); // hopefully wont fail as much
        // Initialize state monitor for reliable state queries
        arm_mgi.startStateMonitor(2.0);  
        gripper_mgi.startStateMonitor(2.0);
        
        // Set workspace bounds (Page 3 documentation)
        arm_mgi.setWorkspace(
            config.workspace.min_x, config.workspace.min_y, config.workspace.min_z,
            config.workspace.max_x, config.workspace.max_y, config.workspace.max_z
        );               

    }


    void apply_path_constraints(moveit::planning_interface::MoveGroupInterface& arm_group_, const rclcpp::Logger& logger)
    {
        // moveit_msgs::msg::Constraints path_constraints;
        // moveit_msgs::msg::JointConstraint lift_constraint;

        // lift_constraint.joint_name = "shoulder_lift_joint";
        // lift_constraint.position = -1.57;
        // lift_constraint.tolerance_above = 1.0;
        // lift_constraint.tolerance_below = 1.5;
        // lift_constraint.weight = 1.0;

        // path_constraints.joint_constraints.push_back(lift_constraint);
        // arm_group_.setPathConstraints(path_constraints);

        // RCLCPP_INFO(logger, "Elbow-up joint constraint applied.");

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "elbow_up_constraint";

        moveit_msgs::msg::JointConstraint lift_constraint;
        lift_constraint.joint_name = "shoulder_lift_joint";
        
        // Target position (Elbow Up)
        lift_constraint.position = -1.50; 
        
        // Tolerances
        lift_constraint.tolerance_above = 1.5; 
        lift_constraint.tolerance_below = 1.0; 
        
        // Weight
        lift_constraint.weight = 1.0;

        path_constraints.joint_constraints.push_back(lift_constraint);

        // Apply to the group
        arm_group_.setPathConstraints(path_constraints);
        
        // Optional: Loosen goal tolerance slightly to help planner succeed with constraints
        arm_group_.setGoalJointTolerance(0.01); 

        RCLCPP_INFO(logger, "Elbow-up joint constraint applied.");
    }

    bool setup_scene(moveit::planning_interface::PlanningSceneInterface& planning_scene_)
    {
        // Add static table collision objects
        // Table 1
        moveit_msgs::msg::CollisionObject table1;
        table1.header.frame_id = kWorldFrame;
        table1.id = "table_1";

        shape_msgs::msg::SolidPrimitive table1_prim;
        table1_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        table1_prim.dimensions = {0.3652, 0.3652, 0.016};

        geometry_msgs::msg::Pose table1_pose;
        table1_pose.position.x = 3.97;
        table1_pose.position.y = -1.10;
        table1_pose.position.z = 0.322;
        table1_pose.orientation.w = 1.0;

        table1.primitives.push_back(table1_prim);
        table1.primitive_poses.push_back(table1_pose);
        table1.operation = moveit_msgs::msg::CollisionObject::ADD;

        // Table 2
        moveit_msgs::msg::CollisionObject table2;
        table2.header.frame_id = kWorldFrame;
        table2.id = "table_2";

        shape_msgs::msg::SolidPrimitive table2_prim;
        table2_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        table2_prim.dimensions = {0.3652, 0.3652, 0.016};

        geometry_msgs::msg::Pose table2_pose;
        table2_pose.position.x = 4.56;
        table2_pose.position.y = -0.50;  // Different Y position for second table
        table2_pose.position.z = 0.322;
        table2_pose.orientation.w = 1.0;

        table2.primitives.push_back(table2_prim);
        table2.primitive_poses.push_back(table2_pose);
        table2.operation = moveit_msgs::msg::CollisionObject::ADD;

        // Apply both tables to the scene
        planning_scene_.applyCollisionObject(table1);
        planning_scene_.applyCollisionObject(table2);

        // Give PlanningSceneInterface time to process
        rclcpp::sleep_for(500ms);

        return true;
    }

    void moveCollisionObject(const std::string& object_id,
                            const geometry_msgs::msg::Pose& new_pose,
                            const std::string& frame_id,
                            moveit::planning_interface::PlanningSceneInterface& planning_scene_)
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = frame_id;
        obj.id = object_id;
        obj.operation = moveit_msgs::msg::CollisionObject::MOVE;
        obj.pose = new_pose;

        planning_scene_.applyCollisionObject(obj);
    }

    std::vector<std::string> getSceneObjectNames(moveit::planning_interface::PlanningSceneInterface& planning_scene_)
    {
        return planning_scene_.getKnownObjectNames();
    }

    bool setGripper(double joint_value, moveit::planning_interface::MoveGroupInterface& gripper_mgi, const rclcpp::Logger& logger)
    {
        RCLCPP_INFO(logger, "Setting gripper to %.3f", joint_value);

        std::map<std::string, double> joint_values;
        joint_values[kActiveJointName] = joint_value;  // <-- ensure this matches your URDF/SRDF joint name
        gripper_mgi.setJointValueTarget(joint_values);
        return execute_move("Set gripper", gripper_mgi, logger);    
    }

    bool execute_move(const std::string& name,
                        moveit::planning_interface::MoveGroupInterface& mgi,
                        const rclcpp::Logger& logger)
    {
        const auto result = mgi.move();
        log_result(logger, name, result);
        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }

    // In motion_util.cpp, update executeCartesianPath:
    bool executeCartesianPath(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        const MotionConfig& config,
        moveit::planning_interface::MoveGroupInterface& arm_mgi,
        const rclcpp::Logger& logger
    ) {
        moveit_msgs::msg::RobotTrajectory trajectory;
        
        // Get current constraints (don't clear them!)
        moveit_msgs::msg::Constraints current_constraints = arm_mgi.getPathConstraints();
        
        // Use constraint-aware version (Page 8, 4th overload)
        double fraction = arm_mgi.computeCartesianPath(
            waypoints,
            config.cartesian_step_sizes[0],  // Use configured step size
            config.jump_threshold,            // Proper jump threshold
            trajectory,
            current_constraints,              // Keep constraints active
            true,                             // avoid_collisions = true
            nullptr                           // error_code (optional)
        );
        
        if (fraction >= config.min_cartesian_fraction) {
            return execute_trajectory("Cartesian path", arm_mgi, trajectory, 
                                    logger);
        }
        
        RCLCPP_WARN(logger, "Cartesian path only achieved %.1f%%", fraction * 100.0);
        return false;
    }

    bool plan_and_execute_to_pose(const geometry_msgs::msg::Pose& pose,
                                moveit::planning_interface::MoveGroupInterface& mgi,
                                const rclcpp::Logger& logger)
    {
        mgi.setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = mgi.plan(plan);
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            log_result(logger, "Plan to pose", plan_result);
            return false;
        }

        const auto exec_result = mgi.execute(plan);
        log_result(logger, "Execute to pose", exec_result);
        return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool attachObject(const std::string& object_id, moveit::planning_interface::MoveGroupInterface& mgi, const rclcpp::Logger& logger)
    {
        const auto ee_link = mgi.getEndEffectorLink();
        if (ee_link.empty()) {
            RCLCPP_ERROR(logger, "Cannot attach: end effector link is empty");
            return false;
        }

        const bool ok = mgi.attachObject(object_id, ee_link);
        if (ok) {
            RCLCPP_INFO(logger, "Attached '%s' to '%s'", object_id.c_str(), ee_link.c_str());
        } else {
            RCLCPP_ERROR(logger, "Failed to attach '%s' to '%s'", object_id.c_str(), ee_link.c_str());
        }
        return ok;
    }

    bool detachObject(const std::string& object_id, moveit::planning_interface::MoveGroupInterface& mgi, const rclcpp::Logger& logger)
    {
        const bool ok = mgi.detachObject(object_id);
        if (ok) {
            RCLCPP_INFO(logger, "Detached '%s'", object_id.c_str());
        } else {
            RCLCPP_ERROR(logger, "Failed to detach '%s'", object_id.c_str());
        }
        return ok;
    }

    bool move_to_joint_configuration(const std::map<std::string, double>& joint_targets,
                                     moveit::planning_interface::MoveGroupInterface& arm_mgi,
                                     const std::string& configuration_name,
                                     const rclcpp::Logger& logger)
    {
        RCLCPP_INFO(logger, "Moving to %s configuration...", configuration_name.c_str());
        
        arm_mgi.setJointValueTarget(joint_targets);
        auto result = arm_mgi.move();
        
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Failed to move to %s configuration", configuration_name.c_str());
            return false;
        }
        
        RCLCPP_INFO(logger, "Successfully moved to %s configuration", configuration_name.c_str());
        return true;
    }


    bool execute_trajectory(const std::string& name,
                            moveit::planning_interface::MoveGroupInterface& mgi,
                            const moveit_msgs::msg::RobotTrajectory& traj,
                            const rclcpp::Logger& logger)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = traj;

        const auto result = mgi.execute(plan);
        log_result(logger, name, result);
        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }

    void log_result(const rclcpp::Logger& logger,
                            const std::string& action,
                            const moveit::core::MoveItErrorCode& code)
    {
        if (code == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, ">> %s: SUCCESS", action.c_str());
        } else {
            RCLCPP_ERROR(logger, ">> %s: FAILED (MoveItErrorCode=%d)", action.c_str(), code.val);
        }
    }

} // namespace motion_util