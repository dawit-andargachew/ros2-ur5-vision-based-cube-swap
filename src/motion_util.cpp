#include "group_6_assignment_2/motion_util.hpp"
#include <chrono>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace motion_util {

    // void configure_moveit(moveit::planning_interface::MoveGroupInterface& arm_mgi,
    //                      moveit::planning_interface::MoveGroupInterface& gripper_mgi) {
    //     arm_mgi.setMaxVelocityScalingFactor(0.5);  // Fixed: was arm_mgi_ (typo)
    //     arm_mgi.setMaxAccelerationScalingFactor(0.5);
    //     arm_mgi.setPlanningTime(20.0);

    //     // (void)arm_mgi.getCurrentState(2.0);  // Added: matches MotionInterface.cpp
    //     // (void)gripper_mgi.getCurrentState(2.0);  // Added: matches MotionInterface.cpp
    // }

    // In motion_util.cpp, update configure_moveit:
    void configure_moveit(moveit::planning_interface::MoveGroupInterface& arm_mgi,
                        moveit::planning_interface::MoveGroupInterface& gripper_mgi) {
        // More balanced settings
        arm_mgi.setMaxVelocityScalingFactor(0.5);  // Increased from 0.3
        arm_mgi.setMaxAccelerationScalingFactor(0.4);
        arm_mgi.setPlanningTime(10.0);  // Reduced from 20.0
        arm_mgi.setNumPlanningAttempts(10);  // Reduced from 50
        
        // Use relaxed tolerances
        arm_mgi.setGoalPositionTolerance(0.015);  // 1.5cm
        arm_mgi.setGoalOrientationTolerance(0.1);  // ~5.7 degrees
        arm_mgi.setGoalJointTolerance(0.02);
        
        // // Set a robust planner
        // arm_mgi.setPlannerId("RRTConnect");
        
        // For Cartesian paths, use:
        arm_mgi.setMaxVelocityScalingFactor(0.3);  // Slower for precision
        arm_mgi.setPlanningTime(15.0);  // Longer for Cartesian
        
        // Gripper settings
        gripper_mgi.setMaxVelocityScalingFactor(0.8);
        gripper_mgi.setMaxAccelerationScalingFactor(0.8);
        gripper_mgi.setPlanningTime(5.0);
        gripper_mgi.setNumPlanningAttempts(3);
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
        // ---- 1) Table collision object (static, can be hardcoded)
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = kWorldFrame;
        table.id = kTableId;

        shape_msgs::msg::SolidPrimitive table_prim;
        table_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        table_prim.dimensions = {0.3652, 0.3652, 0.016}; //static world table

        geometry_msgs::msg::Pose table_pose;
        table_pose.position.x = 3.97;
        table_pose.position.y = -1.10;
        table_pose.position.z = 0.322;
        table_pose.orientation.w = 1.0;

        table.primitives.push_back(table_prim);
        table.primitive_poses.push_back(table_pose);
        table.operation = moveit_msgs::msg::CollisionObject::ADD;

        planning_scene_.applyCollisionObject(table);

        // Give PlanningSceneInterface a beat to help sim
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
    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                            double eef_step,
                            double min_fraction,
                            moveit::planning_interface::MoveGroupInterface& arm_mgi_,
                            const rclcpp::Logger& logger) {
        
        // Temporary relaxation of constraints for Cartesian motion
        moveit_msgs::msg::Constraints original_constraints = arm_mgi_.getPathConstraints();
        arm_mgi_.clearPathConstraints();
        
        // Try multiple times with different step sizes
        std::vector<double> step_sizes = {0.05, 0.03, 0.01};  // Try coarse first
        double best_fraction = 0.0;
        moveit_msgs::msg::RobotTrajectory best_traj;
        
        for (double step : step_sizes) {
            moveit_msgs::msg::RobotTrajectory traj;
            const double jump_thresh = 0.0;
            const double fraction = arm_mgi_.computeCartesianPath(waypoints, step, jump_thresh, traj);
            
            RCLCPP_INFO(logger, "Cartesian path with step %.3f: %.1f%% computed", 
                    step, fraction * 100.0);
            
            if (fraction > best_fraction) {
                best_fraction = fraction;
                best_traj = traj;
            }
            
            if (fraction >= min_fraction) break;
        }
        
        // Restore constraints
        if (!original_constraints.joint_constraints.empty()) {
            arm_mgi_.setPathConstraints(original_constraints);
        }
        
        if (best_fraction < min_fraction) {
            RCLCPP_WARN(logger, "Best Cartesian path only %.1f%% (required %.1f%%)", 
                    best_fraction * 100.0, min_fraction * 100.0);
            // Try with lower requirement
            if (best_fraction > 0.5) {  // If we got more than 50%, try executing
                RCLCPP_INFO(logger, "Attempting to execute partial path (%.1f%%)", 
                        best_fraction * 100.0);
                return execute_trajectory("Execute partial Cartesian path", 
                                        arm_mgi_, best_traj, logger);
            }
            return false;
        }
        
        return execute_trajectory("Execute Cartesian path", arm_mgi_, best_traj, logger);
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