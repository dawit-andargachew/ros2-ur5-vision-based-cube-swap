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

    // In motion_util.cpp, update configure_moveit function:
    void configure_moveit(moveit::planning_interface::MoveGroupInterface& arm_mgi,
                     moveit::planning_interface::MoveGroupInterface& gripper_mgi) {
    // More conservative velocity and acceleration for stability
    arm_mgi.setMaxVelocityScalingFactor(0.3);
    arm_mgi.setMaxAccelerationScalingFactor(0.3);
    
    // Increase planning time to allow more attempts
    arm_mgi.setPlanningTime(30.0);
    
    // Set number of planning attempts
    arm_mgi.setNumPlanningAttempts(10);
    
    // Set goal tolerance (helps with hard-to-reach poses)
    arm_mgi.setGoalPositionTolerance(0.01);  // 1cm position tolerance
    arm_mgi.setGoalOrientationTolerance(0.05);  // ~3 degree orientation tolerance
    arm_mgi.setGoalJointTolerance(0.01);  // Joint tolerance in radians
    
    // Allow replanning during execution if needed
    arm_mgi.allowReplanning(true);
    
    // Set specific planner (RRTConnect is generally robust)
    arm_mgi.setPlannerId("RRTConnectkConfigDefault");
    
    // Configure gripper as well
    gripper_mgi.setMaxVelocityScalingFactor(0.5);
    gripper_mgi.setMaxAccelerationScalingFactor(0.5);
    gripper_mgi.setPlanningTime(10.0);
    gripper_mgi.setNumPlanningAttempts(5);
}


    void apply_path_constraints(moveit::planning_interface::MoveGroupInterface& arm_group_, const rclcpp::Logger& logger)
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

        RCLCPP_INFO(logger, "Elbow-up joint constraint applied.");
    }

    bool setup_scene(moveit::planning_interface::PlanningSceneInterface& planning_scene_)
    {
        // ---- 1) Table collision object
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = kWorldFrame;
        table.id = kTableId;

        shape_msgs::msg::SolidPrimitive table_prim;
        table_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        table_prim.dimensions = {0.3652, 0.3652, 0.016}; //static world table

        geometry_msgs::msg::Pose table_pose; //manual pose b/c static -> but maybe we can calculate based on the initial cube poses
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

        geometry_msgs::msg::Pose cube_pose_{};
        constexpr double kOffsetXLateral = +0.035; //trial and error
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

    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                          double eef_step,
                          double min_fraction,
                          moveit::planning_interface::MoveGroupInterface& arm_mgi_,
                          const rclcpp::Logger& logger
                      )
    {
        moveit_msgs::msg::RobotTrajectory traj;
        const double jump_thresh = 0.0;
        const double fraction = arm_mgi_.computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

        RCLCPP_INFO(logger, "Computed Cartesian path fraction: %.1f%%", fraction * 100.0);  // Fixed: was node_->get_logger()

        if (fraction < min_fraction) {
            RCLCPP_ERROR(logger, "Cartesian path failed (only %.1f%% computed).", fraction * 100.0);  // Fixed: was logger (already correct, but ensuring consistency)
            return false;
        }

        return execute_trajectory("Execute Cartesian path", arm_mgi_, traj, logger);
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