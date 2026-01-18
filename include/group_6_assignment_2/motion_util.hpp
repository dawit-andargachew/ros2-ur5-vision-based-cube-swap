#ifndef MY_PROJECT_MOTION_UTILS_HPP
#define MY_PROJECT_MOTION_UTILS_HPP

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>

namespace motion_util {

// Forward declare or define MotionConfig before functions that use it
struct MotionConfig {
    // Planning parameters
    double max_velocity_scaling_factor = 0.4;
    double max_acceleration_scaling_factor = 0.3;
    double planning_time = 30.0;
    int num_planning_attempts = 20;
    
    // Goal tolerances
    double goal_position_tolerance = 0.015;
    // Note: orientation tolerance was previously 0.15 rad (~8.6°). It was tightened to 0.010 rad (~0.57°)
    // to improve end-effector orientation accuracy (e.g., for precise tool alignment). This ~15x reduction
    // in tolerance can make planning more stringent and may reduce planning success rates in cluttered scenes.
    double goal_orientation_tolerance = 0.010;
    double goal_joint_tolerance = 0.02;
    
    // Gripper parameters
    double gripper_max_velocity_scaling_factor = 0.8;
    double gripper_max_acceleration_scaling_factor = 0.8;
    double gripper_goal_joint_tolerance = 0.02;
    double gripper_planning_time = 5.0;
    int gripper_num_planning_attempts = 5;
    
    // Cartesian path parameters
    std::vector<double> cartesian_step_sizes = {0.05, 0.03, 0.01};
    double min_cartesian_fraction = 0.5;
    // Note: jump_threshold is deprecated in newer MoveIt versions
    // double jump_threshold = 0.0;  // Deprecated - no longer used
    
    // State monitoring and timing
    double state_monitor_wait_time = 2.0;  // seconds to wait for state monitor initialization
    double scene_update_delay = 0.5;  // seconds to wait after applying collision objects
    
    // Workspace bounds
    struct Workspace {
        double min_x = -0.9, min_y = -0.9, min_z = -0.05;
        double max_x = 0.9, max_y = 0.9, max_z = 1.1;
    } workspace;
    
    // Replanning
    bool allow_replanning = true;
    int replan_attempts = 3;
    double replan_delay = 2.0;
};

    void configure_moveit(moveit::planning_interface::MoveGroupInterface& arm_mgi,
                         moveit::planning_interface::MoveGroupInterface& gripper_mgi,
                         const MotionConfig& config);

    void apply_path_constraints(moveit::planning_interface::MoveGroupInterface& arm_group_, const rclcpp::Logger& logger);

    bool setup_scene(moveit::planning_interface::PlanningSceneInterface& planning_scene_);

    void moveCollisionObject(const std::string& object_id,
                            const geometry_msgs::msg::Pose& new_pose,
                            const std::string& frame_id,
                            moveit::planning_interface::PlanningSceneInterface& planning_scene_);

    std::vector<std::string> getSceneObjectNames(moveit::planning_interface::PlanningSceneInterface& planning_scene_);

    bool setGripper(double joint_value, moveit::planning_interface::MoveGroupInterface& gripper_mgi, const rclcpp::Logger& logger);

    bool execute_move(const std::string& name,
                        moveit::planning_interface::MoveGroupInterface& mgi,
                        const rclcpp::Logger& logger);

    bool execute_trajectory(const std::string& name,
                            moveit::planning_interface::MoveGroupInterface& mgi,
                            const moveit_msgs::msg::RobotTrajectory& traj,
                            const rclcpp::Logger& logger);

    bool executeCartesianPath(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        const MotionConfig& config,
        moveit::planning_interface::MoveGroupInterface& arm_mgi,
        const rclcpp::Logger& logger
    );
                       
    bool plan_and_execute_to_pose(const geometry_msgs::msg::Pose& pose,
                                  moveit::planning_interface::MoveGroupInterface& mgi,
                                  const rclcpp::Logger& logger);

    bool attachObject(const std::string& object_id, moveit::planning_interface::MoveGroupInterface& mgi, const rclcpp::Logger& logger);
    bool detachObject(const std::string& object_id, moveit::planning_interface::MoveGroupInterface& mgi, const rclcpp::Logger& logger);

    bool move_to_joint_configuration(const std::map<std::string, double>& joint_targets,
                                     moveit::planning_interface::MoveGroupInterface& arm_mgi,
                                     const std::string& configuration_name,
                                     const rclcpp::Logger& logger);

    void log_result(const rclcpp::Logger& logger,
                            const std::string& action,
                            const moveit::core::MoveItErrorCode& code);

    static constexpr const char* kArmGroupName = "ir_arm";
    static constexpr const char* kGripperGroupName = "ir_gripper";  
    static constexpr const char* kActiveJointName = "robotiq_85_left_knuckle_joint";
    static constexpr const char* kWorldFrame = "world";
    
    // Table collision object dimensions and positions
    struct TableConfig {
        static constexpr double width = 0.3652;
        static constexpr double depth = 0.3652;
        static constexpr double height = 0.016;
        static constexpr double z_position = 0.322;

        // Table 1 position
        static constexpr double table1_x = 4.01;
        static constexpr double table1_y = -1.10;
        
        // Table 2 position
        static constexpr double table2_x = 4.6;
        static constexpr double table2_y = -0.50;
    };
} // namespace motion_util

#endif