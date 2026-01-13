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

    void configure_moveit(moveit::planning_interface::MoveGroupInterface& arm_mgi,
                         moveit::planning_interface::MoveGroupInterface& gripper_mgi);

    void apply_path_constraints(moveit::planning_interface::MoveGroupInterface& arm_group_, const rclcpp::Logger& logger);

    bool setup_scene(moveit::planning_interface::PlanningSceneInterface& planning_scene_);

    void moveCollisionObject(const std::string& object_id,
                            const geometry_msgs::msg::Pose& new_pose,
                            const std::string& frame_id,
                            moveit::planning_interface::PlanningSceneInterface& planning_scene_);

    bool setGripper(double joint_value, moveit::planning_interface::MoveGroupInterface& gripper_mgi, const rclcpp::Logger& logger);

    bool execute_move(const std::string& name,
                        moveit::planning_interface::MoveGroupInterface& mgi,
                        const rclcpp::Logger& logger);

    bool execute_trajectory(const std::string& name,
                            moveit::planning_interface::MoveGroupInterface& mgi,
                            const moveit_msgs::msg::RobotTrajectory& traj,
                            const rclcpp::Logger& logger);

    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                            double eef_step,
                            double min_fraction,
                            moveit::planning_interface::MoveGroupInterface& arm_mgi_,
                            const rclcpp::Logger& logger
                        );
    bool plan_and_execute_to_pose(const geometry_msgs::msg::Pose& pose,
                                  moveit::planning_interface::MoveGroupInterface& mgi,
                                  const rclcpp::Logger& logger);

    bool attachObject(const std::string& object_id, moveit::planning_interface::MoveGroupInterface& mgi, const rclcpp::Logger& logger);
    bool detachObject(const std::string& object_id, moveit::planning_interface::MoveGroupInterface& mgi, const rclcpp::Logger& logger);

    void log_result(const rclcpp::Logger& logger,
                            const std::string& action,
                            const moveit::core::MoveItErrorCode& code);

    static constexpr const char* kArmGroupName = "ir_arm";
    static constexpr const char* kGripperGroupName = "ir_gripper";  
    static constexpr const char* kActiveJointName = "robotiq_85_left_knuckle_joint";
    static constexpr const char* kWorldFrame = "world";
    static constexpr const char* kCubeId1 = "target_cube_1";
    static constexpr const char* kTableId1 = "surface_collision_1";
    static constexpr const char* kCubeId2 = "target_cube_2";
    static constexpr const char* kTableId2 = "surface_collision_2";
    static constexpr double kOpenVal = 0.0;
    static constexpr double kClosedVal = 0.10;  // tune for collisions

    static constexpr double kGripperLengthOffset = 0.18;
    static constexpr double kGraspCenterZOffset = -0.02;
    static constexpr double kPreGraspApproachDist = 0.15;
} // namespace motion_util

#endif