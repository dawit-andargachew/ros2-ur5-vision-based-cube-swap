#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "group_6_assignment_2/action/place.hpp"
#include "group_6_assignment_2/srv/get_cube_poses.hpp"
#include "group_6_assignment_2/motion_util.hpp"
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;
using namespace motion_util;

namespace place_action_cpp
{

class PlaceActionServer : public rclcpp::Node
{
public:
  using Place = group_6_assignment_2::action::Place;
  using GoalHandlePlace = rclcpp_action::ServerGoalHandle<Place>;

  explicit PlaceActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("place_action_server", options),
    arm_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kArmGroupName),
    gripper_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), kGripperGroupName)
  {
    const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    place_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/place_pose", pose_qos);

    MotionConfig config;  // Use default configuration
    configure_moveit(arm_group_, gripper_group_, config);

    cube_pose_client_ =
      this->create_client<group_6_assignment_2::srv::GetCubePoses>("get_cube_poses");

    action_server_ = rclcpp_action::create_server<Place>(
      this,
      "place",
      std::bind(&PlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PlaceActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  // ======================
  // Action server
  // ======================
  rclcpp_action::Server<Place>::SharedPtr action_server_;

  // ======================
  // MoveIt
  // ======================
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_; // useful only if we reintegrate 
                                                                      // the defensive check at the start,
                                                                      // pointless otherwise (CANCEL)

  // ======================
  // Perception
  // ======================
  rclcpp::Client<group_6_assignment_2::srv::GetCubePoses>::SharedPtr cube_pose_client_;

  // ======================
  // Debug visualization
  // ======================
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr place_pose_pub_;

  // ======================
  // Action callbacks
  // ======================
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const Place::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received place goal request for grabbed_cube_id %d onto target_table_id %d",
                goal->grabbed_cube_id, goal->target_table_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePlace>)
  {
    RCLCPP_WARN(get_logger(), "Place goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    std::thread{
      std::bind(&PlaceActionServer::execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    auto result = std::make_shared<Place::Result>();
    uint8_t grabbed_cube_id = goal_handle->get_goal()->grabbed_cube_id;
    uint8_t target_table_id = goal_handle->get_goal()->target_table_id;
    RCLCPP_INFO(get_logger(), "Executing place for grabbed_cube_id: %d onto target_table_id: %d",
                grabbed_cube_id, target_table_id);

    bool success = run_place_sequence(grabbed_cube_id, target_table_id, goal_handle);
    result->success = success;

    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ======================
  // Place sequence
  // ======================

  
  bool run_place_sequence(
    uint8_t grabbed_cube_id,
    uint8_t target_table_id,
    const std::shared_ptr<GoalHandlePlace> goal_handle)
  {

    auto feedback = std::make_shared<Place::Feedback>();

    // check if grabbed_cube_id is attached if not fail and end action
    feedback->status = "Verifying grabbed cube is attached";
    goal_handle->publish_feedback(feedback);
    const std::string current_cube_id = "cube_" + std::to_string(grabbed_cube_id);
    /*
    auto attached_objects = planning_scene_.getAttachedObjects();
    if (attached_objects.find(current_cube_id) == attached_objects.end()) {
      RCLCPP_ERROR(get_logger(), "Grabbed cube '%s' is not attached", current_cube_id.c_str());
      return false;
    }
    */
    //STEP 1 compute goal place pose
    std::optional<geometry_msgs::msg::Pose> place_pose_opt = compute_goal_place_pose(target_table_id);
    if (!place_pose_opt) {
      RCLCPP_ERROR(get_logger(), "Failed to compute goal place pose. Check target_table_id validity");
      return false;
    }
    geometry_msgs::msg::Pose place_pose = *place_pose_opt;
    publish_debug_pose(place_pose, place_pose_pub_);
    RCLCPP_INFO(get_logger(), "Place pose: x=%.3f, y=%.3f, z=%.3f",
                place_pose.position.x, place_pose.position.y, place_pose.position.z);


    /*
    //go home
    feedback->status = "Moving to home position before placing";
    goal_handle->publish_feedback(feedback);
    std::map<std::string, double> home_joint_config = {
      {"shoulder_pan_joint", 0.0},
      {"shoulder_lift_joint", -1.57}, 
      {"elbow_joint", 0.0},
      {"wrist_1_joint", -1.57},
      {"wrist_2_joint", 0.0},
      {"wrist_3_joint", 0.0}
    }; 
    if (!motion_util::move_to_joint_configuration(home_joint_config, arm_group_, "home", get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to move to home position before placing");
      return false;
    }
    if (goal_handle->is_canceling()) return false;
    */


    //STEP 2 move to pre-place pose
    feedback->status = "Moving to pre-place pose";
    goal_handle->publish_feedback(feedback);
    
    if(!motion_util::plan_and_execute_to_pose(place_pose, arm_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to move to pre-place pose");
      return false;
    }
    if (goal_handle->is_canceling()) return false;

    // STEP 5 DETACH OBJECT
    feedback->status = "Detaching object from gripper";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::detachObject(current_cube_id, arm_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to detach object from gripper");
      return false;
    }
    if (goal_handle->is_canceling()) return false;
    rclcpp::sleep_for(1s); // Brief pause to let physics settle


    //STEP 3 OPEN THE ARM
    feedback->status = "Opening gripper to release object";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::setGripper(0.0, gripper_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to open gripper");
      return false;
    }
    if (goal_handle->is_canceling()) return false;

    // STEP 4 RAISE TO SAFE HEIGHT
    // Small clearance move (joint or pose) to make the cartesian not fail hopefully
    /*
    geometry_msgs::msg::Pose clearance_pose =
        arm_group_.getCurrentPose().pose;

    clearance_pose.position.z += 0.02;  // 2 cm

    if (!motion_util::plan_and_execute_to_pose(
            clearance_pose, arm_group_, get_logger()))
    {
      RCLCPP_ERROR(get_logger(), "Failed clearance move");
      return false;
    }
    */

    feedback->status = "Moving up (Cartesian)...";
    goal_handle->publish_feedback(feedback);
    if (!move_up_cartesian(0.05))
    {
      RCLCPP_ERROR(get_logger(), "Failed to raise to safe height");
      return false;
    }
    if (goal_handle->is_canceling()) return false;

    // FINISH
    RCLCPP_INFO(get_logger(), "Place action completed successfully");
    return true;
  
  }

  // compute goal pose function returns a pose
  // simple offset depending on the table id
  std::optional<geometry_msgs::msg::Pose> compute_goal_place_pose(
    uint8_t target_table_id)
  {  
    geometry_msgs::msg::Pose place_pose;
    if (target_table_id == 1) {
      place_pose.position.x = 3.97 + 0.10;
      //place_pose.position.y = -1.10 - 0.10; //old offset
      place_pose.position.y = -1.10 - 0.05;
      place_pose.position.z = 0.446 + 0.18;
    } else if (target_table_id == 2) {
      //place_pose.position.x = 4.56 + 0.10; //old offset
      place_pose.position.x = 4.56 + 0.05;
      place_pose.position.y = -0.50 - 0.10;
      place_pose.position.z = 0.446 + 0.18;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid target_table_id: %d", target_table_id);
      return std::nullopt;
    }

    // set quaternion via RPY
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI, 0.0);
    q.normalize();

    place_pose.orientation.x = q.x();
    place_pose.orientation.y = q.y();
    place_pose.orientation.z = q.z();
    place_pose.orientation.w = q.w();

    return place_pose;

  }

  bool move_up_cartesian(double lift_distance)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose lifted_pose = arm_group_.getCurrentPose().pose;
    lifted_pose.position.z += lift_distance;
    waypoints.push_back(lifted_pose);
    motion_util::MotionConfig config;
    config.cartesian_step_sizes = {0.01};
    config.min_cartesian_fraction = 0.15;
    return executeCartesianPath(waypoints, config, arm_group_, get_logger());
  }


  void publish_debug_pose(
    const geometry_msgs::msg::Pose& pose,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
    const std::string& frame_id = "world")
  {
    if (!publisher) {
      return;
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    msg.pose = pose;
    publisher->publish(msg);
  }
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<place_action_cpp::PlaceActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
