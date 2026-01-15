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
    configure_moveit(arm_group_, gripper_group_);

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
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  // ======================
  // Perception
  // ======================
  rclcpp::Client<group_6_assignment_2::srv::GetCubePoses>::SharedPtr cube_pose_client_;
  geometry_msgs::msg::Pose reference_cube_pose_;

  // ======================
  // Parameters
  // ======================
  const double place_offset_y_ = 0.60;
  const double pre_place_offset_z_ = 0.20;
  const double retreat_offset_z_ = 0.20; // hardcoded offset for table placement

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

    /*
      DEFENSIVE CHECKS
    */

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
    geometry_msgs::msg::Pose place_pose = compute_goal_place_pose(target_table_id);


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


    //STEP 2 move to pre-place pose
    feedback->status = "Moving to pre-place pose";
    goal_handle->publish_feedback(feedback);
    
    if(!motion_util::plan_and_execute_to_pose(place_pose, arm_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to move to pre-place pose");
      return false;
    }
    if (goal_handle->is_canceling()) return false;

    //STEP 3 OPEN THE ARM
    feedback->status = "Opening gripper to release object";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::setGripper(0.0, gripper_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to open gripper");
      return false;
    }
    if (goal_handle->is_canceling()) return false;
    RCLCPP_INFO(get_logger(), "Place action completed successfully");
    return true;

    // STEP 4 DETATCH OBJECT
    feedback->status = "Detaching object from gripper";
    goal_handle->publish_feedback(feedback);
    if (!motion_util::detachObject(current_cube_id, arm_group_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to detach object from gripper");
      return false;
    }
    if (goal_handle->is_canceling()) return false;
    rclcpp::sleep_for(1s); // Brief pause to let physics settle

    // FINISH
    RCLCPP_INFO(get_logger(), "Place action completed successfully");
    return true;
  
  }

  //compute goal pose function returns a pose
  // for now simple offset depending on the table id
   geometry_msgs::msg::Pose compute_goal_place_pose(
    uint8_t target_table_id)
  {  
    geometry_msgs::msg::Pose place_pose;
    if (target_table_id == 1) {
      place_pose.position.x = 3.97 + 0.80;
      place_pose.position.y = -1.10;
      //place_pose.position.z = 0.446 + 0.1;
    } else if (target_table_id == 2) {
      place_pose.position.x = 4.56;
      place_pose.position.y = -0.50 - 0.90;
      //place_pose.position.z = 0.446 + 0.1;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid target_table_id: %d", target_table_id);
    }

    // set quaternion via RPY
     tf2::Quaternion q;
    q.setRPY(0.0, 
             3.1415, 
             0.0);
    q.normalize();

    place_pose.orientation.x = q.x();
    place_pose.orientation.y = q.y();
    place_pose.orientation.z = q.z();
    place_pose.orientation.w = q.w();

    return place_pose;

  }

//   // ======================
//   // Helpers
//   // ======================
//   bool sense_reference_cube(uint8_t cube_id)
//   {
//     if (!cube_pose_client_->wait_for_service(5s)) {
//       RCLCPP_ERROR(get_logger(), "GetCubePoses service unavailable");
//       return false;
//     }

//     auto req = std::make_shared<group_6_assignment_2::srv::GetCubePoses::Request>();
//     req->target_ids.push_back(static_cast<int32_t>(cube_id));

//     auto future = cube_pose_client_->async_send_request(req);
//     if (future.wait_for(10s) != std::future_status::ready) {
//       RCLCPP_ERROR(get_logger(), "Cube pose service timeout");
//       return false;
//     }

//     auto res = future.get();
//     if (!res->success || res->poses.empty()) return false;

//     reference_cube_pose_ = res->poses[0];
//     return true;
//   }

//   geometry_msgs::msg::Pose compute_place_pose()
//   {
//     geometry_msgs::msg::Pose pose = reference_cube_pose_;
//     pose.position.y += place_offset_y_;
//     return pose;
//   }

//   bool move_to_pre_place(const geometry_msgs::msg::Pose& place_pose)
//   {
//     geometry_msgs::msg::Pose pre = place_pose;
//     pre.position.z += pre_place_offset_z_;
//     return plan_and_execute_to_pose(pre, arm_group_, get_logger());
//   }


//   bool retreat_from_place(const geometry_msgs::msg::Pose& place_pose)
//   {
//     geometry_msgs::msg::Pose retreat = place_pose;
//     retreat.position.z += retreat_offset_z_;
//     return executeCartesianPath(
//       {retreat}, 0.01, 0.15, arm_group_, get_logger());
//   }
// 
};

}  // namespace place_action_cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<place_action_cpp::PlaceActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
