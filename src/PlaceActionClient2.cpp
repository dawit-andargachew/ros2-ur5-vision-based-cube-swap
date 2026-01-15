// #include <chrono>
// #include <memory>
// #include <string>
// #include <vector>

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <tf2/LinearMath/Quaternion.h>

// #include "group_6_assignment_2/action/place.hpp"
// #include "group_6_assignment_2/srv/get_cube_poses.hpp"

// using namespace std::chrono_literals;

// class PlaceActionClient : public rclcpp::Node
// {
// public:
//   using Place = group_6_assignment_2::action::Place;
//   using GoalHandlePlace = rclcpp_action::ClientGoalHandle<Place>;
//   using GetCubePoses = group_6_assignment_2::srv::GetCubePoses;

//   explicit PlaceActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
//   : rclcpp::Node("place_action_client", options)
//   {
//     // Action Client to talk to Place Server
//     this->client_ptr_ = rclcpp_action::create_client<Place>(this, "place");
    
//     // Service Client to detect tags (to find where to place)
//     this->cube_poses_client_ = this->create_client<GetCubePoses>("get_cube_poses");
//   }

//   void perform_place_sequence(int32_t cube_to_move_id, int32_t reference_cube_id)
//   {
//     // 1. Find the Pose of the Reference Cube (Cube 10)
//     auto target_pose = get_reference_pose(reference_cube_id);
    
//     if (!target_pose_valid_) {
//       RCLCPP_ERROR(this->get_logger(), "Could not determine placement pose. Aborting.");
//       return;
//     }

//     // 2. Apply Offset (Place NEXT TO the reference cube, not ON it)
//     // Table 2 (Tag 10) is around Y = -0.5. The table is small.
//     // We shift by 12cm in Y to sit beside it.
//     target_pose.position.y += 0.12; 
    
//     // Maintain safe Z height (matches your image data ~0.44)
//     // Z from image is ~0.44. This includes table height + cube center.
//     // We can keep this Z or slightly adjust if we want to hover.
//     target_pose.position.z = 0.45; 

//     // 3. Set Orientation (Gripper facing down)
//     tf2::Quaternion q;
//     q.setRPY(0, -1.57, 0); // Pitch -90 degrees
//     target_pose.orientation.x = q.x();
//     target_pose.orientation.y = q.y();
//     target_pose.orientation.z = q.z();
//     target_pose.orientation.w = q.w();

//     // 4. Send Goal
//     send_goal(cube_to_move_id, target_pose);
//   }

// private:
//   rclcpp_action::Client<Place>::SharedPtr client_ptr_;
//   rclcpp::Client<GetCubePoses>::SharedPtr cube_poses_client_;
//   bool target_pose_valid_ = false;

//   geometry_msgs::msg::Pose get_reference_pose(int32_t id)
//   {
//     geometry_msgs::msg::Pose pose;
    
//     if (!cube_poses_client_->wait_for_service(5s)) {
//       RCLCPP_ERROR(get_logger(), "GetCubePoses service not available");
//       return pose;
//     }

//     auto request = std::make_shared<GetCubePoses::Request>();
//     request->target_ids.push_back(id);

//     auto future = cube_poses_client_->async_send_request(request);
    
//     // Simple blocking wait for demonstration (in real node, use callbacks or state machine)
//     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == 
//         rclcpp::FutureReturnCode::SUCCESS) 
//     {
//       auto response = future.get();
//       if (response->success && !response->poses.empty()) {
//         pose = response->poses[0];
//         target_pose_valid_ = true;
//         RCLCPP_INFO(get_logger(), "Found Reference Tag %d at [x:%.2f, y:%.2f, z:%.2f]", 
//           id, pose.position.x, pose.position.y, pose.position.z);
//       } else {
//         RCLCPP_ERROR(get_logger(), "Service failed to find Tag %d", id);
//       }
//     } else {
//       RCLCPP_ERROR(get_logger(), "Service call failed");
//     }
//     return pose;
//   }

//   void send_goal(int32_t cube_id, geometry_msgs::msg::Pose target_pose)
//   {
//     if (!this->client_ptr_->wait_for_action_server(10s)) {
//       RCLCPP_ERROR(this->get_logger(), "Place Action server not available");
//       return;
//     }

//     auto goal_msg = Place::Goal();
//     goal_msg.cube_id = cube_id;
//     goal_msg.place_pose = target_pose;

//     RCLCPP_INFO(this->get_logger(), "Sending PLACE goal for Cube %d -> [x:%.2f, y:%.2f]", 
//         cube_id, target_pose.position.x, target_pose.position.y);

//     auto send_goal_options = rclcpp_action::Client<Place>::SendGoalOptions();
//     send_goal_options.result_callback =
//       std::bind(&PlaceActionClient::result_callback, this, std::placeholders::_1);

//     this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//   }

//   void result_callback(const GoalHandlePlace::WrappedResult& result)
//   {
//     if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//         RCLCPP_INFO(this->get_logger(), "Cube Placed Successfully!");
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Place Action Failed");
//     }
//     rclcpp::shutdown();
//   }
// };

// int main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<PlaceActionClient>();
  
//   // Logic: "Move Cube 1 to the table where Cube 10 is"
//   // We use Cube 10 as the reference anchor.
//   node->perform_place_sequence(1, 10);
  
//   rclcpp::spin(node); // Note: perform_place_sequence calls shutdown in callback
//   rclcpp::shutdown();
//   return 0;
// }