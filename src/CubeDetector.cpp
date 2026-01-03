/**
 * Node: cube_detector
 * 
 * type: service node
 * 
 * purpose: 
 * Detects cubes in the environment using camera and apriltag data.
 * Provides services to get the positions of detected cubes.
 * each cube can be requested by its unique ID.
 * 
 * 
 * output:
 *  Provides a service "get_cube_position" that returns the position of a cube given its ID.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "group_6_assignment_2/srv/get_cube_poses.hpp"

using GetCubePoses = group_6_assignment_2::srv::GetCubePoses;

using std::placeholders::_1;
using std::placeholders::_2;

class CubeDetector : public rclcpp::Node
{
public:
    CubeDetector() : Node("cube_detector")
    {
        //create the service
        service_ = this->create_service<GetCubePoses>(
            "get_cube_poses",
            std::bind(&CubeDetector::handle_get_cube_poses, this, _1, _2)
        );  

        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Cube Detector Service is ready.");
    }
private:
    static constexpr int FIND_TF_TIMEOUT_MS = 200;

    void handle_get_cube_poses(
        const std::shared_ptr<GetCubePoses::Request> request,
        std::shared_ptr<GetCubePoses::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request for cube poses.");

        response->success = true;
        response->message = "";
        response->detected_ids.clear();
        response->poses.clear();

        if (request->target_ids.empty())
        {
            response->success = false;
            response->message = "No target_ids provided.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        std::vector<int32_t> missing_ids;

        for (const auto& cube_id : request->target_ids)
        {
            //check tag exists and transform lookup
            if(!tf_buffer_->canTransform("world", "tag36h11:" + std::to_string(cube_id), tf2::TimePointZero, std::chrono::milliseconds(FIND_TF_TIMEOUT_MS)))
            {
                RCLCPP_WARN(this->get_logger(), "Transform for tag36h11:%d not available.", cube_id);
                response->success = false;
                missing_ids.push_back(cube_id);
                continue;
            }

           
            

            try
            {
                //lookup transforms for tag in the outputframe "world"
                geometry_msgs::msg::TransformStamped tag_transform = tf_buffer_->lookupTransform(
                    "world", 
                    "tag36h11:" + std::to_string(cube_id), 
                    tf2::TimePointZero
                );
                //convert to pose
                geometry_msgs::msg::Pose tag_pose;
                tag_pose.position.x = tag_transform.transform.translation.x;
                tag_pose.position.y = tag_transform.transform.translation.y;
                tag_pose.position.z = tag_transform.transform.translation.z;
                tag_pose.orientation = tag_transform.transform.rotation;
                //add to response
                response->detected_ids.push_back(cube_id);
                response->poses.push_back(tag_pose);
                RCLCPP_INFO(this->get_logger(), "Found pose for tag36h11:%d.", cube_id);   
                
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not transform tag36h11:%d to world frame: %s", cube_id, ex.what());
                response->success = false;
                missing_ids.push_back(cube_id);
              
            }
        }

        if (response->success)
        {
            response->message = "All requested tags found.";
        }
        else
        {
            std::ostringstream oss;
            oss << "Missing one or more tags. Missing IDs: [";
            for (size_t i = 0; i < missing_ids.size(); ++i)
            {
                oss << missing_ids[i];
                if (i + 1 < missing_ids.size())
                {
                    oss << ", ";
                }
            }
            oss << "]";
            response->message = oss.str();
        }
    }

    rclcpp::Service<GetCubePoses>::SharedPtr service_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubeDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}