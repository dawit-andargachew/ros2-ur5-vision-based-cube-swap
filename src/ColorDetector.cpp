/**
 * Node: color_detector
 * * Type: Service Server & Client
 *
 *  * Purpose: 
 * Determines the color of specific cubes by fusing 3D spatial data with 2D computer vision.
 * Synchronizes with the 'cube_detector' service to retrieve verified cube poses.
 * Projects 3D poses onto the image plane to match physical cubes with detected color blobs.
 *
 * * Output:
 * Provides service "get_cube_colors" returning a mapped string of Cube IDs to Colors.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <mutex>
#include <cmath>

#include "group_6_assignment_2/srv/get_cube_colors.hpp"
#include "group_6_assignment_2/srv/get_cube_poses.hpp"


#include <image_geometry/pinhole_camera_model.hpp>


using GetCubeColors = group_6_assignment_2::srv::GetCubeColors;
using GetCubePoses = group_6_assignment_2::srv::GetCubePoses;
using std::placeholders::_1;
using std::placeholders::_2;

// --- Data Structures ---
struct ColorDef {
    std::string name;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> ranges;
};

struct DetectedObject {
    std::string color;
    int x; // Pixel U
    int y; // Pixel V
    double area;
    int matched_id; // -1 if not matched to a cube ID
};

class ColorDetector : public rclcpp::Node
{
public:
    ColorDetector() : Node("color_detector")
    {
        // 1. Create a Reentrant Callback Group for the Service Client
        // Critical: Prevents deadlock when calling a service from within a service
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // 2. Service Server (Standard Default Group)
        service_ = this->create_service<GetCubeColors>(
            "get_cube_colors",
            std::bind(&ColorDetector::handle_get_cube_colors, this, _1, _2)
        );

        // 3. Service Client (Assigned to the Reentrant Group)
        cube_poses_client_ = this->create_client<GetCubePoses>(
            "get_cube_poses",
            rclcpp::ServicesQoS(),
            client_cb_group_ //critical for calling service within another service call back
        );

        // 4. Subscriptions
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image", qos_profile,
            std::bind(&ColorDetector::image_callback, this, std::placeholders::_1));

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/rgb_camera/camera_info", qos_profile,
            std::bind(&ColorDetector::camera_info_callback, this, std::placeholders::_1));
        
        // 5. TF Buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "ColorDetector ready. Waiting for Camera Info...");
    }

private:
    static constexpr double MIN_AREA_THRESHOLD = 200.0;
    static constexpr const char* CAMERA_FRAME_ID = "external_camera/link/rgb_camera";
    const std::vector<int32_t> TARGET_CUBE_IDS = {1, 10}; 
    static constexpr double MATCH_THRESHOLD = 500.0; // pixels - adjust based on your camera resolution
    static constexpr double MIN_DISTANCE_THRESHOLD = 10.0; // pixels - minimum to consider a match
    
    // --- Callbacks for Data ---
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_image_ = msg;
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);
        camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg); // Deep copy

        // if we have it we can unsubscribe
        if (cam_info_sub_) {
            cam_info_sub_.reset();
            RCLCPP_INFO(this->get_logger(), "Camera Info received. Unsubscribed from Camera Info topic.");
        }   
    }

    // --- Image Processing ---
    std::vector<DetectedObject> detect_colors_in_image(const cv::Mat& frame) {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Defined robust colors with slightly wider ranges for safety
        std::vector<ColorDef> colors = {
            {"RED",    {{ {cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255)}, {cv::Scalar(160, 100, 50), cv::Scalar(180, 255, 255)} }}},
            {"YELLOW", {{ {cv::Scalar(20, 100, 50), cv::Scalar(35, 255, 255)} }}},
            {"GREEN",  {{ {cv::Scalar(40, 50, 50), cv::Scalar(85, 255, 255)} }}},
            {"BLUE",   {{ {cv::Scalar(90, 50, 50), cv::Scalar(130, 255, 255)} }}},
            {"PURPLE", {{ {cv::Scalar(135, 50, 50), cv::Scalar(165, 255, 255)} }}}
        };

        std::vector<DetectedObject> detections;

        for (const auto& c : colors) {
            cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
            for (const auto& range : c.ranges) {
                cv::Mat temp;
                cv::inRange(hsv, range.first, range.second, temp);
                cv::add(mask, temp, mask);
            }

            // Clean up noise (Open/Close)
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            double max_area = 0;
            cv::Point center;
            bool found = false;

            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > max_area) {
                    max_area = area;
                    auto rect = cv::boundingRect(contour);
                    center = cv::Point(rect.x + rect.width/2, rect.y + rect.height/2);
                    found = true;
                }
            }

            if (found && max_area > MIN_AREA_THRESHOLD) {
                detections.push_back({c.name, center.x, center.y, max_area, -1});
            }
        }
        return detections;
    }
    
    bool project_camera_pose_to_pixel(const geometry_msgs::msg::Pose& pose_in_cam_frame,
                                  const image_geometry::PinholeCameraModel& camera_model,
                                  cv::Point2d& output_pixel) 
    {   
        try {
            if (!camera_model.initialized()) {
                RCLCPP_WARN(this->get_logger(), "Camera model not initialized for projection");
                return false;
            }
            
            // Check if point is behind camera (invalid for projection)
            if (pose_in_cam_frame.position.z <= 0) {
                RCLCPP_WARN(this->get_logger(), "Point behind camera (z=%.3f)", pose_in_cam_frame.position.z);
                return false;
            }
            
            cv::Point3d point_3d(
                pose_in_cam_frame.position.x,
                pose_in_cam_frame.position.y,
                pose_in_cam_frame.position.z
            );

            output_pixel = camera_model.project3dToPixel(point_3d);
            
            RCLCPP_DEBUG(this->get_logger(), "Projected 3D point (%.3f, %.3f, %.3f) -> pixel (%.1f, %.1f)",
                        point_3d.x, point_3d.y, point_3d.z,
                        output_pixel.x, output_pixel.y);
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Projection Error: %s", e.what());
            return false;
        }
    }
    

    geometry_msgs::msg::Pose get_pose_in_camera_frame(const geometry_msgs::msg::Pose& world_pose) 
    {
        try {
            RCLCPP_INFO(this->get_logger(), "World pose: (%.3f, %.3f, %.3f)", 
                       world_pose.position.x, world_pose.position.y, world_pose.position.z);
            
            // Check if transform is available
            if (!tf_buffer_->canTransform(CAMERA_FRAME_ID, "world", tf2::TimePointZero, std::chrono::milliseconds(200))) {
                RCLCPP_WARN(this->get_logger(), "Cannot transform world -> %s", CAMERA_FRAME_ID);
                return geometry_msgs::msg::Pose(); // Return empty pose on failure
            }
            
            // Lookup transform from world to camera frame
            geometry_msgs::msg::TransformStamped t_world_to_cam;
            t_world_to_cam = tf_buffer_->lookupTransform(CAMERA_FRAME_ID, "world", tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Transform translation: (%.3f, %.3f, %.3f)", 
                       t_world_to_cam.transform.translation.x, 
                       t_world_to_cam.transform.translation.y, 
                       t_world_to_cam.transform.translation.z);

            // Transform the pose
            geometry_msgs::msg::Pose cam_frame_pose;
            tf2::doTransform(world_pose, cam_frame_pose, t_world_to_cam);
            
            RCLCPP_INFO(this->get_logger(), "Camera frame pose: (%.3f, %.3f, %.3f)", 
                       cam_frame_pose.position.x, cam_frame_pose.position.y, cam_frame_pose.position.z);
            
            return cam_frame_pose;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Transform Error: %s", ex.what());
            return geometry_msgs::msg::Pose(); // Return empty pose on error
        }
    }

    bool get_camera_model(image_geometry::PinholeCameraModel& camera_model) 
    {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);
        if (!camera_info_) {
            RCLCPP_WARN(this->get_logger(), "Camera info not available");
            return false;
        }
        
        // Deep copy to avoid issues
        auto cam_info = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info_);
        if (!camera_model.fromCameraInfo(*cam_info)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera model from camera info");
            return false;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Camera model initialized. Resolution: %dx%d",
                    camera_model.fullResolution().width,
                    camera_model.fullResolution().height);
        
        return true;
    }

    struct MatchResult {
        std::string color;
        double distance;
    };

    MatchResult find_closest_color_match(const cv::Point2d& projected_pixel, 
                                     std::vector<DetectedObject>& color_blobs,
                                     int cube_id)
    {
        double min_distance = std::numeric_limits<double>::max();
        std::string best_color = "UNKNOWN";
        int best_blob_index = -1;
        
        for (size_t i = 0; i < color_blobs.size(); i++) {
            auto& blob = color_blobs[i];
            if (blob.matched_id != -1) continue; // Already matched
            
            double dx = blob.x - projected_pixel.x;
            double dy = blob.y - projected_pixel.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "Cube %d -> %s blob at (%d,%d): distance %.1f px", 
                        cube_id, blob.color.c_str(), blob.x, blob.y, dist);
            
            if (dist < min_distance && dist < MATCH_THRESHOLD) {
                min_distance = dist;
                best_color = blob.color;
                best_blob_index = i;
            }
        }
        
        MatchResult result = {best_color, min_distance};
        
        if (best_color != "UNKNOWN" && min_distance <= MIN_DISTANCE_THRESHOLD) {
            color_blobs[best_blob_index].matched_id = cube_id;
            RCLCPP_INFO(this->get_logger(), 
                    "Matched Cube %d -> %s (distance: %.1f px)", 
                    cube_id, best_color.c_str(), min_distance);
        } else if (best_color != "UNKNOWN") {
            RCLCPP_WARN(this->get_logger(), 
                    "Cube %d: closest blob is %s but distance %.1f > threshold %.1f", 
                    cube_id, best_color.c_str(), min_distance, MIN_DISTANCE_THRESHOLD);
        }
        
        return result;
    }

    // --- Service Callback ---
    void handle_get_cube_colors(
        const std::shared_ptr<GetCubeColors::Request> request,
        std::shared_ptr<GetCubeColors::Response> response)
    {
        (void)request; 
        RCLCPP_INFO(this->get_logger(), "Processing GetCubeColors request...");

        // 1. Check Data
        sensor_msgs::msg::Image::SharedPtr img_msg;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            img_msg = latest_image_;
        }
        if (!img_msg) {
            response->success = false;
            response->message = "No image received yet.";
            return;
        }

        // 2. Call CubeDetector Service
        if (!cube_poses_client_->wait_for_service(std::chrono::seconds(1))) {
            response->success = false;
            response->message = "CubeDetector service not available.";
            return;
        }

        auto poses_req = std::make_shared<GetCubePoses::Request>();
        poses_req->target_ids = TARGET_CUBE_IDS;

        auto future_result = cube_poses_client_->async_send_request(poses_req);
        
        // Wait for result
        if (future_result.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
             response->success = false;
             response->message = "Timed out waiting for CubeDetector.";
             RCLCPP_ERROR(this->get_logger(), "Service call timed out.");
             return;
        }
        
        auto poses_res = future_result.get();
        if (!poses_res->success) {
            response->success = false;
            response->message = "CubeDetector failed: " + poses_res->message;
            return;
        }

        // 3. Process Image
        cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        auto color_blobs = detect_colors_in_image(frame);

        // 4. Match
        std::stringstream ss;
        int matched_count = 0;

        for (size_t i = 0; i < poses_res->detected_ids.size(); ++i) {
            int id = poses_res->detected_ids[i];
            geometry_msgs::msg::Pose world_pose = poses_res->poses[i];
            
            // Convert frame from world to camera frame
            geometry_msgs::msg::Pose cam_frame_pose = get_pose_in_camera_frame(world_pose);

            cv::Point2d projected_pixel;
            
            // Get camera model
            image_geometry::PinholeCameraModel camera_model;
            if (!get_camera_model(camera_model)) {
                RCLCPP_WARN(this->get_logger(), "No camera info available for cube %d", id);
                continue;
            }
            
            if (!project_camera_pose_to_pixel(cam_frame_pose, camera_model, projected_pixel)) {
                RCLCPP_WARN(this->get_logger(), "Failed to project cube %d to pixel", id);
                continue;
            }
            
            RCLCPP_INFO(this->get_logger(), "Cube %d projected to pixel (%.1f, %.1f)", id, projected_pixel.x, projected_pixel.y);
            
            // Find closest color blob to this projected pixel
            auto match_result = find_closest_color_match(projected_pixel, color_blobs, id);
            
            if (match_result.color != "UNKNOWN") {
                if (matched_count > 0) ss << " | ";
                ss << "Cube " << id << ": " << match_result.color;
                matched_count++;
                
                RCLCPP_INFO(this->get_logger(), "Matched Cube %d -> %s (distance: %.1f px)", 
                           id, match_result.color.c_str(), match_result.distance);
            } else {
                RCLCPP_WARN(this->get_logger(), "No color match found for Cube %d (min_dist: %.1f)", 
                           id, match_result.distance);
            }
        }

        if (matched_count == 0) {
            response->success = false;
            response->message = "Could not match any cubes to colors.";
            RCLCPP_WARN(this->get_logger(), "Matching failed. See logs for distances.");
        } else {
            response->success = true;
            response->message = ss.str();
            RCLCPP_INFO(this->get_logger(), "Result: %s", response->message.c_str());
        }
    }

    rclcpp::Service<GetCubeColors>::SharedPtr service_;
    rclcpp::Client<GetCubePoses>::SharedPtr cube_poses_client_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_; 
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::Image::SharedPtr latest_image_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    std::mutex image_mutex_;
    std::mutex cam_info_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetector>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}