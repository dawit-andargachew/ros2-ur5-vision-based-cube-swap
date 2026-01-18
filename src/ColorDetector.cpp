/**
 * Node: color_detector
 * * fixes included:
 * 1. OpenCV Assertion Crash (Deep Copy of CameraInfo)
 * 2. Service Deadlock (Reentrant Callback Group)
 * 3. Matching Logic (Increased Threshold + Debug Logs)
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
            rmw_qos_profile_services_default,
            client_cb_group_
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
    const std::vector<int32_t> TARGET_CUBE_IDS = {1, 10}; 

    // --- Callbacks for Data ---
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_image_ = msg;
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);
        camera_info_ = msg;
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

    // --- Math: Project World Point (3D) -> Image Pixel (2D) ---
    bool project_world_to_pixel(const geometry_msgs::msg::Pose& world_pose, 
                                cv::Point2d& output_pixel,
                                const std::string& camera_frame_id) 
    {
        try {
            // 1. Thread Safety & Data Copy
            sensor_msgs::msg::CameraInfo::SharedPtr info_ptr;
            {
                std::lock_guard<std::mutex> lock(cam_info_mutex_);
                info_ptr = camera_info_;
            }

            if (!info_ptr) {
                RCLCPP_WARN(this->get_logger(), "No Camera Info available for projection.");
                return false;
            }

            // 2. TF Transform: World -> Camera Frame
            if (!tf_buffer_->canTransform(camera_frame_id, "world", tf2::TimePointZero, std::chrono::milliseconds(200))) {
                RCLCPP_WARN(this->get_logger(), "Cannot transform World -> %s", camera_frame_id.c_str());
                return false;
            }
            
            geometry_msgs::msg::TransformStamped t_world_to_cam;
            t_world_to_cam = tf_buffer_->lookupTransform(camera_frame_id, "world", tf2::TimePointZero);

            geometry_msgs::msg::Pose cam_pose;
            tf2::doTransform(world_pose, cam_pose, t_world_to_cam);

            // 3. Project using OpenCV (With Safe Data Copy)
            std::vector<cv::Point3f> object_points;
            object_points.push_back(cv::Point3f(cam_pose.position.x, cam_pose.position.y, cam_pose.position.z));
            
            // --- DEEP COPY TO AVOID CRASH ---
            cv::Mat K(3, 3, CV_64F);
            for(int i=0; i<9; i++) K.at<double>(i/3, i%3) = info_ptr->k[i];

            cv::Mat D;
            if (info_ptr->d.empty()) {
                D = cv::Mat::zeros(1, 5, CV_64F);
            } else {
                D = cv::Mat(info_ptr->d.size(), 1, CV_64F);
                for(size_t i=0; i<info_ptr->d.size(); i++) D.at<double>(i, 0) = info_ptr->d[i];
            }

            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

            std::vector<cv::Point2d> image_points;
            cv::projectPoints(object_points, rvec, tvec, K, D, image_points);

            if (!image_points.empty()) {
                output_pixel = image_points[0];
                return true;
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Projection TF Error: %s", ex.what());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what());
        }
        return false;
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
            
            cv::Point2d projected_pixel;
            if (project_world_to_pixel(world_pose, projected_pixel, img_msg->header.frame_id)) {
                
                // --- DEBUGGING INFO START ---
                RCLCPP_INFO(this->get_logger(), "Cube %d Projected Pixel: (%.1f, %.1f)", id, projected_pixel.x, projected_pixel.y);
                
                double min_dist = 250.0; // Increased threshold
                std::string matched_color = "Unknown";
                int best_blob_idx = -1;

                for (size_t b = 0; b < color_blobs.size(); ++b) {
                    if (color_blobs[b].matched_id != -1) continue; 

                    double dx = color_blobs[b].x - projected_pixel.x;
                    double dy = color_blobs[b].y - projected_pixel.y;
                    double dist = std::sqrt(dx*dx + dy*dy);

                    RCLCPP_INFO(this->get_logger(), "  -> Checking %s blob at (%.1f, %.1f) | Dist: %.1f", 
                        color_blobs[b].color.c_str(), (double)color_blobs[b].x, (double)color_blobs[b].y, dist);

                    if (dist < min_dist) {
                        min_dist = dist;
                        best_blob_idx = b;
                        matched_color = color_blobs[b].color;
                    }
                }
                // --- DEBUGGING INFO END ---
                if (best_blob_idx != -1) {
                    color_blobs[best_blob_idx].matched_id = id;
                    if (matched_count > 0) ss << " | ";
                    ss << "Cube " << id << ": " << matched_color;
                    matched_count++;
                }
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