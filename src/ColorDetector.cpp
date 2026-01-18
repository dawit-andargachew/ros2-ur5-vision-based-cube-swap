/**
 * Node: color_detector
 * 
 * Purpose:
 * Service node that detects and identifies the colors of cubes placed on the table.
 * Provides a service "get_cube_colors" that returns detected cube colors when called.
 * 
 * How It Works (Step-by-Step):
 * 
 * Step 1: Color Detection (Scans Entire Cube Surface)
 *   - Subscribes to camera feed (/rgb_camera/image)
 *   - Converts image to HSV color space for better color detection
 *   - Creates color masks for RED, BLUE, YELLOW, GREEN, PURPLE using HSV ranges
 *   - Finds contours (blobs) of each color in the entire image
 *   - Selects the largest blob per color (filters noise)
 *   - Records center position (x, y) of each color blob
 *   - Note: This detects the CUBE's colored surface, not the AprilTag
 * 
 * Step 2: AprilTag Detection (Separate System)
 *   - Subscribes to AprilTag detections (/apriltag/detections)
 *   - Receives tag IDs (1, 10, etc.) and their pixel positions from AprilTag node
 *   - AprilTag detection runs independently and detects the black/white tag markers
 * 
 * Step 3: Spatial Matching (Links Color to Tag ID)
 *   - For each detected color blob, finds the closest AprilTag detection
 *   - Calculates Euclidean distance: sqrt((x_blob - x_tag)² + (y_blob - y_tag)²)
 *   - Matches if distance < 150 pixels (configurable threshold)
 *   - Assigns the matched AprilTag ID to the color blob
 * 
 * Step 4: TF Validation (Ensures IDs Exist)
 *   - Validates matched AprilTag IDs exist in TF tree (same as cube_detector)
 *   - Checks if transform "world" → "tag36h11:<id>" exists
 *   - Invalidates matches that don't exist in TF
 * 
 * Step 5: Response Formatting
 *   - Formats message like "Cube 1: RED | Cube 10: BLUE"
 *   - Uses actual detected AprilTag IDs (not hardcoded)
 *   - Returns only successfully matched cubes
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <mutex>
#include <set>
#include <cmath>
#include <limits>

#include "group_6_assignment_2/srv/get_cube_colors.hpp"
#include "group_6_assignment_2/srv/get_cube_poses.hpp"

using GetCubeColors = group_6_assignment_2::srv::GetCubeColors;
using std::placeholders::_1;
using std::placeholders::_2;

// Structure to define a color with its Name and HSV Ranges
struct ColorDef {
    std::string name;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> ranges;
};

struct DetectedObject {
    std::string color;
    int x;
    int y;  // Added for spatial matching
    double area;
    int apriltag_id;  // Will store the matched AprilTag ID, -1 if not matched
};

class ColorDetector : public rclcpp::Node
{
public:
    ColorDetector() : Node("color_detector")
    {
        // Create the service
        service_ = this->create_service<GetCubeColors>(
            "get_cube_colors",
            std::bind(&ColorDetector::handle_get_cube_colors, this, _1, _2)
        );

        // Subscribe to camera feed in background to maintain latest image
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image", qos_profile,
            std::bind(&ColorDetector::image_callback, this, std::placeholders::_1));
        
        // Subscribe to AprilTag detections to get tag positions in image
        rclcpp::QoS apriltag_qos(1);
        apriltag_subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/apriltag/detections", apriltag_qos,
            std::bind(&ColorDetector::apriltag_callback, this, std::placeholders::_1));
        
        // Initialize TF2 buffer and listener for validating AprilTag IDs
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(this->get_logger(), "Cube Color Detector Service is ready.");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /apriltag/detections for spatial matching");
    }

private:

    // Define the threshold constant here
    static constexpr double MIN_AREA_THRESHOLD = 200.0;
    static constexpr int FIND_TF_TIMEOUT_MS = 200;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Store the latest image (thread-safe)
        std::lock_guard<std::mutex> lock(latest_image_mutex_);
        latest_image_ = msg;
    }

    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        // Store the latest AprilTag detections (thread-safe)
        std::lock_guard<std::mutex> lock(latest_apriltag_mutex_);
        latest_apriltag_detections_ = msg;
    }

    std::vector<DetectedObject> detect_colors_in_image(const cv::Mat& frame)
    {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        std::vector<ColorDef> colors = {
            {"RED",    {{ {cv::Scalar(0, 120, 70), cv::Scalar(8, 255, 255)}, {cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255)} }}},
            
            {"YELLOW", {{ {cv::Scalar(22, 100, 100), cv::Scalar(33, 255, 255)} }}},
            {"GREEN",  {{ {cv::Scalar(38, 60, 60), cv::Scalar(85, 255, 255)} }}},
            {"BLUE",   {{ {cv::Scalar(90, 60, 60), cv::Scalar(125, 255, 255)} }}},
            {"PURPLE", {{ {cv::Scalar(135, 50, 50), cv::Scalar(165, 255, 255)} }}}
        };

        std::vector<DetectedObject> detections;

        // Iterate through all defined colors
        for (const auto& c : colors) {
            cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
            
            // Combine masks if a color has multiple ranges (like Red)
            for (const auto& range : c.ranges) {
                cv::Mat temp;
                cv::inRange(hsv, range.first, range.second, temp);
                cv::add(mask, temp, mask);
            }

            // Find Contours (Blobs) in the mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // Find the single largest blob for this color to avoid noise
            double max_area = 0;
            cv::Rect max_rect;
            int max_x = -1;
            int max_y = -1;
            
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > max_area) {
                    max_area = area;
                    max_rect = cv::boundingRect(contour);
                    max_x = max_rect.x + max_rect.width / 2;
                    max_y = max_rect.y + max_rect.height / 2;
                }
            }

            // Filter: Must be larger than `MIN_AREA_THRESHOLD` pixels to be considered a real object
            if (max_area > MIN_AREA_THRESHOLD) {
                detections.push_back({c.name, max_x, max_y, max_area, -1});  // -1 means not matched yet
            }
        }

        // Sort detections from Left to Right (by X-coordinate)
        // This gives us a consistent order (Cube 1 is usually left, Cube 10 right).
        std::sort(detections.begin(), detections.end(), 
            [](const DetectedObject& a, const DetectedObject& b) {
                return a.x < b.x; 
            });

        return detections;
    }

    void handle_get_cube_colors(
        const std::shared_ptr<GetCubeColors::Request> request,
        std::shared_ptr<GetCubeColors::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request for cube colors.");

        response->success = false;
        response->message = "";

        // Get the latest image (thread-safe)
        sensor_msgs::msg::Image::SharedPtr latest_image;
        {
            std::lock_guard<std::mutex> lock(latest_image_mutex_);
            if (!latest_image_) {
                response->success = false;
                response->message = "No image received yet. Please wait for camera feed.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            latest_image = latest_image_;
        }

        try {
            cv::Mat frame = cv_bridge::toCvCopy(latest_image, "bgr8")->image;
            
            // Detect colors in the image
            std::vector<DetectedObject> detections = detect_colors_in_image(frame);

            if (detections.empty()) {
                response->success = false;
                response->message = "No cubes detected in the image.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            // Get latest AprilTag detections (thread-safe)
            apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr apriltag_detections;
            {
                std::lock_guard<std::mutex> lock(latest_apriltag_mutex_);
                apriltag_detections = latest_apriltag_detections_;
            }

            if (!apriltag_detections || apriltag_detections->detections.empty()) {
                response->success = false;
                response->message = "No AprilTag detections available. Make sure AprilTag detection is running and tags are visible.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Found %zu AprilTag detections in image", 
                       apriltag_detections->detections.size());
            for (const auto& tag : apriltag_detections->detections) {
                RCLCPP_INFO(this->get_logger(), "  - Tag ID: %d at (%.1f, %.1f)", 
                           tag.id, tag.centre.x, tag.centre.y);
            }
            
            // Match color blobs with AprilTag detections based on spatial proximity
            match_color_blobs_with_apriltags(detections, apriltag_detections);
            
            // Validate matched IDs using TF transforms
            validate_apriltag_ids_with_tf(detections);

            // Format message using actual AprilTag IDs
            std::stringstream message_stream;
            int count = 0;
            for (const auto& obj : detections) {
                // Stop after finding the two main cubes
                if (count >= 2) break;

                if (count > 0) message_stream << " | ";
                
                // Use AprilTag ID if matched, otherwise skip
                if (obj.apriltag_id >= 0) {
                    message_stream << "Cube " << obj.apriltag_id << ": " << obj.color;
                    count++;
                } else {
                    RCLCPP_WARN(this->get_logger(), "No AprilTag ID assigned to %s blob, skipping.", 
                                obj.color.c_str());
                }
            }
            
            if (count == 0) {
                response->success = false;
                response->message = "No cubes could be matched with available AprilTag IDs.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            response->success = true;
            response->message = message_stream.str();
            
            RCLCPP_INFO(this->get_logger(), "Detected: [%s]", response->message.c_str());
        }
        catch (cv_bridge::Exception& e) {
            response->success = false;
            response->message = "cv_bridge exception: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void match_color_blobs_with_apriltags(
        std::vector<DetectedObject>& color_detections,
        const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr& apriltag_detections)
    {
        // Track which tags have been matched to ensure one-to-one matching
        std::set<int> matched_tag_ids;
        
        // Match each color blob with the closest AprilTag detection based on pixel distance
        // Uses spatial proximity - NO assumption about left/right position
        for (auto& color_obj : color_detections) {
            double min_distance = std::numeric_limits<double>::max();
            int matched_tag_id = -1;

            for (const auto& tag : apriltag_detections->detections) {
                // Skip tags that have already been matched (one-to-one matching)
                if (matched_tag_ids.find(tag.id) != matched_tag_ids.end()) {
                    continue;
                }

                // Calculate Euclidean distance between color blob center and AprilTag center
                double dx = color_obj.x - tag.centre.x;
                double dy = color_obj.y - tag.centre.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                // Match with the closest tag within a reasonable threshold (e.g., 150 pixels)
                // can be adjusted based on the camera resolution and setup, but for our case we used 150.
                constexpr double MAX_MATCH_DISTANCE = 150.0;
                if (distance < min_distance && distance < MAX_MATCH_DISTANCE) {
                    min_distance = distance;
                    matched_tag_id = tag.id;
                }
            }

            if (matched_tag_id >= 0) {
                color_obj.apriltag_id = matched_tag_id;
                matched_tag_ids.insert(matched_tag_id);  // Mark this tag as used
                RCLCPP_INFO(this->get_logger(), 
                           "Matched %s color blob at (%d, %d) with AprilTag ID %d (distance: %.2f pixels)",
                           color_obj.color.c_str(), color_obj.x, color_obj.y, matched_tag_id, min_distance);
            } else {
                RCLCPP_WARN(this->get_logger(), 
                           "No AprilTag match found for %s color blob at (%d, %d) within threshold",
                           color_obj.color.c_str(), color_obj.x, color_obj.y);
            }
        }
    }

    void validate_apriltag_ids_with_tf(std::vector<DetectedObject>& color_detections)
    {
        // Validate that matched AprilTag IDs actually exist in the TF tree (like CubeDetector does)
        for (auto& color_obj : color_detections) {
            if (color_obj.apriltag_id >= 0) {
                std::string tag_frame = "tag36h11:" + std::to_string(color_obj.apriltag_id);
                
                // Check if transform exists (same logic as CubeDetector)
                if (!tf_buffer_->canTransform("world", tag_frame, tf2::TimePointZero, 
                                             std::chrono::milliseconds(FIND_TF_TIMEOUT_MS))) {
                    RCLCPP_WARN(this->get_logger(), 
                               "AprilTag ID %d matched but not found in TF tree. Invalidating match.",
                               color_obj.apriltag_id);
                    color_obj.apriltag_id = -1;  // Invalidate the match
                } else {
                    RCLCPP_DEBUG(this->get_logger(), 
                                "Validated AprilTag ID %d exists in TF tree for %s cube",
                                color_obj.apriltag_id, color_obj.color.c_str());
                }
            }
        }
    }

    rclcpp::Service<GetCubeColors>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr latest_apriltag_detections_;
    std::mutex latest_image_mutex_;
    std::mutex latest_apriltag_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
