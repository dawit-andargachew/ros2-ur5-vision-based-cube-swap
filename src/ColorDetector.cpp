/**
 * Node: color_detector
 * 
 * Purpose:
 * Service node that detects and identifies the colors of cubes placed on the table.
 * Provides a service "get_cube_colors" that returns detected cube colors when called.
 * 
 * Strategy:
 * This node subscribes to the camera feed in the background to maintain the latest image.
 * When the service is called, it uses OpenCV to scan for predefined colors (Red, Blue, Green, etc.) 
 * in HSV space. It filters out noise, identifies the largest color blobs, and sorts them spatially 
 * from left to right. Finally, it assigns logical IDs (Cube 1, Cube 10) based on horizontal order.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <mutex>

#include "group_6_assignment_2/srv/get_cube_colors.hpp"

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
    double area;
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
        
        RCLCPP_INFO(this->get_logger(), "Cube Color Detector Service is ready.");
    }

private:

    // Define the threshold constant here
    static constexpr double MIN_AREA_THRESHOLD = 200.0;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Store the latest image (thread-safe)
        std::lock_guard<std::mutex> lock(latest_image_mutex_);
        latest_image_ = msg;
    }

    std::vector<DetectedObject> detect_colors_in_image(const cv::Mat& frame)
    {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Define the list of Robust Colors to search for
        // RED has two ranges because Hue wraps around 0-180 (0-10 and 160-180).
        std::vector<ColorDef> colors = {
            {"RED",    {{ {cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255)}, {cv::Scalar(160, 100, 50), cv::Scalar(180, 255, 255)} }}},
            {"BLUE",   {{ {cv::Scalar(80, 50, 50), cv::Scalar(150, 255, 255)} }}},
            {"YELLOW", {{ {cv::Scalar(20, 100, 50), cv::Scalar(35, 255, 255)} }}},
            {"PURPLE", {{ {cv::Scalar(130, 50, 50), cv::Scalar(160, 255, 255)} }}},
            {"GREEN",  {{ {cv::Scalar(40, 50, 50), cv::Scalar(85, 255, 255)} }}}
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
            
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > max_area) {
                    max_area = area;
                    max_rect = cv::boundingRect(contour);
                    max_x = max_rect.x + max_rect.width / 2;
                }
            }

            // Filter: Must be larger than `MIN_AREA_THRESHOLD` pixels to be considered a real object
            if (max_area > MIN_AREA_THRESHOLD) {
                detections.push_back({c.name, max_x, max_area});
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

            // Format message like "Cube 1: RED | Cube 10: BLUE"
            std::stringstream message_stream;
            int count = 0;
            for (const auto& obj : detections) {
                // Assign Logical ID based on order: 1st object -> ID 1, 2nd object -> ID 10
                int id_label = (count == 0) ? 1 : 10;
                
                // Stop after finding the two main cubes
                if (count >= 2) break;

                if (count > 0) message_stream << " | ";
                message_stream << "Cube " << id_label << ": " << obj.color;
                count++;
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

    rclcpp::Service<GetCubeColors>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::mutex latest_image_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
