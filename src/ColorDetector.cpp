#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ColorDetector : public rclcpp::Node
{
public:
    ColorDetector() : Node("color_detector")
    {
        // Subscribe to the RGB camera topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image", 10,
            std::bind(&ColorDetector::image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Color Detector (Robust Vision) started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image message to OpenCV image
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // // Save image for debugging (once every ~30 frames)
            // static int frame_count = 0;
            // if (frame_count++ % 30 == 0) {
            //     cv::imwrite("camera_view.png", frame);
            //     RCLCPP_INFO(this->get_logger(), "Saved camera view to 'camera_view.png' in workspace root.");
            // }

            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            // Red Masks
            cv::Mat mask_red1, mask_red2, mask_red;
            cv::inRange(hsv_frame, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask_red1);
            cv::inRange(hsv_frame, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask_red2);
            cv::addWeighted(mask_red1, 1.0, mask_red2, 1.0, 0.0, mask_red);

            // Blue Mask
            cv::Mat mask_blue;
            // Widen range: Hue 80-150, Saturation 20-255, Value 20-255
            cv::inRange(hsv_frame, cv::Scalar(80, 20, 20), cv::Scalar(150, 255, 255), mask_blue);

            int red_count = cv::countNonZero(mask_red);
            int blue_count = cv::countNonZero(mask_blue);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "DEBUG: Red Pixels: %d | Blue Pixels: %d", red_count, blue_count);

            // Process
            process_contours(mask_red, "RED");
            process_contours(mask_blue, "BLUE");
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void process_contours(const cv::Mat& mask, const std::string& color_name)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > 0) // Reduced to 10 to debug visibility
            {
                cv::Moments M = cv::moments(contour);
                if (M.m00 > 0)
                {
                    // Infer ID based on Color (Specific to this assignment)
                    int id = (color_name == "RED") ? 1 : 10;

                    // Print Color and ID (throttled to avoid spam)
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Cube ID: %2d | Color: %s", 
                        id, color_name.c_str());
               
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetector>());
    rclcpp::shutdown();
    return 0;
}
