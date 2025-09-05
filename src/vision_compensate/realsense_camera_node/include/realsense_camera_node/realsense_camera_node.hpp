#ifndef REALSENSE_CAMERA_NODE_REALSENSE_CAMERA_NODE_HPP
#define REALSENSE_CAMERA_NODE_REALSENSE_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include "realsense_camera_node/camera_config.hpp"

namespace realsense_camera_node
{

class RealSenseMultiCameraNode : public rclcpp::Node
{
public:
    RealSenseMultiCameraNode();
    ~RealSenseMultiCameraNode();

private:
    // Initialization
    bool initializeCamera();
    void loadParameters();
    void setupPublishers();
    void setupSubscribers();

    // Camera operations
    cv::Mat get_realsense_image();
    void captureAndPublish();
    void displayImage(const cv::Mat& image);

    // Subscriber callbacks
    void detectionCommandCallback(const std_msgs::msg::String::SharedPtr msg);

    // Parameter handling
    void parameterCallback(const std::vector<rclcpp::Parameter>& parameters);
    bool validateParameters(const CameraConfig& config);
    ModelConfig getModelConfig(CameraModel model);

    // Cleanup
    void cleanup();
    
    // Helper methods
    void logCompensationError(int error_code, const std::string& golden_path);

    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_cmd_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // RealSense components
    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::pipeline_profile profile_;
    bool camera_initialized_;

    // Configuration
    CameraConfig config_;
    
    // OpenCV
    bool show_display_;
    std::string window_name_;
};

} // namespace realsense_camera_node

#endif // REALSENSE_CAMERA_NODE_REALSENSE_CAMERA_NODE_HPP