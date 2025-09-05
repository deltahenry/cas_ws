#ifndef REALSENSE_MULTI_CAMERA__REALSENSE_MULTI_CAMERA_NODE_HPP_
#define REALSENSE_MULTI_CAMERA__REALSENSE_MULTI_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "realsense_multi_camera/camera_config.hpp"

namespace realsense_multi_camera
{

class RealSenseMultiCameraNode : public rclcpp::Node
{
public:
    RealSenseMultiCameraNode();
    ~RealSenseMultiCameraNode();

private:
    // Initialization methods
    bool initializeCamera();
    void loadParameters();
    void setupPublishers();
    
    // Camera operations
    void captureAndPublish();
    void displayImage(const cv::Mat& image);
    void publishImage(const cv::Mat& image, const std::chrono::high_resolution_clock::time_point& timestamp);
    void publishCameraInfo(const std::chrono::high_resolution_clock::time_point& timestamp);
    
    // Parameter handling
    void parameterCallback(const std::vector<rclcpp::Parameter>& parameters);
    bool validateParameters(const CameraConfig& config);
    void updateCameraSettings();
    
    // Cleanup
    void cleanup();
    
    // Error handling
    void handleCameraError(const std::exception& e);
    bool reconnectCamera();
    
    // Member variables
    rs2::pipeline pipeline_;
    rs2::config rs_config_;
    rs2::pipeline_profile profile_;
    bool camera_initialized_;
    bool show_display_;
    
    CameraConfig config_;
    
    // ROS2 components
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Frame tracking
    uint32_t frame_counter_;
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    double current_fps_;
    
    // Error handling
    int retry_count_;
    static const int MAX_RETRY_COUNT = 5;
    std::chrono::high_resolution_clock::time_point last_retry_time_;
};

}  // namespace realsense_multi_camera

#endif  // REALSENSE_MULTI_CAMERA__REALSENSE_MULTI_CAMERA_NODE_HPP_