#include "realsense_multi_camera/realsense_multi_camera_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <iostream>
#include <stdexcept>
#include <thread>

namespace realsense_multi_camera
{

// CameraDetector implementation
CameraModel CameraDetector::detectConnectedCamera()
{
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        if (devices.size() == 0) {
            RCLCPP_WARN(rclcpp::get_logger("camera_detector"), "No RealSense devices found");
            return CameraModel::UNKNOWN;
        }
        
        auto device = devices[0];
        std::string product_line = device.get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
        
        if (product_line.find("D400") != std::string::npos) {
            std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
            if (name.find("D405") != std::string::npos) {
                return CameraModel::D405;
            } else if (name.find("D435") != std::string::npos) {
                return CameraModel::D435;
            }
        }
        
        RCLCPP_WARN(rclcpp::get_logger("camera_detector"), "Unsupported camera model: %s", product_line.c_str());
        return CameraModel::UNKNOWN;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_detector"), "Error detecting camera: %s", e.what());
        return CameraModel::UNKNOWN;
    }
}

std::vector<CameraModel> CameraDetector::getAllConnectedCameras()
{
    std::vector<CameraModel> cameras;
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        for (auto& device : devices) {
            std::string product_line = device.get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
            if (isModelSupported(product_line)) {
                std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
                if (name.find("D405") != std::string::npos) {
                    cameras.push_back(CameraModel::D405);
                } else if (name.find("D435") != std::string::npos) {
                    cameras.push_back(CameraModel::D435);
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_detector"), "Error getting connected cameras: %s", e.what());
    }
    
    return cameras;
}

std::string CameraDetector::modelToString(CameraModel model)
{
    switch (model) {
        case CameraModel::D405: return "D405";
        case CameraModel::D435: return "D435";
        case CameraModel::AUTO: return "auto";
        default: return "unknown";
    }
}

CameraModel CameraDetector::stringToModel(const std::string& model_str)
{
    if (model_str == "D405") return CameraModel::D405;
    if (model_str == "D435") return CameraModel::D435;
    if (model_str == "auto") return CameraModel::AUTO;
    return CameraModel::UNKNOWN;
}

bool CameraDetector::isModelSupported(const std::string& product_line)
{
    return product_line.find("D400") != std::string::npos;
}

// CameraConfig validation
bool CameraConfig::validate() const
{
    if (camera_settings.width <= 0 || camera_settings.height <= 0) {
        return false;
    }
    if (camera_settings.fps <= 0 || camera_settings.fps > 120) {
        return false;
    }
    if (model_config.min_distance < 0 || model_config.max_distance <= model_config.min_distance) {
        return false;
    }
    if (ros2.queue_size <= 0) {
        return false;
    }
    return true;
}

// RealSenseMultiCameraNode implementation
RealSenseMultiCameraNode::RealSenseMultiCameraNode()
    : Node("realsense_multi_camera_node")
    , camera_initialized_(false)
    , show_display_(true)
    , frame_counter_(0)
    , current_fps_(0.0)
    , retry_count_(0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RealSense Multi Camera Node");
    
    loadParameters();
    setupPublishers();
    
    if (initializeCamera()) {
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / config_.camera_settings.fps));
        timer_ = this->create_wall_timer(period, std::bind(&RealSenseMultiCameraNode::captureAndPublish, this));
        
        RCLCPP_INFO(this->get_logger(), "Camera initialized successfully. Model: %s", 
                    CameraDetector::modelToString(config_.model).c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
    }
}

RealSenseMultiCameraNode::~RealSenseMultiCameraNode()
{
    cleanup();
}

void RealSenseMultiCameraNode::loadParameters()
{
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    
    // Camera model parameter
    param_desc.description = "Camera model (D405, D435, auto)";
    this->declare_parameter("model", "D405", param_desc);
    
    // Camera settings
    param_desc.description = "Image width";
    this->declare_parameter("width", 640, param_desc);
    param_desc.description = "Image height";
    this->declare_parameter("height", 480, param_desc);
    param_desc.description = "Frame rate";
    this->declare_parameter("fps", 30, param_desc);
    
    // Display settings
    param_desc.description = "Show OpenCV display window";
    this->declare_parameter("show_display", true, param_desc);
    
    // ROS2 settings
    param_desc.description = "Camera topic name";
    this->declare_parameter("topic_name", "camera/rgb/image_raw", param_desc);
    param_desc.description = "Camera frame ID";
    this->declare_parameter("frame_id", "camera_link", param_desc);
    param_desc.description = "Publisher queue size";
    this->declare_parameter("queue_size", 1, param_desc);
    
    // Load parameters into config
    std::string model_str = this->get_parameter("model").as_string();
    config_.model = CameraDetector::stringToModel(model_str);
    
    if (config_.model == CameraModel::AUTO) {
        config_.model = CameraDetector::detectConnectedCamera();
    }
    
    config_.camera_settings.width = this->get_parameter("width").as_int();
    config_.camera_settings.height = this->get_parameter("height").as_int();
    config_.camera_settings.fps = this->get_parameter("fps").as_int();
    
    show_display_ = this->get_parameter("show_display").as_bool();
    
    config_.ros2.topic_name = this->get_parameter("topic_name").as_string();
    config_.ros2.frame_id = this->get_parameter("frame_id").as_string();
    config_.ros2.queue_size = this->get_parameter("queue_size").as_int();
    
    // Set model-specific configurations
    switch (config_.model) {
        case CameraModel::D405:
            config_.model_config = {0.13, 4.0, 87, 58};
            break;
        case CameraModel::D435:
            config_.model_config = {0.11, 10.0, 87, 58};
            break;
        default:
            config_.model_config = {0.1, 5.0, 87, 58};
    }
    
    config_.display = {"RealSense Camera", true, true};
    
    // Validate configuration
    if (!validateParameters(config_)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera configuration");
        throw std::runtime_error("Invalid camera configuration");
    }
}

void RealSenseMultiCameraNode::setupPublishers()
{
    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_publisher_ = image_transport_->advertise(config_.ros2.topic_name, config_.ros2.queue_size);
    
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info", config_.ros2.queue_size);
    
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this);
    camera_info_manager_->setCameraName("realsense_camera");
}

bool RealSenseMultiCameraNode::initializeCamera()
{
    try {
        rs_config_.enable_stream(RS2_STREAM_COLOR, 
                                config_.camera_settings.width,
                                config_.camera_settings.height,
                                RS2_FORMAT_BGR8,
                                config_.camera_settings.fps);
        
        profile_ = pipeline_.start(rs_config_);
        camera_initialized_ = true;
        
        // Get camera intrinsics for camera info
        auto stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        
        // Set camera info
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header.frame_id = config_.ros2.frame_id;
        camera_info.width = intrinsics.width;
        camera_info.height = intrinsics.height;
        camera_info.distortion_model = "plumb_bob";
        
        // Set camera matrix
        camera_info.k[0] = intrinsics.fx;
        camera_info.k[2] = intrinsics.ppx;
        camera_info.k[4] = intrinsics.fy;
        camera_info.k[5] = intrinsics.ppy;
        camera_info.k[8] = 1.0;
        
        // Set distortion coefficients
        camera_info.d.resize(5);
        for (int i = 0; i < 5; i++) {
            camera_info.d[i] = intrinsics.coeffs[i];
        }
        
        camera_info_manager_->setCameraInfo(camera_info);
        
        RCLCPP_INFO(this->get_logger(), "Camera initialized: %dx%d @ %d fps",
                    config_.camera_settings.width, config_.camera_settings.height,
                    config_.camera_settings.fps);
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera: %s", e.what());
        return false;
    }
}

void RealSenseMultiCameraNode::captureAndPublish()
{
    if (!camera_initialized_) {
        if (reconnectCamera()) {
            return;
        } else {
            return;
        }
    }
    
    try {
        rs2::frameset frames = pipeline_.wait_for_frames(5000);
        rs2::frame color_frame = frames.get_color_frame();
        
        if (!color_frame) {
            RCLCPP_WARN(this->get_logger(), "No color frame received");
            return;
        }
        
        auto timestamp = std::chrono::high_resolution_clock::now();
        
        // Convert to OpenCV Mat
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        // Display image if enabled
        if (show_display_) {
            displayImage(image);
        }
        
        // Publish image
        publishImage(image, timestamp);
        publishCameraInfo(timestamp);
        
        // Update FPS
        frame_counter_++;
        if (frame_counter_ % 30 == 0) {
            auto current_time = timestamp;
            if (frame_counter_ > 30) {
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_frame_time_);
                current_fps_ = 30000.0 / duration.count();
            }
            last_frame_time_ = current_time;
        }
        
        // Reset retry count on successful capture
        retry_count_ = 0;
        
    } catch (const std::exception& e) {
        handleCameraError(e);
    }
}

void RealSenseMultiCameraNode::displayImage(const cv::Mat& image)
{
    cv::Mat display_image = image.clone();
    
    if (config_.display.show_info_overlay) {
        // Add info overlay
        std::string info = "Model: " + CameraDetector::modelToString(config_.model) + 
                          " | FPS: " + std::to_string(static_cast<int>(current_fps_)) +
                          " | Frame: " + std::to_string(frame_counter_);
        
        cv::putText(display_image, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
    
    cv::imshow(config_.display.window_name, display_image);
    cv::waitKey(1);
}

void RealSenseMultiCameraNode::publishImage(const cv::Mat& image, const std::chrono::high_resolution_clock::time_point& timestamp)
{
    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    image_msg->header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count());
    image_msg->header.frame_id = config_.ros2.frame_id;
    
    image_publisher_.publish(image_msg);
}

void RealSenseMultiCameraNode::publishCameraInfo(const std::chrono::high_resolution_clock::time_point& timestamp)
{
    auto camera_info = camera_info_manager_->getCameraInfo();
    camera_info.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count());
    camera_info.header.frame_id = config_.ros2.frame_id;
    
    camera_info_publisher_->publish(camera_info);
}

void RealSenseMultiCameraNode::parameterCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    for (const auto& param : parameters) {
        if (param.get_name() == "show_display") {
            show_display_ = param.as_bool();
        }
    }
}

bool RealSenseMultiCameraNode::validateParameters(const CameraConfig& config)
{
    return config.validate();
}

void RealSenseMultiCameraNode::handleCameraError(const std::exception& e)
{
    RCLCPP_ERROR(this->get_logger(), "Camera error: %s", e.what());
    
    if (retry_count_ < MAX_RETRY_COUNT) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_last_retry = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_retry_time_);
        
        if (time_since_last_retry.count() >= 1) {
            retry_count_++;
            last_retry_time_ = current_time;
            
            RCLCPP_WARN(this->get_logger(), "Attempting to reconnect camera (attempt %d/%d)", 
                        retry_count_, MAX_RETRY_COUNT);
            
            cleanup();
            camera_initialized_ = false;
        }
    } else {
        RCLCPP_FATAL(this->get_logger(), "Max retry count reached. Shutting down node.");
        rclcpp::shutdown();
    }
}

bool RealSenseMultiCameraNode::reconnectCamera()
{
    try {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return initializeCamera();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Reconnect failed: %s", e.what());
        return false;
    }
}

void RealSenseMultiCameraNode::cleanup()
{
    try {
        if (camera_initialized_) {
            pipeline_.stop();
            camera_initialized_ = false;
        }
        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Cleanup warning: %s", e.what());
    }
}

}  // namespace realsense_multi_camera

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<realsense_multi_camera::RealSenseMultiCameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Failed to create node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}