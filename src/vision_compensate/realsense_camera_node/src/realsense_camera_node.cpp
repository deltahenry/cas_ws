#include "realsense_camera_node/realsense_camera_node.hpp"
#include "vision.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace realsense_camera_node
{

RealSenseMultiCameraNode::RealSenseMultiCameraNode()
    : Node("realsense_multi_camera_node"), camera_initialized_(false), show_display_(true)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RealSense Camera Node");
    
    // Load parameters from YAML
    loadParameters();
    
    // Initialize camera
    if (!initializeCamera()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
        return;
    }
    
    // Setup publishers and subscribers
    setupPublishers();
    setupSubscribers();
    
    // Setup timer for continuous capture
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / config_.camera_settings.fps),
        std::bind(&RealSenseMultiCameraNode::captureAndPublish, this));
        
    RCLCPP_INFO(this->get_logger(), "RealSense Camera Node initialized successfully");
}

RealSenseMultiCameraNode::~RealSenseMultiCameraNode()
{
    cleanup();
}

void RealSenseMultiCameraNode::loadParameters()
{
    // Declare parameters with default values
    this->declare_parameter("realsense_camera.model", "D405");
    this->declare_parameter("realsense_camera.camera_settings.width", 640);
    this->declare_parameter("realsense_camera.camera_settings.height", 480);
    this->declare_parameter("realsense_camera.camera_settings.fps", 30);
    this->declare_parameter("realsense_camera.display.window_name", "RealSense Camera");
    this->declare_parameter("realsense_camera.display.show_info_overlay", true);
    this->declare_parameter("realsense_camera.display.auto_resize", true);
    this->declare_parameter("realsense_camera.ros2.topic_name", "camera/rgb/image_raw");
    this->declare_parameter("realsense_camera.ros2.frame_id", "camera_link");
    this->declare_parameter("realsense_camera.ros2.queue_size", 1);
    this->declare_parameter("show_display", true);
    
    // Load parameters
    std::string model_str = this->get_parameter("realsense_camera.model").as_string();
    config_.model = CameraDetector::stringToModel(model_str);
    
    config_.camera_settings.width = this->get_parameter("realsense_camera.camera_settings.width").as_int();
    config_.camera_settings.height = this->get_parameter("realsense_camera.camera_settings.height").as_int();
    config_.camera_settings.fps = this->get_parameter("realsense_camera.camera_settings.fps").as_int();
    
    config_.display.window_name = this->get_parameter("realsense_camera.display.window_name").as_string();
    config_.display.show_info_overlay = this->get_parameter("realsense_camera.display.show_info_overlay").as_bool();
    config_.display.auto_resize = this->get_parameter("realsense_camera.display.auto_resize").as_bool();
    
    config_.ros2.topic_name = this->get_parameter("realsense_camera.ros2.topic_name").as_string();
    config_.ros2.frame_id = this->get_parameter("realsense_camera.ros2.frame_id").as_string();
    config_.ros2.queue_size = this->get_parameter("realsense_camera.ros2.queue_size").as_int();
    
    show_display_ = this->get_parameter("show_display").as_bool();
    window_name_ = config_.display.window_name;
    
    // Get model-specific configuration
    config_.model_config = getModelConfig(config_.model);
    
    // Validate parameters
    if (!validateParameters(config_)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid parameters detected");
    }
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded - Model: %s, Resolution: %dx%d, FPS: %d",
                CameraDetector::modelToString(config_.model).c_str(),
                config_.camera_settings.width,
                config_.camera_settings.height,
                config_.camera_settings.fps);
}

ModelConfig RealSenseMultiCameraNode::getModelConfig(CameraModel model)
{
    ModelConfig model_config;
    
    switch (model) {
        case CameraModel::D405:
            model_config.min_distance = 0.13;
            model_config.max_distance = 4.0;
            model_config.fov_horizontal = 87.0;
            model_config.fov_vertical = 58.0;
            break;
        case CameraModel::D435:
            model_config.min_distance = 0.11;
            model_config.max_distance = 10.0;
            model_config.fov_horizontal = 87.0;
            model_config.fov_vertical = 58.0;
            break;
        default:
            // Default to D405 settings
            model_config.min_distance = 0.13;
            model_config.max_distance = 4.0;
            model_config.fov_horizontal = 87.0;
            model_config.fov_vertical = 58.0;
            break;
    }
    
    return model_config;
}

bool RealSenseMultiCameraNode::initializeCamera()
{
    try {
        // Configure camera
        cfg_.enable_stream(RS2_STREAM_COLOR, 
                          config_.camera_settings.width,
                          config_.camera_settings.height,
                          RS2_FORMAT_BGR8,
                          config_.camera_settings.fps);
        
        // Start pipeline
        profile_ = pipe_.start(cfg_);
        camera_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Camera initialized successfully");
        return true;
        
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing camera: %s", e.what());
        return false;
    }
}

void RealSenseMultiCameraNode::setupPublishers()
{
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        config_.ros2.topic_name, config_.ros2.queue_size);
        
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info", config_.ros2.queue_size);
}

void RealSenseMultiCameraNode::setupSubscribers()
{
    detection_cmd_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/detection_cmd", 10,
        std::bind(&RealSenseMultiCameraNode::detectionCommandCallback, this, std::placeholders::_1));
}

cv::Mat RealSenseMultiCameraNode::get_realsense_image()
{
    if (!camera_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "Camera not initialized");
        return cv::Mat();
    }
    
    try {
        // Wait for frames
        rs2::frameset frames = pipe_.wait_for_frames();
        
        // Get color frame
        rs2::frame color_frame = frames.get_color_frame();
        
        // Convert to OpenCV Mat
        cv::Mat image(cv::Size(config_.camera_settings.width, config_.camera_settings.height),
                     CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        return image.clone();
        
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "RealSense error in get_realsense_image: %s", e.what());
        return cv::Mat();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error capturing image: %s", e.what());
        return cv::Mat();
    }
}

void RealSenseMultiCameraNode::captureAndPublish()
{
    // Get image using function-based approach
    cv::Mat image = get_realsense_image();
    
    if (image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to capture image");
        return;
    }
    
    // Display image if enabled
    if (show_display_) {
        displayImage(image);
    }
    
    // Convert to ROS message and publish
    try {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = config_.ros2.frame_id;
        
        sensor_msgs::msg::Image::SharedPtr img_msg = 
            cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        
        image_publisher_->publish(*img_msg);
        
        // Publish camera info
        sensor_msgs::msg::CameraInfo camera_info_msg;
        camera_info_msg.header = header;
        camera_info_msg.width = config_.camera_settings.width;
        camera_info_msg.height = config_.camera_settings.height;
        
        camera_info_publisher_->publish(camera_info_msg);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing image: %s", e.what());
    }
}

void RealSenseMultiCameraNode::displayImage(const cv::Mat& image)
{
    try {
        cv::Mat display_image = image.clone();
        
        // Add info overlay if enabled
        if (config_.display.show_info_overlay) {
            std::string info_text = "Model: " + CameraDetector::modelToString(config_.model) +
                                   " | " + std::to_string(config_.camera_settings.width) + "x" +
                                   std::to_string(config_.camera_settings.height) +
                                   " | " + std::to_string(config_.camera_settings.fps) + " FPS";
            
            cv::putText(display_image, info_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow(window_name_, display_image);
        cv::waitKey(1);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error displaying image: %s", e.what());
    }
}

void RealSenseMultiCameraNode::detectionCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received detection command: %s", msg->data.c_str());
    
    // Process command-specific actions
    if (msg->data == "start_detect") {
        // Trigger image capture for real-time compensation
        cv::Mat image = get_realsense_image();
        if (!image.empty()) {
            RCLCPP_INFO(this->get_logger(), "Image captured for real-time detection");
            
            std::string filename = "/tmp/realsense_capture_" + 
                                 std::to_string(this->now().seconds()) + ".jpg";
            cv::imwrite(filename, image);
            RCLCPP_INFO(this->get_logger(), "Image saved to: %s", filename.c_str());
            
            std::string golden_path = "/tmp/golden_reference.jpg";
            double X_offset, Z_offset;
            
            int result = bfc::batteryFrameLocationCompensation(golden_path, image, X_offset, Z_offset);
            
            if (result == bfc::kOk) {
                RCLCPP_INFO(this->get_logger(), "[start_detect] Battery frame compensation successful - X_offset: %.2f, Z_offset: %.2f", 
                           X_offset, Z_offset);
            } else {
                RCLCPP_WARN(this->get_logger(), "[start_detect] Battery frame compensation failed with error code: %d", result);
                logCompensationError(result, golden_path);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "[start_detect] Failed to capture image from camera");
        }
    }
    else if (msg->data == "test_golden") {
            // Test using two completely fixed static image files (非動態取圖)
            // No camera involvement - purely static file processing
            std::string golden_path = "/home/harveywang/image_sample/2D_golden(x=0,z=112).png";
            std::string current_image_path = "/home/harveywang/image_sample/2D(x=25,z=117).png";
            
            RCLCPP_INFO(this->get_logger(), "[test_golden] Testing 2 fixed images - No camera capture involved");
            RCLCPP_INFO(this->get_logger(), "Fixed Image 1 (Golden): %s", golden_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Fixed Image 2 (Current): %s", current_image_path.c_str());
            
            double X_offset, Z_offset;
            int result = bfc::batteryFrameLocationCompensation(golden_path, current_image_path, X_offset, Z_offset);
            
            if (result == bfc::kOk) {
                RCLCPP_INFO(this->get_logger(), "[test_golden] Static file compensation successful - X_offset: %.2f, Z_offset: %.2f", 
                           X_offset, Z_offset);
            } else {
                RCLCPP_WARN(this->get_logger(), "[test_golden] Static file compensation failed with error code: %d", result);
                logCompensationError(result, golden_path);
            }
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Unknown detection command: %s", msg->data.c_str());
        RCLCPP_INFO(this->get_logger(), "Available commands: 'start_detect', 'test_golden'");
    }
}

bool RealSenseMultiCameraNode::validateParameters(const CameraConfig& config)
{
    // Validate camera settings
    if (config.camera_settings.width <= 0 || config.camera_settings.height <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid image dimensions");
        return false;
    }
    
    if (config.camera_settings.fps <= 0 || config.camera_settings.fps > 60) {
        RCLCPP_ERROR(this->get_logger(), "Invalid FPS value");
        return false;
    }
    
    if (config.ros2.queue_size <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid queue size");
        return false;
    }
    
    return true;
}

void RealSenseMultiCameraNode::cleanup()
{
    try {
        if (camera_initialized_) {
            pipe_.stop();
            camera_initialized_ = false;
        }
        
        if (show_display_) {
            cv::destroyAllWindows();
        }
        
        RCLCPP_INFO(this->get_logger(), "Cleanup completed");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error during cleanup: %s", e.what());
    }
}

void RealSenseMultiCameraNode::logCompensationError(int error_code, const std::string& golden_path)
{
    // Log specific error messages based on error code
    switch (error_code) {
        case bfc::kErrReadImage:
            RCLCPP_ERROR(this->get_logger(), "Error: Could not read golden reference image at %s", golden_path.c_str());
            break;
        case bfc::kErrChannelMissing:
            RCLCPP_ERROR(this->get_logger(), "Error: Missing color channels in image");
            break;
        case bfc::kErrRightSideDetect:
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to detect right side of battery frame");
            break;
        case bfc::kErrLeftSideDetect:
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to detect left side of battery frame");
            break;
        case bfc::kErrFourPointsNotFound:
            RCLCPP_ERROR(this->get_logger(), "Error: Could not find four corner points");
            break;
        case bfc::kErrLineIntersection:
            RCLCPP_ERROR(this->get_logger(), "Error: Line intersection calculation failed");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Error: Unknown compensation error");
            break;
    }
}

// Camera Detector Implementation
CameraModel CameraDetector::detectConnectedCamera()
{
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        if (devices.size() == 0) {
            return CameraModel::UNKNOWN;
        }
        
        // Get first device
        rs2::device device = devices[0];
        std::string product_name = device.get_info(RS2_CAMERA_INFO_NAME);
        
        if (product_name.find("D405") != std::string::npos) {
            return CameraModel::D405;
        } else if (product_name.find("D435") != std::string::npos) {
            return CameraModel::D435;
        }
        
        return CameraModel::UNKNOWN;
        
    } catch (const std::exception& e) {
        return CameraModel::UNKNOWN;
    }
}

std::vector<CameraModel> CameraDetector::getAllConnectedCameras()
{
    std::vector<CameraModel> cameras;
    
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        for (auto&& device : devices) {
            std::string product_name = device.get_info(RS2_CAMERA_INFO_NAME);
            
            if (product_name.find("D405") != std::string::npos) {
                cameras.push_back(CameraModel::D405);
            } else if (product_name.find("D435") != std::string::npos) {
                cameras.push_back(CameraModel::D435);
            }
        }
        
    } catch (const std::exception& e) {
        // Return empty vector on error
    }
    
    return cameras;
}

std::string CameraDetector::modelToString(CameraModel model)
{
    switch (model) {
        case CameraModel::D405: return "D405";
        case CameraModel::D435: return "D435";
        case CameraModel::AUTO: return "AUTO";
        default: return "UNKNOWN";
    }
}

CameraModel CameraDetector::stringToModel(const std::string& model_str)
{
    if (model_str == "D405") return CameraModel::D405;
    if (model_str == "D435") return CameraModel::D435;
    if (model_str == "auto" || model_str == "AUTO") return CameraModel::AUTO;
    return CameraModel::UNKNOWN;
}

} // namespace realsense_camera_node

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<realsense_camera_node::RealSenseMultiCameraNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
