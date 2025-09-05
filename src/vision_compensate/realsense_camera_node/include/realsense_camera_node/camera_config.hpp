#ifndef REALSENSE_CAMERA_NODE_CAMERA_CONFIG_HPP
#define REALSENSE_CAMERA_NODE_CAMERA_CONFIG_HPP

#include <string>

namespace realsense_camera_node
{

enum class CameraModel {
    D405,
    D435,
    AUTO,
    UNKNOWN
};

struct ModelConfig {
    double min_distance;
    double max_distance;
    double fov_horizontal;
    double fov_vertical;
};

struct CameraSettings {
    int width;
    int height;
    int fps;
};

struct DisplaySettings {
    std::string window_name;
    bool show_info_overlay;
    bool auto_resize;
};

struct ROS2Settings {
    std::string topic_name;
    std::string frame_id;
    int queue_size;
};

struct CameraConfig {
    CameraModel model;
    CameraSettings camera_settings;
    ModelConfig model_config;
    DisplaySettings display;
    ROS2Settings ros2;
};

class CameraDetector {
public:
    static CameraModel detectConnectedCamera();
    static std::vector<CameraModel> getAllConnectedCameras();
    static std::string modelToString(CameraModel model);
    static CameraModel stringToModel(const std::string& model_str);
};

} // namespace realsense_camera_node

#endif // REALSENSE_CAMERA_NODE_CAMERA_CONFIG_HPP