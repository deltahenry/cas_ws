#ifndef REALSENSE_MULTI_CAMERA__CAMERA_CONFIG_HPP_
#define REALSENSE_MULTI_CAMERA__CAMERA_CONFIG_HPP_

#include <string>
#include <memory>
#include <vector>
#include <librealsense2/rs.hpp>

namespace realsense_multi_camera
{

enum class CameraModel {
    D405,
    D435,
    AUTO,
    UNKNOWN
};

struct ModelSpecificConfig {
    double min_distance;
    double max_distance;
    int fov_horizontal;
    int fov_vertical;
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
    ModelSpecificConfig model_config;
    DisplaySettings display;
    ROS2Settings ros2;
    
    bool validate() const;
};

class CameraDetector {
public:
    static CameraModel detectConnectedCamera();
    static std::vector<CameraModel> getAllConnectedCameras();
    static std::string modelToString(CameraModel model);
    static CameraModel stringToModel(const std::string& model_str);
    
private:
    static bool isModelSupported(const std::string& product_line);
};

}  // namespace realsense_multi_camera

#endif  // REALSENSE_MULTI_CAMERA__CAMERA_CONFIG_HPP_