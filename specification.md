# ROS2 Vision Compensation System Specification

## Overview
This specification describes the implementation of two ROS2 Python packages for vision-based compensation using RealSense cameras.

## Package Structure
```
src/
├── realsense_camera_driver/
└── vision_compensation/
```

## Package 1: realsense_camera_driver

### Description
A Python-based ROS2 package to drive RealSense cameras and publish RGB images.

### Configuration
- **Configuration file**: `parameter.yaml`
- **Camera model**: Configurable (D405, D435, etc.)
- **Camera count**: Configurable (1, 2, 3, ...)
- **FPS**: Configurable (default: 15)

### Topics
- **Publisher**: `/camera_image`
  - Type: `CV.MAT`
  - Frequency: 10Hz

### Parameters
```yaml
camera:
  model: "D435"  # D405, D435, etc.
  count: 1       # Number of cameras
  fps: 15        # Frames per second
```

## Package 2: vision_compensation

### Description
A ROS2 package that performs vision-based compensation by comparing current images with golden samples.

### Subscribers
1. **Topic**: `/detection_cmd`
   - Type: `String`
   - Callback: `detection_cmd_callback`

2. **Topic**: `/camera_image`
   - Type: `CV.MAT`
   - Callback: Stores image in `self.image_now`

### Publisher
- **Topic**: `/compensate_pose`
- **Type**: `Float32MultiArray`
- **Usage example**:
```python
def compensate_pose_callback(self, msg: Float32MultiArray):
    if len(msg.data) >= 2:
        X, Z = msg.data[:2]
        self.get_logger().info(f"Received compensation: X={X:.2f}, Z={Z:.2f}")
```

### Commands

#### 1. start_detect
- **Trigger**: `msg.data == "start_detect"`
- **Function**: `x, z = compensate_cabinet(image_golden, image_now)`
- **Description**: Compare golden sample with current image
- **Calibration coefficients**: 
  - `X_out *= 0.53`
  - `Z_out *= 0.18`
- **Golden sample path**: `~/image_sample2D_golden(x=0,z=112).png`

#### 2. start_detect_0
- **Trigger**: `msg.data == "start_detect_0"`
- **Function**: `x, z = compensate_cabinet_test(image_golden, image_test)`
- **Description**: Compare golden sample with same image (test mode)
- **Calibration coefficients**: 
  - `X_out *= 0.53`
  - `Z_out *= 0.18`
- **Golden sample path**: `~/image_sample2D_golden(x=0,z=112).png`
- **Test image path**: `~/image_sample2D_golden(x=0,z=112).png` (same as golden)

#### 3. start_test
- **Trigger**: `msg.data == "start_test"`
- **Function**: `x, z = compensate_cabinet_test(image_golden, image_test)`
- **Description**: Compare golden sample with test image file
- **Calibration coefficients**: 
  - `X_out *= 0.53`
  - `Z_out *= 0.18`
- **Golden sample path**: `~/image_sample/2D_golden(x=0,z=112).png`
- **Test image path**: `~/image_sample/2D(x=25,z=117).png`

### Vision Processing Module

#### File: `vision.py`
Convert C++ utility functions from `utility/vision.cpp` and `vision.hpp` to Python:

1. **Function**: `compensate_cabinet(image_golden, image_now)`
   - **Original C++**: `int batteryFrameLocationCompensation(const std::string& golden_path_name, const std::string& current_image_path, double& X_out, double& Z_out)`
   - **Returns**: `x, z` coordinates

2. **Function**: `compensate_cabinet_test(image_golden, image_test)`
   - **Original C++**: `int batteryFrameLocationCompensation(const std::string& golden_path_name, cv::Mat current, double& X_out, double& Z_out)`
   - **Returns**: `x, z` coordinates

## Dependencies
- ROS2 (Python)
- OpenCV (cv2)
- RealSense SDK
- std_msgs
- sensor_msgs

## File Paths
- Golden sample (detect): `~/image_sample2D_golden(x=0,z=112).png`
- Golden sample (detect_0): `~/image_sample2D_golden(x=0,z=112).png`
- Test image (detect_0): `~/image_sample2D_golden(x=0,z=112).png`
- Golden sample (test): `~/image_sample/2D_golden(x=0,z=112).png`
- Test image (test): `~/image_sample/2D(x=25,z=117).png`