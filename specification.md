# ROS2 Vision Compensation System Specification

## Overview
<<<<<<< HEAD
This specification describes the implementation of two ROS2 Python packages for vision-based compensation using RealSense cameras.
=======
This specification describes the implementation of three ROS2 Python packages for vision-based compensation using RealSense cameras.
>>>>>>> test2

## Package Structure
```
src/
├── realsense_camera_driver/
<<<<<<< HEAD
└── vision_compensation/
=======
├── vision_compensation/
└── show_image_node/
>>>>>>> test2
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
A ROS2 package that performs vision-based compensation by comparing current images with golden samples and provides image display and save functionality.

### Subscribers
1. **Topic**: `/detection_cmd`
   - Type: `String`
   - Callback: `detection_cmd_callback`

2. **Topic**: `/camera_image`
   - Type: `CV.MAT`
   - Callback: Stores image in `self.image_now`

<<<<<<< HEAD
### Publisher
- **Topic**: `/compensate_pose`
- **Type**: `Float32MultiArray`
- **Usage example**:
=======
### Publishers
1. **Topic**: `/compensate_pose`
   - Type: `Float32MultiArray`
   - Description: Publish compensation X,Z values

2. **Topic**: `/image_golden_with_corner`
   - Type: `Image`
   - Description: Golden image with corner annotations

3. **Topic**: `/image_now_with_corner`
   - Type: `Image`
   - Description: Current image with corner annotations

### Usage Example
>>>>>>> test2
```python
def compensate_pose_callback(self, msg: Float32MultiArray):
    if len(msg.data) >= 2:
        X, Z = msg.data[:2]
        self.get_logger().info(f"Received compensation: X={X:.2f}, Z={Z:.2f}")
```

### Features
- **Current Image Display**: Show current camera image
<<<<<<< HEAD
=======
- **Corner Detection and Annotation**: Visualize detected corners on images
>>>>>>> test2
- **Image Processing**: Vision-based compensation calculations
- **Image Saving**: Save current image with timestamp

### Commands

#### 1. start_detect
- **Trigger**: `msg.data == "start_detect"`
- **Function**: `x, z = compensate_cabinet(image_golden, image_now)`
- **Description**: Compare golden sample with current image
<<<<<<< HEAD
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
=======
- **Publishers**: Both `/image_golden_with_corner` and `/image_now_with_corner`
- **Calibration coefficients**: 
  - `X_out *= 0.53`
  - `Z_out *= 0.18`
- **Golden sample path**: `~/image_sample/2D_golden(x=0,z=112).png`

#### 2. start_detect_0
- **Trigger**: `msg.data == "start_detect_0"`
- **Function**: `x, z = compensate_cabinet(image_golden, image_test)`
- **Description**: Compare golden sample with same image (test mode)
- **Publishers**: Both `/image_golden_with_corner` and `/image_now_with_corner`
- **Calibration coefficients**: 
  - `X_out *= 0.53`
  - `Z_out *= 0.18`
- **Golden sample path**: `~/image_sample/2D_golden(x=0,z=112).png`
- **Test image path**: `~/image_sample/2D_golden(x=0,z=112).png` (same as golden)
>>>>>>> test2

#### 3. start_test
- **Trigger**: `msg.data == "start_test"`
- **Function**: `x, z = compensate_cabinet_test(image_golden, image_test)`
- **Description**: Compare golden sample with test image file
<<<<<<< HEAD
=======
- **Publishers**: Both `/image_golden_with_corner` and `/image_now_with_corner`
>>>>>>> test2
- **Calibration coefficients**: 
  - `X_out *= 0.53`
  - `Z_out *= 0.18`
- **Golden sample path**: `~/image_sample/2D_golden(x=0,z=112).png`
- **Test image path**: `~/image_sample/2D(x=25,z=117).png`

#### 4. save_image
- **Trigger**: `msg.data == "save_image"`
- **Function**: Save current image to PNG file
- **Description**: Save current camera image with timestamp filename
- **Filename format**: `YYYY_MMDD_HHMMSS.png` (e.g., `2025_0908_173026.png`)
- **Save location**: Current working directory

### Vision Processing Module

#### File: `vision.py`
<<<<<<< HEAD
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
- datetime (for timestamp generation)

## File Paths
- Golden sample (detect): `~/image_sample2D_golden(x=0,z=112).png`
- Golden sample (detect_0): `~/image_sample2D_golden(x=0,z=112).png`
- Test image (detect_0): `~/image_sample2D_golden(x=0,z=112).png`
- Golden sample (test): `~/image_sample/2D_golden(x=0,z=112).png`
- Test image (test): `~/image_sample/2D(x=25,z=117).png`
- Saved images: `YYYY_MMDD_HHMMSS.png` in current directory
=======
Translate C# utility functions from `utility/vision.cs` to Python:

1. **Main Function**: `compensate_cabinet(image_golden, image_now)`
   - **Original C#**: `CompareAverageCornersAndDraw` from `BlobEdgePipelineV2` class
   - **Purpose**: Compare golden sample with current image
   - **Input**: Two images (golden reference and current camera image)
   - **Output**: `x, z` coordinates representing the offset
   - **Algorithm**: Uses blob detection and edge analysis to find corner points and calculate position differences

2. **Test Function**: `compensate_cabinet_test(image_golden, image_test)`
   - **Original C#**: Same `CompareAverageCornersAndDraw` function
   - **Purpose**: Compare golden sample with test image file
   - **Input**: Two image file paths
   - **Output**: `x, z` coordinates

3. **Class**: `BlobEdgePipeline`
   - **Purpose**: Container for vision pipeline parameters
   - **Configuration**: Load parameters from `vision_parameter.yaml`

#### File: `example.py`
Translate C# example code from `utility/example.cs` to demonstrate usage:
- Shows how to configure the vision pipeline parameters
- Demonstrates calling the comparison function
- Example of processing results and displaying output
- **Display output images**: Must display the output images with corner annotations

#### Configuration File: `vision_parameter.yaml`
```yaml
# Vision pipeline parameters
blob_edge_pipeline:
  MinAreaLR: 10000          # Minimum area for left/right blobs
  LeftRightThreshold: 165   # Threshold for left/right detection
  MinAreaTB: 150           # Minimum area for top/bottom blobs
  TopBottomThreshold: 150  # Threshold for top/bottom detection
  dilateIterationsLR: 0    # Dilation iterations for LR
  TrimPercent: 15          # Trimming percentage
  BandScale: 0.98          # Band scaling factor
  MinSamplesForFit: 10     # Minimum samples for line fitting
```

### Vision Algorithm Details
The vision processing uses a blob edge detection pipeline with the following stages:
1. **Left/Right Edge Detection**: Threshold ~150-165, minimum area 10000
2. **Top/Bottom Edge Detection**: Threshold ~120-150, minimum area 150-300
3. **Corner Detection**: Find intersections of detected edges
4. **Position Calculation**: Compare average corner positions between golden and current images

## Dependencies
- ROS2 (Python)
- OpenCV (cv2) - for image processing and computer vision algorithms
- RealSense SDK (pyrealsense2) - for camera interface
- std_msgs - for String messages
- std_msgs.msg.Float32MultiArray - for pose compensation data
- sensor_msgs - for image message types
- cv_bridge - for ROS-OpenCV image conversion
- datetime - for timestamp generation
- numpy - for numerical operations in vision processing

## Package 3: show_image_node

### Description
A ROS2 node that displays visualization images from the vision compensation system.

### Subscribers
1. **Topic**: `/image_golden_with_corner`
   - Type: `Image`
   - Description: Golden image with corner annotations

2. **Topic**: `/image_now_with_corner`
   - Type: `Image`
   - Description: Current image with corner annotations

### Features
- **Dual Image Display**: Shows both images in a single window
- **Layout**: Images displayed vertically (top and bottom)
- **Real-time Update**: Updates display when new images are received

## File Structure
```
src/
├── realsense_camera_driver/
│   ├── package.xml
│   ├── setup.py
│   ├── realsense_camera_driver/
│   │   ├── __init__.py
│   │   └── realsense_camera_node.py
│   └── config/
│       └── parameter.yaml
├── vision_compensation/
│   ├── package.xml
│   ├── setup.py
│   ├── vision_compensation/
│   │   ├── __init__.py
│   │   ├── vision_compensation_node.py
│   │   ├── vision.py (translated from utility/vision.cs)
│   │   └── example.py (translated from utility/example.cs)
│   └── config/
│       └── vision_parameter.yaml
└── show_image_node/
    ├── package.xml
    ├── setup.py
    └── show_image_node/
        ├── __init__.py
        └── show_image_node.py
```

## File Paths
- Golden sample (detect): `~/image_sample/2D_golden(x=0,z=112).png`
- Golden sample (detect_0): `~/image_sample/2D_golden(x=0,z=112).png`
- Test image (detect_0): `~/image_sample/2D_golden(x=0,z=112).png`
- Golden sample (test): `~/image_sample/2D_golden(x=0,z=112).png`
- Test image (test): `~/image_sample/2D(x=25,z=117).png`
- Saved images: `YYYY_MMDD_HHMMSS.png` in current directory

## Translation Notes
- **Source files**: `utility/vision.cs` and `utility/example.cs`
- **Target location**: Both translated files should be placed in the `vision_compensation` package
- **Main function to translate**: `CompareAverageCornersAndDraw` → `compensate_cabinet`
- **Class creation**: Create `BlobEdgePipeline` class for parameter management
- **Configuration**: Parameters stored in `vision_parameter.yaml`
- **C# to Python considerations**: Convert Emgu CV (C#) calls to OpenCV (Python) equivalents
>>>>>>> test2
