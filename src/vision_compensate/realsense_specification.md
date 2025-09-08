# RealSense Multi-Model ROS2 Node Specification

## Project Overview

This specification defines a ROS2 node written in C++ that can drive multiple RealSense camera models (D405, D435, etc.) based on configuration parameters loaded from a YAML file. The node captures RGB images and displays them using OpenCV.

## Requirements

### Functional Requirements

1. **Multi-Model Camera Support**
   - Support RealSense D405 and D435 models
   - Dynamically select camera model from YAML configuration
   - Auto-detect available cameras if model not specified

2. **Parameter Management**
   - Load configuration from `parameter.yaml` file
   - Support runtime parameter updates
   - Validate parameter values

3. **Image Processing**
   - Capture RGB images from selected RealSense camera through function-based approach
   - Implement `get_realsense_image()` function that returns image data
   - Replace stream-based camera reading with function-based approach
   - Display images in real-time using OpenCV
   - Publish images as ROS2 messages

4. **Command Subscription and Result Publishing**
   - Subscribe to `/detection_cmd` topic with string message type
   - Implement callback function to handle detection commands
   - Support "start_detect" command for real-time battery frame compensation
   - Support "test_golden" command for file-based battery frame compensation
   - Publish compensation results to `/compensate_pose` topic using Float32MultiArray
   - Send X, Z coordinates when "start_detect" command is processed

5. **Build System**
   - CMake-based build configuration
   - Automated compilation and execution scripts
   - Dependency management

6. **C# to C++ Translation**
   - Translate `/home/harveywang/ros2/vision_bridge_node/src/camera_compensate/vision.cs` to `vision.cpp`
   - Maintain exactly the two batteryFrameLocationCompensation function signatures as specified
   - Preserve core image processing functionality from C# EmguCV to C++ OpenCV
   - Ensure seamless integration with ROS2 detection command callbacks

### Non-Functional Requirements

1. **Performance**
   - Minimum 30 FPS operation
   - Low latency image processing
   - Efficient memory usage

2. **Reliability**
   - Robust error handling
   - Camera disconnect/reconnect handling
   - Graceful shutdown

3. **Maintainability**
   - Modular code structure
   - Comprehensive documentation
   - Clear separation of concerns

## System Architecture

### Component Structure

```
realsense_multi_camera/
├── config/
│   └── parameter.yaml
├── src/
│   └── realsense_multi_camera_node.cpp
├── include/
│   └── realsense_multi_camera/
│       ├── realsense_multi_camera_node.hpp
│       └── camera_config.hpp
├── launch/
│   └── realsense_camera.launch.py
├── CMakeLists.txt
├── package.xml
└── scripts/
    ├── build_and_run.bash
    └── setup_environment.bash
```

### Key Classes

1. **RealSenseMultiCameraNode**
   - Main ROS2 node class
   - Parameter management
   - Camera lifecycle management

2. **CameraConfig**
   - Configuration data structure
   - Parameter validation
   - Model-specific settings

## Configuration Specification

### parameter.yaml Structure

```yaml
realsense_camera:
  # Camera model selection
  model: "D405"  # Options: "D405", "D435", "auto"
  
  # Camera settings
  camera_settings:
    width: 640
    height: 480
    fps: 30
    
  # Model-specific configurations
  model_configs:
    D405:
      min_distance: 0.13  # meters
      max_distance: 4.0   # meters
      fov_horizontal: 87  # degrees
      fov_vertical: 58    # degrees
      
    D435:
      min_distance: 0.11  # meters
      max_distance: 10.0  # meters
      fov_horizontal: 87  # degrees
      fov_vertical: 58    # degrees
      
  # Display settings
  display:
    window_name: "RealSense Camera"
    show_info_overlay: true
    auto_resize: true
    
  # ROS2 settings
  ros2:
    topic_name: "camera/rgb/image_raw"
    frame_id: "camera_link"
    queue_size: 1
```

## API Specification

### ROS2 Interface

#### Published Topics
- `/camera/rgb/image_raw` (sensor_msgs::Image) - RGB image data
- `/camera/camera_info` (sensor_msgs::CameraInfo) - Camera calibration info
- `/compensate_pose` (std_msgs::Float32MultiArray) - Battery frame compensation results (X, Z coordinates)

#### Subscribed Topics
- `/detection_cmd` (std_msgs::String) - Detection command input
  - Supported commands:
    - `"start_detect"` - Real-time battery frame compensation using live camera
    - `"test_golden"` - Static file-based compensation using two fixed images

#### Parameters
- `model` (string) - Camera model selection
- `width` (int) - Image width
- `height` (int) - Image height
- `fps` (int) - Frame rate
- `show_display` (bool) - Enable/disable OpenCV display

#### Services
- `/camera/set_parameters` - Update camera parameters at runtime
- `/camera/get_camera_info` - Retrieve camera information

### C++ API

#### Core Methods

```cpp
class RealSenseMultiCameraNode {
public:
    RealSenseMultiCameraNode();
    ~RealSenseMultiCameraNode();
    
private:
    // Initialization
    bool initializeCamera();
    void loadParameters();
    void setupPublishers();
    
    // Camera operations
    cv::Mat get_realsense_image();
    void captureAndPublish();
    void displayImage(const cv::Mat& image);
    
    // Subscriber callbacks
    void detectionCommandCallback(const std_msgs::msg::String::SharedPtr msg);
    
    // Publisher management
    void setupCompensatePublisher();
    
    // Parameter handling
    void parameterCallback(const std::vector<rclcpp::Parameter>& parameters);
    bool validateParameters(const CameraConfig& config);
    
    // Cleanup
    void cleanup();
};
```

## Implementation Details

### C# to C++ Translation

The project requires translating an existing C# vision processing implementation to C++:

#### Source File
- **Original**: `/home/harveywang/ros2/vision_bridge_node/src/camera_compensate/vision.cs` (actual location)
- **Referenced as**: `/home/harveywang/ros2/vision_bridge_node/src/vision.cs` (in requirement)
- **Target**: `/home/harveywang/ros2/vision_bridge_node/src/camera_compensate/vision.cpp`

#### Key Translation Requirements
1. **EmguCV to OpenCV**: Convert C# EmguCV calls to C++ OpenCV equivalents
2. **Function Signatures**: Maintain the two batteryFrameLocationCompensation function signatures
3. **Image Processing Pipeline**: Preserve the core algorithm functionality
4. **Integration**: Ensure seamless integration with the ROS2 node

#### Original C# Structure
```csharp
internal class vision_1
{
    public int batteryframelocationcompensation(string golden_path_name, 
                                              string current_image_path_name, 
                                              ref double X, ref double Z)
    {
        // Image loading with EmguCV
        Image<Bgr, byte> color_golden = new Image<Bgr, byte>(golden_path_name);
        Image<Gray, byte>[] gray_golden = color_golden.Split();
        
        // Core processing algorithm
        findoutbatteryframecorner(gray_golden[2], thr1, thr2, thr3, area, ref point_golden);
        
        // Calculate offsets
        X = x_golden_average - x_current_average;
        Z = y_golden_average - y_current_average;
        return 0;
    }
}
```

#### C++ Translation Target - Exact Required Function Signatures
```cpp
namespace bfc {
    // Required by msg->data == "test_golden"
    int batteryFrameLocationCompensation(const std::string& golden_path_name,
                                       const std::string& current_image_path,
                                       double& X_out, double& Z_out);
                                       
    // Required by msg->data == "start_detect" 
    int batteryFrameLocationCompensation(const std::string& golden_path_name,
                                       cv::Mat current,
                                       double& X_out, double& Z_out);
}
```

**Critical Requirement**: These exact function signatures must be maintained as specified in the requirement lines 7-8.

### Battery Frame Location Compensation Integration

The system integrates battery frame location compensation functionality through the detection command callback:

#### Command Types:
- **"start_detect"**: Performs real-time compensation using live camera image
- **"test_golden"**: Performs compensation using two specific fixed image files with known coordinates for validation

#### Function Signatures:
```cpp
// Real-time compensation with cv::Mat
int batteryFrameLocationCompensation(const std::string& golden_path_name, 
                                   cv::Mat current, 
                                   double& X_out, double& Z_out);

// File-based compensation with image paths
int batteryFrameLocationCompensation(const std::string& golden_path_name, 
                                   const std::string& current_image_path, 
                                   double& X_out, double& Z_out);
```

### Function-Based Image Capture

The camera implementation uses a function-based approach rather than continuous streaming. This replaces traditional stream-based camera reading:

```cpp
class RealSenseMultiCameraNode {
private:
    // Function-based image capture (replaces stream-based reading)
    cv::Mat get_realsense_image() {
        // Configure and capture single frame from RealSense camera
        // Return processed RGB image as cv::Mat
        // Handle errors gracefully
        // This function is called when image is needed
    }
    
    // Callback for detection commands subscription
    void detectionCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Process detection command received from /detection_cmd topic
        // Trigger image capture if needed using get_realsense_image()
        // Execute corresponding action based on command
        RCLCPP_INFO(this->get_logger(), "Received detection command: %s", msg->data.c_str());
        
        if (msg->data == "start_detect") {
            // Real-time battery frame compensation using current captured image
            cv::Mat current_image = get_realsense_image();
            double X_out, Z_out;
            int result = batteryFrameLocationCompensation(golden_path, current_image, X_out, Z_out);
            
            // Publish compensation results to /compensate_pose topic
            if (result == bfc::kOk) {
                std_msgs::Float32MultiArray pose_msg;
                pose_msg.data = {static_cast<float>(X_out), static_cast<float>(Z_out)};
                compensate_pose_publisher_->publish(pose_msg);
            }
        }
        else if (msg->data == "test_golden") {
            // Validation using two specific fixed image files with known coordinates
            // Expected: X_diff=25, Z_diff=5 -> X_out≈13.25, Z_out≈0.9 after calibration
            std::string golden_path = "/home/harveywang/image_sample/2D_golden(x=0,z=112).png";
            std::string current_image_path = "/home/harveywang/image_sample/2D(x=25,z=117).png";
            double X_out, Z_out;
            int result = batteryFrameLocationCompensation(golden_path, current_image_path, X_out, Z_out);
            // Result includes calibration: X_out = (detected_diff) * 0.53, Z_out = (detected_diff) * 0.18
        }
    }
};
```

### Camera Model Detection

```cpp
enum class CameraModel {
    D405,
    D435,
    AUTO,
    UNKNOWN
};

class CameraDetector {
public:
    static CameraModel detectConnectedCamera();
    static std::vector<CameraModel> getAllConnectedCameras();
    static std::string modelToString(CameraModel model);
};
```

### Error Handling Strategy

1. **Camera Connection Errors**
   - Retry mechanism with exponential backoff
   - Fallback to available camera models
   - User notification through logs

2. **Parameter Validation**
   - Range checking for numeric parameters
   - Model compatibility validation
   - Default value fallback

3. **Runtime Errors**
   - Frame drop detection
   - Memory allocation failure handling
   - Graceful degradation

## Build and Deployment

### Dependencies

#### System Dependencies
- Ubuntu 22.04 (recommended)
- ROS2 Humble
- Intel RealSense SDK 2.54+
- OpenCV 4.5+

#### ROS2 Package Dependencies
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>camera_info_manager</depend>
```

### Build Process

1. **Automated Build and Run**
   ```bash
   ./scripts/build_and_run.bash
   ```

2. **Manual Environment Setup** (if needed)
   ```bash
   ./scripts/setup_environment.bash
   ```

3. **Manual Compilation** (if needed)
   ```bash
   colcon build --packages-select realsense_multi_camera
   ```

4. **Manual Execution** (if needed)
   ```bash
   ros2 launch realsense_multi_camera realsense_camera.launch.py
   ```

### Automation Scripts

#### build_and_run.bash Features
- Automated compilation using colcon build
- Automated execution of the ROS2 node
- Dependency verification
- Build status reporting
- Parameter validation
- Launch options
- Debug mode support
- Environment setup and sourcing

## Testing Strategy

### Unit Tests
- Parameter loading and validation
- Camera model detection
- Error handling scenarios

### Integration Tests
- End-to-end image capture pipeline
- ROS2 message publishing
- Parameter update mechanism

### Performance Tests
- Frame rate consistency
- Memory usage profiling
- CPU usage monitoring

## Quality Assurance

### Code Standards
- Google C++ Style Guide compliance
- Comprehensive documentation
- Static analysis with clang-tidy

### Validation Criteria
- Successful operation with both D405 and D435
- Stable 30+ FPS performance
- Clean shutdown without memory leaks
- Parameter changes without restart

## Documentation Requirements

### User Documentation
- Installation guide
- Configuration tutorial
- Troubleshooting guide
- API reference

### Developer Documentation
- Architecture overview
- Contributing guidelines
- Testing procedures
- Release notes

## Deployment Considerations

### Hardware Requirements
- USB 3.0+ port for camera connection
- Minimum 4GB RAM
- Multi-core CPU recommended

### Software Environment
- ROS2 workspace setup
- Proper udev rules for camera access
- Display server for OpenCV visualization

## C# to C++ Migration Status

### Translation Progress
- ✅ **C# Analysis**: Original vision.cs file analyzed and understood
- ✅ **Function Signatures**: Two batteryFrameLocationCompensation overloads implemented
- ✅ **Core Algorithm**: EmguCV to OpenCV translation completed
- ✅ **Integration**: Successfully integrated with ROS2 node callback system
- ✅ **Testing**: Both function overloads tested and working

### Key Differences from C# Original
1. **Language Migration**: C# EmguCV → C++ OpenCV
2. **Memory Management**: Automatic garbage collection → Manual memory management
3. **Error Handling**: C# exceptions → C++ return codes with detailed error enums
4. **Integration**: Standalone C# class → ROS2 integrated C++ namespace
5. **Performance**: Enhanced with robust edge fitting and statistical outlier detection
6. **Calibration**: Added scaling factors (X *= 0.53, Z *= 0.18) for real-world coordinate conversion
7. **Validation**: Specific test images with known coordinates for algorithm validation

## Future Enhancements

### Planned Features
- Support for additional RealSense models (D455, L515)
- Multiple camera support
- Advanced image processing filters
- Web-based configuration interface

### Extensibility Points
- Plugin architecture for new camera models
- Configurable image processing pipeline
- Custom display overlays
- Integration with robot navigation stack

## Usage Examples

### Battery Frame Compensation Commands

#### Real-time Compensation
```bash
# Trigger real-time battery frame location compensation
ros2 topic pub --once /detection_cmd std_msgs/String "data: 'start_detect'"
```
This command will:
1. Capture current image from RealSense camera
2. Compare with golden reference image
3. Calculate X and Z offsets with calibration coefficients (X *= 0.53, Z *= 0.18)
4. Publish results to `/compensate_pose` topic as Float32MultiArray [X, Z]
5. Log results or error messages

**Subscribing to Results:**
```python
def compensate_pose_callback(self, msg: Float32MultiArray):
    if len(msg.data) >= 2:
        X, Z = msg.data[:2]
        self.get_logger().info(f"Received compensation: X={X:.2f}, Z={Z:.2f}")
```

#### Static File-based Compensation
```bash
# Trigger static file-based battery frame location compensation
ros2 topic pub --once /detection_cmd std_msgs/String "data: 'test_golden'"
```
This command will:
1. Load two pre-existing fixed image files with known coordinates
2. Uses golden reference: `/home/harveywang/image_sample/2D_golden(x=0,z=112).png`
3. Uses current image: `/home/harveywang/image_sample/2D(x=25,z=117).png`
4. Perform compensation analysis between the two static files
5. Apply calibration coefficients: X_out *= 0.53, Z_out *= 0.18
6. Calculate X and Z offsets with expected result: X≈25*0.53=13.25, Z≈5*0.18=0.9
7. Log results or error messages

**Important**: This mode uses two fixed static image files (非動態取圖), not dynamic camera capture. The system processes pre-existing files with known coordinate differences for validation.

#### Expected Output

**For start_detect command:**
```
[INFO] [realsense_multi_camera_node]: Image captured for real-time detection
[INFO] [realsense_multi_camera_node]: [start_detect] Battery frame compensation successful - X_offset: 12.45, Z_offset: -8.32
# Published to /compensate_pose topic: [12.45, -8.32]
```

**For test_golden command:**
```
[INFO] [realsense_multi_camera_node]: [test_golden] Testing 2 fixed images - No camera capture involved
[INFO] [realsense_multi_camera_node]: Fixed Image 1 (Golden): 2D_golden(x=0,z=112).png
[INFO] [realsense_multi_camera_node]: Fixed Image 2 (Current): 2D(x=25,z=117).png
[INFO] [realsense_multi_camera_node]: [test_golden] Static file compensation successful - X_offset: 13.25, Z_offset: 0.90
# No publishing to /compensate_pose for test_golden - validation only
```

or in case of errors:
```
[WARN] [realsense_multi_camera_node]: Battery frame compensation failed with error code: -10
[ERROR] [realsense_multi_camera_node]: Error: Could not read golden reference image
```