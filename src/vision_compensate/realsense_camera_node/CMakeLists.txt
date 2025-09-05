cmake_minimum_required(VERSION 3.8)
project(realsense_camera_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../src/camera_compensate)

# Find RealSense libraries from ROS2 installation
find_library(REALSENSE2_LIBRARY
    NAMES realsense2
    PATHS /opt/ros/humble/lib/x86_64-linux-gnu
    REQUIRED
)

# Include RealSense headers from ROS2 installation
include_directories(/opt/ros/humble/include)

# Create executable
add_executable(realsense_camera_node
  src/realsense_camera_node.cpp
  ../src/camera_compensate/vision.cpp
)

# Link dependencies
ament_target_dependencies(realsense_camera_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Link OpenCV and RealSense libraries
target_link_libraries(realsense_camera_node
  ${OpenCV_LIBRARIES}
  ${REALSENSE2_LIBRARY}
)

# Install targets
install(TARGETS
  realsense_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()