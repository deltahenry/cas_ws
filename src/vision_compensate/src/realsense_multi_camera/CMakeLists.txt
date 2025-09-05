cmake_minimum_required(VERSION 3.8)
project(realsense_multi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(realsense_multi_camera_node
  src/realsense_multi_camera_node.cpp
)

# Target dependencies
ament_target_dependencies(realsense_multi_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
  realsense2
)

# Link OpenCV
target_link_libraries(realsense_multi_camera_node ${OpenCV_LIBS})

# Install targets
install(TARGETS
  realsense_multi_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install scripts
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts/
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()