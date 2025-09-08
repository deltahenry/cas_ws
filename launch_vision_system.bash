#!/bin/bash

# Launch script for ROS2 Vision Compensation System
# This script sources ROS2, builds the workspace, and launches both nodes

echo "=== ROS2 Vision Compensation System Launch Script ==="

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Source workspace
echo "Sourcing workspace..."
source install/setup.bash

# Build packages (optional - uncomment if you want to build on every launch)
# echo "Building packages..."
# colcon build --packages-select realsense_camera_driver vision_compensation

echo "Launching ROS2 Vision Compensation System..."
echo "Press Ctrl+C to stop all nodes"

# Launch both nodes in the background
echo "Starting RealSense camera driver..."
ros2 launch realsense_camera_driver realsense_camera.launch.py &
CAMERA_PID=$!

# Wait a moment for camera to initialize
sleep 2

echo "Starting vision compensation node..."
ros2 launch vision_compensation vision_compensation.launch.py &
VISION_PID=$!

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Stopping all nodes..."
    kill $CAMERA_PID 2>/dev/null
    kill $VISION_PID 2>/dev/null
    wait
    echo "All nodes stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo ""
echo "=== System Status ==="
echo "Camera Driver PID: $CAMERA_PID"
echo "Vision Compensation PID: $VISION_PID"
echo ""
echo "Available commands to test:"
echo "  ros2 topic pub /detection_cmd std_msgs/String 'data: start_detect'"
echo "  ros2 topic pub /detection_cmd std_msgs/String 'data: start_detect_0'"
echo "  ros2 topic pub /detection_cmd std_msgs/String 'data: start_test'"
echo ""
echo "Monitor topics:"
echo "  ros2 topic echo /camera_image"
echo "  ros2 topic echo /compensate_pose"
echo ""
echo "Press Ctrl+C to stop all nodes"

# Wait for processes to finish
wait