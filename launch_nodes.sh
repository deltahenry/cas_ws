#!/bin/bash

# Launch ROS2 nodes in separate terminals

# Launch realsense camera driver
gnome-terminal --title="RealSense Camera Driver" -- bash -c "
cd /home/harveywang/ros2/cas_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run realsense_camera_driver realsense_camera_node
exec bash"

# Wait a moment before launching the next node
sleep 2

# Launch vision compensation node
gnome-terminal --title="Vision Compensation" -- bash -c "
cd /home/harveywang/ros2/cas_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vision_compensation vision_compensation_node
exec bash"

# Wait a moment before launching the image display node
sleep 2

# Launch show image node
gnome-terminal --title="Show Images" -- bash -c "
cd /home/harveywang/ros2/cas_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run show_image_node show_image_node
exec bash"

echo "Launched ROS2 nodes in separate terminals"