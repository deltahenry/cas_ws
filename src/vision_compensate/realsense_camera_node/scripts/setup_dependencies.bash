#!/bin/bash

# RealSense Camera Node - Dependency Setup Script
# This script installs the necessary dependencies for the RealSense camera node

set -e

echo "=== RealSense Camera Node - Dependency Setup ==="

# Check if running on Ubuntu
if ! command -v apt &> /dev/null; then
    echo "Error: This script is designed for Ubuntu/Debian systems with apt package manager"
    exit 1
fi

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install RealSense SDK dependencies
echo "Installing RealSense SDK dependencies..."
sudo apt install -y \
    librealsense2-dev \
    librealsense2-utils \
    librealsense2-dkms

# Install OpenCV if not present
echo "Installing OpenCV..."
sudo apt install -y \
    libopencv-dev \
    libopencv-contrib-dev

# Install ROS2 dependencies
echo "Installing ROS2 dependencies..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs

# Install build tools
echo "Installing build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config

echo "=== Dependencies installation completed ==="
echo "You can now run the build script: ./scripts/build_and_run.bash"