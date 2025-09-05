#!/bin/bash

# RealSense Multi Camera - Environment Setup Script
# This script sets up the environment and dependencies for the RealSense multi-camera node

set -e

echo "=== RealSense Multi Camera Environment Setup ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "This script should not be run as root"
   exit 1
fi

# Check Ubuntu version
print_info "Checking Ubuntu version..."
ubuntu_version=$(lsb_release -rs)
if [[ "$ubuntu_version" != "22.04" ]]; then
    print_warning "This script is optimized for Ubuntu 22.04. Current version: $ubuntu_version"
    print_info "Continuing anyway, but you may encounter issues..."
fi

# Check ROS2 installation
print_info "Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 not found. Please install ROS2 Humble first."
    print_info "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
else
    print_success "ROS2 found"
fi

# Source ROS2 environment
print_info "Sourcing ROS2 environment..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    print_success "ROS2 Humble environment sourced"
else
    print_error "ROS2 Humble setup script not found"
    exit 1
fi

# Update package list
print_info "Updating package list..."
sudo apt update

# Install system dependencies
print_info "Installing system dependencies..."

# Basic development tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    curl \
    wget

# OpenCV dependencies
print_info "Installing OpenCV dependencies..."
sudo apt install -y \
    libopencv-dev \
    python3-opencv

# Install Intel RealSense SDK
print_info "Installing Intel RealSense SDK..."

# Register the server's public key
if ! apt-key list | grep -q "Intel"; then
    print_info "Adding Intel RealSense public key..."
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
fi

# Add the repository
if ! grep -q "librealsense" /etc/apt/sources.list.d/librealsense.list 2>/dev/null; then
    print_info "Adding Intel RealSense repository..."
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo jammy main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt update
fi

# Install RealSense packages
print_info "Installing RealSense packages..."
sudo apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# Install ROS2 dependencies
print_info "Installing ROS2 dependencies..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-sensor-msgs \
    ros-humble-ament-cmake \
    ros-humble-rclcpp

# Set up udev rules for RealSense cameras
print_info "Setting up udev rules for RealSense cameras..."
sudo cp /usr/lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to video group (for camera access)
print_info "Adding user to video group..."
sudo usermod -a -G video $USER

# Verify RealSense installation
print_info "Verifying RealSense installation..."
if command -v realsense-viewer &> /dev/null; then
    print_success "RealSense SDK installed successfully"
    print_info "You can test with: realsense-viewer"
else
    print_warning "RealSense viewer not found, but SDK may still be installed"
fi

# Check for connected RealSense devices
print_info "Checking for connected RealSense devices..."
if command -v rs-enumerate-devices &> /dev/null; then
    device_count=$(rs-enumerate-devices | grep -c "Device info:" || true)
    if [ "$device_count" -gt 0 ]; then
        print_success "Found $device_count RealSense device(s)"
        rs-enumerate-devices
    else
        print_warning "No RealSense devices detected. Make sure camera is connected."
    fi
else
    print_warning "rs-enumerate-devices not found"
fi

# Create workspace directories if they don't exist
WORKSPACE_DIR="$HOME/ros2_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    print_info "Creating ROS2 workspace at $WORKSPACE_DIR..."
    mkdir -p "$WORKSPACE_DIR/src"
fi

# Setup environment variables
print_info "Setting up environment variables..."
echo "# RealSense Multi Camera Environment" >> ~/.bashrc
echo "export REALSENSE_MULTI_CAMERA_PATH=$PWD" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Check display server (for OpenCV visualization)
if [ -n "$DISPLAY" ]; then
    print_success "Display server detected: $DISPLAY"
elif [ -n "$WAYLAND_DISPLAY" ]; then
    print_success "Wayland display detected: $WAYLAND_DISPLAY"
else
    print_warning "No display server detected. OpenCV visualization may not work."
    print_info "If running over SSH, use: ssh -X username@hostname"
fi

# Final setup instructions
print_success "Environment setup completed successfully!"
echo ""
print_info "Next steps:"
echo "  1. Log out and log back in (or run: newgrp video)"
echo "  2. Source your ROS2 environment: source /opt/ros/humble/setup.bash"
echo "  3. Build the package: colcon build --packages-select realsense_multi_camera"
echo "  4. Test the installation: ros2 launch realsense_multi_camera realsense_camera.launch.py"
echo ""
print_info "Troubleshooting:"
echo "  - If camera not detected: Check USB connection and run 'rs-enumerate-devices'"
echo "  - If permission denied: Make sure you're in the video group and udev rules are applied"
echo "  - For build issues: Check that all dependencies are installed"
echo ""
print_success "Setup complete! Happy coding!"