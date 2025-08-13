#!/bin/bash

# ROS2 Compensation Node Execution Script
# This script builds and runs the compensation calculation node

set -e  # Exit on any error

echo "=== ROS2 Compensation Node Setup and Execution ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Source ROS2 environment
print_status "Sourcing ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    print_status "ROS2 Humble sourced"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    print_status "ROS2 Foxy sourced"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    print_status "ROS2 Galactic sourced"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
    print_status "ROS2 Iron sourced"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    print_status "ROS2 Jazzy sourced"
else
    print_error "No ROS2 installation found in /opt/ros/"
    print_error "Please install ROS2 or source manually: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

print_status "ROS2 Distribution: $ROS_DISTRO"

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

print_status "Working in directory: $WORKSPACE_DIR"

# Build the package
print_status "Building object_frame_compensation_package..."
cd "$WORKSPACE_DIR"

if colcon build --packages-select object_frame_compensation_package --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    print_status "Build successful!"
else
    print_error "Build failed!"
    exit 1
fi

# Source the workspace
print_status "Sourcing workspace..."
source "$WORKSPACE_DIR/install/setup.bash"

# Check if launch file exists
LAUNCH_FILE="$WORKSPACE_DIR/install/object_frame_compensation_package/share/object_frame_compensation_package/launch/object_frame_compensation.launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    print_error "Launch file not found: $LAUNCH_FILE"
    exit 1
fi

print_status "Starting object frame compensation node..."
print_warning "Press Ctrl+C to stop the node"

# Launch the node
ros2 launch object_frame_compensation_package object_frame_compensation.launch.py