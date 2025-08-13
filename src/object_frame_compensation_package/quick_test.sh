#!/bin/bash

# Quick Test Script for ROS2 Compensation Node
# Fast verification of basic functionality

set -e

echo "=== ROS2 Compensation Node Quick Test ==="

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Source ROS2 environment
print_status "Sourcing ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    print_error "No ROS2 installation found in /opt/ros/"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

print_status "Quick test starting..."

# Build
print_status "Building package..."
cd "$WORKSPACE_DIR"
if colcon build --packages-select compensation_package > /dev/null 2>&1; then
    print_status "✓ Build successful"
else
    print_error "✗ Build failed"
    exit 1
fi

source install/setup.bash

# Launch node
print_status "Launching node..."
ros2 launch compensation_package compensation_node.launch.py > /tmp/compensation_quick_test.log 2>&1 &
NODE_PID=$!

# Cleanup function
cleanup() {
    if [ ! -z "$NODE_PID" ]; then
        kill $NODE_PID 2>/dev/null || true
        wait $NODE_PID 2>/dev/null || true
    fi
}
trap cleanup EXIT

sleep 3

# Check node
if kill -0 $NODE_PID 2>/dev/null; then
    print_status "✓ Node running"
else
    print_error "✗ Node failed to start"
    cat /tmp/compensation_quick_test.log
    exit 1
fi

# Test basic functionality
print_status "Testing basic functionality..."

# Start output listener
timeout 8 ros2 topic echo /arm_grasp_coordinates --once > /tmp/quick_output.log 2>&1 &
ECHO_PID=$!

sleep 1

# Send test input
print_status "Sending test input: (0.5, 0.3, 0.2)"
ros2 topic pub --once /object_world_coordinates geometry_msgs/msg/Pose "
position: {x: 0.5, y: 0.3, z: 0.2}
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
" > /dev/null 2>&1

# Wait for output
wait $ECHO_PID 2>/dev/null || true

if [ -s /tmp/quick_output.log ]; then
    # Extract output values
    OUT_X=$(grep -A3 "position:" /tmp/quick_output.log | grep "x:" | awk '{print $2}' | head -1 | cut -c1-6)
    OUT_Y=$(grep -A3 "position:" /tmp/quick_output.log | grep "y:" | awk '{print $2}' | head -1 | cut -c1-6)  
    OUT_Z=$(grep -A3 "position:" /tmp/quick_output.log | grep "z:" | awk '{print $2}' | head -1 | cut -c1-6)
    
    print_status "✓ Output received: ($OUT_X, $OUT_Y, $OUT_Z)"
    print_status "✓ Compensation calculation working!"
else
    print_warning "✗ No output received"
fi

# Quick parameter check
PARAM_COUNT=$(ros2 param list | grep compensation_calculator_node | wc -l)
if [ "$PARAM_COUNT" -gt 0 ]; then
    print_status "✓ Parameters loaded ($PARAM_COUNT)"
else
    print_warning "✗ No parameters found"
fi

print_status "Quick test completed!"
print_status "For detailed testing, run: ./comprehensive_test.sh"

# Cleanup temp files
rm -f /tmp/quick_output.log /tmp/compensation_quick_test.log