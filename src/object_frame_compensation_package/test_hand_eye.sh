#!/bin/bash

# Simple test script for hand-eye compensation node

echo "=== Hand-Eye Compensation Node Test ==="

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    print_error "ROS2 Humble not found"
    exit 1
fi

# Source workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"
source install/setup.bash

print_status "Testing hand-eye compensation node..."

# Start node in background
ros2 run hand_eye_compensation_package hand_eye_compensation_node > /tmp/hand_eye_test.log 2>&1 &
NODE_PID=$!

cleanup() {
    if [ ! -z "$NODE_PID" ]; then
        kill $NODE_PID 2>/dev/null || true
        wait $NODE_PID 2>/dev/null || true
    fi
}
trap cleanup EXIT

sleep 2

# Check if node is running
if kill -0 $NODE_PID 2>/dev/null; then
    print_status "✓ Node started successfully"
else
    print_error "✗ Node failed to start"
    cat /tmp/hand_eye_test.log
    exit 1
fi

# Test basic functionality
print_status "Testing topics..."

# Check topics exist
if ros2 topic list 2>/dev/null | grep -q "hand_eye_compensation_node/object_camera_pose"; then
    print_status "✓ Input topic exists"
else
    print_warning "✗ Input topic not found"
fi

if ros2 topic list 2>/dev/null | grep -q "hand_eye_compensation_node/target_arm_pose"; then
    print_status "✓ Output topic exists"
else
    print_warning "✗ Output topic not found"
fi

# Test parameter loading
PARAM_COUNT=$(ros2 param list 2>/dev/null | grep hand_eye_compensation_node | wc -l)
if [ "$PARAM_COUNT" -gt 0 ]; then
    print_status "✓ Parameters loaded ($PARAM_COUNT parameters)"
else
    print_warning "✗ No parameters found"
fi

# Send test message
print_status "Sending test object pose..."

timeout 5 ros2 topic echo /hand_eye_compensation_node/target_arm_pose --once > /tmp/hand_eye_output.log 2>&1 &
ECHO_PID=$!

sleep 1

ros2 topic pub --once /hand_eye_compensation_node/object_camera_pose geometry_msgs/Pose "
position:
  x: 0.1
  y: 0.05
  z: 0.3
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
" > /dev/null 2>&1

wait $ECHO_PID 2>/dev/null || true

if [ -s /tmp/hand_eye_output.log ]; then
    OUT_X=$(grep -A3 "position:" /tmp/hand_eye_output.log | grep "x:" | awk '{print $2}' | head -1 | cut -c1-6)
    OUT_Y=$(grep -A3 "position:" /tmp/hand_eye_output.log | grep "y:" | awk '{print $2}' | head -1 | cut -c1-6)  
    OUT_Z=$(grep -A3 "position:" /tmp/hand_eye_output.log | grep "z:" | awk '{print $2}' | head -1 | cut -c1-6)
    
    print_status "✓ Compensation calculated: target($OUT_X, $OUT_Y, $OUT_Z)"
else
    print_warning "✗ No compensation output received"
fi

print_status "Basic test completed!"

# Cleanup
rm -f /tmp/hand_eye_test.log /tmp/hand_eye_output.log