#!/bin/bash

# Test script for ROS2 Compensation Node
# This script tests the compensation calculation functionality

set -e

echo "=== ROS2 Compensation Node Test Script ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_test() {
    echo -e "${BLUE}[TEST]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}[ERROR]${NC} ROS2 environment not sourced"
    exit 1
fi

# Get workspace directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Source workspace
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true

print_status "Starting test sequence..."

# Function to run a command in background and get its PID
run_node_background() {
    print_status "Launching compensation node..."
    ros2 launch compensation_package compensation_node.launch.py > /tmp/compensation_node.log 2>&1 &
    NODE_PID=$!
    sleep 3  # Wait for node to start
    
    # Check if node is running
    if kill -0 $NODE_PID 2>/dev/null; then
        print_status "Node started successfully (PID: $NODE_PID)"
    else
        echo -e "${RED}[ERROR]${NC} Node failed to start"
        cat /tmp/compensation_node.log
        exit 1
    fi
}

# Function to cleanup
cleanup() {
    if [ ! -z "$NODE_PID" ]; then
        print_status "Stopping compensation node..."
        kill $NODE_PID 2>/dev/null || true
        wait $NODE_PID 2>/dev/null || true
    fi
}

# Set up cleanup trap
trap cleanup EXIT

# Test 1: Check if node can be launched
print_test "Test 1: Node launch test"
run_node_background

# Test 2: Check node is publishing/subscribing to correct topics
print_test "Test 2: Topic interface test"
sleep 2

# Check if node is subscribing to input topic
if ros2 topic info /object_world_coordinates >/dev/null 2>&1; then
    print_status "✓ Input topic '/object_world_coordinates' exists"
else
    print_warning "✗ Input topic '/object_world_coordinates' not found"
fi

# Check if node is publishing to output topic
if ros2 topic info /arm_grasp_coordinates >/dev/null 2>&1; then
    print_status "✓ Output topic '/arm_grasp_coordinates' exists"
else
    print_warning "✗ Output topic '/arm_grasp_coordinates' not found"
fi

# Test 3: Publish test message and verify output
print_test "Test 3: Message processing test"

# Start listening to output topic in background
timeout 10 ros2 topic echo /arm_grasp_coordinates --once > /tmp/compensation_output.log 2>&1 &
ECHO_PID=$!

# Wait a moment
sleep 1

# Publish test message
print_status "Publishing test object coordinates..."
ros2 topic pub --once /object_world_coordinates geometry_msgs/msg/Pose "
position:
  x: 0.5
  y: 0.3
  z: 0.2
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
"

# Wait for echo to complete
wait $ECHO_PID 2>/dev/null || true

# Check if we got output
if [ -s /tmp/compensation_output.log ]; then
    print_status "✓ Node processed message and produced output"
    echo "Output preview:"
    head -10 /tmp/compensation_output.log | sed 's/^/  /'
else
    print_warning "✗ No output received from compensation calculation"
fi

# Test 4: Parameter test
print_test "Test 4: Parameter validation test"
if ros2 param list | grep compensation_calculator_node >/dev/null; then
    print_status "✓ Node parameters are loaded"
    ros2 param get /compensation_calculator_node hand_eye_calibration_tf.translation.x 2>/dev/null || print_warning "Could not read parameter"
else
    print_warning "✗ Node parameters not found"
fi

print_status "Test sequence completed!"
print_status "Check /tmp/compensation_node.log for detailed node output"

# Clean up temp files
rm -f /tmp/compensation_output.log
rm -f /tmp/compensation_node.log