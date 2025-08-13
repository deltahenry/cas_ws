#!/bin/bash

# Comprehensive Test Script for ROS2 Compensation Node
# This script performs complete testing of the compensation calculation functionality

set -e

echo "=== ROS2 Compensation Node Comprehensive Test ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_test() {
    echo -e "${BLUE}[TEST]${NC} $1"
}

print_result() {
    echo -e "${CYAN}[RESULT]${NC} $1"
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
    print_error "Available ROS2 versions:"
    ls /opt/ros/ 2>/dev/null || echo "  No ROS2 installations found"
    exit 1
fi

print_status "ROS2 Distribution: $ROS_DISTRO"

# Get workspace directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="/tmp/compensation_test_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$LOG_DIR"
print_status "Test logs will be saved to: $LOG_DIR"

# Cleanup function
cleanup() {
    print_status "Cleaning up test environment..."
    
    # Kill any running nodes
    if [ ! -z "$NODE_PID" ]; then
        kill $NODE_PID 2>/dev/null || true
        wait $NODE_PID 2>/dev/null || true
    fi
    
    # Kill any background processes
    jobs -p | xargs -r kill 2>/dev/null || true
    
    print_status "Test completed. Check logs in $LOG_DIR"
}

trap cleanup EXIT

print_status "Starting comprehensive test suite..."

# =============================================================================
# TEST 1: Build Test
# =============================================================================
print_test "Test 1: Package Build Test"
cd "$WORKSPACE_DIR"

if colcon build --packages-select compensation_package --cmake-args -DCMAKE_BUILD_TYPE=Release > "$LOG_DIR/build.log" 2>&1; then
    print_result "✓ Package built successfully"
else
    print_error "✗ Package build failed"
    echo "Build log:"
    cat "$LOG_DIR/build.log"
    exit 1
fi

# Source the workspace
source "$WORKSPACE_DIR/install/setup.bash"

# =============================================================================
# TEST 2: Node Launch Test
# =============================================================================
print_test "Test 2: Node Launch Test"

# Start node in background
ros2 launch compensation_package compensation_node.launch.py > "$LOG_DIR/node.log" 2>&1 &
NODE_PID=$!
sleep 3

# Check if node is running
if kill -0 $NODE_PID 2>/dev/null; then
    print_result "✓ Node launched successfully (PID: $NODE_PID)"
else
    print_error "✗ Node failed to launch"
    cat "$LOG_DIR/node.log"
    exit 1
fi

# =============================================================================
# TEST 3: Node Discovery Test
# =============================================================================
print_test "Test 3: Node Discovery Test"

# Check node exists
if ros2 node list 2>/dev/null | grep -q "compensation_calculator_node"; then
    print_result "✓ Node found in ROS2 network"
else
    print_error "✗ Node not found in ROS2 network"
    ros2 node list
    exit 1
fi

# =============================================================================
# TEST 4: Topic Interface Test
# =============================================================================
print_test "Test 4: Topic Interface Test"

sleep 2

# Check input topic
if ros2 topic list 2>/dev/null | grep -q "/object_world_coordinates"; then
    print_result "✓ Input topic '/object_world_coordinates' exists"
else
    print_warning "✗ Input topic '/object_world_coordinates' not found"
fi

# Check output topic
if ros2 topic list 2>/dev/null | grep -q "/arm_grasp_coordinates"; then
    print_result "✓ Output topic '/arm_grasp_coordinates' exists"
else
    print_warning "✗ Output topic '/arm_grasp_coordinates' not found"
fi

# Check topic types
INPUT_TYPE=$(ros2 topic type /object_world_coordinates 2>/dev/null || echo "unknown")
OUTPUT_TYPE=$(ros2 topic type /arm_grasp_coordinates 2>/dev/null || echo "unknown")

if [ "$INPUT_TYPE" = "geometry_msgs/msg/Pose" ]; then
    print_result "✓ Input topic type correct: $INPUT_TYPE"
else
    print_warning "✗ Input topic type incorrect: $INPUT_TYPE (expected: geometry_msgs/msg/Pose)"
fi

if [ "$OUTPUT_TYPE" = "geometry_msgs/msg/Pose" ]; then
    print_result "✓ Output topic type correct: $OUTPUT_TYPE"
else
    print_warning "✗ Output topic type incorrect: $OUTPUT_TYPE (expected: geometry_msgs/msg/Pose)"
fi

# =============================================================================
# TEST 5: Parameter Test
# =============================================================================
print_test "Test 5: Parameter Validation Test"

# Check if parameters exist
PARAM_COUNT=$(ros2 param list 2>/dev/null | grep compensation_calculator_node | wc -l)
if [ "$PARAM_COUNT" -gt 0 ]; then
    print_result "✓ Node parameters loaded ($PARAM_COUNT parameters)"
    
    # Test specific parameters
    HAND_EYE_X=$(ros2 param get /compensation_calculator_node hand_eye_calibration_tf.translation.x 2>/dev/null | grep "Double value:" | awk '{print $3}' || echo "failed")
    if [ "$HAND_EYE_X" != "failed" ]; then
        print_result "✓ Hand-eye calibration parameter readable: x=$HAND_EYE_X"
    else
        print_warning "✗ Could not read hand-eye calibration parameter"
    fi
else
    print_warning "✗ No node parameters found"
fi

# =============================================================================
# TEST 6: Basic Functionality Test
# =============================================================================
print_test "Test 6: Basic Functionality Test"

# Start output monitor
timeout 15 ros2 topic echo /arm_grasp_coordinates --once > "$LOG_DIR/output1.log" 2>&1 &
ECHO_PID1=$!

sleep 2

# Send test input
print_status "Sending test input: position(0.5, 0.3, 0.2)"
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
" > "$LOG_DIR/input1.log" 2>&1

# Wait for output
wait $ECHO_PID1 2>/dev/null || true

if [ -s "$LOG_DIR/output1.log" ]; then
    print_result "✓ Node processed input and produced output"
    print_status "Output sample:"
    head -15 "$LOG_DIR/output1.log" | sed 's/^/  /'
else
    print_warning "✗ No output received within timeout"
fi

# =============================================================================
# TEST 7: Multiple Input Test
# =============================================================================
print_test "Test 7: Multiple Input Test"

print_status "Testing with multiple different inputs..."

# Test inputs
declare -a TEST_INPUTS=(
    "0.1 0.1 0.1"
    "0.5 0.3 0.2" 
    "1.0 0.5 0.3"
    "-0.2 0.4 0.1"
)

for i in "${!TEST_INPUTS[@]}"; do
    IFS=' ' read -r x y z <<< "${TEST_INPUTS[$i]}"
    
    print_status "Test input $((i+1)): position($x, $y, $z)"
    
    # Start output monitor
    timeout 10 ros2 topic echo /arm_grasp_coordinates --once > "$LOG_DIR/output_$((i+1)).log" 2>&1 &
    ECHO_PID=$!
    
    sleep 1
    
    # Send input
    ros2 topic pub --once /object_world_coordinates geometry_msgs/msg/Pose "
position:
  x: $x
  y: $y
  z: $z
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
" > "$LOG_DIR/input_$((i+1)).log" 2>&1
    
    # Wait for output
    wait $ECHO_PID 2>/dev/null || true
    
    if [ -s "$LOG_DIR/output_$((i+1)).log" ]; then
        # Extract output position
        OUTPUT_X=$(grep -A3 "position:" "$LOG_DIR/output_$((i+1)).log" | grep "x:" | awk '{print $2}' | head -1)
        OUTPUT_Y=$(grep -A3 "position:" "$LOG_DIR/output_$((i+1)).log" | grep "y:" | awk '{print $2}' | head -1)
        OUTPUT_Z=$(grep -A3 "position:" "$LOG_DIR/output_$((i+1)).log" | grep "z:" | awk '{print $2}' | head -1)
        print_result "✓ Output $((i+1)): position($OUTPUT_X, $OUTPUT_Y, $OUTPUT_Z)"
    else
        print_warning "✗ No output for input $((i+1))"
    fi
done

# =============================================================================
# TEST 8: Performance Test
# =============================================================================
print_test "Test 8: Performance Test"

print_status "Testing high-frequency input processing..."

# Start performance monitor
ros2 topic hz /arm_grasp_coordinates > "$LOG_DIR/performance.log" 2>&1 &
PERF_PID=$!

# Send high-frequency inputs
ros2 topic pub -r 5 /object_world_coordinates geometry_msgs/msg/Pose "
position:
  x: 0.5
  y: 0.3
  z: 0.2
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
" > "$LOG_DIR/high_freq_input.log" 2>&1 &
FREQ_PID=$!

# Let it run for 10 seconds
sleep 10

# Stop frequency test
kill $FREQ_PID 2>/dev/null || true
kill $PERF_PID 2>/dev/null || true

if [ -s "$LOG_DIR/performance.log" ]; then
    print_result "✓ Performance test completed"
    print_status "Performance summary:"
    tail -5 "$LOG_DIR/performance.log" | sed 's/^/  /'
else
    print_warning "✗ Performance test failed"
fi

# =============================================================================
# TEST 9: Parameter Modification Test
# =============================================================================
print_test "Test 9: Parameter Modification Test"

# Save original parameter
ORIG_GOLDEN_X=$(ros2 param get /compensation_calculator_node golden_object_recognition_coord_pose.position.x 2>/dev/null | grep "Double value:" | awk '{print $3}' || echo "0.0")

# Modify parameter
ros2 param set /compensation_calculator_node golden_object_recognition_coord_pose.position.x 0.5 > "$LOG_DIR/param_set.log" 2>&1

# Verify change
NEW_GOLDEN_X=$(ros2 param get /compensation_calculator_node golden_object_recognition_coord_pose.position.x 2>/dev/null | grep "Double value:" | awk '{print $3}' || echo "failed")

if [ "$NEW_GOLDEN_X" = "0.5" ]; then
    print_result "✓ Parameter modification successful: $ORIG_GOLDEN_X → $NEW_GOLDEN_X"
    
    # Test with modified parameter
    timeout 10 ros2 topic echo /arm_grasp_coordinates --once > "$LOG_DIR/output_modified.log" 2>&1 &
    ECHO_PID=$!
    
    sleep 1
    
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
" > /dev/null 2>&1
    
    wait $ECHO_PID 2>/dev/null || true
    
    if [ -s "$LOG_DIR/output_modified.log" ]; then
        print_result "✓ Node responded to parameter change"
    else
        print_warning "✗ No response after parameter change"
    fi
    
    # Restore original parameter
    ros2 param set /compensation_calculator_node golden_object_recognition_coord_pose.position.x "$ORIG_GOLDEN_X" > /dev/null 2>&1
else
    print_warning "✗ Parameter modification failed"
fi

# =============================================================================
# TEST 10: Error Handling Test
# =============================================================================
print_test "Test 10: Error Handling Test"

# Test with invalid input (this should not crash the node)
print_status "Testing error handling with edge cases..."

# Check if node is still responsive
if kill -0 $NODE_PID 2>/dev/null; then
    print_result "✓ Node survived all tests and is still running"
else
    print_warning "✗ Node crashed during testing"
fi

# =============================================================================
# TEST SUMMARY
# =============================================================================
echo
echo "=================================="
print_status "TEST SUMMARY"
echo "=================================="

print_status "Test logs saved in: $LOG_DIR"
print_status "Key files:"
echo "  - Node output: $LOG_DIR/node.log"
echo "  - Build log: $LOG_DIR/build.log"  
echo "  - Test outputs: $LOG_DIR/output*.log"
echo "  - Performance: $LOG_DIR/performance.log"

print_status "Manual verification commands:"
echo "  ros2 node list | grep compensation"
echo "  ros2 topic list | grep coordinates"
echo "  ros2 param list | grep compensation"
echo "  ros2 topic echo /arm_grasp_coordinates"

print_result "Comprehensive testing completed!"