#!/bin/bash

# RealSense Camera Node - Build and Run Script
# This script automatically compiles and executes the RealSense camera node

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"
WORKSPACE_DIR="$( cd "$PROJECT_DIR/.." &> /dev/null && pwd )"

echo -e "${BLUE}=== RealSense Multi-Camera Node - Build and Run Script ===${NC}"
echo "Project directory: $PROJECT_DIR"
echo "Workspace directory: $WORKSPACE_DIR"

# Function to print status
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ROS2 is sourced
check_ros2_environment() {
    print_status "Checking ROS2 environment..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS2 not sourced. Attempting to source..."
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
            print_status "ROS2 Humble sourced successfully"
        else
            print_error "ROS2 installation not found. Please install ROS2 Humble."
            exit 1
        fi
    else
        print_status "ROS2 $ROS_DISTRO is already sourced"
    fi
}

# Check dependencies
check_dependencies() {
    print_status "Checking dependencies..."
    
    # Check for RealSense SDK (either system or ROS2)
    if ! pkg-config --exists realsense2; then
        if [ ! -f "/opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so" ]; then
            print_error "RealSense SDK not found. Run: sudo apt install librealsense2-dev"
            print_error "Or run: ./scripts/setup_dependencies.bash"
            exit 1
        else
            print_status "RealSense SDK found in ROS2 installation"
        fi
    else
        print_status "RealSense SDK found via pkg-config"
    fi
    
    # Check for OpenCV
    if ! pkg-config --exists opencv4; then
        print_error "OpenCV not found. Run: sudo apt install libopencv-dev"
        print_error "Or run: ./scripts/setup_dependencies.bash"
        exit 1
    fi
    
    print_status "All dependencies found"
}

# Build the package
build_package() {
    print_status "Building package..."
    
    cd "$WORKSPACE_DIR"
    
    # Source ROS2 if workspace setup exists
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        print_status "Workspace environment sourced"
    fi
    
    # Build the specific package
    if colcon build --packages-select realsense_camera_node --cmake-args -DCMAKE_BUILD_TYPE=Release; then
        print_status "Build completed successfully"
    else
        print_error "Build failed"
        exit 1
    fi
    
    # Source the built package
    source install/setup.bash
    print_status "Package environment sourced"
}

# Check for connected RealSense cameras
check_realsense_cameras() {
    print_status "Checking for connected RealSense cameras..."
    
    if command -v rs-enumerate-devices &> /dev/null; then
        echo "Connected RealSense devices:"
        rs-enumerate-devices || print_warning "No RealSense cameras detected"
    else
        print_warning "rs-enumerate-devices not found. Cannot check for cameras."
    fi
}

# Run the node
run_node() {
    print_status "Starting RealSense multi-camera node..."
    
    cd "$WORKSPACE_DIR"
    
    # Parse command line arguments
    SHOW_DISPLAY="true"
    NODE_ARGS=""
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --no-display)
                SHOW_DISPLAY="false"
                shift
                ;;
            --debug)
                NODE_ARGS="--ros-args --log-level debug"
                shift
                ;;
            *)
                print_warning "Unknown argument: $1"
                shift
                ;;
        esac
    done
    
    print_status "Launch arguments: show_display:=$SHOW_DISPLAY"
    
    # Launch the node
    ros2 launch realsense_camera_node realsense_camera.launch.py \
        show_display:=$SHOW_DISPLAY \
        $NODE_ARGS
}

# Parse command line options
SKIP_BUILD=false
HELP=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --help|-h)
            HELP=true
            shift
            ;;
        *)
            break
            ;;
    esac
done

# Show help
if [ "$HELP" = true ]; then
    echo "Usage: $0 [OPTIONS] [NODE_OPTIONS]"
    echo ""
    echo "OPTIONS:"
    echo "  --skip-build     Skip the build process"
    echo "  --help, -h       Show this help message"
    echo ""
    echo "NODE_OPTIONS:"
    echo "  --no-display     Disable OpenCV display window"
    echo "  --debug          Enable debug logging"
    echo ""
    echo "Examples:"
    echo "  $0                    # Build and run with display"
    echo "  $0 --no-display      # Build and run without display"
    echo "  $0 --skip-build      # Skip build and run"
    echo "  $0 --debug           # Enable debug logging"
    exit 0
fi

# Main execution
main() {
    check_ros2_environment
    check_dependencies
    check_realsense_cameras
    
    if [ "$SKIP_BUILD" = false ]; then
        build_package
    else
        print_status "Skipping build process"
        cd "$WORKSPACE_DIR"
        if [ -f "install/setup.bash" ]; then
            source install/setup.bash
        fi
    fi
    
    run_node "$@"
}

# Run main function with all arguments
main "$@"