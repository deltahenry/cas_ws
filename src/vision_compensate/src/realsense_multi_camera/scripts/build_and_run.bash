#!/bin/bash

# RealSense Multi Camera - Build and Run Script
# This script builds and runs the RealSense multi-camera node with various options

set -e

echo "=== RealSense Multi Camera Build and Run Script ==="

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

# Default parameters
MODEL="D405"
WIDTH=640
HEIGHT=480
FPS=30
SHOW_DISPLAY=true
DEBUG_MODE=false
BUILD_ONLY=false
RUN_ONLY=false
CLEAN_BUILD=false
VERBOSE=false

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --model MODEL           Camera model (D405, D435, auto) [default: D405]"
    echo "  --width WIDTH           Image width [default: 640]"
    echo "  --height HEIGHT         Image height [default: 480]"
    echo "  --fps FPS              Frame rate [default: 30]"
    echo "  --no-display           Disable OpenCV display window"
    echo "  --debug                Enable debug mode"
    echo "  --build-only           Only build, don't run"
    echo "  --run-only             Only run (skip build)"
    echo "  --clean                Clean build before building"
    echo "  --verbose              Verbose output"
    echo "  --help                 Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Build and run with default settings"
    echo "  $0 --model D435 --fps 60             # Run with D435 camera at 60 FPS"
    echo "  $0 --clean --build-only              # Clean build only"
    echo "  $0 --run-only --no-display           # Run without display"
    echo "  $0 --debug --verbose                 # Debug mode with verbose output"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --model)
            MODEL="$2"
            shift 2
            ;;
        --width)
            WIDTH="$2"
            shift 2
            ;;
        --height)
            HEIGHT="$2"
            shift 2
            ;;
        --fps)
            FPS="$2"
            shift 2
            ;;
        --no-display)
            SHOW_DISPLAY=false
            shift
            ;;
        --debug)
            DEBUG_MODE=true
            shift
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --run-only)
            RUN_ONLY=true
            shift
            ;;
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Validate parameters
if [[ ! "$MODEL" =~ ^(D405|D435|auto)$ ]]; then
    print_error "Invalid model: $MODEL. Must be D405, D435, or auto"
    exit 1
fi

if [[ ! "$WIDTH" =~ ^[0-9]+$ ]] || [ "$WIDTH" -le 0 ]; then
    print_error "Invalid width: $WIDTH. Must be a positive integer"
    exit 1
fi

if [[ ! "$HEIGHT" =~ ^[0-9]+$ ]] || [ "$HEIGHT" -le 0 ]; then
    print_error "Invalid height: $HEIGHT. Must be a positive integer"
    exit 1
fi

if [[ ! "$FPS" =~ ^[0-9]+$ ]] || [ "$FPS" -le 0 ] || [ "$FPS" -gt 120 ]; then
    print_error "Invalid FPS: $FPS. Must be between 1 and 120"
    exit 1
fi

# Function to check dependencies
check_dependencies() {
    print_info "Checking dependencies..."
    
    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please install ROS2 first."
        exit 1
    fi
    
    # Check colcon
    if ! command -v colcon &> /dev/null; then
        print_error "colcon not found. Please install: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    
    # Check RealSense SDK
    if ! ldconfig -p | grep -q librealsense2; then
        print_error "RealSense SDK not found. Please run the setup_environment.bash script first."
        exit 1
    fi
    
    print_success "All dependencies found"
}

# Function to check for RealSense cameras
check_cameras() {
    print_info "Checking for RealSense cameras..."
    
    if command -v rs-enumerate-devices &> /dev/null; then
        device_count=$(rs-enumerate-devices | grep -c "Device info:" || true)
        if [ "$device_count" -gt 0 ]; then
            print_success "Found $device_count RealSense device(s)"
            if [ "$VERBOSE" = true ]; then
                rs-enumerate-devices
            fi
        else
            print_warning "No RealSense devices detected."
            print_info "Make sure your camera is connected and you have the necessary permissions."
            print_info "You may need to run: sudo udevadm control --reload-rules && sudo udevadm trigger"
        fi
    else
        print_warning "rs-enumerate-devices not found. Cannot check for cameras."
    fi
}

# Function to source ROS2 environment
source_ros2() {
    print_info "Sourcing ROS2 environment..."
    
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        print_success "ROS2 Humble environment sourced"
    else
        print_error "ROS2 Humble setup script not found"
        exit 1
    fi
}

# Function to build the package
build_package() {
    print_info "Building realsense_multi_camera package..."
    
    # Navigate to workspace root
    CURRENT_DIR=$(pwd)
    while [[ "$CURRENT_DIR" != "/" && ! -f "$CURRENT_DIR/src/realsense_multi_camera/package.xml" ]]; do
        CURRENT_DIR=$(dirname "$CURRENT_DIR")
    done
    
    if [ "$CURRENT_DIR" = "/" ]; then
        print_error "Could not find workspace root with realsense_multi_camera package"
        exit 1
    fi
    
    cd "$CURRENT_DIR"
    print_info "Workspace root: $CURRENT_DIR"
    
    # Clean build if requested
    if [ "$CLEAN_BUILD" = true ]; then
        print_info "Cleaning build directories..."
        rm -rf build install log
        print_success "Build directories cleaned"
    fi
    
    # Build command options
    BUILD_ARGS="--packages-select realsense_multi_camera"
    
    if [ "$DEBUG_MODE" = true ]; then
        BUILD_ARGS="$BUILD_ARGS --cmake-args -DCMAKE_BUILD_TYPE=Debug"
        print_info "Building in debug mode"
    fi
    
    if [ "$VERBOSE" = true ]; then
        BUILD_ARGS="$BUILD_ARGS --event-handlers console_direct+"
    fi
    
    # Build the package
    print_info "Running: colcon build $BUILD_ARGS"
    if ! colcon build $BUILD_ARGS; then
        print_error "Build failed"
        exit 1
    fi
    
    print_success "Build completed successfully"
    
    # Source the local workspace
    if [ -f install/setup.bash ]; then
        source install/setup.bash
        print_success "Local workspace sourced"
    else
        print_warning "install/setup.bash not found. Build may have failed silently."
    fi
}

# Function to run the node
run_node() {
    print_info "Running RealSense Multi Camera Node..."
    print_info "Configuration:"
    print_info "  Model: $MODEL"
    print_info "  Resolution: ${WIDTH}x${HEIGHT}"
    print_info "  FPS: $FPS"
    print_info "  Display: $SHOW_DISPLAY"
    print_info "  Debug: $DEBUG_MODE"
    
    # Prepare launch arguments
    LAUNCH_ARGS="model:=$MODEL width:=$WIDTH height:=$HEIGHT fps:=$FPS show_display:=$SHOW_DISPLAY"
    
    print_info "Launching with arguments: $LAUNCH_ARGS"
    print_info "Press Ctrl+C to stop"
    echo ""
    
    # Launch the node
    if [ "$DEBUG_MODE" = true ]; then
        export RCUTILS_LOGGING_SEVERITY=DEBUG
    fi
    
    ros2 launch realsense_multi_camera realsense_camera.launch.py $LAUNCH_ARGS
}

# Main execution
print_info "Starting RealSense Multi Camera build and run process"
print_info "Configuration: Model=$MODEL, ${WIDTH}x${HEIGHT}@${FPS}fps, Display=$SHOW_DISPLAY"

# Check dependencies (unless run-only mode)
if [ "$RUN_ONLY" = false ]; then
    check_dependencies
fi

# Source ROS2 environment
source_ros2

# Check for cameras
check_cameras

# Build phase
if [ "$RUN_ONLY" = false ]; then
    build_package
fi

# Run phase
if [ "$BUILD_ONLY" = false ]; then
    echo ""
    print_info "Starting node in 3 seconds... (Press Ctrl+C to cancel)"
    sleep 3
    run_node
fi

print_success "Script completed successfully"