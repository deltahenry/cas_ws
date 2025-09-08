#!/bin/bash

# Portable RealSense Multi Camera - Environment Setup Script
# This script attempts to set up the environment across different Linux distributions

set -e

echo "=== Portable RealSense Multi Camera Environment Setup ==="

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

# Detect OS and package manager
detect_os() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$NAME
        VER=$VERSION_ID
        CODENAME=$VERSION_CODENAME
        
        # Detect package manager
        if command -v apt-get &> /dev/null; then
            PKG_MANAGER="apt"
            PKG_INSTALL="apt-get install -y"
            PKG_UPDATE="apt-get update"
        elif command -v yum &> /dev/null; then
            PKG_MANAGER="yum"
            PKG_INSTALL="yum install -y"
            PKG_UPDATE="yum update"
        elif command -v dnf &> /dev/null; then
            PKG_MANAGER="dnf"
            PKG_INSTALL="dnf install -y"
            PKG_UPDATE="dnf update"
        elif command -v pacman &> /dev/null; then
            PKG_MANAGER="pacman"
            PKG_INSTALL="pacman -S --noconfirm"
            PKG_UPDATE="pacman -Sy"
        else
            print_error "Unsupported package manager"
            return 1
        fi
        
        print_info "Detected OS: $OS $VER ($PKG_MANAGER)"
        return 0
    else
        print_error "Cannot detect OS version"
        return 1
    fi
}

# Detect ROS2 distribution
detect_ros2() {
    print_info "Detecting ROS2 installation..."
    
    # Try to find ROS2 installation
    for ros_distro in humble iron galactic foxy; do
        if [ -f "/opt/ros/$ros_distro/setup.bash" ]; then
            ROS_DISTRO_DETECTED=$ros_distro
            print_success "Found ROS2 $ros_distro"
            return 0
        fi
    done
    
    # Check if ROS2 is already sourced
    if [ ! -z "$ROS_DISTRO" ]; then
        ROS_DISTRO_DETECTED=$ROS_DISTRO
        print_success "ROS2 $ROS_DISTRO already sourced"
        return 0
    fi
    
    print_error "No ROS2 installation found"
    return 1
}

# Check if running as root
check_privileges() {
    if [[ $EUID -eq 0 ]]; then
        print_warning "Running as root. This may cause permission issues."
        SUDO_CMD=""
    else
        if ! command -v sudo &> /dev/null; then
            print_error "sudo not available and not running as root"
            print_info "Please install sudo or run as root"
            return 1
        fi
        SUDO_CMD="sudo"
    fi
    return 0
}

# Install basic dependencies based on package manager
install_basic_deps() {
    print_info "Installing basic dependencies..."
    
    case $PKG_MANAGER in
        "apt")
            $SUDO_CMD $PKG_UPDATE
            $SUDO_CMD $PKG_INSTALL \
                build-essential \
                cmake \
                git \
                pkg-config \
                curl \
                wget \
                libopencv-dev \
                python3-opencv
            ;;
        "yum"|"dnf")
            $SUDO_CMD $PKG_UPDATE
            $SUDO_CMD $PKG_INSTALL \
                gcc-c++ \
                make \
                cmake \
                git \
                pkgconfig \
                curl \
                wget \
                opencv-devel \
                python3-opencv
            ;;
        "pacman")
            $SUDO_CMD $PKG_UPDATE
            $SUDO_CMD $PKG_INSTALL \
                base-devel \
                cmake \
                git \
                pkgconf \
                curl \
                wget \
                opencv \
                python-opencv
            ;;
        *)
            print_error "Unsupported package manager: $PKG_MANAGER"
            return 1
            ;;
    esac
    
    print_success "Basic dependencies installed"
}

# Install ROS2 dependencies
install_ros2_deps() {
    print_info "Installing ROS2 dependencies..."
    
    # Source ROS2 environment first
    if [ -f "/opt/ros/$ROS_DISTRO_DETECTED/setup.bash" ]; then
        source "/opt/ros/$ROS_DISTRO_DETECTED/setup.bash"
        print_success "ROS2 $ROS_DISTRO_DETECTED environment sourced"
    fi
    
    case $PKG_MANAGER in
        "apt")
            $SUDO_CMD $PKG_INSTALL \
                ros-$ROS_DISTRO_DETECTED-cv-bridge \
                ros-$ROS_DISTRO_DETECTED-image-transport \
                ros-$ROS_DISTRO_DETECTED-camera-info-manager \
                ros-$ROS_DISTRO_DETECTED-sensor-msgs \
                ros-$ROS_DISTRO_DETECTED-ament-cmake \
                ros-$ROS_DISTRO_DETECTED-rclcpp
            ;;
        *)
            print_warning "ROS2 packages installation for $PKG_MANAGER not implemented"
            print_info "Please install ROS2 packages manually"
            ;;
    esac
}

# Install RealSense SDK
install_realsense_sdk() {
    print_info "Installing Intel RealSense SDK..."
    
    case $PKG_MANAGER in
        "apt")
            # Check if already using ROS2 RealSense packages
            if dpkg -l | grep -q "ros-.*-librealsense2"; then
                print_info "ROS2 RealSense packages already installed"
                return 0
            fi
            
            # Add Intel RealSense repository for Debian/Ubuntu
            if ! grep -q "librealsense" /etc/apt/sources.list.d/librealsense.list 2>/dev/null; then
                print_info "Adding Intel RealSense repository..."
                $SUDO_CMD mkdir -p /etc/apt/keyrings
                curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | $SUDO_CMD tee /etc/apt/keyrings/librealsense.pgp > /dev/null
                
                # Determine the correct codename
                if [ "$CODENAME" = "jammy" ] || [ "$CODENAME" = "focal" ] || [ "$CODENAME" = "bionic" ]; then
                    REPO_CODENAME=$CODENAME
                else
                    REPO_CODENAME="jammy"  # Default fallback
                    print_warning "Using jammy repository for unsupported Ubuntu version"
                fi
                
                echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $REPO_CODENAME main" | \
                $SUDO_CMD tee /etc/apt/sources.list.d/librealsense.list
                $SUDO_CMD $PKG_UPDATE
            fi
            
            $SUDO_CMD $PKG_INSTALL \
                librealsense2-dkms \
                librealsense2-utils \
                librealsense2-dev \
                librealsense2-dbg
            ;;
        *)
            print_warning "RealSense SDK installation for $PKG_MANAGER not implemented"
            print_info "Please install RealSense SDK manually from:"
            print_info "https://github.com/IntelRealSense/librealsense"
            ;;
    esac
}

# Set up camera permissions
setup_camera_permissions() {
    print_info "Setting up camera permissions..."
    
    # Add user to video group
    if groups $USER | grep -q '\bvideo\b'; then
        print_info "User already in video group"
    else
        $SUDO_CMD usermod -a -G video $USER
        print_success "User added to video group"
    fi
    
    # Set up udev rules (if available)
    if [ -f "/usr/lib/udev/rules.d/99-realsense-libusb.rules" ]; then
        $SUDO_CMD cp /usr/lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
        $SUDO_CMD udevadm control --reload-rules || true
        $SUDO_CMD udevadm trigger || true
        print_success "udev rules configured"
    else
        print_warning "RealSense udev rules not found"
    fi
}

# Verify installation
verify_installation() {
    print_info "Verifying installation..."
    
    # Check RealSense SDK
    if command -v realsense-viewer &> /dev/null; then
        print_success "RealSense SDK installed successfully"
    elif pkg-config --exists realsense2 2>/dev/null; then
        print_success "RealSense SDK found via pkg-config"
    else
        print_warning "RealSense SDK verification failed"
    fi
    
    # Check OpenCV
    if pkg-config --exists opencv4 2>/dev/null; then
        print_success "OpenCV 4 found"
    elif pkg-config --exists opencv 2>/dev/null; then
        print_success "OpenCV found"
    else
        print_warning "OpenCV not found via pkg-config"
    fi
    
    # Check for connected cameras
    if command -v rs-enumerate-devices &> /dev/null; then
        device_count=$(rs-enumerate-devices 2>/dev/null | grep -c "Device info:" || true)
        if [ "$device_count" -gt 0 ]; then
            print_success "Found $device_count RealSense device(s)"
        else
            print_warning "No RealSense devices detected"
        fi
    fi
}

# Main execution
main() {
    print_info "Starting portable environment setup..."
    
    # Check basic requirements
    if ! detect_os; then
        exit 1
    fi
    
    if ! detect_ros2; then
        exit 1
    fi
    
    if ! check_privileges; then
        exit 1
    fi
    
    # Install components
    install_basic_deps || print_warning "Basic dependencies installation failed"
    install_ros2_deps || print_warning "ROS2 dependencies installation failed"
    install_realsense_sdk || print_warning "RealSense SDK installation failed"
    setup_camera_permissions || print_warning "Camera permissions setup failed"
    
    # Verify
    verify_installation
    
    print_success "Portable environment setup completed!"
    echo ""
    print_info "Next steps:"
    echo "  1. Log out and log back in (or run: newgrp video)"
    echo "  2. Source your ROS2 environment: source /opt/ros/$ROS_DISTRO_DETECTED/setup.bash"
    echo "  3. Build the package: colcon build --packages-select realsense_multi_camera"
    echo "  4. Test the installation"
    
    if [ "$SUDO_CMD" != "" ]; then
        print_info "Note: You may need to log out and back in for group changes to take effect"
    fi
}

# Run main function
main "$@"