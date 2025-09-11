from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    realsense_pkg_share = get_package_share_directory('realsense_camera_driver')
    
    # Camera config file path
    camera_config_file = os.path.join(realsense_pkg_share, 'config', 'parameter.yaml')
    
    # Launch arguments
    camera_config_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=camera_config_file,
        description='Path to camera config file'
    )
    
    # RealSense camera node
    realsense_camera_node = Node(
        package='realsense_camera_driver',
        executable='realsense_camera_node',
        name='realsense_camera_node',
        parameters=[LaunchConfiguration('camera_config_file')],
        output='screen'
    )
    
    # Vision compensation node
    vision_compensation_node = Node(
        package='vision_compensation',
        executable='vision_compensation_node',
        name='vision_compensation_node',
        output='screen'
    )
    
    return LaunchDescription([
        camera_config_arg,
        realsense_camera_node,
        vision_compensation_node
    ])