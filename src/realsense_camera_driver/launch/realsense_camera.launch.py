from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_camera_driver')
    
    config_file = os.path.join(pkg_share, 'config', 'parameter.yaml')
    
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )
    
    realsense_camera_node = Node(
        package='realsense_camera_driver',
        executable='realsense_camera_node',
        name='realsense_camera_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        config_arg,
        realsense_camera_node
    ])