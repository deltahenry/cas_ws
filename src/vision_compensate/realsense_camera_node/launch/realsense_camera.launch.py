from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('realsense_camera_node')
    
    # Parameters file path
    params_file = os.path.join(pkg_dir, 'config', 'parameter.yaml')
    
    # Launch arguments
    show_display_arg = DeclareLaunchArgument(
        'show_display',
        default_value='true',
        description='Enable/disable OpenCV display window'
    )
    
    # Node
    realsense_node = Node(
        package='realsense_camera_node',
        executable='realsense_camera_node',
        name='realsense_multi_camera_node',
        parameters=[
            params_file,
            {'show_display': LaunchConfiguration('show_display')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        show_display_arg,
        realsense_node
    ])