from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'object_frame_compensation_package'
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'object_frame_params.yaml'
        ),
        description='Full path to the object frame compensation parameter file'
    )
    
    object_frame_node = Node(
        package='object_frame_compensation_package',
        executable='object_frame_compensation_node',
        name='object_frame_compensation_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        config_file_arg,
        object_frame_node,
    ])