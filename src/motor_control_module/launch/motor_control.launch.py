from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='motion_control_module',
        #     executable='control',
        #     name='motion_control',
        #     output='screen'
        # ),
        Node(
            package='motion_control_module',
            executable='control_v2',
            name='motion_control_v2',
            output='screen'
        ),
        Node(
            package='motor_control_module',
            executable='esp32_motor_control',
            name='motor_control',
            output='screen'
        ),
    ])
