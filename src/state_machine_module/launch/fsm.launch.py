from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_module',
            executable='component_control',
            name='component_control',
            output='screen'
        ),
        Node(
            package='task_module',
            executable='rough_align',
            name='rough_align',
            output='screen'
        ),
    ])
