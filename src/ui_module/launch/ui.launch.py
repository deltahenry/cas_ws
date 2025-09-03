from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ui_app',
            executable='ui_node',
            name='ui_node',
            output='screen'
        ),
        Node(
            package='ui_module',
            executable='ui_motor_control',
            name='ui_motor_control',
            output='screen'
        ),
        Node(
            package='ui_module',
            executable='ui_forklift',
            name='ui_forklift',
            output='screen'
        ),
        # Node(
        #     package='ui_module',
        #     executable='ui_recipe',
        #     name='ui_recipe',
        #     output='screen'
        # ),
        Node(
            package='ui_module',
            executable='ui_limit',
            name='ui_limit',
            output='screen'
        ),

    ])
