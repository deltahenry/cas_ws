from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='state_machine_module',
            executable='run',
            name='run_node',
            output='screen'
        ),
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
        Node(
            package='task_module',
            executable='precise_align',
            name='precise_align',
            output='screen'
        ),
        Node(
            package='task_module',
            executable='pick',
            name='pick',
            output='screen'
        ),
        Node(
            package='task_module',
            executable='assembly',
            name='assembly',
            output='screen'
        ),
        Node(
            package='task_module',
            executable='recipe_node',
            name='recipe_node',
            output='screen'
        ),

    
    ])
