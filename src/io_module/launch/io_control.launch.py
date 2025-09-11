from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='io_module',
        #     executable='io_node',
        #     name='io_node',
        #     output='screen'
        # ), 
        Node(
            package='io_module',
            executable='height',
            name='height_sensor',
            output='screen'
        ),
        Node(
            package='io_module',
            executable='depth_rangefinder',
            name='depth_rangefinder',
            output='screen'
        ),
        Node(
            package='io_module',
            executable='forklift',
            name='forklift_control',
            output='screen'
        ), 
        Node(
            package='io_module',
            executable='gripper',
            name='gripper_control',
            output='screen'
        ), 
        Node(
            package='io_module',
            executable='limit_control',
            name='limit_control',
            output='screen'
        ), 
        
      
    ])
