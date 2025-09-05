#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='D405',
        description='Camera model (D405, D435, auto)'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Image height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frame rate'
    )
    
    show_display_arg = DeclareLaunchArgument(
        'show_display',
        default_value='true',
        description='Show OpenCV display window'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='camera/rgb/image_raw',
        description='Camera topic name'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Camera frame ID'
    )
    
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='1',
        description='Publisher queue size'
    )
    
    # Get package share directory
    package_share = FindPackageShare('realsense_multi_camera')
    
    # Parameter file path
    param_file = PathJoinSubstitution([
        package_share,
        'config',
        'parameter.yaml'
    ])
    
    # RealSense Multi Camera Node
    realsense_node = Node(
        package='realsense_multi_camera',
        executable='realsense_multi_camera_node',
        name='realsense_multi_camera_node',
        parameters=[
            param_file,
            {
                'model': LaunchConfiguration('model'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'show_display': LaunchConfiguration('show_display'),
                'topic_name': LaunchConfiguration('topic_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'queue_size': LaunchConfiguration('queue_size'),
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('camera/rgb/image_raw', LaunchConfiguration('topic_name')),
        ]
    )
    
    # Log launch information
    log_info = LogInfo(
        msg=['Launching RealSense Multi Camera Node with model: ', LaunchConfiguration('model')]
    )
    
    return LaunchDescription([
        model_arg,
        width_arg,
        height_arg,
        fps_arg,
        show_display_arg,
        topic_name_arg,
        frame_id_arg,
        queue_size_arg,
        log_info,
        realsense_node,
    ])