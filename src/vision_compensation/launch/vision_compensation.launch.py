from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vision_compensation_node = Node(
        package='vision_compensation',
        executable='vision_compensation_node',
        name='vision_compensation_node',
        output='screen'
    )
    
    return LaunchDescription([
        vision_compensation_node
    ])