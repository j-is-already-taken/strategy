from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picamera_ros2',
            executable='picamera_pub_exec',
            name='picamera_node'
        ),
        Node(
            package='teamJ_software',
            executable='detect_yellow',
            name='detect_yellow_node'
        )
    ])
