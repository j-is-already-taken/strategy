from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omni_wheel',
            executable='omni_wheel_action_server',
            name='omni_wheel_action_server',
            output='screen'
        ),
        Node(
            package='omni_wheel',
            executable='omni_wheel_action_client',
            name='omni_wheel_action_client',
            output='screen'
        ),
        Node(
            package='servo_control',
            executable='servo_control_action_server',
            name='servo_control_action_server',
            output='screen'
        ),
        Node(
            package='servo_control',
            executable='servo_control_action_client',
            name='servo_control_action_client',
            output='screen'
        ),
    ])
