from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
        ),
        Node(
            package='odrive_control', 
            executable='switching_control_teleop',
            name='switching_control_teleop',
            parameters=[
                {
                    'use_joy': True
                }
            ],
        ),
    ])
