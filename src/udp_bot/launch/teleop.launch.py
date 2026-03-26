from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp_bot',
            executable='teleop_robot.py',
            name='teleop_robot',
            output='screen',
        ),
    ])
