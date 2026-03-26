from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Use ros2 run teleop_twist_keyboard for controlling the simulated robot
    teleop = ExecuteProcess(
        cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
             '--ros-args', '-r', '/cmd_vel:=/cmd_vel'],
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([
        teleop,
    ])
