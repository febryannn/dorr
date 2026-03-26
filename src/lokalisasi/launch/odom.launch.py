from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    udp_bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('udp_bot'), 'launch', 'udp_bot.launch.py')
        )
    )

    odom_node = Node(
        package='lokalisasi',
        executable='odom',
        name='odom',
        output='screen',
    )

    return LaunchDescription([
        udp_bot_launch,
        odom_node,
    ])
