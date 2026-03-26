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

    # TODO: Include marvelmind_nav launch when ROS2 port is available
    # marvelmind_launch = IncludeLaunchDescription(...)

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    fusion_node = Node(
        package='lokalisasi',
        executable='fusion_gps_odom',
        name='fusion_gps_odom',
        output='screen',
    )

    return LaunchDescription([
        udp_bot_launch,
        static_tf,
        fusion_node,
    ])
