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

    # TODO: Include mcl_3dl launch when ROS2 port is available
    # mcl_3dl_launch = IncludeLaunchDescription(...)

    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    )

    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
    )

    rgbd_cam_loc_node = Node(
        package='lokalisasi',
        executable='rgbd_cam_loc',
        name='rgbd_cam_loc',
        output='screen',
        parameters=[{
            'init_robotx': 0.6,
            'init_roboty': 2.7,
            'init_robotw': 0.0,
            'min_cam_d': -6.0,
            'max_cam_d': 6.0,
            'maxf_points_out': 15.0,
        }],
    )

    return LaunchDescription([
        udp_bot_launch,
        world_to_map_tf,
        base_to_camera_tf,
        rgbd_cam_loc_node,
    ])
