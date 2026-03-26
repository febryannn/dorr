import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_visualization')
    robot_description_share = get_package_share_directory('robot_description')

    static_map = LaunchConfiguration('static_map')

    declare_static_map = DeclareLaunchArgument(
        'static_map',
        default_value='True',
        description='Whether to use a static map'
    )

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_share, 'launch', 'robot.launch.py')
        )
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(pkg_share, 'maps', 'p206.yaml'),
            'frame_id': 'map',
        }],
        condition=IfCondition(static_map),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'map_visual_new.rviz')],
    )

    return LaunchDescription([
        declare_static_map,
        include_robot,
        map_server_node,
        rviz_node,
    ])
