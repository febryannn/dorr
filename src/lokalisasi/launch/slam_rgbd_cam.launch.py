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
        arguments=['0.13', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
    )

    slam_rgbd_cam_node = Node(
        package='lokalisasi',
        executable='slam_rgbd_cam',
        name='slam_rgbd_cam',
        output='screen',
        parameters=[{
            'init_robotx': -2.8,
            'init_roboty': 2.1,
            'init_robotw': -90.0,
            'min_cam_d': -9.0,
            'max_cam_d': 9.0,
            'maxf_points_out': 10.0,
        }],
    )

    pointcloud_to_laserscan1 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan1',
        remappings=[
            ('cloud_in', '/cloud'),
            ('scan', '/scan1'),
        ],
        parameters=[{
            'target_frame': 'camera_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 3.0,
            'angle_min': -0.4,
            'angle_max': 0.4,
            'angle_increment': 0.0314159,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 7.0,
            'use_inf': False,
            'inf_epsilon': -0.05,
            'concurrency_level': 1,
        }],
    )

    pointcloud_to_laserscan2 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan2',
        remappings=[
            ('cloud_in', '/cloud'),
            ('scan', '/scan2'),
        ],
        parameters=[{
            'target_frame': 'camera_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 3.0,
            'angle_min': 2.74,
            'angle_max': 3.14,
            'angle_increment': 0.0314159,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 7.0,
            'use_inf': False,
            'inf_epsilon': -0.05,
            'concurrency_level': 1,
        }],
    )

    pointcloud_to_laserscan3 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan3',
        remappings=[
            ('cloud_in', '/cloud'),
            ('scan', '/scan3'),
        ],
        parameters=[{
            'target_frame': 'camera_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 3.0,
            'angle_min': -3.14,
            'angle_max': -2.74,
            'angle_increment': 0.0314159,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 7.0,
            'use_inf': False,
            'inf_epsilon': -0.05,
            'concurrency_level': 1,
        }],
    )

    lidar_slam_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lidar_slam_3d'), 'launch', 'lidar_slam_3d.launch.py')
        )
    )

    # NOTE: jsk_pcl_utils/PointCloudToPCD nodelet not available in ROS2
    # Use pcl_ros or custom node to save PCD files if needed

    return LaunchDescription([
        udp_bot_launch,
        world_to_map_tf,
        base_to_camera_tf,
        slam_rgbd_cam_node,
        pointcloud_to_laserscan1,
        pointcloud_to_laserscan2,
        pointcloud_to_laserscan3,
        lidar_slam_3d_launch,
    ])
