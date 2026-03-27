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

    # Kinect v1 horizontal FOV = 57 deg = 0.9948 rad (half = 28.5 deg = 0.4974 rad)
    # 5 pointcloud_to_laserscan nodes matching actual Kinect v1 FOV

    # scan1: front kinect (facing 0 deg) -> -28.5 to +28.5 deg
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
            'angle_min': -0.4974,
            'angle_max': 0.4974,
            'angle_increment': 0.0314159,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 7.0,
            'use_inf': False,
            'inf_epsilon': -0.05,
            'concurrency_level': 1,
        }],
    )

    # scan2: right kinect (facing -90 deg = -1.5708 rad) -> -118.5 to -61.5 deg
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
            'angle_min': -2.0682,
            'angle_max': -1.0734,
            'angle_increment': 0.0314159,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 7.0,
            'use_inf': False,
            'inf_epsilon': -0.05,
            'concurrency_level': 1,
        }],
    )

    # scan3: left kinect (facing +90 deg = 1.5708 rad) -> +61.5 to +118.5 deg
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
            'angle_min': 1.0734,
            'angle_max': 2.0682,
            'angle_increment': 0.0314159,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 7.0,
            'use_inf': False,
            'inf_epsilon': -0.05,
            'concurrency_level': 1,
        }],
    )

    # scan4: back kinect (facing 180 deg) positive side -> +151.5 to +180 deg
    pointcloud_to_laserscan4 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan4',
        remappings=[
            ('cloud_in', '/cloud'),
            ('scan', '/scan4'),
        ],
        parameters=[{
            'target_frame': 'camera_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 3.0,
            'angle_min': 2.6442,
            'angle_max': 3.14159,
            'angle_increment': 0.0314159,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 7.0,
            'use_inf': False,
            'inf_epsilon': -0.05,
            'concurrency_level': 1,
        }],
    )

    # scan5: back kinect (facing 180 deg) negative side -> -180 to -151.5 deg
    pointcloud_to_laserscan5 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan5',
        remappings=[
            ('cloud_in', '/cloud'),
            ('scan', '/scan5'),
        ],
        parameters=[{
            'target_frame': 'camera_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 3.0,
            'angle_min': -3.14159,
            'angle_max': -2.6442,
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

    return LaunchDescription([
        udp_bot_launch,
        world_to_map_tf,
        slam_rgbd_cam_node,
        pointcloud_to_laserscan1,
        pointcloud_to_laserscan2,
        pointcloud_to_laserscan3,
        pointcloud_to_laserscan4,
        pointcloud_to_laserscan5,
        lidar_slam_3d_launch,
    ])
