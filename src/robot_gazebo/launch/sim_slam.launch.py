import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_pointcloud_concatenate = get_package_share_directory('pointcloud_concatenate')

    # Launch Gazebo simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_gazebo, 'launch', 'sim.launch.py')
        )
    )

    # Point cloud concatenation (4 kinect cameras)
    concat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pointcloud_concatenate, 'launch', 'concat.launch.py')
        ),
        launch_arguments={
            'cloud_in1': '/kinect1/kinect1/depth/points',
            'cloud_in2': '/kinect2/kinect2/depth/points',
            'cloud_in3': '/kinect3/kinect3/depth/points',
            'cloud_in4': '/kinect4/kinect4/depth/points',
            'cloud_out': '/cloud',
            'use_sim_time': 'true',
        }.items(),
    )

    # === pointcloud_to_laserscan (5 nodes, same angles as real robot) ===
    # Kinect v1 horizontal FOV = 57 deg = 0.9948 rad (half = 28.5 deg = 0.4974 rad)

    laserscan_common_params = {
        'target_frame': 'camera_link',
        'transform_tolerance': 0.01,
        'min_height': 0.1,
        'max_height': 3.0,
        'angle_increment': 0.0314159,
        'scan_time': 0.1,
        'range_min': 0.45,
        'range_max': 7.0,
        'use_inf': False,
        'inf_epsilon': -0.05,
        'concurrency_level': 1,
        'use_sim_time': True,
    }

    # scan1: front kinect (facing 0 deg) -> -28.5 to +28.5 deg
    pointcloud_to_laserscan1 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan1',
        remappings=[('cloud_in', '/cloud'), ('scan', '/scan1')],
        parameters=[{**laserscan_common_params,
                     'angle_min': -0.4974, 'angle_max': 0.4974}],
    )

    # scan2: right kinect (facing -90 deg) -> -118.5 to -61.5 deg
    pointcloud_to_laserscan2 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan2',
        remappings=[('cloud_in', '/cloud'), ('scan', '/scan2')],
        parameters=[{**laserscan_common_params,
                     'angle_min': -2.0682, 'angle_max': -1.0734}],
    )

    # scan3: left kinect (facing +90 deg) -> +61.5 to +118.5 deg
    pointcloud_to_laserscan3 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan3',
        remappings=[('cloud_in', '/cloud'), ('scan', '/scan3')],
        parameters=[{**laserscan_common_params,
                     'angle_min': 1.0734, 'angle_max': 2.0682}],
    )

    # scan4: back kinect (facing 180 deg) positive side -> +151.5 to +180 deg
    pointcloud_to_laserscan4 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan4',
        remappings=[('cloud_in', '/cloud'), ('scan', '/scan4')],
        parameters=[{**laserscan_common_params,
                     'angle_min': 2.6442, 'angle_max': 3.14159}],
    )

    # scan5: back kinect (facing 180 deg) negative side -> -180 to -151.5 deg
    pointcloud_to_laserscan5 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan5',
        remappings=[('cloud_in', '/cloud'), ('scan', '/scan5')],
        parameters=[{**laserscan_common_params,
                     'angle_min': -3.14159, 'angle_max': -2.6442}],
    )

    # === lidar_slam_3d (SLAM node) ===
    lidar_slam_3d_node = Node(
        package='lidar_slam_3d',
        executable='lidar_slam_3d',
        name='lidar_slam_3d',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'sensor_frame': 'camera_link',
            'publish_freq': 0.4,
            'min_scan_distance': 0.5,
            'point_cloud_topic': '/cloud',
            'odom_topic': '/odom',
            'laser_topic1': '/scan1',
            'laser_topic2': '/scan2',
            'laser_topic3': '/scan3',
            'laser_topic4': '/scan4',
            'laser_topic5': '/scan5',
            'enable_floor_filter': False,
        }],
    )

    # RViz2 for visualization
    rviz_config = os.path.join(pkg_robot_gazebo, 'rviz', 'sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        sim_launch,
        # Delay to wait for Gazebo to fully start
        TimerAction(
            period=5.0,
            actions=[
                concat_launch,
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                pointcloud_to_laserscan1,
                pointcloud_to_laserscan2,
                pointcloud_to_laserscan3,
                pointcloud_to_laserscan4,
                pointcloud_to_laserscan5,
                lidar_slam_3d_node,
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[rviz_node],
        ),
    ])
