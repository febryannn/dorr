from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_slam_3d',
            executable='lidar_slam_3d',
            name='lidar_slam_3d',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'sensor_frame': 'camera_link',
                'publish_freq': 0.4,
                'min_scan_distance': 0.5,
                'point_cloud_topic': 'cloud',
                'odom_topic': 'odom',
                'laser_topic1': 'scan1',
                'laser_topic2': 'scan2',
                'laser_topic3': 'scan3',
                'laser_topic4': 'scan4',
                'enable_floor_filter': False,
            }],
        ),
    ])
