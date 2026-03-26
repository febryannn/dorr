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
                'enable_floor_filter': False,
            }],
        ),
        Node(
            package='lidar_slam_3d',
            executable='slam3d_to_grid_node',
            name='slam_to_grid',
            output='screen',
            parameters=[{
                'position_x': 0.5,
                'position_y': 3.0,
                'cell_size': 0.02,
                'length_x': 13.0,
                'length_y': 9.0,
                'cloud_in_topic': '/map_cloud',
                'mapi_topic_name': 'mapgrid_i',
                'maph_topic_name': 'mapgrid_h',
                'intensity_factor': 125.0,
                'height_factor': 125.0,
            }],
        ),
    ])
