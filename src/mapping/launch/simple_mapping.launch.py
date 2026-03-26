from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Static transform: base_link -> camera_depth_optical_frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_depth_optical_frame',
        ],
    )

    # Simple mapping node
    simple_mapping_node = Node(
        package='mapping',
        executable='simple_mapping',
        name='simple_mapping',
        output='screen',
        parameters=[{
            'maxf_points_out': 1.0,
            'maxf_map_out': 1.0,
            'min_cam_d': 0.6,
            'max_cam_d': 8.0,
        }],
    )

    return LaunchDescription([
        static_tf_node,
        simple_mapping_node,
    ])
