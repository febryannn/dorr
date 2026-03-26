from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame',
            default_value='camera_link',
            description='Target frame for pointcloud transform'
        ),
        DeclareLaunchArgument(
            'cloud_in1',
            default_value='/kinect1/kinect1/depth/points',
            description='Input pointcloud topic 1'
        ),
        DeclareLaunchArgument(
            'cloud_in2',
            default_value='/kinect2/kinect2/depth/points',
            description='Input pointcloud topic 2'
        ),
        DeclareLaunchArgument(
            'cloud_in3',
            default_value='/astra/depth/points',
            description='Input pointcloud topic 3'
        ),
        DeclareLaunchArgument(
            'cloud_out',
            default_value='full_pointcloud',
            description='Output concatenated pointcloud topic'
        ),

        Node(
            package='pointcloud_concatenate',
            executable='pointcloud_concatenate_node',
            name='pc_concat',
            output='screen',
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),
                'clouds': 3,
                'hz': 10.0,
            }],
            remappings=[
                ('cloud_in1', LaunchConfiguration('cloud_in1')),
                ('cloud_in2', LaunchConfiguration('cloud_in2')),
                ('cloud_in3', LaunchConfiguration('cloud_in3')),
                ('cloud_out', LaunchConfiguration('cloud_out')),
            ],
        ),
    ])
