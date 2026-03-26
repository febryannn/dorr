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
            default_value='/kinect1/kinect1/depth_registered/points',
            description='Input pointcloud topic 1 (front)'
        ),
        DeclareLaunchArgument(
            'cloud_in2',
            default_value='/kinect2/kinect2/depth_registered/points',
            description='Input pointcloud topic 2 (back)'
        ),
        DeclareLaunchArgument(
            'cloud_in3',
            default_value='/kinect3/kinect3/depth_registered/points',
            description='Input pointcloud topic 3 (right)'
        ),
        DeclareLaunchArgument(
            'cloud_in4',
            default_value='/kinect4/kinect4/depth_registered/points',
            description='Input pointcloud topic 4 (left)'
        ),
        DeclareLaunchArgument(
            'cloud_out',
            default_value='full_pointcloud',
            description='Output concatenated pointcloud topic'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='pointcloud_concatenate',
            executable='pointcloud_concatenate_node',
            name='pc_concat',
            output='screen',
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),
                'clouds': 4,
                'hz': 10.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('cloud_in1', LaunchConfiguration('cloud_in1')),
                ('cloud_in2', LaunchConfiguration('cloud_in2')),
                ('cloud_in3', LaunchConfiguration('cloud_in3')),
                ('cloud_in4', LaunchConfiguration('cloud_in4')),
                ('cloud_out', LaunchConfiguration('cloud_out')),
            ],
        ),
    ])
