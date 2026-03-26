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
        }.items(),
    )

    # Teleop keyboard for controlling the robot
    teleop_note = Node(
        package='ros2cli',
        executable='ros2',
        arguments=['topic', 'pub', '--once', '/cmd_vel',
                   'geometry_msgs/msg/Twist',
                   '{linear: {x: 0.0}, angular: {z: 0.0}}'],
        output='screen',
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        sim_launch,
        # Delay concat to wait for Gazebo to fully start
        TimerAction(
            period=5.0,
            actions=[concat_launch],
        ),
        TimerAction(
            period=3.0,
            actions=[rviz_node],
        ),
    ])
