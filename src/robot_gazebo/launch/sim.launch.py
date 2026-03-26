import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Process simulation URDF with xacro
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'robot_sim.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # World file
    world_file = os.path.join(pkg_robot_gazebo, 'worlds', 'indoor.sdf')

    # Bridge config
    bridge_config = os.path.join(pkg_robot_gazebo, 'config', 'ros_gz_bridge.yaml')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_robot_description, '..')
    )

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'agv',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
    )

    # ros_gz_bridge (parameter bridge)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Static transform: map -> odom (identity, for simulation)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time,
        gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        map_to_odom_tf,
    ])
