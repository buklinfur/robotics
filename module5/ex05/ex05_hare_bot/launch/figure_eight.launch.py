import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Берём всё "робот+мир+бридж+urdf" из ex04_hare_bot (робота не меняем)
    ex04_share = get_package_share_directory('ex04_hare_bot')

    world = os.path.join(ex04_share, 'worlds', 'ex04_world.sdf')
    bridge_yaml = os.path.join(ex04_share, 'config', 'bridge.yaml')
    xacro_file = os.path.join(ex04_share, 'urdf', 'robot_hare_wheel_bot.urdf.xacro')

    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ['<?xml version="1.0"?>\n', os.popen(f'xacro {xacro_file}').read()]
        }]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_yaml}]
    )

    motion = Node(
        package='ex05_hare_bot',
        executable='figure_eight',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        motion,
    ])
