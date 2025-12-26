import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ex04_hare_bot')

    world = os.path.join(pkg_share, 'worlds', 'ex04_world.sdf')
    bridge_yaml = os.path.join(pkg_share, 'config', 'bridge.yaml')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_hare_wheel_bot.urdf.xacro')

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
            'robot_description': Command(['xacro', ' ', xacro_file])
        }]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_yaml}]
    )

    circle_node = Node(
        package='ex04_hare_bot',
        executable='circle_movement',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    rviz_config = os.path.join(pkg_share, 'config', 'ex04.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )


    return LaunchDescription([
        rviz_arg,
        gazebo,
        bridge,
        robot_state_publisher,
        rviz,
        circle_node,
    ])
