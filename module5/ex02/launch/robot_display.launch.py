import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # По заданию запускаем уже СГЕНЕРИРОВАННЫЙ URDF
    urdf_path = os.path.join(
        get_package_share_directory('ex02'),
        'urdf',
        'robot.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # RViz конфиг берём из ex01 (интеграция с упражнением 01)
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('ex01'),
        'rviz',
        'view_robot.rviz'
    ])

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg]
        ),
    ])
