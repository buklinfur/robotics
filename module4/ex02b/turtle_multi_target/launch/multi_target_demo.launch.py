from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 1) Симулятор
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                name="sim",
                output="screen",
            ),
            # 2) Спавним только turtle2 и turtle3 (turtle1 уже существует по умолчанию)
            TimerAction(
                period=0.5,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/spawn",
                            "turtlesim/srv/Spawn",
                            '"{x: 5.0, y: 5.0, theta: 0.0, name: turtle2}"',
                        ],
                        shell=True,
                    )
                ],
            ),
            TimerAction(
                period=1.2,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/spawn",
                            "turtlesim/srv/Spawn",
                            '"{x: 8.0, y: 8.0, theta: 0.0, name: turtle3}"',
                        ],
                        shell=True,
                    )
                ],
            ),
            # 3) TF от поз черепах -> world->turtlei
            Node(
                package="turtle_multi_target",
                executable="turtle_tf_broadcaster.py",
                name="turtle_tf_broadcaster",
                output="screen",
            ),
            # 4) Вращающиеся цели
            Node(
                package="turtle_multi_target",
                executable="target_switcher.py",
                name="target_switcher",
                output="screen",
                parameters=[
                    {"radius": 2.0},
                    {"direction": 1},  # 1 по часовой, -1 против
                ],
            ),
            # 5) Контроллер turtle2, авто-переключение цели
            Node(
                package="turtle_multi_target",
                executable="turtle_controller.py",
                name="turtle_controller",
                output="screen",
                parameters=[{"switch_distance": 1.5}],
            ),
        ]
    )
