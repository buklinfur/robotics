from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "radius", default_value="2.0", description="Radius of carrot rotation"
            ),
            DeclareLaunchArgument(
                "direction_of_rotation",
                default_value="1",
                description="1 for CW, -1 for CCW",
            ),
            Node(package="turtlesim", executable="turtlesim_node", name="sim"),
            # управление первой черепахой
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="turtlesim",
                        executable="turtle_teleop_key",
                        name="teleop",
                        prefix="xterm -e",
                    )
                ],
            ),
            # broadcaster для turtle1
            Node(
                package="ex02",
                executable="turtle_tf2_broadcaster",
                name="broadcaster1",
                parameters=[{"turtle": "turtle1"}],
            ),
            # broadcaster для turtle2 (добавляем)
            TimerAction(
                period=3.0,  # запускаем чуть позже, чтобы она успела создаться
                actions=[
                    Node(
                        package="ex02",
                        executable="turtle_tf2_broadcaster",
                        name="broadcaster2",
                        parameters=[{"turtle": "turtle2"}],
                    )
                ],
            ),
            # морковка
            Node(
                package="ex02",
                executable="carrot_broadcaster",
                name="carrot",
                parameters=[
                    {
                        "radius": LaunchConfiguration("radius"),
                        "direction_of_rotation": LaunchConfiguration(
                            "direction_of_rotation"
                        ),
                    }
                ],
            ),
            # listener (создаёт turtle2)
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ex02",
                        executable="turtle_tf2_listener",
                        name="listener",
                    )
                ],
            ),
        ]
    )
