import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "ex03"
    pkg_share = get_package_share_directory(pkg)

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation clock"
    )
    declare_gz_args = DeclareLaunchArgument(
        "gz_args", default_value="-r empty.sdf",
        description="Arguments passed to Gazebo (gz sim)"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")

    xacro_file = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    rviz_cfg = os.path.join(pkg_share, "rviz", "robot.rviz")

    robot_description = {"robot_description": Command(["xacro", " ", xacro_file])}

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[

            ("/world/empty/clock", "/clock"),
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty",
            "-name", "robot_hare_wheel_bot",
            "-allow_renaming", "true",
            "-param", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.4",
        ],
        parameters=[robot_description],
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gz_args,
        gz,
        rsp,
        bridge,
        delayed_spawn,
        rviz,
    ])
