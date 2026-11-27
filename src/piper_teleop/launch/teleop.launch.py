"""启动遥操作桥。

    ros2 launch piper_teleop teleop.launch.py
    ros2 launch piper_teleop teleop.launch.py rate:=50.0 max_step_deg:=1.5

前提:follower 端的驱动已用命名空间 /follower 启动,
      这样它会订阅 /follower/control/joint_states。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("src_topic", default_value="/leader/feedback/joint_states"),
        DeclareLaunchArgument("dst_topic", default_value="/follower/control/joint_states"),
        DeclareLaunchArgument("rate", default_value="50.0"),
        DeclareLaunchArgument("max_step_deg", default_value="1.5"),
        DeclareLaunchArgument("alpha", default_value="0.2"),
        DeclareLaunchArgument("watchdog_ms", default_value="300.0"),
        DeclareLaunchArgument("passthrough_gripper", default_value="true"),
    ]
    node = Node(
        package="piper_teleop", executable="teleop_bridge", output="screen",
        parameters=[{
            "src_topic": LaunchConfiguration("src_topic"),
            "dst_topic": LaunchConfiguration("dst_topic"),
            "rate": LaunchConfiguration("rate"),
            "max_step_deg": LaunchConfiguration("max_step_deg"),
            "alpha": LaunchConfiguration("alpha"),
            "watchdog_ms": LaunchConfiguration("watchdog_ms"),
            "passthrough_gripper": LaunchConfiguration("passthrough_gripper"),
        }],
    )
    return LaunchDescription(args + [node])
