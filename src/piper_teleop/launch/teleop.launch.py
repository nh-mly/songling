"""遥操作统一入口 —— 一条命令起一套。

    # A 机(leader):驱动 + 使能 + 回零 + 零力拖动
    ros2 launch piper_teleop teleop.launch.py role:=leader

    # B 机(follower):驱动 + 使能 + 回零 + 开控制门闸
    ros2 launch piper_teleop teleop.launch.py role:=follower

    # 任一台:启桥(把 leader 的关节角转发给 follower)
    ros2 launch piper_teleop teleop.launch.py role:=bridge

前置:两台都要 source scripts/teleop_env.sh(domain 66 + SUBNET)。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _driver_and_setup(context, role, speed, can_port, arm_type, effector,
                      home_tol, monitor):
    """leader / follower:起驱动 + 一键准备节点。"""
    ctrl_share = FindPackageShare("agx_arm_ctrl")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ctrl_share, "launch", "start_single_agx_arm.launch.py"])),
        launch_arguments={
            "namespace": role,
            "can_port": can_port,
            "arm_type": arm_type,
            "effector_type": effector,
            "speed_percent": speed,
            "auto_enable": "false",
            "control_enabled": "false",
        }.items(),
    )

    setup = Node(
        package="piper_teleop", executable="teleop_setup", output="screen",
        parameters=[{
            "role": role,
            "namespace": f"/{role}",
            "home_tolerance_deg": float(home_tol),
            "monitor_period_s": float(monitor),
        }],
    )
    return [driver, setup]


def _bridge():
    """只起桥,参数从 config/teleop_params.yaml 读。"""
    cfg_path = os.path.join(
        get_package_share_directory("piper_teleop"), "config", "teleop_params.yaml")
    with open(cfg_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    params = cfg.get("teleop_bridge", {}).get("ros__parameters", {})

    return [Node(package="piper_teleop", executable="teleop_bridge",
                 output="screen", parameters=[params])]


def launch_setup(context, *args, **kwargs):
    role = LaunchConfiguration("role").perform(context).strip().lower()

    if role == "bridge":
        return _bridge()
    if role in ("leader", "follower"):
        speed = (LaunchConfiguration("speed_leader").perform(context)
                 if role == "leader"
                 else LaunchConfiguration("speed_follower").perform(context))
        return _driver_and_setup(
            context, role, speed,
            LaunchConfiguration("can_port").perform(context),
            LaunchConfiguration("arm_type").perform(context),
            LaunchConfiguration("effector_type").perform(context),
            LaunchConfiguration("home_tolerance_deg").perform(context),
            LaunchConfiguration("monitor_period_s").perform(context),
        )
    raise RuntimeError(f"role 必须是 leader / follower / bridge,收到: {role!r}")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("role", default_value="leader",
                              description="leader | follower | bridge"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("arm_type", default_value="piper"),
        DeclareLaunchArgument("effector_type", default_value="agx_gripper"),
        DeclareLaunchArgument("speed_leader", default_value="20",
                              description="leader 手拖用低速即可"),
        DeclareLaunchArgument("speed_follower", default_value="60",
                              description="follower 跟随需要更快"),
        DeclareLaunchArgument("home_tolerance_deg", default_value="3.0",
                              description="回零判定阈值"),
        DeclareLaunchArgument("monitor_period_s", default_value="10.0",
                              description="准备完成后的状态打印周期,0=退出"),
        OpaqueFunction(function=launch_setup),
    ])
