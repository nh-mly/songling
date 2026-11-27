"""M1: 在 MuJoCo 里跑起 Piper,暴露 ros2_control 接口。

    ros2 launch piper_mujoco mujoco_sim.launch.py
    ros2 launch piper_mujoco mujoco_sim.launch.py headless:=true

之后可以:
    ros2 control list_controllers
    ros2 topic echo /joint_states --once
    # 发轨迹到 arm_controller(见 README)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("piper_mujoco")
    desc_pkg = FindPackageShare("agx_arm_description")
    headless = LaunchConfiguration("headless").perform(context)

    # 用 xacro 展开(里面 include 了官方 URDF,并带上 MuJoCo 硬件接口)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_share, "urdf", "piper_mj.urdf.xacro"]),
    ])
    robot_description_str = robot_description_content.perform(context)

    # headless 参数需要覆盖到 URDF 里 —— 用一个临时 xacro 参数实现太重,
    # 这里直接字符串替换(仅这一个 bool)。
    if headless.lower() == "true":
        robot_description_str = robot_description_str.replace(
            '<param name="headless">false</param>',
            '<param name="headless">true</param>')

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    controllers_file = PathJoinSubstitution([pkg_share, "config", "mj_controllers.yaml"])
    plugins_file = PathJoinSubstitution([pkg_share, "config", "mj_plugins.yaml"])

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="both",
             parameters=[robot_description, {"use_sim_time": True}]),

        # MuJoCo 自带的 ros2_control_node(替代标准 controller_manager)
        Node(package="mujoco_ros2_control", executable="ros2_control_node",
             output="both",
             parameters=[
                 {"use_sim_time": True},
                 ParameterFile(controllers_file),
                 ParameterFile(plugins_file),
             ],
             on_exit=Shutdown()),

    ]

    for c in ("joint_state_broadcaster", "arm_controller", "gripper_controller"):
        nodes.append(Node(package="controller_manager", executable="spawner",
                          arguments=[c, "--param-file", controllers_file],
                          output="both"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="不显示 MuJoCo 窗口"),
        OpaqueFunction(function=launch_setup),
    ])
