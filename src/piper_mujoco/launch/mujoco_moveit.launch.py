"""M2: MuJoCo + MoveIt + RViz。

    ros2 launch piper_mujoco mujoco_moveit.launch.py
    ros2 launch piper_mujoco mujoco_moveit.launch.py headless:=true

与真机相同的链路:
    RViz → move_group → FollowJointTrajectory → JTC → MuJoCo 物理
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

import os
import yaml


def launch_setup(context, *args, **kwargs):
    pkg = FindPackageShare("piper_mujoco")

    # 官方工具会把 URDF/SRDF/kinematics/joint_limits/controllers
    # 打包成 move_group 需要的完整参数集合
    moveit_config = MoveItConfigsBuilder(
        "agx_arm", package_name="agx_arm_moveit"
    ).to_moveit_configs()

    # 静态 agx_arm.srdf 里没有 disable_collisions(0 条),会导致
    # CheckStartStateCollision 在零位就报自碰撞 → 用 xacro 展开出完整 SRDF
    srdf_xml = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("agx_arm_moveit"), "config", "agx_arm.srdf.xacro"]),
        " arm_type:=piper effector_type:=agx_gripper",
    ]).perform(context)

    # MoveItConfigsBuilder 默认的 agx_arm.urdf.xacro 里 effector_type 默认是 none
    # → 展开后没有夹爪。这里改用我们自己的 xacro(含 agx_gripper)。
    robot_desc = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("piper_mujoco"), "urdf", "piper_mj.urdf.xacro"]),
    ]).perform(context)

    # MoveItConfigsBuilder 默认找 moveit_controllers.yaml,
    # 我们的文件叫 moveit_controllers_gripper.yaml,需要显式加载
    ctrl_path = os.path.join(
        FindPackageShare("agx_arm_moveit").perform(context),
        "config", "moveit_controllers_gripper.yaml")
    with open(ctrl_path, encoding="utf-8") as f:
        controllers = yaml.safe_load(f)

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="both",
        parameters=[
            moveit_config.to_dict(),
            {"robot_description": ParameterValue(robot_desc, value_type=str)},
            {"robot_description_semantic": ParameterValue(srdf_xml, value_type=str)},
            dict(controllers),          # moveit_controller_manager / moveit_simple_controller_manager 需平铺到顶层
            {"use_sim_time": True},
            {"planning_pipelines": ["ompl"]},
            {"allow_trajectory_execution": True},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            {"trajectory_execution.allowed_start_tolerance": 0.05},
        ],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", output="both",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("agx_arm_moveit"), "config", "moveit.rviz"])],
        parameters=[
            {"robot_description": ParameterValue(robot_desc, value_type=str)},
            {"robot_description_semantic": ParameterValue(srdf_xml, value_type=str)},
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "mujoco_sim.launch.py"])),
            launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
        ),
        move_group,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
