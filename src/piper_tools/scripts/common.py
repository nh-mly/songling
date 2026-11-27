#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Piper 接口的唯一事实来源。

关节名 / 话题 / 服务 / 单位换算 / 限位,全从这里取。
别在别的脚本里硬编码这些字符串 —— 上游有三套命名(gripper / gripper_joint1 / gripper_joint2),
散着写迟早对不上。

限位来源: piAgxArm/api/constants.py ROBOT_JOINT_LIMIT_PRESET["piper"] 与
         agx_arm_urdf/piper/urdf/piper_description.urdf <limit>,两者一致。
"""

import math

# ---------------------------------------------------------------- 关节
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
GRIPPER_JOINT = "gripper"

# 度为单位,与 URDF 完全一致(joint1 ±150°, joint2 0~180°, ...)
LIMITS_DEG = {
    "joint1": (-150.0, 150.0),
    "joint2": (0.0, 180.0),
    "joint3": (-170.0, 0.0),
    "joint4": (-100.0, 100.0),
    "joint5": (-70.0, 70.0),
    "joint6": (-170.0, 170.0),
}

# ---------------------------------------------------------------- 话题 / 服务
TOPIC_FEEDBACK = "/feedback/joint_states"
TOPIC_CMD = "/control/joint_states"
TOPIC_ARM_STATUS = "/feedback/arm_status"
TOPIC_TCP = "/feedback/tcp_pose"

SVC_ENABLE = "/enable_agx_arm"   # std_srvs/SetBool
SVC_HOME = "/move_home"          # std_srvs/Empty
SVC_GATE = "/control_enable"     # std_srvs/SetBool
SVC_ESTOP = "/emergency_stop"    # std_srvs/Empty


# ---------------------------------------------------------------- 工具函数
def deg2rad(d):
    return d * math.pi / 180.0


def rad2deg(r):
    return r * 180.0 / math.pi


def resolve_joint(token):
    """把 '3' / 'joint3' 统一成 'joint3'。"""
    token = str(token).strip()
    if token.startswith("joint"):
        name = token
    else:
        name = f"joint{int(token)}"
    if name not in JOINT_NAMES:
        raise ValueError(f"未知关节: {token}(可选 {', '.join(JOINT_NAMES)})")
    return name


def check_limit(name, value_rad):
    """返回 (ok, 说明)。超限只提示,不在这里强制。"""
    lo, hi = LIMITS_DEG[name]
    v = rad2deg(value_rad)
    if v < lo or v > hi:
        return False, f"{name}={v:+.2f}° 超出 [{lo:+.0f}°, {hi:+.0f}°]"
    return True, f"{name}={v:+.2f}°"


def fmt_joints(values_rad):
    return "  ".join(
        f"{n.split('joint')[-1]}:{rad2deg(v):+7.2f}°"
        for n, v in zip(JOINT_NAMES, values_rad)
    )
