#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""status —— 只读地看一眼机械臂现在什么状态。

不使能、不发命令、不改任何东西。用来在每次动作前后做"我知道现在是几度"。

用法:
    ros2 run piper_tools status              # 读一次
    ros2 run piper_tools status --watch 2    # 每 2 秒刷新
    ros2 run piper_tools status --json       # 机器可读
"""

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray

import common


class StatusNode(Node):
    def __init__(self):
        super().__init__("piper_status")
        self.joints = {}
        self.gripper = None
        self.stamp = None
        self.arm_status = None
        self.create_subscription(JointState, common.TOPIC_FEEDBACK, self._on_joints, 10)
        self.create_subscription(DiagnosticArray, common.TOPIC_ARM_STATUS, self._on_status, 10)

    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in common.JOINT_NAMES:
                self.joints[name] = pos
            elif name == common.GRIPPER_JOINT:
                self.gripper = pos
        self.stamp = self.get_clock().now()

    def _on_status(self, msg):
        if msg.status:
            st = msg.status[-1]
            # DiagnosticStatus.level 是 byte(rclpy 里是 bytes,b'\x00'),不能直接 int()
            lvl = st.level
            if isinstance(lvl, (bytes, bytearray)):
                lvl = lvl[0] if lvl else 0
            else:
                lvl = int(lvl)
            self.arm_status = {"name": st.name, "level": lvl, "message": st.message}

    def ready(self):
        return all(n in self.joints for n in common.JOINT_NAMES)


def emit(node, as_json):
    vals = [node.joints[n] for n in common.JOINT_NAMES]
    if as_json:
        print(json.dumps({
            "joints_rad": dict(zip(common.JOINT_NAMES, [round(v, 6) for v in vals])),
            "joints_deg": {n: round(common.rad2deg(v), 3) for n, v in zip(common.JOINT_NAMES, vals)},
            "gripper_m": None if node.gripper is None else round(node.gripper, 6),
            "arm_status": node.arm_status,
        }, ensure_ascii=False))
        return
    print(f"关节  {common.fmt_joints(vals)}")
    for n, v in zip(common.JOINT_NAMES, vals):
        ok, detail = common.check_limit(n, v)
        flag = " " if ok else " !"
        print(f"  {flag} {detail}")
    if node.gripper is not None:
        print(f"夹爪  {node.gripper * 1000:+.1f} mm(取绝对值)")
    if node.arm_status:
        print(f"状态  {node.arm_status['name']} level={node.arm_status['level']} {node.arm_status['message']}")


def main():
    ap = argparse.ArgumentParser(description="只读查看 Piper 状态")
    ap.add_argument("--watch", type=float, default=0, help="每 N 秒刷新一次,0 = 只读一次")
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    ap.add_argument("--timeout", type=float, default=5.0, help="等待第一帧的超时(秒)")
    args = ap.parse_args()

    rclpy.init()
    node = StatusNode()

    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    while rclpy.ok() and not node.ready():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds / 1e9 > deadline:
            print(f"超时: {args.timeout}s 内没收到完整关节反馈\n"
                  f"  是不是没起节点? ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py ...\n"
                  f"  话题对不对? ros2 topic list | grep feedback", file=sys.stderr)
            node.destroy_node(); rclpy.shutdown(); return 1

    if args.watch <= 0:
        emit(node, args.json)
        node.destroy_node(); rclpy.shutdown(); return 0

    import time
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            emit(node, args.json)
            if not args.json:
                print("-" * 60)
            time.sleep(max(0.0, args.watch - 0.05))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
