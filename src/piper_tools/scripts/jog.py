#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""jog —— 单关节安全点动。

为什么需要它
------------
`agx_arm_ctrl` 收 `control/joint_states` 时是**整组下发**:

    joints = [arm_joints.get(name, 0) for name in self.arm_joint_names]

意思是:你只发 joint6,其余五个会被当成 0 一起发下去 —— 手臂会甩到零位。
手工 `ros2 topic pub` 只发一个关节,是最容易出事的操作。

本工具的做法:先读当前全部关节角,只改目标那一个,补齐 6 个再发。

用法:
    ros2 run piper_tools jog --joint 6 --delta 5 --dry-run   # 先看目标
    ros2 run piper_tools jog --joint 6 --delta 5              # 真发
    ros2 run piper_tools jog --joint 3 --delta -10 --duration 2 --rate 30

安全默认:
    * 单次 |delta| > 15° 拒绝执行(要的话显式 --force)
    * 目标超出 URDF 限位时拒绝执行(--force 可越过,但请先看 --dry-run)
    * --dry-run 只打印,不发任何消息
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import common

MAX_DELTA_DEG = 15.0


class JogNode(Node):
    def __init__(self):
        super().__init__("piper_jog")
        self.current = {}
        self.pub = self.create_publisher(JointState, common.TOPIC_CMD, 10)
        self.create_subscription(JointState, common.TOPIC_FEEDBACK, self._on_fb, 10)

    def _on_fb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in common.JOINT_NAMES:
                self.current[name] = pos

    def ready(self):
        return all(n in self.current for n in common.JOINT_NAMES)

    def wait_feedback(self, timeout):
        t0 = time.time()
        while rclpy.ok() and not self.ready():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                return False
        return True


def main():
    ap = argparse.ArgumentParser(description="Piper 单关节安全点动")
    ap.add_argument("--joint", required=True, help="joint1..joint6 或 1..6")
    ap.add_argument("--delta", type=float, required=True, help="增量,单位度(+/-)")
    ap.add_argument("--dry-run", action="store_true", help="只打印目标,不发命令")
    ap.add_argument("--force", action="store_true", help="越过 delta/限位 保护")
    ap.add_argument("--duration", type=float, default=1.0, help="连发多久(秒),默认 1.0")
    ap.add_argument("--rate", type=float, default=50.0, help="发送频率 Hz,默认 50")
    ap.add_argument("--timeout", type=float, default=5.0, help="等反馈超时(秒)")
    args = ap.parse_args()

    try:
        joint = common.resolve_joint(args.joint)
    except ValueError as e:
        print(f"参数错误: {e}", file=sys.stderr)
        return 2

    if abs(args.delta) > MAX_DELTA_DEG and not args.force:
        print(f"拒绝执行: 单次 |delta|={abs(args.delta):.1f}° > {MAX_DELTA_DEG:.0f}°。\n"
              f"  分几次动,或显式加 --force。", file=sys.stderr)
        return 2

    rclpy.init()
    node = JogNode()
    if not node.wait_feedback(args.timeout):
        print(f"超时: {args.timeout}s 内没收到 6 个关节的反馈。\n"
              f"  检查: ros2 topic echo {common.TOPIC_FEEDBACK} --once\n"
              f"  节点起了吗: ros2 node list", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    target = {n: node.current[n] for n in common.JOINT_NAMES}
    before = target[joint]
    target[joint] = before + common.deg2rad(args.delta)

    ok, detail = common.check_limit(joint, target[joint])
    print(f"当前  {common.fmt_joints([node.current[n] for n in common.JOINT_NAMES])}")
    print(f"目标  {common.fmt_joints([target[n] for n in common.JOINT_NAMES])}")
    print(f"改动  {joint}: {common.rad2deg(before):+.2f}° → {common.rad2deg(target[joint]):+.2f}°")
    if not ok:
        print(f"警告: {detail}", file=sys.stderr)
        if not args.force:
            print("拒绝执行(超限位)。确认无误后加 --force。", file=sys.stderr)
            node.destroy_node(); rclpy.shutdown(); return 2

    if args.dry_run:
        print("--dry-run:没发任何消息。")
        node.destroy_node(); rclpy.shutdown(); return 0

    msg = JointState()
    msg.name = list(common.JOINT_NAMES)
    msg.position = [target[n] for n in common.JOINT_NAMES]

    period = 1.0 / max(1.0, args.rate)
    n_sent = 0
    t_end = time.time() + max(0.1, args.duration)
    try:
        while rclpy.ok() and time.time() < t_end:
            msg.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(msg)
            n_sent += 1
            rclpy.spin_once(node, timeout_sec=period)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        print(f"已发送 {n_sent} 帧到 {common.TOPIC_CMD}")
        node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
