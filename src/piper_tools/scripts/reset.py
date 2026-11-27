#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reset —— 回零位(调用官方的 /move_home 服务)。

为什么不用自己发零位:
    官方 `agx_arm_ctrl` 的 `move_home` 走的是它自己维护的收敛逻辑和状态检查,
    自己发一帧全 0 的 joint_states 会绕过这些检查。能用服务就别发话题。

前提:
    * 节点已启动且**已使能**(没使能时服务会回 warn,手臂不动)
    * 急停未拍下

用法:
    ros2 run piper_tools reset --dry-run     # 只看现在在哪、要调什么
    ros2 run piper_tools reset --yes         # 真回零
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

import common


class ResetNode(Node):
    def __init__(self):
        super().__init__("piper_reset")
        self.current = {}
        self.create_subscription(JointState, common.TOPIC_FEEDBACK, self._on_fb, 10)

    def _on_fb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in common.JOINT_NAMES:
                self.current[name] = pos

    def ready(self):
        return all(n in self.current for n in common.JOINT_NAMES)


def main():
    ap = argparse.ArgumentParser(description="Piper 回零")
    ap.add_argument("--yes", action="store_true", help="确认执行(不带则等同 --dry-run)")
    ap.add_argument("--dry-run", action="store_true", help="只打印,不调用")
    ap.add_argument("--timeout", type=float, default=5.0, help="等反馈超时(秒)")
    args = ap.parse_args()

    do_it = args.yes and not args.dry_run

    rclpy.init()
    node = ResetNode()

    t0 = time.time()
    while rclpy.ok() and not node.ready() and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.ready():
        print(f"当前位置  {common.fmt_joints([node.current[n] for n in common.JOINT_NAMES])}")
    else:
        print(f"警告: {args.timeout}s 内没收到完整关节反馈,无法确认当前位置。", file=sys.stderr)

    cli = node.create_client(Empty, common.SVC_HOME)
    if not cli.wait_for_service(timeout_sec=3.0):
        print(f"服务不可用: {common.SVC_HOME}\n"
              f"  真机节点没起? ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py ...",
              file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    if not do_it:
        print(f"--dry-run(未加 --yes):将调用 {common.SVC_HOME}。")
        node.destroy_node(); rclpy.shutdown(); return 0

    print(f"调用 {common.SVC_HOME} ...")
    fut = cli.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if fut.done() and fut.result() is not None:
        print("调用完成。手臂应开始回零;用 status 观察角度变化。")
    else:
        print("调用超时或失败。检查节点的日志输出。", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
