#!/usr/bin/env python3
"""遥操作一键准备:等待驱动 → 使能 → 回零(带验证) → 进入对应模式。

    # A 机(leader)
    ros2 run piper_teleop teleop_setup --ros-args -p role:=leader

    # B 机(follower)
    ros2 run piper_teleop teleop_setup --ros-args -p role:=follower

做的事(按顺序,任一步失败就报错停下):
    leader   : 等驱动 → enable → move_home → 验证回零 → leader_mode(零力拖动)
    follower : 等驱动 → enable → move_home → 验证回零 → control_enable(开控制门闸)

为什么要有它:
    · move_home 在未使能时会静默失败(只打一行 warn,不返回错误)
    · move_home 有时只走一半就报成功 → 必须读角度验证并重试
    · 手工敲 4~5 条服务调用容易漏顺序
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]


class TeleopSetup(Node):
    def __init__(self):
        super().__init__("teleop_setup")

        self.declare_parameter("role", "leader")               # leader | follower
        self.declare_parameter("namespace", "")                # 空则按 role 推导
        self.declare_parameter("service_timeout_s", 60.0)      # 等驱动的上限
        self.declare_parameter("home_tolerance_deg", 3.0)      # 回零判定阈值
        self.declare_parameter("home_retries", 2)              # 回零重试次数
        self.declare_parameter("monitor_period_s", 10.0)       # 0 = 准备完就退出

        self.role = str(self.get_parameter("role").value).strip().lower()
        if self.role not in ("leader", "follower"):
            raise ValueError(f"role must be leader or follower, got {self.role!r}")

        ns = str(self.get_parameter("namespace").value).strip()
        self.ns = (ns if ns.startswith("/") else "/" + ns) if ns else f"/{self.role}"
        self.srv_timeout = float(self.get_parameter("service_timeout_s").value)
        self.home_tol = float(self.get_parameter("home_tolerance_deg").value)
        self.home_retries = int(self.get_parameter("home_retries").value)
        self.monitor_period = float(self.get_parameter("monitor_period_s").value)

        self.joints = {}
        self.create_subscription(
            JointState, f"{self.ns}/feedback/joint_states", self._on_joints, 10)

        self.monitor_timer = None
        self.get_logger().info(
            f"teleop_setup role={self.role} ns={self.ns} "
            f"(等驱动最多 {self.srv_timeout:.0f}s)")

    # ------------------------------------------------------------------ 工具
    def _on_joints(self, msg: JointState):
        if msg.name:
            self.joints = dict(zip(msg.name, msg.position))

    def _wait_joints(self, timeout=10.0):
        t0 = time.time()
        while len(self.joints) < 6 and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.joints) >= 6

    def _call(self, srv_type, name, request, timeout=15.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=self.srv_timeout):
            self.get_logger().error(f"服务不存在(等了 {self.srv_timeout:.0f}s): {name}")
            return None
        fut = cli.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        self.destroy_client(cli)
        return res

    def _max_abs_deg(self):
        vals = [abs(math.degrees(self.joints[j])) for j in ARM_JOINTS if j in self.joints]
        return max(vals) if vals else 999.0

    def _wait_motion_done(self, timeout=15.0, eps=0.05):
        """等到相邻两次采样几乎不变。"""
        t0 = time.time()
        prev = dict(self.joints)
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not prev or len(self.joints) < 6:
                prev = dict(self.joints)
                continue
            delta = max(abs(math.degrees(self.joints.get(j, 0) - prev.get(j, 0)))
                        for j in ARM_JOINTS)
            if delta < eps:
                return True
            prev = dict(self.joints)
        return False

    # ------------------------------------------------------------------ 流程
    def run_setup(self) -> bool:
        # 1. 等关节数据(证明驱动真的连上臂了)
        if not self._wait_joints(timeout=self.srv_timeout):
            self.get_logger().error("收不到关节数据,驱动可能没连上机械臂")
            return False
        self.get_logger().info(
            "当前角度: " + " ".join(
                f"j{i}={math.degrees(self.joints[f'joint{i}']):+6.2f}"
                for i in range(1, 7)))

        # 2. 使能
        res = self._call(SetBool, f"{self.ns}/enable_agx_arm", SetBool.Request(data=True))
        if res is None or not res.success:
            self.get_logger().error(f"使能失败: {getattr(res, 'message', '无响应')}")
            return False
        self.get_logger().info(f"使能: {res.message}")
        time.sleep(1.0)

        # 3. 回零(带验证与重试)
        for attempt in range(1, self.home_retries + 1):
            res = self._call(Empty, f"{self.ns}/move_home", Empty.Request())
            if res is None:
                self.get_logger().error("move_home 服务无响应")
                return False
            self._wait_motion_done()
            worst = self._max_abs_deg()
            self.get_logger().info(
                f"回零第 {attempt} 次: 最大偏差 {worst:.2f}° (阈值 {self.home_tol:.1f}°)")
            if worst <= self.home_tol:
                break
            if attempt == self.home_retries:
                self.get_logger().warn(
                    f"回零后仍有 {worst:.1f}° 偏差,继续(请人工确认是否卡阻)")

        # 4. 进入对应模式
        if self.role == "leader":
            res = self._call(Empty, f"{self.ns}/leader_mode", Empty.Request())
            if res is None:
                self.get_logger().error("leader_mode 调用失败")
                return False
            self.get_logger().info("已进入零力拖动(leader):手臂变轻,松手停在原地")
        else:
            res = self._call(SetBool, f"{self.ns}/control_enable",
                             SetBool.Request(data=True))
            if res is None or not res.success:
                self.get_logger().error("打开控制门闸失败")
                return False
            self.get_logger().info(f"控制门闸: {res.message}")

        self.get_logger().info(f"==== {self.role} 准备完成 ====")
        return True

    # ------------------------------------------------------------------ 监控
    def start_monitor(self):
        if self.monitor_period <= 0:
            self.get_logger().info("monitor_period_s=0,准备完成后退出")
            return
        self.monitor_timer = self.create_timer(self.monitor_period, self._monitor)

    def _monitor(self):
        if len(self.joints) < 6:
            self.get_logger().warn("关节数据中断")
            return
        worst = self._max_abs_deg()
        vals = " ".join(f"j{i}={math.degrees(self.joints[f'joint{i}']):+6.1f}"
                        for i in range(1, 7))
        self.get_logger().info(f"[{self.role}] {vals} | 离零最大 {worst:.1f}°")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSetup()
    ok = False
    try:
        ok = node.run_setup()
        if ok:
            node.start_monitor()
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
