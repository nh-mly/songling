#!/usr/bin/env python3
"""遥操作一键准备:等待驱动 → 使能 → 回零(带验证) → 进入对应模式。

    # A 机(leader)
    ros2 run piper_teleop teleop_setup --ros-args -p role:=leader

    # B 机(follower)
    ros2 run piper_teleop teleop_setup --ros-args -p role:=follower

做的事(按顺序,任一步失败就报错停下):
    leader   : 等驱动 → 规范化(0xFC) → enable → 开门闸 → 回零 → 验证 → leader_mode(零力拖动)
    follower : 等驱动 → 规范化(0xFC) → enable → 开门闸 → 回零 → 验证

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

    def _wait_motion_done(self, timeout=15.0, eps=0.05, min_wait=1.5, stable_n=5):
        """等到运动结束。

        注意:必须有 min_wait —— 指令刚发出时臂还没开始动,
        立刻判断会误认为"已到位"(这是 move_home 报成功却没动的根源)。
        还需要连续 stable_n 次采样都稳定,避免单帧抖动误判。
        """
        t0 = time.time()
        prev = dict(self.joints)
        stable = 0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - t0
            if not prev or len(self.joints) < 6:
                prev = dict(self.joints)
                continue
            delta = max(abs(math.degrees(self.joints.get(j, 0) - prev.get(j, 0)))
                        for j in ARM_JOINTS)
            if elapsed > min_wait and delta < eps:
                stable += 1
                if stable >= stable_n:
                    return True
            else:
                stable = 0
            prev = dict(self.joints)
        return False

    def _publish_zeros(self, ns, duration=0.6):
        """通过 control/move_j 把六个关节命令到 0(比 move_home 服务可靠)。"""
        from std_msgs.msg import Header
        pub = self.create_publisher(JointState, f"{ns}/control/move_j", 10)
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINTS)
        msg.position = [0.0] * 6
        t0 = time.time()
        while time.time() - t0 < duration:
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.destroy_publisher(pub)

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

        # 1.5 规范化:清除上一次遗留的 Leader 模式
        # set_leader_mode() 发的 0xFA 会被控制器持久化,下次上电臂仍处于 Leader 模式,
        # 此时它把关节角当"控制帧(0x15x)"广播,不发反馈帧(0x2Ax),
        # 驱动会报 "Failed to get firmware version" 并退出。
        # 修复:先设回 Follower 模式(0xFC)。注意 0x00 是空操作,不能用来复位。
        res = self._call(Empty, f"{self.ns}/follower_mode", Empty.Request())
        if res is not None:
            self.get_logger().info("已规范化:清除遗留的 Leader 模式(0xFC)")
        time.sleep(0.5)

        # 2. 使能
        res = self._call(SetBool, f"{self.ns}/enable_agx_arm", SetBool.Request(data=True))
        if res is None or not res.success:
            self.get_logger().error(f"使能失败: {getattr(res, 'message', '无响应')}")
            return False
        self.get_logger().info(f"使能: {res.message}")
        time.sleep(1.0)

        # 2.5 打开控制门闸(后面用 control/move_j 回零需要它)
        res = self._call(SetBool, f"{self.ns}/control_enable", SetBool.Request(data=True))
        if res is not None:
            self.get_logger().info(f"控制门闸: {res.message}")

        # 3. 回零(带验证与重试)
        # 不用 move_home 服务:它调 move_j 后立即判断"没动=到位",
        # 导致日志报成功但臂没动(实测踩过)。改走 control/move_j + 显式等待。
        for attempt in range(1, self.home_retries + 1):
            self._publish_zeros(self.ns)
            self._wait_motion_done()
            worst = self._max_abs_deg()
            self.get_logger().info(
                f"回零第 {attempt} 次: 最大偏差 {worst:.2f}° (阈值 {self.home_tol:.1f}°)")
            if worst <= self.home_tol:
                break
            if attempt == self.home_retries:
                # 回零失败绝不能继续进零力/跟随模式:
                # 姿态未知时把臂交给桥,可能造成大幅扫动或撞机
                self.get_logger().error(
                    f"回零失败: 重试 {self.home_retries} 次后仍有 {worst:.1f}° 偏差"
                    f"(阈值 {self.home_tol:.1f}°)。已中止,不进入 {self.role} 模式。"
                    f"请检查: 机械臂是否卡住/被压住? 零位是否被外力挡住?")
                return False

        # 4. 进入对应模式
        if self.role == "leader":
            # 注意:不要调用 leader_mode(0xFA)!
            # 实测 0xFA 会被控制器持久化,且该模式下臂不再发 0x2Ax 反馈帧,
            # 驱动会判定 "Agx_arm feedback lost" 并停发所有数据(包括 leader 数据)。
            # 正确做法:臂保持普通模式,由人按下机械臂上的拖动/示教按键开启重力补偿,
            # 此时反馈帧照常上报(实测 200 Hz),驱动可正常读取。
            self.get_logger().info("=" * 56)
            self.get_logger().info("现在请按下机械臂上的【拖动/示教按键】开启重力补偿")
            self.get_logger().info("(LED 亮起即生效,臂会变轻,可手拖)")
            self.get_logger().info("=" * 56)
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
