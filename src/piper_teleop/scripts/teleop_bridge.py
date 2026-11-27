#!/usr/bin/env python3
"""主从遥操作桥:把 leader 的关节角转发给 follower。

数据流
    /leader/feedback/joint_states   (leader 手拖出来的角度, 200 Hz)
        -> 死区 -> 低通 -> 限速截断 -> 发布
    /follower/control/joint_states  (follower 的位置指令)

安全设计
    · 看门狗 PAUSE : 超过 watchdog_ms 没有新帧 -> 停发(follower 保持原位);
                     数据恢复后自动继续,不会跳。
    · 限速        : 每帧相对"上次发出的值"最多变化 max_step_deg(硬截断)。
    · 跳变 FAULT  : 单帧原始变化超过 max_step*jump_factor -> 锁死停发,
                     必须显式调用 /teleop_reset 才能恢复(防误动作)。
    · 整组下发    : 每帧发全 joint1..6(+gripper),缺的名字不会被当成 0。

用法
    ros2 run piper_teleop teleop_bridge
    ros2 run piper_teleop teleop_bridge --ros-args \
        -p max_step_deg:=1.5 -p rate:=50.0 -p watchdog_ms:=300

    ros2 service call /teleop_reset std_srvs/srv/Trigger "{}"
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER = "gripper"


class TeleopBridge(Node):
    def __init__(self):
        super().__init__("teleop_bridge")

        # ---------------- 参数 ----------------
        self.declare_parameter("src_topic", "/leader/feedback/joint_states")
        self.declare_parameter("dst_topic", "/follower/control/joint_states")
        self.declare_parameter("rate", 50.0)                  # 转发频率 Hz( WiFi 用 50)
        self.declare_parameter("max_step_deg", 1.5)           # 每帧最大变化
        self.declare_parameter("max_vel_deg_s", 600.0)        # leader 角速度上限(判异常)
        self.declare_parameter("deadband_deg", 0.05)
        self.declare_parameter("alpha", 0.2)                  # 一阶低通, 1.0 = 关
        self.declare_parameter("watchdog_ms", 300.0)
        self.declare_parameter("joint_offset_deg", [0.0] * 6)  # 零位补偿
        self.declare_parameter("passthrough_gripper", True)
        self.declare_parameter("report_period_s", 5.0)
        self.declare_parameter("follower_fb_topic", "/follower/feedback/joint_states")
        self.declare_parameter("init_max_diff_deg", 15.0)   # 开局姿态差上限

        gp = self.get_parameter
        self.src_topic = gp("src_topic").value
        self.dst_topic = gp("dst_topic").value
        self.rate = float(gp("rate").value)
        self.max_step = math.radians(float(gp("max_step_deg").value))
        self.max_vel = math.radians(float(gp("max_vel_deg_s").value))
        self.deadband = math.radians(float(gp("deadband_deg").value))
        self.alpha = float(gp("alpha").value)
        self.watchdog_s = float(gp("watchdog_ms").value) / 1000.0
        self.offsets = [math.radians(v) for v in gp("joint_offset_deg").value]
        self.use_gripper = bool(gp("passthrough_gripper").value)
        self.report_period = float(gp("report_period_s").value)
        self.follower_fb_topic = gp("follower_fb_topic").value
        self.init_max_diff = math.radians(float(gp("init_max_diff_deg").value))

        # ---------------- 状态 ----------------
        self.latest = None            # 最新的 leader 值 {name: rad}
        self.latest_stamp = 0.0
        self.last_sent = None         # 上次真正发出的 rad 列表
        self.last_target = None       # 上次收到的 leader 目标
        self.last_target_t = 0.0
        self.paused = True            # 看门狗触发时为 True(可自动恢复)
        self.fault = False            # 跳变锁死(需 /teleop_reset)
        self.started = False
        self.sent_count = 0
        self.skip_count = 0
        self.last_report = time.monotonic()

        # ---------------- 通信 ----------------
        sub_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, self.src_topic, self._on_leader, sub_qos)
        self.pub = self.create_publisher(JointState, self.dst_topic, 10)
        self.follower_pos = None
        self.create_subscription(JointState, self.follower_fb_topic,
                                 self._on_follower, sub_qos)
        self.create_service(Trigger, "teleop_reset", self._on_reset)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self.get_logger().info(
            f"bridge {self.src_topic} -> {self.dst_topic} @ {self.rate:.0f} Hz | "
            f"max_step={math.degrees(self.max_step):.2f}deg/"
            f"{self.rate:.0f}Hz max_vel={math.degrees(self.max_vel):.0f}deg/s "
            f"alpha={self.alpha:.2f} watchdog={self.watchdog_s*1000:.0f}ms")

    # ------------------------------------------------------------------
    def _on_leader(self, msg: JointState):
        if not msg.name:
            return
        self.latest = dict(zip(msg.name, msg.position))
        self.latest_stamp = time.monotonic()

    def _on_follower(self, msg: JointState):
        if msg.name:
            self.follower_pos = dict(zip(msg.name, msg.position))

    def _on_reset(self, request, response):
        self.fault = False
        self.paused = True
        self.last_sent = None      # 下一帧以当前 leader 值为基准,避免跳变
        self.last_target = None
        self.get_logger().info("teleop reset: FAULT 已清除,等待 leader 数据重新对齐")
        response.success = True
        response.message = "fault cleared"
        return response

    def _target_vector(self):
        """leader 最新值 + 零位补偿。缺名字时返回 None(整组不发)。"""
        out = []
        for i, name in enumerate(ARM_JOINTS):
            v = self.latest.get(name)
            if v is None:
                return None
            out.append(v + self.offsets[i])
        if self.use_gripper:
            v = self.latest.get(GRIPPER)
            if v is None:
                return None
            out.append(v)
        return out

    # ------------------------------------------------------------------
    def _tick(self):
        now = time.monotonic()

        if self.fault:
            return
        if self.latest is None:
            return

        # --- 看门狗:停发,但可自动恢复 ---
        if now - self.latest_stamp > self.watchdog_s:
            if not self.paused:
                self.get_logger().warn(
                    f"watchdog: {self.watchdog_s*1000:.0f} ms 无新帧,暂停转发")
            self.paused = True
            return
        if self.paused:
            self.paused = False
            self.last_sent = None   # 重新对齐,避免恢复瞬间跳变
            self.get_logger().info("leader 数据恢复,重新对齐后继续")

        target = self._target_vector()
        if target is None:
            return

        # 首次:检查 follower 与 leader 的姿态差,过大就拒绝启动
        if self.last_sent is None:
            if self.follower_pos is not None:
                diffs = []
                for i, name in enumerate(ARM_JOINTS + ([GRIPPER] if self.use_gripper else [])):
                    v = self.follower_pos.get(name)
                    if v is not None:
                        diffs.append(abs(target[i] - v))
                if diffs:
                    worst = max(diffs)
                    if worst > self.init_max_diff:
                        self.fault = True
                        worst_name = (ARM_JOINTS + ([GRIPPER] if self.use_gripper else []))[
                            diffs.index(worst)]
                        self.get_logger().error(
                            f"FAULT: 开局姿态差 {math.degrees(worst):.1f}deg > "
                            f"{math.degrees(self.init_max_diff):.1f}deg(最差 {worst_name})。"
                            f"请先把两台摆到接近的姿态,再 "
                            f"ros2 service call /teleop_reset std_srvs/srv/Trigger")
                        return
            self.last_sent = list(target)
            if not self.started:
                self.started = True
                self.get_logger().info("已对齐,开始转发(TELEOP)")
            return

        # --- 异常检测:按角速度(而不是每帧变化) ---
        # 拖得快是合法的,只有超过 max_vel_deg_s 才认为是数据异常
        if self.last_target is not None:
            dt = max(now - self.last_target_t, 1e-3)
            vel = max(abs(t - s) for t, s in zip(target, self.last_target)) / dt
            if vel > self.max_vel:
                self.fault = True
                self.get_logger().error(
                    f"FAULT: leader 角速度 {math.degrees(vel):.0f}deg/s > "
                    f"{math.degrees(self.max_vel):.0f}deg/s,已停发。"
                    f"确认安全后 ros2 service call /teleop_reset std_srvs/srv/Trigger")
                return
        self.last_target = list(target)
        self.last_target_t = now

        # --- 低通 ---
        if self.alpha < 1.0:
            filt = [self.alpha * t + (1.0 - self.alpha) * s
                    for t, s in zip(target, self.last_sent)]
        else:
            filt = list(target)

        # --- 限速 ---
        limited = [s + math.copysign(min(abs(t - s), self.max_step), t - s)
                   for t, s in zip(filt, self.last_sent)]

        # --- 死区 ---
        if max(abs(t - s) for t, s in zip(limited, self.last_sent)) < self.deadband:
            self.skip_count += 1
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + ([GRIPPER] if self.use_gripper else [])
        msg.position = limited
        self.pub.publish(msg)

        self.last_sent = limited
        self.sent_count += 1

        # --- 周期汇报 ---
        if now - self.last_report >= self.report_period:
            self.last_report = now
            lag = max(abs(a - b) for a, b in zip(target, limited))
            self.get_logger().info(
                f"已发 {self.sent_count} 帧 | 跳过 {self.skip_count} | "
                f"本帧滞后 {math.degrees(lag):.2f}deg")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
