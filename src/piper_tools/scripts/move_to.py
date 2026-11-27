#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_to —— 给末端位姿(法兰),规划 → 检查 → 执行 → 闭环修正。

关于姿态约定(踩过的坑)
----------------------
工具**朝下** = `--rpy 0 180 0`(绕 Y 轴 180°)。
`--rpy 180 0 0` 是绕 X 轴滚转,不是朝下 —— 之前用错导致全图 IK 无解。

关于 TCP
--------
--xyz 是 **TCP**(两指夹持中心)位置;脚本沿工具轴后退 --tcp 米得到法兰目标。
--tcp 默认 0.1425(模型: flange→gripper_base 0.0045 + →手指 0.138)。
想直接给法兰坐标就写 --tcp 0。

闭环修正
--------
真机跟踪、机械间隙、TCP 估算都会带来几毫米误差。执行后用 TF 量实际位置,
把差值加到目标上再走一次,默认最多 3 次、收敛到 3 mm。
`--no-correct` 关闭。

用法
----
    ros2 run piper_tools move_to --xyz 0.25 0.0 0.15 --tcp 0 --dry-run
    ros2 run piper_tools move_to --xyz 0.25 0.0 0.15 --tcp 0 --vel 0.02
    ros2 run piper_tools move_to --xyz 0.25 0.0 0.15 --tcp 0 --no-correct

⚠️ 真机会动。首次务必先 --dry-run。
"""

import argparse
import copy
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                             BoundingVolume, MoveItErrorCodes)
import shape_msgs.msg as shape_msgs

import common


def rpy_to_quat(r_deg, p_deg, y_deg):
    r, p, y = (math.radians(v) for v in (r_deg, p_deg, y_deg))
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class MoveToNode(Node):
    def __init__(self):
        super().__init__("piper_move_to")
        self.joints = {}
        self.create_subscription(JointState, "/feedback/joint_states", self._on_fb, 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def _on_fb(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n in common.JOINT_NAMES:
                self.joints[n] = p

    def ready(self):
        return all(n in self.joints for n in common.JOINT_NAMES)

    def flange_xyz(self, timeout=3.0):
        """读 base_link → link6 的平移。失败返回 None。"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self.buffer.lookup_transform("base_link", "link6", rclpy.time.Time())
                t = tf.transform.translation
                return (t.x, t.y, t.z)
            except Exception:
                continue
        return None


def make_constraints(fx, fy, fz, quat, pos_tol, ang_tol):
    cons = Constraints()

    pc = PositionConstraint()
    pc.header.frame_id = "base_link"
    pc.link_name = "link6"
    pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
    bv = BoundingVolume()
    sph = shape_msgs.SolidPrimitive()
    sph.type = shape_msgs.SolidPrimitive.SPHERE
    sph.dimensions = [float(pos_tol)]
    bv.primitives = [sph]
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(fx), float(fy), float(fz)
    p.orientation.w = 1.0
    bv.primitive_poses = [p]
    pc.constraint_region = bv
    pc.weight = 1.0
    cons.position_constraints = [pc]

    oc = OrientationConstraint()
    oc.header.frame_id = "base_link"
    oc.link_name = "link6"
    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = quat
    oc.absolute_x_axis_tolerance = ang_tol
    oc.absolute_y_axis_tolerance = ang_tol
    oc.absolute_z_axis_tolerance = ang_tol
    oc.weight = 1.0
    cons.orientation_constraints = [oc]
    return cons


def main():
    ap = argparse.ArgumentParser(description="Piper 末端位姿规划 + 闭环修正")
    ap.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"), required=True,
                    help="TCP 目标位置(m)")
    ap.add_argument("--rpy", nargs=3, type=float, default=[0.0, 180.0, 0.0],
                    help="法兰姿态(度)。工具朝下 = 0 180 0(不是 180 0 0)")
    ap.add_argument("--tcp", type=float, default=0.1425, help="法兰到 TCP 距离(m),默认 0.1425")
    ap.add_argument("--vel", type=float, default=0.05)
    ap.add_argument("--acc", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=3.0, help="限位余量(度),小于则拒绝")
    ap.add_argument("--pos-tol", type=float, default=0.01, help="规划位置容差(m),默认 10mm")
    ap.add_argument("--ang-tol", type=float, default=0.1, help="规划姿态容差(rad),默认 0.1")
    ap.add_argument("--ae-tol", type=float, default=0.003, help="闭环收敛阈值(m),默认 3mm")
    ap.add_argument("--max-iter", type=int, default=3, help="闭环最多迭代次数")
    ap.add_argument("--no-correct", action="store_true", help="关闭闭环修正")
    ap.add_argument("--dry-run", action="store_true", help="只规划,不执行")
    ap.add_argument("--timeout", type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = MoveToNode()
    t0 = time.time()
    while rclpy.ok() and not node.ready() and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not node.ready():
        print("超时:读不到 6 轴反馈", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    quat = rpy_to_quat(*args.rpy)
    ox, oy, oz = quat_rotate(quat, (0.0, 0.0, args.tcp))
    tx, ty, tz = args.xyz
    # 初始法兰目标
    fx, fy, fz = tx - ox, ty - oy, tz - oz

    print(f"  TCP 目标   : ({tx:.4f},{ty:.4f},{tz:.4f}) rpy={args.rpy}")
    print(f"  TCP 偏移   : {args.tcp*1000:.1f} mm → 法兰 ({fx:.4f},{fy:.4f},{fz:.4f})")

    mcg = ActionClient(node, MoveGroup, "move_action")
    if not mcg.wait_for_server(timeout_sec=10):
        print("move_action 不可用(move_group 起了吗?)", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1
    exa = ActionClient(node, ExecuteTrajectory, "execute_trajectory")
    if not args.dry_run and not exa.wait_for_server(timeout_sec=10):
        print("execute_trajectory 不可用", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    # 记下原始目标,闭环时以它为基准
    gx, gy, gz = fx, fy, fz
    ok_all = True
    measured = None

    for it in range(1, (1 if args.no_correct or args.dry_run else args.max_iter) + 1):
        if it > 1:
            print(f"\n  --- 第 {it} 次修正 ---")
        print(f"  法兰目标   : ({fx:.4f},{fy:.4f},{fz:.4f})  规划中 ...")

        goal = MoveGroup.Goal()
        g = goal.request
        g.group_name = "arm"
        g.num_planning_attempts = 20
        g.allowed_planning_time = 8.0
        g.max_velocity_scaling_factor = args.vel
        g.max_acceleration_scaling_factor = args.acc
        goal.planning_options.plan_only = True
        g.goal_constraints.append(
            make_constraints(fx, fy, fz, quat, args.pos_tol, args.ang_tol))

        send = mcg.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=15)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("  ❌ 目标被拒绝"); ok_all = False; break
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res, timeout_sec=60)
        r = res.result()
        if r is None or r.result.error_code.val != MoveItErrorCodes.SUCCESS \
                or not r.result.planned_trajectory.joint_trajectory.points:
            code = r.result.error_code.val if r else "timeout"
            print(f"  ❌ 规划失败:error_code={code}")
            print("     -1 规划失败 / -2 无效 / -10 起始状态越界 / -31 无 IK 解")
            ok_all = False; break

        mt = copy.deepcopy(r.result.planned_trajectory)
        names = list(mt.joint_trajectory.joint_names)
        last = mt.joint_trajectory.points[-1]
        target = dict(zip(names, last.positions))
        dur = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        print(f"  ✅ {len(mt.joint_trajectory.points)} 点 / {dur:.1f} 秒")
        print("  末点关节  : " + "  ".join(
            f"j{n.replace('joint','')}={math.degrees(target[n]):+6.1f}°"
            for n in sorted(names, key=lambda s: int(s.replace('joint', '')) if s.replace('joint', '').isdigit() else 99)))

        bad, worst = [], None
        for n in names:
            if n not in common.LIMITS_DEG:
                continue
            lo, hi = common.LIMITS_DEG[n]
            deg = math.degrees(target[n])
            m = min(deg - lo, hi - deg)
            if worst is None or m < worst[1]:
                worst = (n, m)
            if m < args.margin:
                bad.append(f"{n}={deg:+.1f}° 余量 {m:.1f}°")
        if bad:
            print("  ❌ 拒绝执行:关节离限位太近")
            for b in bad:
                print(f"       {b}")
            ok_all = False; break
        if worst:
            print(f"  限位余量  : {worst[1]:.1f}° ({worst[0]})")

        if args.dry_run:
            print("  --dry-run:已规划但未执行")
            node.destroy_node(); rclpy.shutdown(); return 0

        print("  执行中 ...")
        eg = ExecuteTrajectory.Goal(); eg.trajectory = mt
        es = exa.send_goal_async(eg)
        rclpy.spin_until_future_complete(node, es, timeout_sec=15)
        egh = es.result()
        if egh is None or not egh.accepted:
            print("  ❌ 执行目标被拒绝"); ok_all = False; break
        er = egh.get_result_async()
        rclpy.spin_until_future_complete(node, er, timeout_sec=dur + 30)
        err = er.result()
        ok = err is not None and err.result.error_code.val == MoveItErrorCodes.SUCCESS
        print(f"  执行      : {'SUCCESS' if ok else 'FAILED'}")
        if not ok:
            ok_all = False; break

        # ---- 闭环:量实际 ----
        for _ in range(20):          # 等状态稳定
            rclpy.spin_once(node, timeout_sec=0.05)
        measured = node.flange_xyz()
        if measured is None:
            print("  读取 TF 失败,跳过闭环修正")
            break
        dx, dy, dz = gx - measured[0], gy - measured[1], gz - measured[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        print(f"  实测法兰  : ({measured[0]:.4f},{measured[1]:.4f},{measured[2]:.4f})")
        print(f"  偏差      : ({dx*1000:+.1f},{dy*1000:+.1f},{dz*1000:+.1f}) mm,|e|={dist*1000:.1f} mm")

        if args.no_correct:
            break
        if dist <= args.ae_tol:
            print(f"  ✅ 收敛(|e| ≤ {args.ae_tol*1000:.0f} mm)")
            break
        if it == args.max_iter:
            print(f"  ⚠️ 达到最大迭代次数 {args.max_iter},残留 {dist*1000:.1f} mm")
            break
        # 下一轮目标 = 目标 + 偏差(把实际往目标方向推)
        fx, fy, fz = fx + dx, fy + dy, fz + dz

    node.destroy_node(); rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
