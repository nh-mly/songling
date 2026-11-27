#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 MoveIt 的规划场景加一张"桌面",让规划器知道桌子存在。

为什么需要
----------
默认规划场景里什么都没有 —— 桌面是"空气"。所以规划器会给出"合法"但穿桌的轨迹
(2026-11-26 真机就是这么撞上的)。

桌子按 base_link 的 z=0 平面建模(机械臂底座就装在桌面上)。
加完之后:碰撞轨迹会被拒绝,`move_to` 会规划失败而不是撞上去。

用法
----
    ros2 run piper_tools add_table                 # 默认 1.2×1.2 m,顶面 z=0
    ros2 run piper_tools add_table --size 0.8      # 改尺寸
    ros2 run piper_tools add_table --top 0.005     # 桌面顶面抬高 5mm(留余量)
    ros2 run piper_tools add_table --remove        # 移除
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene

TABLE_ID = "table"


def main():
    ap = argparse.ArgumentParser(description="给规划场景加桌面")
    ap.add_argument("--size", type=float, default=1.2, help="桌面边长(m),默认 1.2")
    ap.add_argument("--thickness", type=float, default=0.05, help="板厚(m),默认 0.05")
    ap.add_argument("--top", type=float, default=0.0, help="顶面高度(m),默认 0(= base_link 平面)")
    ap.add_argument("--remove", action="store_true", help="移除桌面")
    args = ap.parse_args()

    rclpy.init()
    node = Node("add_table")

    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("  ❌ /apply_planning_scene 不可用(move_group 起了吗?)", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    scene = PlanningScene()
    scene.is_diff = True
    obj = CollisionObject()
    obj.header.frame_id = "base_link"
    obj.id = TABLE_ID

    if args.remove:
        obj.operation = CollisionObject.REMOVE
        print("  移除桌面 ...")
    else:
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(args.size), float(args.size), float(args.thickness)]
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        # 让顶面落在 --top
        pose.position.z = float(args.top) - args.thickness / 2.0
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
        print(f"  桌面: {args.size}×{args.size} m,顶面 z={args.top*1000:.1f} mm,"
              f"厚 {args.thickness*1000:.0f} mm")

    scene.world.collision_objects = [obj]

    req = ApplyPlanningScene.Request()
    req.scene = scene
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    ok = res is not None and res.success
    print(f"  {'✅ 已应用' if ok else '❌ 应用失败'}")
    node.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
