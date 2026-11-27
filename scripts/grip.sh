#!/usr/bin/env bash
# =============================================================================
# 夹爪测试 —— 张/合到指定宽度
#
#   ./scripts/grip.sh 0.06     # 张到 60 mm
#   ./scripts/grip.sh 0.04     # 张到 40 mm
#   ./scripts/grip.sh 0.00     # 完全闭合
#   ./scripts/grip.sh 0.06 2.0 # 指定时长(秒)
#
# 为什么要开/关门闸:
#   agx_arm_control_gate 只监听 arm_controller 的 action。
#   夹爪走的是 gripper_controller,门闸不会自己开 → 必须手动开,用完立刻关。
#   脚本用 try/finally 保证异常退出时也把闸关上。
# =============================================================================
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WIDTH="${1:-0.06}"
DURATION="${2:-2.0}"

set +u
. /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] && . "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${PIPER_ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export LC_NUMERIC="${LC_NUMERIC:-C.UTF-8}"
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
mkdir -p "$ROS_LOG_DIR" 2>/dev/null

python3 - "$WIDTH" "$DURATION" 2>&1 <<'PY' | grep -vE 'RTPS|SHM Transport|Function|deprecated|localhost_only'
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState

WIDTH = float(sys.argv[1]); DUR = float(sys.argv[2])
if not (0.0 <= WIDTH <= 0.10):
    print(f"  [警告] 宽度 {WIDTH} 超出常见范围 [0, 0.10] m")

rclpy.init(); node = Node("grip_cmd")
gs = {}
node.create_subscription(JointState, "/feedback/joint_states",
                         lambda m: gs.update(dict(zip(m.name, m.position))), 10)

def spin_until(pred, timeout):
    t = time.time()
    while time.time() - t < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if pred(): return True
    return False

if not spin_until(lambda: 'gripper' in gs, 8):
    print("  ❌ 读不到夹爪反馈 —— 驱动起了吗? ros2 run piper_tools status")
    raise SystemExit(1)
print(f"  当前: {gs['gripper']*1000:+.1f} mm")

cli = node.create_client(SetBool, "/control_enable")
if not cli.wait_for_service(timeout_sec=10):
    print("  ❌ /control_enable 不可用 —— 驱动没起?"); raise SystemExit(1)

opened = False
try:
    f = cli.call_async(SetBool.Request(data=True))
    rclpy.spin_until_future_complete(node, f, timeout_sec=5)
    opened = bool(f.result() and f.result().success)
    print(f"  门闸: {'已打开' if opened else '打开失败'}")

    ac = ActionClient(node, FollowJointTrajectory,
                      "/gripper_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=15):
        print("  ❌ gripper_controller 15 秒内没出现(控制器 active 吗? ros2 control list_controllers)")
        raise SystemExit(1)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ["gripper"]
    pt = JointTrajectoryPoint()
    pt.positions = [WIDTH]
    pt.time_from_start.sec = int(DUR)
    pt.time_from_start.nanosec = int((DUR - int(DUR)) * 1e9)
    goal.trajectory.points = [pt]

    print(f"  目标: {WIDTH*1000:.0f} mm,用时 {DUR:.1f}s")
    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10)
    gh = send.result()
    if gh is None or not gh.accepted:
        print("  ❌ 目标被拒绝")
    else:
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res, timeout_sec=30)
        r = res.result()
        code = r.result.error_code if r else -999
        print(f"  结果: error_code={code} ({'SUCCESS' if code == 0 else 'FAILED'})")

    spin_until(lambda: False, 2.0)
    print(f"  现在: {gs['gripper']*1000:+.1f} mm")
finally:
    if opened:
        f = cli.call_async(SetBool.Request(data=False))
        rclpy.spin_until_future_complete(node, f, timeout_sec=5)
        msg = f.result().message if f.result() else '超时'
        print(f"  门闸: {msg}")
PY
