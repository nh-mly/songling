#!/usr/bin/env bash
# =============================================================================
# 真机 + RViz 一键操作
#
#   ./scripts/real_arm.sh status     看状态(只读)
#   ./scripts/real_arm.sh check      全链路检查
#   ./scripts/real_arm.sh driver     起驱动【终端A,常驻】
#   ./scripts/real_arm.sh enable     使能
#   ./scripts/real_arm.sh home       回零(仅当 MoveIt 未跑)
#   ./scripts/real_arm.sh moveit     起 MoveIt+RViz【终端C,常驻】
#   ./scripts/real_arm.sh rviz       只起 RViz(接已在跑的 move_group)
#
# 内置三条安全规则(都是 2026-11-25 那次事故换来的):
#   1. 驱动默认 control_enabled:=false  → 门闸关着,谁发话题都推不动
#   2. MoveIt 默认 auto_control_gate:=true → 只有执行轨迹时才开门闸
#   3. 起 MoveIt 前检查真机是否接近零位   → mock 初始是 0,两边不一致会跳
# =============================================================================
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARM_TYPE="${ARM_TYPE:-piper}"
EFFECTOR="${EFFECTOR:-agx_gripper}"
CAN_PORT="${CAN_PORT:-can0}"
SPEED="${SPEED:-20}"

# ------------------------------------------------------------------ 环境
_env() {
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
}

die()  { printf '\033[31m[FAIL]\033[0m %s\n' "$*" >&2; exit 1; }
ok()   { printf '\033[32m[ OK ]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[WARN]\033[0m %s\n' "$*"; }
hdr()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
q()    { "$@" 2>/dev/null; }

_has_node()  { q ros2 node list | grep -qx "$1"; }
_driver_up() { q ros2 node list | grep -q 'agx_arm_ctrl'; }
_moveit_up() { q ros2 node list | grep -q 'move_group'; }

# 读真机六轴(rad),失败返回空
_real_rad() {
  # 读 6 轴(rad)。重试 3 次:刚起来时话题可能还没稳定
  local i out
  for i in 1 2 3; do
    out=$(timeout 8 ros2 topic echo /feedback/joint_states --once --field position 2>/dev/null \
          | grep -oE '\[[-0-9eE., ]+\]' | tr -d '[] ' | tr ',' '\n' | head -6)
    [ -n "$out" ] && { printf '%s\n' "$out"; return 0; }
    sleep 1
  done
  return 1
}

# ------------------------------------------------------------------ 子命令
cmd_status() {
  _env
  _driver_up || die "驱动没起(先 ./scripts/real_arm.sh driver)"
  ros2 run piper_tools status
}

cmd_check() {
  _env
  hdr "硬件"
  ip -br link show type can 2>/dev/null | sed 's/^/  /' || warn "没有 CAN 接口"
  lsusb 2>/dev/null | grep -q 1d50 && ok "USB-CAN 在位" || warn "没看到 1d50 适配器"

  hdr "节点"
  local nodes; nodes=$(q ros2 node list)
  for n in agx_arm_ctrl_single_node controller_manager move_group control_gate rviz; do
    echo "$nodes" | grep -q "$n" && ok "$n" || echo "  --  $n"
  done

  hdr "控制器"
  q ros2 control list_controllers | sed 's/^/  /'

  hdr "门闸(control_enabled,应为 False)"
  q ros2 param get /agx_arm_ctrl_single_node control_enabled | sed 's/^/  /'

  hdr "真机 vs mock"
  local real mock
  real=$(_real_rad)
  mock=$(q ros2 topic echo /control/joint_states --once --field position 2>/dev/null \
         | grep -o '\[[-0-9., e]*\]' | tr -d '[] ' | tr ',' '\n' \
         | grep -v '^$' | tail -6)
  if [ -n "$real" ]; then
    echo -n "  真机(deg): "
    echo "$real" | awk '{printf "%+7.2f ", $1*57.2958} END{print ""}'
  else
    warn "读不到真机反馈"
  fi
  if [ -n "$mock" ]; then
    echo -n "  mock(deg): "
    echo "$mock" | awk '{printf "%+7.2f ", $1*57.2958} END{print ""}'
  fi
  if [ -n "$real" ] && [ -n "$mock" ]; then
    echo -n "  差值(deg): "
    paste <(echo "$real") <(echo "$mock") | awk '{printf "%+7.2f ", ($1-$2)*57.2958} END{print ""}'
  fi
}

cmd_enable() {
  _env
  _driver_up || die "驱动没起"
  hdr "使能(关节上电锁住当前姿态,不会移动)"
  ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
  echo
  sleep 1
  ros2 run piper_tools status | grep 关节
}

cmd_home() {
  _env
  _driver_up || die "驱动没起"
  if _moveit_up; then
    cat >&2 <<'EOF'
[拒绝] MoveIt 正在运行,不能用 /move_home。

原因: /move_home 绕过门闸直连真机,而 MoveIt 的 mock 状态不会更新。
      两边错位后,下次点 Execute 会从"mock 以为的位置"开始 → 跳变。

正确做法: 在 RViz 里 Goal State 选 home,然后 Plan → Execute。
          或者先 Ctrl-C 掉 MoveIt,再回来跑这个命令。
EOF
    exit 1
  fi
  hdr "回零(MoveIt 未运行)"
  ros2 service call /move_home std_srvs/srv/Empty "{}"
  sleep 1
  ros2 run piper_tools status | grep 关节
}

cmd_driver() {
  _env
  hdr "前置检查"
  ip link show "$CAN_PORT" 2>/dev/null | grep -q 'UP' \
    || die "$CAN_PORT 没 up,先跑: sudo ./scripts/can_up.sh"
  ok "$CAN_PORT UP"
  ip -details link show "$CAN_PORT" 2>/dev/null | grep -q 'bitrate 1000000' \
    && ok "bitrate 1 Mbit" || warn "bitrate 不是 1000000"
  _driver_up && die "已经有一个驱动在跑,先 Ctrl-C 掉"

  hdr "启动驱动(control_enabled=false,门闸关闭)"
  echo "  arm_type=$ARM_TYPE  effector=$EFFECTOR  can=$CAN_PORT  speed=$SPEED%"
  echo
  exec ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    arm_type:="$ARM_TYPE" can_port:="$CAN_PORT" effector_type:="$EFFECTOR" \
    speed_percent:="$SPEED" auto_enable:=false control_enabled:=false
}

cmd_moveit() {
  _env
  hdr "前置检查"
  _driver_up || die "驱动没起(先 ./scripts/real_arm.sh driver)"
  ok "驱动在跑"

  if _moveit_up; then die "MoveIt 已经在跑,先 Ctrl-C 掉"; fi

  local real
  real=$(_real_rad)
  [ -n "$real" ] || die "读不到 /feedback/joint_states,驱动可能没连上 CAN"

  local maxdeg
  maxdeg=$(echo "$real" | awk '{v=$1*57.2958; if(v<0)v=-v; if(v>m)m=v} END{printf "%.1f", m+0}')
  echo -n "  真机当前(deg): "
  echo "$real" | awk '{printf "%+7.2f ", $1*57.2958} END{print ""}'
  if awk -v m="$maxdeg" 'BEGIN{exit !(m>5)}'; then
    warn "最大关节偏离零位 $maxdeg°"
    cat >&2 <<'EOF'

[拒绝启动] mock 硬件的初始位置是 0,真机不在零位时启动 MoveIt,
          第一次点 Execute 会先补这个差 → 大跳(2026-11-25 事故)。

先回零(在本终端另一个窗口):
    ./scripts/real_arm.sh home

如果机械臂已经处于安全姿态、你确实想从当前位置开始,
编辑 src/agx_arm_ros/src/agx_arm_moveit/config/initial_positions.yaml
把 joint1..6 改成当前值,然后 scripts/build.sh --packages-select agx_arm_moveit
EOF
    exit 1
  fi
  ok "真机接近零位(最大 $maxdeg°),与 mock 一致"

  hdr "启动 MoveIt + RViz(follow=true, auto_control_gate=true)"
  echo
  exec ros2 launch agx_arm_moveit demo.launch.py \
    arm_type:="$ARM_TYPE" effector_type:="$EFFECTOR" \
    follow:=true auto_control_gate:=true
}

cmd_rviz() {
  _env
  _moveit_up || die "move_group 没在跑,先用 moveit 子命令起整套"
  hdr "只起 RViz(接已在跑的 move_group)"
  exec ros2 launch agx_arm_moveit moveit_rviz.launch.py
}

# ------------------------------------------------------------------ 入口
case "${1:-help}" in
  status) cmd_status ;;
  check)  cmd_check ;;
  enable) cmd_enable ;;
  home)   cmd_home ;;
  driver) cmd_driver ;;
  moveit) cmd_moveit ;;
  rviz)   cmd_rviz ;;
  help|-h|--help)
    sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    ;;
  *) die "未知子命令: $1(试试 help)" ;;
esac
