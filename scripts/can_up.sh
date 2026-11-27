#!/usr/bin/env bash
# =============================================================================
# 激活 CAN 并验证 —— 封装官方 can_config.sh,避免波特率写错
#
#   sudo ./scripts/can_up.sh              # can0, 1 Mbit(默认)
#   sudo ./scripts/can_up.sh can1         # 指定接口名
#   sudo ./scripts/can_up.sh can0 1000000 # 指定波特率(不要改,Piper 固定 1M)
#
# 只做三件事:查模块 → 激活 → 验帧。不改 /etc,不加 udev 规则,重启后需重跑。
# =============================================================================
set -uo pipefail
IFACE="${1:-can0}"
BITRATE="${2:-1000000}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

die()  { printf '\033[31m[FAIL]\033[0m %s\n' "$*"; exit 1; }
ok()   { printf '\033[32m[ OK ]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[WARN]\033[0m %s\n' "$*"; }

[ "$(id -u)" -eq 0 ] || die "需要 root: sudo $0 $*"

echo "[1/3] 检查 USB-CAN 模块"
if ! ip link show type can 2>/dev/null | grep -q .; then
  warn "没有检测到任何 CAN 接口"
  echo "  USB 设备:"
  lsusb 2>/dev/null | grep -iE 'can|ch340|cp210|8dev|peak|innomaker|zlg' || echo "    (没看到疑似 USB-CAN)"
  echo "  内核日志尾部:"
  dmesg 2>/dev/null | tail -5 | sed 's/^/    /'
  echo
  echo "  排查顺序:"
  echo "    1) 模块插紧了吗  lsusb"
  echo "    2) 驱动加载了吗  lsmod | grep gs_usb   (没有就 sudo modprobe gs_usb)"
  echo "    3) 权限          见 /etc/udev/rules.d(非 root 用户需要 udev 规则)"
  die "放弃激活"
fi
ok "检测到: $(ip -br link show type can | awk '{print $1}' | tr '\n' ' ')"

echo
echo "[2/3] 激活 $IFACE @ ${BITRATE}"
OFFICIAL="$WS/src/agx_arm_ros/scripts/can_config.sh"
if [ -f "$OFFICIAL" ]; then
  echo "  用官方脚本: $OFFICIAL"
  bash "$OFFICIAL" "$IFACE" "$BITRATE" || warn "官方脚本返回非 0,继续用 ip link 兜底"
fi

# 兜底:不依赖官方脚本的行为
if ! ip -details link show "$IFACE" 2>/dev/null | grep -q "bitrate $BITRATE"; then
  echo "  手动配置 ip link"
  ip link set "$IFACE" down 2>/dev/null
  ip link set "$IFACE" type can bitrate "$BITRATE" || die "设置 bitrate 失败"
  ip link set "$IFACE" up || die "up 失败"
fi

ACTUAL=$(ip -details link show "$IFACE" 2>/dev/null | grep -oP 'bitrate \K\d+')
if [ "${ACTUAL:-}" = "$BITRATE" ]; then
  ok "$IFACE up,bitrate=$ACTUAL"
else
  die "bitrate 不符: 期望 $BITRATE,实际 ${ACTUAL:-未知}"
fi

echo
echo "[3/3] 抓帧验证(3 秒)"
command -v candump >/dev/null || die "没装 can-utils"
COUNT=$(timeout 3 candump -n 20 "$IFACE" 2>/dev/null | tee /tmp/can_probe.txt | wc -l)
if [ "$COUNT" -gt 0 ]; then
  ok "$COUNT 帧"
  echo "  看到的 ID:"
  awk '{print $2}' /tmp/can_probe.txt | sort -u | head -8 | sed 's/^/    /'
  echo
  # 不同固件的关节反馈 ID 不同,只要求至少出现一类状态/关节帧
  if grep -qiE '(^|[[:space:]])(2A1|2A5|2A6|2A7|2A8)([[:space:]]|$)' /tmp/can_probe.txt; then
    ok "检测到状态/关节反馈帧"
  else
    warn "只看到其它 ID,没看到 2A1/2A5~2A8"
    echo "  按顺序查:"
    echo "    1) 机械臂上电了吗(24V,<=26V)"
    echo "    2) 急停按钮是否被拍下"
    echo "    3) CAN H/L 是否接反"
    echo "    4) 波特率是否 1 Mbit"
    echo "    5) 固件是否与脚本预期不同(用驱动日志的 fw_version 确认)"
  fi
else
  warn "3 秒内 0 帧"
  echo "  接口是 up 的但没数据,按顺序查:"
  echo "    1) 机械臂上电了吗(24V,≤26V)"
  echo "    2) 急停按钮是不是被拍下了"
  echo "    3) CAN H/L 是否接反"
  echo "    4) 波特率是否 1 Mbit(不是 500k)"
  echo "    5) 是不是插到了别的 can 口(ls /sys/class/net | grep can)"
fi
