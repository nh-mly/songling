#!/usr/bin/env bash
# =============================================================================
# Piper 工作区健康检查 —— 一条命令回答"模型/包/xacro/mesh 有没有问题"
#
#   用法:  ./scripts/check.sh            # 自动定位工作区(脚本上一级)
#          ./scripts/check.sh ~/107/songlin
#
# 检查内容(按"踩过的坑"顺序):
#   1. 目录与子模块      —— 子模块空着会 build 成功但 RViz 没模型
#   2. ROS 环境与包      —— 4 个包是否可被发现
#   3. xacro 展开        —— 展开失败 = 启动必崩
#   4. URDF 结构         —— check_urdf
#   5. mesh 可达性       —— package:// 全部指向真实文件
#   6. 工具包            —— piper_tools 是否装上
# =============================================================================
set -uo pipefail

WS="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
FAIL=0
OK()   { printf '  \033[32m OK \033[0m %s\n' "$*"; }
BAD()  { printf '  \033[31mFAIL\033[0m %s\n' "$*"; FAIL=1; }
WARN() { printf '  \033[33mWARN\033[0m %s\n' "$*"; }

echo "工作区: $WS"
echo

# ---------------------------------------------------------------- 1. 目录
echo "[1/6] 目录与子模块"
# 兼容两种布局: src/agx_arm_ros(推荐) 或 根目录 agx_arm_ros(未整理)
ARM_DIR="$WS/src/agx_arm_ros"
[ -d "$ARM_DIR" ] || ARM_DIR="$WS/agx_arm_ros"
URDF_DIR="$ARM_DIR/src/agx_arm_description/agx_arm_urdf"
[ -d "$ARM_DIR" ]            || BAD "找不到 agx_arm_ros(既不在 src/ 也不在根目录)"
[ -d "$URDF_DIR" ]           || BAD "缺少 agx_arm_urdf 子模块目录"
if [ -d "$URDF_DIR" ]; then
  n=$(ls -A "$URDF_DIR" 2>/dev/null | wc -l)
  if [ "$n" -eq 0 ]; then
    BAD "子模块是空的(0 个条目) → build 会成功,但 RViz 只有坐标系。跑: cd src/agx_arm_ros && git submodule update --init --recursive"
  else
    OK "子模块 $n 个条目"
  fi
  m=$(find "$URDF_DIR" -name '*.stl' -o -name '*.STL' -o -name '*.dae' 2>/dev/null | wc -l)
  [ "$m" -gt 0 ] && OK "mesh 文件 $m 个" || BAD "没有 mesh 文件"
fi

# ---------------------------------------------------------------- 2. 环境与包
echo
echo "[2/6] ROS 环境与包"
# shellcheck disable=SC1091
set +u
. /opt/ros/jazzy/setup.bash 2>/dev/null || BAD "找不到 /opt/ros/jazzy/setup.bash"
set -u
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u; . "$WS/install/setup.bash"; set -u
  OK "已 source $WS/install/setup.bash"
else
  WARN "还没编译(无 install/setup.bash),下面会退回到源码路径"
fi
for p in agx_arm_msgs agx_arm_ctrl agx_arm_description agx_arm_moveit; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then OK "包 $p"; else BAD "包 $p 找不到(没编译?)"; fi
done

# ---------------------------------------------------------------- 3. xacro
echo
echo "[3/6] xacro 展开"
XACRO_SRC="$URDF_DIR/piper/urdf/piper_with_gripper_description.xacro"
XACRO_INST="$(ros2 pkg prefix agx_arm_description 2>/dev/null)/share/agx_arm_description/agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro"
if [ -f "$XACRO_INST" ]; then XACRO_FILE="$XACRO_INST"; else XACRO_FILE="$XACRO_SRC"; fi
TMP_URDF=$(mktemp /tmp/piper_check_XXXX.urdf)
if command -v xacro >/dev/null 2>&1; then
  if xacro "$XACRO_FILE" > "$TMP_URDF" 2>/tmp/piper_xacro_err.txt; then
    OK "展开成功 ($(wc -l < "$TMP_URDF") 行): $(basename "$XACRO_FILE")"
  else
    BAD "xacro 展开失败,见 /tmp/piper_xacro_err.txt"; tail -3 /tmp/piper_xacro_err.txt | sed 's/^/       /'
  fi
else
  BAD "没装 xacro(sudo apt install ros-jazzy-xacro)"
fi

# ---------------------------------------------------------------- 4. URDF
echo
echo "[4/6] URDF 结构"
if command -v check_urdf >/dev/null 2>&1 && [ -s "$TMP_URDF" ]; then
  if check_urdf "$TMP_URDF" >/tmp/piper_check_urdf.txt 2>&1; then
    OK "check_urdf 通过"
  else
    BAD "check_urdf 失败"; tail -3 /tmp/piper_check_urdf.txt | sed 's/^/       /'
  fi
else
  WARN "没有 check_urdf,跳过"
fi

# ---------------------------------------------------------------- 5. mesh
echo
echo "[5/6] mesh 可达性(package:// 解析)"
if [ -s "$TMP_URDF" ]; then
  miss=0; total=0
  while IFS= read -r uri; do
    [ -z "$uri" ] && continue
    total=$((total + 1))
    pkg=${uri#package://}; pkg=${pkg%%/*}
    rest=${uri#package://"$pkg"/}
    prefix=$(ros2 pkg prefix "$pkg" 2>/dev/null)
    if [ -z "$prefix" ] || [ ! -f "$prefix/share/$pkg/$rest" ]; then
      miss=$((miss + 1))
      [ "$miss" -le 3 ] && WARN "缺失: $uri"
    fi
  done < <(grep -o 'package://[^"]*' "$TMP_URDF" | sort -u)
  if [ "$miss" -eq 0 ]; then OK "$total 个 mesh 引用全部可解析"
  else BAD "$miss / $total 个 mesh 引用找不到"; fi
else
  WARN "URDF 未生成,跳过"
fi
rm -f "$TMP_URDF"

# ---------------------------------------------------------------- 6. 工具
echo
echo "[6/6] piper_tools"
if ros2 pkg prefix piper_tools >/dev/null 2>&1; then
  OK "包 piper_tools 已装"
  for e in status jog reset; do
    if ros2 pkg executables piper_tools 2>/dev/null | grep -q " $e$"; then OK "  可执行 $e"; else WARN "  缺可执行 $e"; fi
  done
else
  WARN "piper_tools 未编译(可选)"
fi

# ---------------------------------------------------------------- 汇总
echo
if [ "$FAIL" -eq 0 ]; then
  printf '\033[32m全部通过\033[0m\n'; exit 0
else
  printf '\033[31m有检查未通过\033[0m\n'; exit 1
fi
