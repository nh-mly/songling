#!/usr/bin/env bash
# =============================================================================
# 编译包装 —— 避免"在根目录 colcon build 把 vendor 里的东西也扫进来"
#
#   ./scripts/build.sh                       # 全部(仅 src/)
#   ./scripts/build.sh --packages-select piper_tools
#   ./scripts/build.sh --clean               # 先删 build/install/log
# =============================================================================
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

CLEAN=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --clean) CLEAN=1 ;;
    *) ARGS+=("$a") ;;
  esac
done

if [ "$CLEAN" = "1" ]; then
  echo "清理 build/ install/ log/ ..."
  rm -rf "$WS/build" "$WS/install" "$WS/log"
fi

set +u
. /opt/ros/jazzy/setup.bash
set -u

echo "工作区: $WS"
echo "源目录: $WS/src"
colcon build --base-paths "$WS/src" --symlink-install "${ARGS[@]}"

echo
echo "完成。使用前:"
echo "  source $WS/install/setup.bash"
