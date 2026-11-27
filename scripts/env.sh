# 用法(zsh 或 bash 都可以):
#   source ~/107/songlin/scripts/env.sh
#
# 一次搞定:ROS 环境 + 工作区 overlay + 网络隔离 + locale。
# 已设置的值不会被覆盖(方便临时切 domain)。

_piper_srcfile="${BASH_SOURCE[0]:-$0}"
_piper_dir="$(cd "$(dirname "$_piper_srcfile")/.." && pwd)"

# ---------------------------------------------------------------- ROS
if [ -n "${ZSH_VERSION:-}" ] && [ -f /opt/ros/jazzy/setup.zsh ]; then
  . /opt/ros/jazzy/setup.zsh
elif [ -n "${BASH_VERSION:-}" ]; then
  . /opt/ros/jazzy/setup.bash
else
  . /opt/ros/jazzy/setup.sh
fi

# ---------------------------------------------------------------- 工作区
if [ -f "$_piper_dir/install/setup.bash" ]; then
  if [ -n "${ZSH_VERSION:-}" ]; then
    . "$_piper_dir/install/setup.zsh"
  else
    . "$_piper_dir/install/setup.bash"
  fi
else
  echo "[env.sh] 警告: $_piper_dir/install 不存在,先跑 scripts/build.sh" >&2
fi

# ---------------------------------------------------------------- 网络隔离
# 实验室同网段有人在跑 piper_driver / mujoco_arm,domain 0 上会互相污染:
#  - 它往 /joint_states 发 joint1..joint8 → RViz 报 "joint7 is not known" 直接崩
#  - 它订阅 /control/joint_states → 我们 MoveIt 的指令可能打到别人的机械臂上
# domain 可以用 PIPER_ROS_DOMAIN_ID 覆盖;discovery 强制 LOCALHOST(隔离是安全要求)
export ROS_DOMAIN_ID="${PIPER_ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# 需要连局域网其他机器时:source 之后手动
#   export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# ---------------------------------------------------------------- locale
# MoveIt 在"小数点是逗号"的 locale 下会崩。C.UTF-8 / en_US.UTF-8 都安全。
case "${LC_NUMERIC:-}" in
  C.UTF-8|en_US.UTF-8|POSIX) : ;;
  *) export LC_NUMERIC=en_US.UTF-8 ;;
esac

# ---------------------------------------------------------------- PATH
case ":$PATH:" in
  *":$HOME/.local/bin:"*) ;;
  *) export PATH="$HOME/.local/bin:$PATH" ;;
esac

unset _piper_srcfile _piper_dir

echo "[env.sh] ROS_DISTRO=$ROS_DISTRO  DOMAIN=$ROS_DOMAIN_ID  RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
