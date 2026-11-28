#!/usr/bin/env bash
# =============================================================================
# 遥操作环境（bash / zsh 通用）
#
#   source ~/107/songlin/scripts/teleop_env.sh
#
# 做三件事:
#   1. 加载 ROS 2 环境
#   2. 加载工作区 install/
#   3. 设跨机通信参数(domain 66 + SUBNET)
#
# 注意:旧版用 ${BASH_SOURCE[0]} 推导路径,在 zsh 下为空导致找不到工作区,
#      现改为固定路径 $HOME/107/songlin,并区分 zsh/bash 的 setup 文件。
# =============================================================================

_ws="${HOME}/107/songlin"

# 1. ROS 2
if [ -n "${ZSH_VERSION:-}" ]; then
  . /opt/ros/jazzy/setup.zsh
else
  . /opt/ros/jazzy/setup.bash
fi

# 2. 工作区
if [ -n "${ZSH_VERSION:-}" ] && [ -f "${_ws}/install/setup.zsh" ]; then
  . "${_ws}/install/setup.zsh"
elif [ -f "${_ws}/install/setup.bash" ]; then
  . "${_ws}/install/setup.bash"
else
  echo "[teleop_env] 警告: 找不到 ${_ws}/install/setup.*，工作区可能没编译" >&2
fi

# 3. 跨机通信
export ROS_DOMAIN_ID=66
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export LC_NUMERIC=en_US.UTF-8

echo "[teleop_env] DOMAIN=${ROS_DOMAIN_ID}  RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}  WS=${_ws}"
unset _ws
