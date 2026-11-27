# 遥操作环境：跨机通信
export ROS_DOMAIN_ID=66
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export LC_NUMERIC=en_US.UTF-8
. /opt/ros/jazzy/setup.bash 2>/dev/null
_here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -f "$_here/install/setup.bash" ] && . "$_here/install/setup.bash"
echo "[teleop_env] DOMAIN=$ROS_DOMAIN_ID  RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
