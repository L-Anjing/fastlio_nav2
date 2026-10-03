#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
FASTLIO_SETUP="$ROOT_DIR/install/setup.bash"


# 回放 rosbag 时设 USE_SIM_TIME=1，并用 ros2 bag play <bag> --clock
if [ "${USE_SIM_TIME:-0}" -eq 1 ]; then SIM_TIME=true; else SIM_TIME=false; fi

# 默认只在本机通信：避免同网段其他机器的 ROS 节点串进来
# （ROS_DOMAIN_ID 未设置时默认是 0，别人 domain 0 的节点会出现在你的话题里）。
# 需要跨机通信时：ROS_LOCALHOST_ONLY=0 ./launch_gnome_chain.sh
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

for required in "$ROS_SETUP" "$FASTLIO_SETUP" "$NAV2_PARAMS"; do
  if [[ ! -f "$required" ]]; then
    echo "missing required file: $required" >&2
    exit 1
  fi
done


# 每个节点开一个独立 gnome-terminal 窗口。
# 命令结束后不关窗（末尾 read），日志和报错都留在窗口里。
launch_in_terminal() {
  local title="$1"
  local cmd="$2"
  echo "[$title] launching..."
  if ! gnome-terminal --title="$title" -- bash -c "
    source '$ROS_SETUP'
    source '$FASTLIO_SETUP'
    cd '$ROOT_DIR'
    export FASTLIO_NAV2_ROOT='$ROOT_DIR'
    export SIM_TIME='$SIM_TIME'
    export ROS_LOCALHOST_ONLY='$ROS_LOCALHOST_ONLY'
    echo '===== $title ====='
    $cmd
    echo
    echo '-------------------'
    echo '[$title] 进程已退出，按回车键关闭窗口...'
    read
  "; then
    echo "failed to open terminal window: $title" >&2
  fi
  sleep 1
}

echo "Starting step-by-step chain in gnome-terminal windows..."

launch_in_terminal "01_mid_360_driver2" \
  "ros2 launch livox_ros_driver2 msg_MID360_launch.py"

launch_in_terminal "02_fast_lio(mapping)" \
  "ros2 launch fast_lio mapping_mid360.launch.py use_rviz:=true"


echo "All requested terminals have been opened."
