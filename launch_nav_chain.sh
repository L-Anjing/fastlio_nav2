#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
FASTLIO_SETUP="$ROOT_DIR/install/setup.bash"
NAV2_PARAMS="$ROOT_DIR/src/navigation/params/nav2_params.yaml"

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

# 先验 PCD：建图时按编译进二进制的 ROOT_DIR 存到 src/fast_lio/PCD/scans.pcd，而 colcon
# 只在 build 时把它拷进 install，两边很容易对不上（install 里是空的 → icp_relocalizer 抛
# "读不了 PCD 文件" 直接退出 → map->odom 从未发布 → Nav2 一直等 map）。这里直接指到
# 源目录并提前检查，避免又出现"窗口一闪而过、nav2 一直转圈"。
MAP_PCD="${MAP_PCD:-$ROOT_DIR/src/fast_lio/PCD/scans.pcd}"
if [[ ! -f "$MAP_PCD" ]]; then
  echo "missing prior map: $MAP_PCD" >&2
  echo "先生成先验图（建图结束 Ctrl+C 会存 scans.pcd），或用 MAP_PCD=/path/to/scans.pcd 指定。" >&2
  exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is not installed" >&2
  exit 1
fi

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

launch_in_terminal "02_fast_lio(reloc)" \
  "ros2 launch fast_lio reloc_mid360.launch.py map_pcd:=$MAP_PCD use_rviz:=false"

launch_in_terminal "03_ground_segmentation" \
  "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"

launch_in_terminal "04_nav2" \
  "ros2 launch navigation bringup_navigation.py use_sim_time:=$SIM_TIME params_file:=$NAV2_PARAMS nav_rviz:=true"

echo "All requested terminals have been opened."
