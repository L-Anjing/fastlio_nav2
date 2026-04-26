# fastlio_nav2（实车建图与导航）

本仓库当前只维护一条实车主链路：**Livox Mid360 + FAST-LIO-SAM-G + Nav2 + TEB + AgileX Ranger Mini V3**。

目标：

- FAST-LIO-SAM-G 负责建图、定位、`/Odometry`、`/scan`、在线 `/map` 和 `map -> odom -> base_link` TF。
- Nav2 负责全局规划、局部避障和速度控制。
- `staged_navigate_to_pose` 负责三阶段导航：起步朝向、TEB 行驶、最终朝向。
- AgileX 驱动只消费 `/cmd_vel_chassis` 控制底盘。

主入口：`src/nav_bringup/launch/bringup_real.launch.py`

## 1. 当前主链路

### 1.1 FAST-LIO-SAM-G 输出

FAST-LIO-SAM-G 位于：

```text
src/localization/FAST-SLAM-G
```

ROS 2 包名仍为：

```text
fast_lio_sam
```

主用配置：

```text
src/nav_bringup/config/reality/fast_lio_sam_gridmap_mid360_real.yaml
```

当前主链路依赖它输出：

- `/Odometry`
- `/OdometryHighFreq`
- `/scan`
- `/map`
- `lio_sam/mapping/cloud_global_2d`
- `map -> odom -> base_link`

### 1.2 Nav2 三阶段导航

RViz 发出的 `/navigate_to_pose` 不直接给 Nav2，而是先进入：

```text
src/nav_bringup/scripts/staged_navigate_to_pose.py
```

当前 action wiring：

```text
/navigate_to_pose      -> staged_navigate_to_pose
/navigate_to_pose_raw  -> bt_navigator
```

三阶段行为：

1. **start yaw**：先原地旋转，让机器人 `+x` / 前置相机朝向轨迹方向。
2. **path stage**：发送位置目标给 `/navigate_to_pose_raw`，由 Nav2 + TEB 行驶到目标位置。
3. **final yaw**：到达位置后，staged 节点独立发布 `/cmd_vel_nav`，调整到 RViz 目标方向。

因为前置相机必须朝向 `+x`，当前配置已关闭 `linear.y` 横移：

```yaml
max_vel_y: 0.0
acc_lim_y: 0.0
```

### 1.3 速度链路

```text
TEB / staged final yaw
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> fake_vel_transform
  -> /cmd_vel_chassis
  -> ranger_base_node
```

`ranger_base_node` 由 `agilex_ws` 提供，当前 launch 传入：

```text
publish_odom_tf:=false
odom_topic_name:=/chassis/odom
cmd_vel_topic_name:=/cmd_vel_chassis
```

这样底盘不抢 FAST-LIO-SAM-G 的 `map/odom/base_link` TF。

## 2. 编译与环境

### 2.1 软件环境

- Ubuntu 22.04
- ROS 2 Humble
- Livox SDK2 / livox_ros_driver2
- GTSAM
- AgileX Ranger ROS2 workspace：`~/workspace/agilex_ws`
- 本仓库：`~/workspace/fastlio_nav2`

### 2.2 编译

```bash
cd ~/workspace/fastlio_nav2
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
```

只改 bringup / navigation 配置时可以快速编译：

```bash
colcon build --packages-select nav_bringup navigation --symlink-install
```

### 2.3 推荐 source 顺序

每个终端都按这个顺序：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/agilex_ws/install/setup.bash
source ~/workspace/fastlio_nav2/install/setup.bash
```

如果 action / topic 信息异常，先清 ROS daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

## 3. 启动

### 3.1 CAN 准备

```bash
cd ~/workspace/agilex_ws
sudo bash src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash
```

### 3.2 建图模式

```bash
ros2 launch nav_bringup bringup_real.launch.py \
  mode:=mapping \
  lio:=fastlio_sam \
  lio_rviz:=True \
  nav_rviz:=False
```

保存当前在线 `/map`：

```bash
cd ~/workspace/fastlio_nav2
./save_grid_map.sh YOUR_WORLD_NAME
```

### 3.3 导航模式（推荐）

```bash
ros2 launch nav_bringup bringup_real.launch.py \
  mode:=fastlio_nav \
  lio:=fastlio_sam \
  launch_chassis:=True \
  chassis_port_name:=can0 \
  chassis_robot_model:=ranger_mini_v3 \
  lio_rviz:=False \
  nav_rviz:=True
```

说明：

- `lio_rviz:=False`：减少一个 RViz，降低 CPU/GPU 压力。
- `nav_rviz:=True`：打开 Nav2 RViz。当前 Nav2 RViz 已延迟启动，给 FAST-LIO-SAM-G 先发布地图。
- `launch_chassis:=False`：只测试导航链路，不控制真实底盘。

## 4. 关键配置

### 4.1 车体与雷达测量

```text
src/nav_bringup/config/reality/measurement_params_real.yaml
```

当前测量值：

```yaml
vehicle_body:
  size: [0.751, 0.500, 0.200]
  nav2_safety_margin: 0.05
base_link_to_livox:
  xyz: [0.250, 0.056, 0.405]
  rpy: [0.0, 0.0, 0.0]
```

`nav2_safety_margin` 会在 launch 时外扩 Nav2/TEB footprint。

### 4.2 Nav2 / TEB

```text
src/nav_bringup/config/reality/nav2_params_real.yaml
```

当前重点：

- `xy_goal_tolerance: 0.08`：位置到达阈值 8 cm。
- `yaw_goal_tolerance: 6.28`：Nav2 path stage 不管最终朝向，最终朝向由 staged 节点处理。
- `max_vel_y: 0.0`：关闭横移，保证前置相机 `+x` 面向轨迹。
- `failure_tolerance: 5.0`：避免 TEB 短暂不可行时立刻 abort。
- global costmap 当前只用 `static_layer + inflation_layer`，已关闭重的 STVL 全局层来加速启动。
- local costmap 订阅 `/scan` 做近场障碍物避障。

### 4.3 FAST-LIO-SAM-G map / scan

```text
src/nav_bringup/config/reality/fast_lio_sam_gridmap_mid360_real.yaml
```

当前关注：

- `frontend_scan.topic`：launch 覆盖为 `/scan`。
- `frontend_scan.mask_enabled`：用于屏蔽雷达后方钢板方向。
- `occupancy_map.input_topic`: `lio_sam/mapping/cloud_global_2d`
- `occupancy_map.output_topic`: `/map`

## 5. 快速检查命令

### 5.1 TF 重复检查

```bash
ros2 topic info /tf -v
ros2 topic info /tf_static -v
```

如果看到同名节点重复出现，例如多个 `robot_state_publisher`、`fast_lio_gridmap`、`ranger_base_node`，先杀掉旧进程：

```bash
pkill -f ros2
pkill -f component_container
pkill -f rviz2
pkill -f staged_navigate
pkill -f fastlio_mapping
pkill -f robot_state_publisher
```

### 5.3 速度链路检查

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_chassis
```

三阶段 final yaw 时应看到 `angular.z` 从 `/cmd_vel_nav` 传到 `/cmd_vel_chassis`。

### 5.4 地图检查

```bash
ros2 topic hz /map
ros2 topic echo /map --once | grep -E 'width|height|resolution'
```

RViz 中 `100 x 100` 通常是 local costmap，不代表 `/map` 消失。


## 6. 文档索引

- `docs/config_single_source.md`：所有实车参数应该改哪里。
- `docs/chassis_integration.md`：AgileX 底盘接入与速度链路。
- `docs/remaining_issues.md`：当前未解决问题与明天继续工作的检查清单。
