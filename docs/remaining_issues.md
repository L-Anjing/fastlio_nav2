# Remaining Issues / 明日继续处理清单

日期：2026-04-26

当前工程主链路已经收敛到：

```text
FAST-LIO-SAM-G -> /Odometry + /scan + /map + TF
Nav2 + TEB -> /cmd_vel_nav
velocity_smoother -> /cmd_vel
fake_vel_transform -> /cmd_vel_chassis
AgileX ranger_base_node -> 底盘
```

三阶段导航已接入：

```text
/navigate_to_pose -> staged_navigate_to_pose
/navigate_to_pose_raw -> bt_navigator
```

正常日志应出现：

```text
Starting independent start yaw alignment
Start yaw aligned
bt_navigator: Begin navigating
bt_navigator: Goal succeeded
Starting independent final yaw alignment
Final yaw aligned
```

## 1. 已解决/当前状态

### 1.1 Action wiring

已验证正确：

```text
/navigate_to_pose server: staged_navigate_to_pose
/navigate_to_pose_raw server: bt_navigator
```

### 1.2 三阶段导航

已能看到 start yaw、TEB path、final yaw 三阶段日志。

### 1.3 RotationShim 已移除

`FollowPath` 当前直接使用：

```yaml
plugin: teb_local_planner::TebLocalPlannerROS
```

不再使用 `nav2_rotation_shim_controller` 包 TEB。

### 1.4 y 横移已关闭

因为前置相机需要 `+x` 面向轨迹，当前关闭横移：

```yaml
max_vel_y: 0.0
acc_lim_y: 0.0
```

## 2. 主要遗留问题

### 2.1 local costmap TF/message filter 丢包

频繁日志：

```text
Message Filter dropping message: frame 'lidar' ... earlier than all the data in the transform cache
```

影响：

- local costmap 可能吃不到稳定 `/scan`。
- 避障可能漏障碍或延迟。
- TEB 可能错误判断轨迹可行/不可行。

明天优先检查：

```bash
ros2 topic hz /scan
ros2 topic info /scan -v
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map lidar
ros2 run tf2_ros tf2_echo base_link lidar
```

重点判断：

- `/scan.header.frame_id` 是 `lidar` 还是 `livox_frame`。
- TF tree 是否真的有 `lidar` frame。
- `/scan` timestamp 是否明显落后 TF cache。
- FAST-LIO-SAM-G 发布 `/scan` 时是否用了旧 stamp。

可能修复方向：

- 统一 `/scan` frame 到 URDF 中存在的 `livox_frame`，或补 `lidar -> livox_frame` 静态 TF。
- 修改 FAST-LIO-SAM-G `/scan` stamp，使用当前 ROS time 或与 TF 同步的 stamp。
- 增大 local costmap observation `transform_tolerance`。

### 2.2 TEB 原地打转 / 长时间 Passing new path

现象：

```text
Passing new path to controller
```

但机器人可能原地打转或不明显朝目标移动。

已做：

- 三阶段 start yaw。
- 关闭 y 横移。
- forward-drive 权重从历史过强值降到 `10.0`。
- Nav2 final yaw 交给 staged 节点。

明天检查：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_chassis
```

看路径阶段是否一直只有 `angular.z`，没有 `linear.x`。

如果一直转：

- 降低 `weight_kinematics_forward_drive: 10.0 -> 5.0`。
- 放宽 `start_yaw_tolerance: 0.20 -> 0.25`，避免刚开始角度过度追求。
- 检查 global plan 是否频繁反向或贴障碍。

### 2.3 TEB trajectory is not feasible

现象：

```text
TebLocalPlannerROS: trajectory is not feasible. Resetting planner...
Controller patience exceeded
Collision Ahead - Exiting Spin
```

已做平衡：

```yaml
nav2_safety_margin: 0.05
min_obstacle_dist: 0.25
inflation_dist: 0.40
inflation_radius: 0.45
failure_tolerance: 5.0
feasibility_check_no_poses: 2
```

明天检查 RViz：

- 打开 `/local_costmap/costmap`。
- 看机器人 footprint 是否一开始就压在障碍物里。
- 看后方钢板是否仍进入 `/scan` 或 local costmap。

可能修复方向：

- 调整 `frontend_scan.mask_angle_min/max`。
- 调整 `/scan` 的 `obstacle_min_range`。
- 若障碍物漏检导致碰撞，不要继续减小安全距离；先修感知。

### 2.4 `/map` 尺寸频繁变化导致 global costmap resize

日志：

```text
StaticLayer: Resizing costmap to ...
Trying to create a map of size ...
```

当前原因：`cloud_to_occupancy` 每次根据点云 min/max 重新计算 OccupancyGrid 尺寸。

影响：

- global costmap 反复 resize。
- RViz 视觉闪烁。
- CPU 负载增加。

明天可做：

- 在 `cloud_to_Occupancy.cpp` 中保持 map origin/width/height 单调扩张或固定窗口。
- 降低 `/map` 发布频率。
- 只有地图边界变化超过阈值才 resize。

相关文件：

```text
src/localization/FAST-SLAM-G/src/cloud_to_Occupancy.cpp
src/nav_bringup/config/reality/fast_lio_sam_gridmap_mid360_real.yaml
```

### 2.5 RViz / CPU 负载

当前已做：

- Nav2 RViz 延迟 12s 启动。
- global STVL layer 已从主配置中移除。
- 推荐只开 `nav_rviz:=True`，关闭 `lio_rviz`。

推荐启动：

```bash
ros2 launch nav_bringup bringup_real.launch.py \
  mode:=fastlio_nav \
  lio:=fastlio_sam \
  launch_chassis:=True \
  lio_rviz:=False \
  nav_rviz:=True
```

## 3. 明天建议调试顺序

1. **先不发导航目标**，确认 TF/topic/action 干净。
2. 检查 `/scan` 是否被 local costmap 持续丢弃。
3. 检查 `/local_costmap/costmap` 中机器人 footprint 周围是否异常占据。
4. 发一个 0.5m 小目标，观察 `/cmd_vel_nav` 是否有稳定 `linear.x`。
5. 再测带 final yaw 的目标，确认 final yaw 阶段只转向不行驶。
6. 最后再测避障。

## 4. 常用命令

```bash
# 杀旧进程
pkill -f ros2
pkill -f component_container
pkill -f rviz2
pkill -f staged_navigate
pkill -f fastlio_mapping
pkill -f robot_state_publisher

# action wiring
ros2 action info /navigate_to_pose
ros2 action info /navigate_to_pose_raw

# TF
ros2 topic info /tf -v
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link livox_frame

# scan/map
ros2 topic hz /scan
ros2 topic info /scan -v
ros2 topic hz /map
ros2 topic echo /map --once | grep -E 'width|height|resolution'

# speed chain
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_chassis
```

## 5. Do Not Forget

- 每个终端 source 顺序：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/agilex_ws/install/setup.bash
source ~/workspace/fastlio_nav2/install/setup.bash
```

- 不要同时启动两次 bringup。
- `ranger_base_node.publish_odom_tf` 必须保持 false。
- 当前不维护旧定位/建图链路。
