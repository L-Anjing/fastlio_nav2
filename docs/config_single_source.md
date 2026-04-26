# 配置唯一化与参数修改说明

当前实车主链路只维护：**FAST-LIO-SAM-G + Nav2 + TEB + AgileX Ranger Mini V3**。

FAST-LIO-SAM-G 输出 `/Odometry`、`/scan`、`/map` 和 TF；Nav2 负责规划、避障、速度输出；`staged_navigate_to_pose` 将 RViz goal 拆成起步朝向、TEB 行驶、最终朝向三阶段。

不维护 ICP、AMCL、slam_toolbox、Point-LIO、外部 pointcloud_to_laserscan 等旧链路。

## 1. 总原则

- 总启动入口：`src/nav_bringup/launch/bringup_real.launch.py`
- 物理测量入口：`src/nav_bringup/config/reality/measurement_params_real.yaml`
- FAST-LIO-SAM-G 实车参数：`src/nav_bringup/config/reality/fast_lio_sam_gridmap_mid360_real.yaml`
- Nav2 / TEB 参数：`src/nav_bringup/config/reality/nav2_params_real.yaml`
- 地面/障碍物分割：`src/nav_bringup/config/reality/segmentation_real.yaml`
- Livox 网络参数：`src/nav_bringup/config/reality/MID360_config.json`

## 2. 参数职责表

| 修改内容 | 文件 | 说明 |
| --- | --- | --- |
| 车体尺寸 | `measurement_params_real.yaml` | `vehicle_body.size`，单位 m |
| Nav2 虚拟安全边界 | `measurement_params_real.yaml` | `vehicle_body.nav2_safety_margin`，启动时外扩 footprint |
| 雷达安装位置 | `measurement_params_real.yaml` | `base_link_to_livox.xyz/rpy` |
| FAST-LIO-SAM-G 输入/发布 | `fast_lio_sam_gridmap_mid360_real.yaml` | 雷达、IMU、`/scan`、`/map` |
| 后方钢板 mask | `fast_lio_sam_gridmap_mid360_real.yaml` | `frontend_scan.mask_*` |
| Nav2 到点阈值 | `nav2_params_real.yaml` | `xy_goal_tolerance` |
| TEB 速度/避障 | `nav2_params_real.yaml` | `max_vel_*`、`min_obstacle_dist`、`inflation_*` |
| RViz 延迟启动 | `navigation/launch/bringup_navigation.py` | 当前延迟 12s 打开 Nav2 RViz |
| 三阶段动作 | `staged_navigate_to_pose.py` | `/navigate_to_pose` proxy |

## 3. `measurement_params_real.yaml`

当前值：

```yaml
robot_measurements:
  vehicle_body:
    size: [0.751, 0.500, 0.200]
    nav2_safety_margin: 0.05
  base_link_to_livox:
    xyz: [0.250, 0.056, 0.405]
    rpy: [0.0, 0.0, 0.0]
```

说明：

- `size[0]` 是 x/车长，当前 `0.751m`。
- `size[1]` 是 y/车宽，当前 `0.500m`。
- `size[2]` 主要用于 URDF 可视化，对 Nav2 平面避障影响小。
- `nav2_safety_margin` 会把 Nav2 footprint 每边外扩，当前 `5cm`。
- `base_link_to_livox` 同时用于 URDF TF、ground segmentation sensor height。

## 4. FAST-LIO-SAM-G 参数

主文件：

```text
src/nav_bringup/config/reality/fast_lio_sam_gridmap_mid360_real.yaml
```

重点字段：

- `common.lid_topic`：Livox 点云输入。
- `common.imu_topic`：launch 覆盖为 `/livox/imu`。
- `publish.frontend_scan_pub_en`：启用前端 `/scan`。
- `frontend_scan.topic`：launch 覆盖为 `/scan`。
- `frontend_scan.mask_enabled`：用于屏蔽雷达 `-x` 后方钢板。
- `occupancy_map.input_topic`: `lio_sam/mapping/cloud_global_2d`
- `occupancy_map.output_topic`: `/map`

注意：如果 Nav2 看不到地图，优先查：

```bash
ros2 topic hz /lio_sam/mapping/cloud_global_2d
ros2 topic hz /map
ros2 topic info /map -v
```

## 5. Nav2 / TEB 参数

主文件：

```text
src/nav_bringup/config/reality/nav2_params_real.yaml
```

当前策略：

- `FollowPath` 直接使用 `teb_local_planner::TebLocalPlannerROS`。
- 不再用 `nav2_rotation_shim_controller` 包 TEB，防止和 staged start/final yaw 打架。
- `xy_goal_tolerance: 0.08`，位置阈值 8cm。
- `yaw_goal_tolerance: 6.28`，Nav2 path stage 不负责最终朝向。
- `max_vel_y: 0.0`，关闭横移，前置相机需要 `+x` 面向轨迹。
- `weight_kinematics_forward_drive: 10.0`，偏好向前，不使用之前过强的 `100.0`。
- `failure_tolerance: 5.0`，避免 TEB 短暂不可行立刻失败。

当前避障相关：

```yaml
min_obstacle_dist: 0.25
inflation_dist: 0.40
dynamic_obstacle_inflation_dist: 0.45
local/global inflation_radius: 0.45
```

如果碰撞：先检查 `/scan` 是否漏检或把车体点/钢板点打进 costmap，不要只盲目加大 inflation。

如果 TEB 频繁 infeasible：先看机器人 footprint 是否已经压在 local costmap 障碍物里。

## 6. 三阶段导航参数

文件：

```text
src/nav_bringup/scripts/staged_navigate_to_pose.py
src/nav_bringup/launch/bringup_real.launch.py
```

当前 launch 参数：

- `start_yaw_enabled: True`
- `start_yaw_tolerance: 0.20`
- `final_yaw_enabled: True`
- `final_yaw_tolerance: 0.25`
- `cmd_vel_topic: /cmd_vel_nav`

正常日志：

```text
Accepted staged goal
Starting independent start yaw alignment
Start yaw aligned
bt_navigator: Begin navigating
bt_navigator: Goal succeeded
Starting independent final yaw alignment
Final yaw aligned
```

## 7. 底盘与 TF 原则

- FAST-LIO-SAM-G 是唯一主定位 TF 来源。
- `ranger_base_node.publish_odom_tf` 必须为 `false`。
- `/chassis/odom` 可以发布，但不参与 Nav2 主定位。
- 如果 `/tf` 中同名 node 重复，优先杀旧进程再重启。

