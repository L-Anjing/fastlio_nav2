# TEB 控制器在本工程中的正确使用说明

本文只针对本仓库（`fastlio_nav2`）的实车配置，不讨论通用示例。

实车配置统一从 `src/nav_bringup/config/reality/` 维护。FAST-LIO-SAM-G 在 `reality` 中只保留主用 `fast_lio_sam_gridmap_mid360_real.yaml`；TEB 相关参数只改 `nav2_params_real.yaml` 和由 `measurement_params_real.yaml` 自动同步的车体尺寸。

当前唯一维护的导航链路是 FAST-LIO-SAM-G + Nav2：FAST-LIO-SAM-G 提供 `/Odometry`、`/scan`、实时 `/map` 和 `map -> odom -> base_link`，TEB 只负责局部轨迹优化和速度输出。

当前控制策略已经改为三阶段：起步朝向和最终朝向由 `staged_navigate_to_pose` 独立控制，TEB 只负责中间路径阶段。不要再用 `nav2_rotation_shim_controller` 包住 TEB，否则容易和 staged start/final yaw 打架。

前置相机安装在机器人 `+x` 方向，因此当前配置关闭 `linear.y` 横移，让 `+x` 尽量朝向轨迹。虽然底盘具备全向能力，但当前任务不使用横移导航。

## 1. TEB 在本工程里的接入位置

- 主配置文件：`src/nav_bringup/config/reality/nav2_params_real.yaml`
- 控制器插件键：`controller_server.ros__parameters.FollowPath.plugin`
- 当前值：`teb_local_planner::TebLocalPlannerROS`

这意味着：

- 全局规划器给 Nav2 路径
- `FollowPath`（TEB）负责局部轨迹优化与速度输出
- 最终通过本工程控制链路下发到底盘

当前不要改回：

```yaml
plugin: nav2_rotation_shim_controller::RotationShimController
```

如果需要调整起步/最终朝向，改 `src/nav_bringup/scripts/staged_navigate_to_pose.py` 或 `bringup_real.launch.py` 中 staged 参数。

## 2. 使用前必须满足的 4 个条件

### 2.1 TF 链必须连续

至少要稳定提供：

- `map -> odom`
- `odom -> base_link`
- `base_link -> livox_frame`

注意：本工程当前 Nav2 参考基座是 `base_link`。`base_link_fake` 只作为旧调试/小陀螺兼容 TF 保留，默认不参与 Nav2 控制。

### 2.2 障碍物输入要正常

TEB 主要通过 costmap 感知障碍，至少要保证：

- `/scan` 连续（由 `fast_lio_sam` 内置前端扫描发布）
- `/segmentation/obstacle` 连续（3D 障碍点云）

若这两个输入缺失，TEB 常见表现是“看起来能规划但实车直冲/急停/贴障”。

### 2.3 车体模型必须一致

需要同时一致：

- costmap 里的机器人几何（`robot_radius` 或 `footprint`）
- TEB 的 `footprint_model`
- URDF 中车体尺寸（用于可视化和碰撞语义对齐）

### 2.4 速度命令链路通

建议确认：

- `/cmd_vel` 有输出
- 底盘实际订阅的控制话题有输出（例如 `/cmd_vel_chassis`）

## 3. 方形车迁移：参数改法（推荐）

你是方形底盘时，不建议继续只用 `robot_radius`。

本工程当前已经把“车体物理尺寸”统一收口到：

- `src/nav_bringup/config/reality/measurement_params_real.yaml`

优先改：

- `robot_measurements.vehicle_body.size`

然后由 `bringup_real.launch.py` 自动下发到：

- URDF 车体盒体
- Nav2 local/global costmap footprint
- TEB `footprint_model`

### 3.1 Costmap 改为 footprint

优先入口：`src/nav_bringup/config/reality/measurement_params_real.yaml`

当前这台车的自动推导结果是：

```yaml
robot_measurements:
  vehicle_body:
    size: [0.6852, 0.57, 1.3345]
```

对应 footprint：

```yaml
footprint: "[[0.3426,0.285],[0.3426,-0.285],[-0.3426,-0.285],[-0.3426,0.285]]"
```

如果你换车，推荐做法不是手改 `nav2_params_real.yaml`，而是改 `vehicle_body.size`。

只有在你明确想脱离这套自动同步逻辑时，才直接去改：

- `local_costmap.local_costmap.ros__parameters.footprint`
- `global_costmap.global_costmap.ros__parameters.footprint`

### 3.2 FollowPath 增加 footprint_model

位置：`controller_server.ros__parameters.FollowPath`

当前自动推导结果：

```yaml
footprint_model:
  type: "polygon"
  vertices: [[0.3426,0.285],[0.3426,-0.285],[-0.3426,-0.285],[-0.3426,0.285]]
```

要求：TEB `vertices` 必须与 costmap `footprint` 对齐。

在本工程当前版本里，这部分由 launch 自动同步，不建议手工分别维护。

## 4. TEB 调参顺序（按风险从低到高）

### 4.1 先调运动上限（先稳）

- `max_vel_x`
- `max_vel_y`
- `max_vel_theta`
- `acc_lim_x`
- `acc_lim_y`
- `acc_lim_theta`

建议：新车上线先从保守值开始，再逐步提升。

### 4.2 再调避障距离（先不撞）

- `min_obstacle_dist`
- `inflation_dist`
- `dynamic_obstacle_inflation_dist`
- costmap `inflation_radius`

经验：

- `inflation_dist >= min_obstacle_dist`
- 方形车建议额外留 0.02~0.05m 裕量

### 4.3 再调平滑和响应（再变快）

- `dt_ref`
- `dt_hysteresis`
- `no_inner_iterations`
- `no_outer_iterations`
- `controller_frequency`

现象到参数映射：

- 轨迹跟随慢：适当减小 `dt_ref` 或提高 `controller_frequency`
- 高频抖动：适当增大 `dt_ref`、降低速度上限、增大避障距离

### 4.4 最后调恢复行为（防卡住）

- `oscillation_recovery`
- `oscillation_v_eps`
- `oscillation_omega_eps`
- `oscillation_recovery_min_duration`

用于抑制原地抖动、反复切换局部轨迹等问题。

## 5. 本工程特有注意点：base_link

`nav2_params_real.yaml` 中以下组件都使用 `base_link`：

- `bt_navigator.robot_base_frame`
- `local_costmap.robot_base_frame`
- `global_costmap.robot_base_frame`
- `recoveries_server.robot_base_frame`

如果你想重新启用 `base_link_fake` 导航模式，必须整体联动修改这些字段，并同步恢复 `fake_vel_transform` 中的速度坐标旋转逻辑；只改一处会导致 TF 不一致和控制异常。

## 6. 快速自检命令

```bash
# 1) 检查 TF
ros2 run tf2_ros tf2_echo map base_link

# 2) 检查障碍输入
ros2 topic hz /scan
ros2 topic hz /segmentation/obstacle

# 3) 检查控制输出
ros2 topic hz /cmd_vel
```

## 7. 常见问题与处理

### 7.1 轨迹抖动、左右来回修正

优先检查：

- `measurement_params_real.yaml` 的 `vehicle_body.size` 是否正确
- `footprint` 与 `footprint_model` 是否一致
- `dt_ref` 是否过小
- `max_vel_*` / `acc_lim_*` 是否过激

### 7.2 会贴障、擦边

优先增大：

- `min_obstacle_dist`
- `inflation_dist`
- `inflation_radius`

### 7.3 能规划但不走

优先检查：

- `map -> odom -> base_link` TF 是否持续更新
- 底盘控制节点是否正确消费速度指令话题
- FAST-LIO-SAM-G 输出 `/Odometry`、`/scan` 和 `map -> odom -> base_link` 是否正常

## 8. 关联文档

- 仓库主说明：`README.md`
- 实车启动：`src/nav_bringup/launch/bringup_real.launch.py`
- Nav2 参数：`src/nav_bringup/config/reality/nav2_params_real.yaml`
