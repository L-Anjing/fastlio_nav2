# TEB 控制器在本工程中的正确使用说明

本文只针对本仓库（`fastlio_nav2`）的实车配置，不讨论通用示例。

## 1. TEB 在本工程里的接入位置

- 主配置文件：`src/nav_bringup/config/reality/nav2_params_real.yaml`
- 控制器插件键：`controller_server.ros__parameters.FollowPath.plugin`
- 当前值：`teb_local_planner::TebLocalPlannerROS`

这意味着：

- 全局规划器给 Nav2 路径
- `FollowPath`（TEB）负责局部轨迹优化与速度输出
- 最终通过本工程控制链路下发到底盘

## 2. 使用前必须满足的 4 个条件

### 2.1 TF 链必须连续

至少要稳定提供：

- `map -> odom`
- `odom -> base_link`
- `base_link -> livox_frame`
- `base_link -> base_link_fake`

注意：本工程 Nav2 参考基座是 `base_link_fake`，不是直接 `base_link`。

### 2.2 障碍物输入要正常

TEB 主要通过 costmap 感知障碍，至少要保证：

- `/scan` 连续（由点云转激光）
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

### 3.1 Costmap 改为 footprint

文件：`src/nav_bringup/config/reality/nav2_params_real.yaml`

位置：

- `local_costmap.local_costmap.ros__parameters`
- `global_costmap.global_costmap.ros__parameters`

做法：

1. 注释 `robot_radius`
2. 增加 `footprint`

示例（车长 0.56m、车宽 0.46m、安全余量 2cm）：

```yaml
footprint: "[[0.30,0.25],[0.30,-0.25],[-0.30,-0.25],[-0.30,0.25]]"
```

### 3.2 FollowPath 增加 footprint_model

位置：`controller_server.ros__parameters.FollowPath`

```yaml
footprint_model:
  type: "polygon"
  vertices: [[0.30,0.25],[0.30,-0.25],[-0.30,-0.25],[-0.30,0.25]]
```

要求：TEB `vertices` 必须与 costmap `footprint` 对齐。

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

## 5. 本工程特有注意点：base_link_fake

`nav2_params_real.yaml` 中以下组件都使用 `base_link_fake`：

- `bt_navigator.robot_base_frame`
- `local_costmap.robot_base_frame`
- `global_costmap.robot_base_frame`
- `recoveries_server.robot_base_frame`

如果你改回 `base_link`，必须整体联动修改；只改一处会导致 TF 不一致和控制异常。

## 6. 快速自检命令

```bash
# 1) 检查 TF
ros2 run tf2_ros tf2_echo map base_link_fake

# 2) 检查障碍输入
ros2 topic hz /scan
ros2 topic hz /segmentation/obstacle

# 3) 检查控制输出
ros2 topic hz /cmd_vel
```

## 7. 常见问题与处理

### 7.1 轨迹抖动、左右来回修正

优先检查：

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

- `base_link_fake` 相关 TF 是否持续更新
- 底盘控制节点是否正确消费速度指令话题
- `localization` 与 `mode:=nav` 是否匹配

## 8. 关联文档

- 仓库主说明：`/home/pi/workspace2/fastlio_nav2/README.md`
- 实车启动：`src/nav_bringup/launch/bringup_real.launch.py`
- Nav2 参数：`src/nav_bringup/config/reality/nav2_params_real.yaml`
