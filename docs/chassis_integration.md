# AgileX Ranger Mini V3 底盘接入说明

本文说明当前 FAST-LIO-SAM-G + Nav2 主链路中，AgileX Ranger Mini V3 底盘如何接入。

## 1. 当前边界

本仓库负责：

- FAST-LIO-SAM-G 定位/建图
- Nav2 规划与避障
- 三阶段导航 action proxy
- 输出速度到 `/cmd_vel_chassis`

AgileX workspace 负责：

- CAN 通信
- Ranger 底盘驱动
- 实际电机执行

## 2. 启动前准备

```bash
cd ~/workspace/agilex_ws
sudo bash src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash
```

每个终端 source：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/agilex_ws/install/setup.bash
source ~/workspace/fastlio_nav2/install/setup.bash
```

## 3. 推荐启动

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

## 4. 速度链路

```text
/controller_server 或 staged_navigate_to_pose
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> fake_vel_transform
  -> /cmd_vel_chassis
  -> ranger_base_node
```

`fake_vel_transform` 当前主要做速度出口适配，不再做旧的 `base_link_fake` 小陀螺速度旋转逻辑。

## 5. 当前底盘运动策略

虽然 Ranger Mini V3 底盘具备全向能力，但当前任务要求前置相机朝向机器人 `+x`，因此当前配置关闭横移：

```yaml
max_vel_y: 0.0
acc_lim_y: 0.0
velocity_smoother.max_velocity: [0.35, 0.0, 0.85]
```

导航策略：

1. staged start yaw 让 `+x` / 前置相机朝向轨迹。
2. TEB 主要沿 `+x` 行驶。
3. staged final yaw 到点后对准最终工作方向。

如果未来需要利用全向避障，可重新打开小的 `max_vel_y`，例如 `0.05~0.10`，但会牺牲相机始终朝轨迹方向的稳定性。

## 6. TF 规则

`ranger_base_node` 必须不发布 `odom -> base_link` TF：

```text
publish_odom_tf: 0
```

FAST-LIO-SAM-G 负责主 TF：

```text
map -> odom -> base_link
```

启动后检查：

```bash
ros2 topic info /tf -v
```

如果看到多个同名 publisher，先杀干净旧 ROS 进程。

## 7. 常用检查

### 7.1 CAN / 底盘节点

```bash
ip link show can0
ros2 topic echo /cmd_vel_chassis
ros2 topic echo /chassis/odom
```

### 7.2 速度是否传到各层

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_chassis
```

### 7.3 方向是否正确

ROS 约定：

- `linear.x > 0`：机器人 `+x` 前进。
- `linear.y` 当前应为 `0`。
- `angular.z > 0`：逆时针旋转。

如果方向反了，先检查 AgileX 驱动层坐标映射，不要在 Nav2 中随意改符号。

## 8. 明天继续关注

- 是否仍然原地打转。
- 是否 TEB 进入 infeasible。
- 是否 local costmap 中机器人 footprint 起始就在障碍物里。
- `/scan` 是否被 TF message filter 丢弃，导致避障不稳定。

详细问题见 `docs/remaining_issues.md`。
