# fake_vel_transform

`fake_vel_transform` 是本工程的底盘速度出口适配节点。

当前实车链路中，Nav2 使用 `robot_base_frame: base_link`，因此节点默认不再做速度坐标旋转，而是将 Nav2 平滑后的 `/cmd_vel` 透传为底盘侧接口 `/cmd_vel_chassis`。

当前主链路为 FAST-LIO-SAM-G + Nav2 三阶段导航：`staged_navigate_to_pose` 和 TEB 都输出到 `/cmd_vel_nav`，`velocity_smoother` 平滑后发布 `/cmd_vel`，本节点只负责把 `/cmd_vel` 送到底盘侧。

## 输入

- `/cmd_vel`：Nav2 / velocity_smoother 输出的 `geometry_msgs/msg/Twist`
- `/local_plan`：局部路径，仅用于保留历史 `base_link_fake` 计算逻辑

## 输出

- `/cmd_vel_chassis`：给底盘驱动订阅的最终速度指令
- `base_link -> base_link_fake`：保留发布，用于兼容旧 RViz / 调试链路

## 当前行为

- `linear.x`、`linear.y`、`angular.z` 原样透传；当前 Nav2 配置已关闭 `linear.y`，因为前置相机需要机器人 `+x` 面向轨迹。
- `spin_speed` 当前在 `nav_bringup` 中设为 `0.0`。
- 若未来重新启用小陀螺 / `base_link_fake` 导航模式，需要同步恢复速度旋转逻辑和 Nav2 的 `robot_base_frame`。

## 调试提示

如果机器人不动或方向异常，按顺序检查：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_chassis
```

如果 `/cmd_vel` 有速度而 `/cmd_vel_chassis` 没有，优先检查本节点是否启动；如果 `/cmd_vel_chassis` 有速度而底盘不动，优先检查 AgileX CAN/驱动。
