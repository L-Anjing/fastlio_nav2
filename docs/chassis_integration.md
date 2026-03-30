# 底盘对接说明（fastlio_nav2）

本文档用于说明：

- 本工程的控制链路在哪里看
- 最终输出给底盘的控制话题是什么
- 对接底盘时必须关注的关键参数
- 如何快速自检对接是否成功

## 1. 一句话结论

本工程在导航模式下会输出底盘速度指令：

- 最终建议对接话题：`/cmd_vel_chassis`
- 消息类型：`geometry_msgs/msg/Twist`

`Twist` 字段语义（ROS 标准）：

- `linear.x`：前进速度（m/s）
- `linear.y`：左向速度（m/s，麦轮/全向底盘使用）
- `angular.z`：绕 z 轴角速度（rad/s）

## 2. 控制链路（去哪里看）

### 2.1 主流程

1. Nav2 控制器（TEB）计算控制量
2. Nav2 velocity_smoother 平滑控制量
3. 输出到 `/cmd_vel`
4. `fake_vel_transform` 订阅 `/cmd_vel` 并发布 `/cmd_vel_chassis`
5. 你的底盘驱动节点订阅 `/cmd_vel_chassis`，转换为 CAN/UART/电机命令

### 2.2 关键源码入口

- Nav2 速度重映射：

  - `src/navigation/navigation/launch/navigation_launch.py`
  - 关键点：`controller_server` 先发 `cmd_vel_nav`，`velocity_smoother` 再发 `cmd_vel`
- 底盘最终控制输出：

  - `src/navigation/fake_vel_transform/src/fake_vel_transform.cpp`
  - 常量定义：
    - `CMD_VEL_TOPIC = /cmd_vel`
    - `AFTER_TF_CMD_VEL = /cmd_vel_chassis`
- 实车总启动入口：

  - `src/nav_bringup/launch/bringup_real.launch.py`
  - 其中会启动 `fake_vel_transform_node`

## 3. 对接底盘你会拿到什么

本工程可直接给你：

- 规划后速度指令：`/cmd_vel_chassis`（给电控）
- 定位与姿态：`/Odometry`、TF（`map/odom/base_link*`）
- 感知障碍输入：`/scan`、`/segmentation/obstacle`

本工程不负责：

- 电机闭环（电流环/速度环）
- 底层总线协议封装（CAN/UART/RS485）
- 驱动安全机制（急停、看门狗、失联刹停）

## 4. 关键参数（对接时最常改）

## 4.1 `nav2_params_real.yaml`

文件：`src/nav_bringup/config/reality/nav2_params_real.yaml`

必须关注：

- 坐标/里程计

  - `bt_navigator.robot_base_frame`（当前为 `base_link_fake`）
  - `bt_navigator.odom_topic`（当前为 `/Odometry`）
- TEB 速度上限

  - `FollowPath.max_vel_x`
  - `FollowPath.max_vel_y`
  - `FollowPath.max_vel_theta`
- TEB 加速度上限

  - `FollowPath.acc_lim_x`
  - `FollowPath.acc_lim_y`
  - `FollowPath.acc_lim_theta`
- 安全距离

  - `FollowPath.min_obstacle_dist`
  - `FollowPath.inflation_dist`
  - `local/global_costmap` 的 `inflation_radius`
- 平滑器限幅（非常关键，避免电控端收到过猛指令）

  - `velocity_smoother.max_velocity`
  - `velocity_smoother.min_velocity`
  - `velocity_smoother.max_accel`
  - `velocity_smoother.max_decel`

## 4.2 `fake_vel_transform`

文件：`src/navigation/fake_vel_transform/src/fake_vel_transform.cpp`

关键点：

- `spin_speed` 参数在 `bringup_real.launch.py` 里当前设为 `0.0`
- 当前实现是“透传” `/cmd_vel -> /cmd_vel_chassis`
- 同时发布 TF：`base_link -> base_link_fake`

## 5. 底盘对接最小实现建议

1. 写一个 `chassis_driver` 节点，订阅 `/cmd_vel_chassis`。
2. 将 `Twist` 转换为底盘控制量（轮速/转速/舵角）。
3. 在驱动侧做二次限幅与安全保护（建议保留）。
4. 建立失联保护：若超过 N ms 未收到新速度，主动下发零速。
5. 先低速联调，再逐步提升 `max_vel_*` 与 `max_accel`。

## 6. 快速联调与验收

启动导航后，建议按顺序检查：

```bash
# 1) Nav2 输出是否正常
ros2 topic hz /cmd_vel

# 2) 底盘接口话题是否正常
ros2 topic hz /cmd_vel_chassis

# 3) 查看实时命令值
ros2 topic echo /cmd_vel_chassis

# 4) 检查定位输入给 Nav2 是否正常
ros2 topic hz /Odometry

# 5) 检查 TF
ros2 run tf2_ros tf2_echo map base_link_fake
```

验收标准（最低）：

- 下发导航目标后 `/cmd_vel_chassis` 有连续输出
- 速度方向与底盘实际运动方向一致
- 障碍接近时速度可下降，最终无碰撞
- 丢话题或停车时底盘能在预期时间内停下

## 7. 常见问题

### 7.1 有规划但底盘不动

优先检查：

- 底盘驱动是否订阅了 `/cmd_vel_chassis`
- 底盘驱动是否把 `Twist` 单位按 m/s、rad/s 处理
- 电控是否有额外使能条件未满足

### 7.2 运动方向反了

优先检查：

- 底盘坐标系定义是否与 ROS REP-103 一致
- `linear.y` 的正负方向映射是否写反
- `angular.z` 正方向（逆时针为正）是否一致

### 7.3 速度变化过猛

优先检查：

- `velocity_smoother.max_accel/max_decel`
- TEB 的 `acc_lim_*`
- 底盘驱动层是否又做了冲突的滤波

## 8. 工程边界

- 本工程：感知、定位、规划、速度指令输出
- 底盘工程：执行控制、安全机制、驱动容错

这样在换车或换电控时，只需要复用 `/cmd_vel_chassis` 接口即可平滑迁移。
