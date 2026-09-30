# FAST-LIO：MID-360 Mapping

本包是 ROS 2 下的 Livox MID-360 FAST-LIO 前端，代码和构建系统均为纯 ROS 2，提供建图里程计模式和基于 PCL ICP 的重定位模式（启动阶段完成配准）。


## 坐标变换约定

整个工程统一规定：`T_B^A` 表示 B 坐标系相对 A 坐标系的位姿，并将 B 系坐标变换到 A 系：

```text
p^A = T_B^A p^B
T_C^A = T_B^A T_C^B
(T_B^A)^-1 = T_A^B
```

ROS TF 的 `parent -> child` 对应 `T_child^parent`。

MID-360 的 IMU 和点云坐标轴方向相同，但原点不同。官方给出的 IMU 原点在点云坐标系中的位置是 `[0.011, 0.02329, -0.04412] m`，因此 YAML 直接配置 `T_imu^lidar`：

- `sensor_extrinsic/imu_in_lidar_R`
- `sensor_extrinsic/imu_in_lidar_T`

FAST-LIO 内部需要 `T_lidar^imu`，程序会自动计算 `(T_imu^lidar)^-1`。雷达安装位置配置为 `T_lidar^base`：

- `robot_extrinsic/lidar_to_robot_R`
- `robot_extrinsic/lidar_to_robot_T`

机器人中心里程计按下式计算：

```text
T_base^odom
  = T_imu^odom T_lidar^imu T_base^lidar
  = T_imu^odom (T_imu^lidar)^-1 (T_lidar^base)^-1
```

公开 TF 链路为：

```text
odom -> base_link -> livox_frame -> imu_link
   \-> base_link_hf
```

| TF | 含义 |
| --- | --- |
| `odom -> base_link` | LiDAR 帧率更新的机器人中心里程计 |
| `odom -> base_link_hf` | IMU 高频传播得到的机器人中心里程计，供串口反馈使用 |
| `base_link -> livox_frame` | YAML 中的雷达安装外参 `T_lidar^base`；`livox_frame` 与 Livox 驱动消息的 `frame_id` 一致 |
| `livox_frame -> imu_link` | MID-360 官方内部外参 `T_imu^lidar` |

整条链路由 `fastlio_mapping` 自己发布，不依赖 `robot_state_publisher` / URDF：这样原始点云 `/livox/lidar`、地面分割和 Nav2 拿到的 `livox_frame` 与定位输出在同一棵 TF 树上。

`odometry/zero_at_start: true` 时，第一帧有效机器人中心位姿被设为 `odom` 原点，起始 XYZ/RPY 为零。`mapping/extrinsic_est_en` 保留 FAST-LIO 的在线外参估计代码，但 MID-360 默认设置为 `false`，此时 EKF、机器人中心里程计和静态 TF 使用同一组 YAML 外参。若改为 `true`，EKF 内部的 `T_lidar^imu` 可能变化，而公开的 `livox_frame -> imu_link` 仍是 YAML 静态值，调试 TF 时需要注意二者不再严格等价。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
colcon build 
source install/setup.bash
```

分别启动：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping_mid360.launch.py
ros2 run tf_to_serial tf_to_serial_node 
```

不需要 RViz 时：

```bash
ros2 launch fast_lio mapping_mid360.launch.py use_rviz:=false
```

一键脚本在仓库根目录：`postion_odom.sh`（Livox 驱动、FAST-LIO、串口节点三段按脚本里是否注释来启停）。

## 参数配置

主配置文件为 `config/mapping/mid360.yaml`，launch 会将嵌套 YAML 展平成代码使用的 `/` 参数名。

| 参数 | 作用 |
| --- | --- |
| `common/lid_topic`、`common/imu_topic` | 点云和 IMU 输入话题 |
| `preprocess/scan_rate` | 实际雷达帧率 |
| `point_filter_num` | 点云抽样间隔，增大可降低计算量 |
| `filter_size_surf`、`filter_size_map` | 点云/地图体素尺寸 |
| `max_iteration` | 每帧滤波迭代上限 |
| `mapping/*_cov` | IMU 噪声与偏置随机游走参数 |
| `sensor_extrinsic/*` | 官方 `T_imu^lidar` |
| `robot_extrinsic/*` | 雷达在机器人中心系中的 `T_lidar^base` |
| `publish/*` | 点云、路径和调试输出开关 |
| `diagnostics/odom_log_interval_sec` | XYZ/RPY 日志周期；`0` 为关闭 |

### 保存 PCD

通过 YAML 控制：

```yaml
pcd_save:
    pcd_save_en: true
    interval: -1
```

`interval: -1` 表示 Ctrl+C 后把累计全局地图保存为 `PCD/scans.pcd`；正整数表示每 N 帧分段保存。长时间运行时，`-1` 会持续占用内存。

### Reloc 模式

`reloc_mid360.launch.py` 先加载完整 mapping 配置，再叠加 `config/reloc/mid360.yaml`
（`reloc_en: true`），同时额外起两个节点：

```bash
ros2 launch fast_lio reloc_mid360.launch.py [map_pcd:=/路径/scans.pcd]
```

- `icp_relocalizer`：用 PCL ICP 把实时点云对齐到先验图，算出 `T_base^map` 后发到
  `/reloc/cloud_align`，成功后发静态 `map -> odom`

输入话题类型为 `geometry_msgs/msg/PoseStamped`，默认话题 `/reloc/cloud_align`。输入必须是
`T_base^odom`，即 `odom` 坐标系中的机器人中心位姿；程序会通过 `T_lidar^base` 和
`T_imu^lidar` 转换为 EKF 使用的 `T_imu^odom`。收到位姿后会暂停高频输出、清空旧 IMU
位姿缓冲并重建局部地图。匹配由本包的 ICP 节点完成，参数在
`config/reloc/relocalization.yaml`。

用先验图导航时 Nav2 要切回 `map` 帧：`bt_navigator` 和两个 costmap 的 `global_frame`
改成 `map`，`global_costmap.plugins` 加上 `static_layer`（订阅 `/map`）；`map -> odom`
由 `icp_relocalizer` 重定位成功后发布的静态变换提供。

## 高频里程计 /OdometryHighFreq

`/OdometryHighFreq` 由独立线程以固定 200 Hz（5 ms）节拍发布，目标是"均匀节拍、时间戳严格递增、无重复位姿"的高频位姿流，供串口反馈/控制环使用。实现要点：

- **基态（`hf_base`）**：每帧 LiDAR 校正（IEKF）完成后，把校正后的 EKF 状态写入共享基态，时间戳记为该帧 `lidar_end_time`；重定基（`zero_at_start`）和重定位时同样刷新基态。
- **IMU 环形缓冲**：IMU 回调把原始样本（时间戳、角速度、加速度）写入环形缓冲，回调保持轻量，不受主线程扫描处理阻塞影响。
- **逐样本积分**：发布线程每个节拍取出缓冲中时间戳比基态新的样本，用与 EKF 相同的运动学（`get_f`，含加速度 G 归一化 `G_m_s2 / |mean_acc|`）逐样本前推。位姿因此始终包含全部真实 IMU 测量，校正间漂移与 EKF 内部预测一致。
- **空拍填充**：某拍没有新 IMU 样本（节拍与 IMU 相位错位）时，用最新 IMU 输入做一小步常数外推（步长 = 0.8 × 实测采样周期，上限 4.5 ms），保证每拍时间戳严格递增、不重复。
- **单调性钳制**：扫描校正重播种后，若计算出的时间戳落后于上一拍已发布值，把时间戳前推一个正常采样周期（约 5 ms），既杜绝时间回退，又把校正处的毫米级位姿跳变摊到正常步长上，不会产生微分速度尖峰。
- **断流保护**：连续 40 拍没有新样本被积分（IMU 断流）时停止填充、保持位姿，避免纯外推漂移。
- 整个链路不依赖系统时钟，bag 回放（消息时间戳与当前系统时间不一致）与实机运行行为一致。

实测（MID-360，bag 回放，~117 s、含实际运动）：发布 200 Hz 均匀、时间戳严格递增（dt=0 与回退均为 0）、真实位姿更新 200 Hz；dt p95 ≈ 6 ms、无 >20 ms 缺口；2D 速度 max ≈ 2 m/s（无尖峰）；低速段高频预测与 LiDAR 校正的偏差 mean ≈ 0.75 mm、p95 ≈ 2.1 mm，yaw 偏差 mean ≈ 0.1°。每帧校正瞬间存在毫米级位姿跳变，属预测→校正的固有行为。

## 运行检查

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /OdometryHighFreq
ros2 run tf2_ros tf2_echo odom base_link_hf
ros2 topic echo /Odometry --once
```

`/Odometry` 使用 Reliable QoS；`/OdometryHighFreq` 使用 Best Effort、Keep Last 1。串口节点直接读取 `odom -> base_link_hf`，不会再创建第二套零点。
