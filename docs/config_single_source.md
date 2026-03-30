# 配置唯一化与参数修改说明

本文档只回答一件事：本工程里遇到某类问题时，到底该改哪个配置文件，以及为什么改它。

目标是把 `src/nav_bringup/config/reality/` 作为实车主链路的唯一配置源，并把常见修改路径说明清楚，避免“同一个含义改两三份文件”的情况。

## 1. 总原则

- 实车相关配置，优先只改 `src/nav_bringup/config/reality/`
- 下游包内的同名配置，如果已经软链接到 `reality`，不要再把它当成源文件维护
- 启动入口统一从 [bringup_real.launch.py](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/launch/bringup_real.launch.py#L1) 进入
- 如果是“物理安装关系”，优先改 [measurement_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/measurement_params_real.yaml#L1)
- 如果是“算法参数”，再去对应算法自己的 yaml 改

## 2. 已完成的唯一化

以下下游路径已经统一到 `src/nav_bringup/config/reality/`：

- FAST_LIO
  - `src/localization/FAST_LIO/config_ROS2/mapping/mid360.yaml`
  - `src/localization/FAST_LIO/config_ROS2/reloc/mid360.yaml`
  - `src/localization/FAST_LIO/config_ROS2/debug_mid360.yaml`
  - 统一源： [fastlio_mid360_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/fastlio_mid360_real.yaml#L1)
- ICP
  - `src/localization/icp_registration/config/icp.yaml`
  - 统一源： [icp_registration_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/icp_registration_real.yaml#L1)
- Nav2
  - `src/navigation/navigation/params/nav2_params.yaml`
  - 统一源： [nav2_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/nav2_params_real.yaml#L1)
- slam_toolbox 在线建图
  - `src/navigation/navigation/params/mapper_params_online_async.yaml`
  - 统一源： [mapper_params_online_async_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/mapper_params_online_async_real.yaml#L1)
- 地面分割
  - `src/perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/launch/segmentation_params.yaml`
  - 统一源： [segmentation_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/segmentation_real.yaml#L1)
- Livox MID360 驱动
  - `src/driver/livox_ros_driver2/src/config/MID360_config.json`
  - 统一源： [MID360_config.json](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/MID360_config.json#L1)

结论：上面这些入口以后都按 `reality` 目录中的文件改。

## 3. 配置文件职责总表

| 需求                                                  | 只改这个文件                                                                                                                                 | 原因                   |
| ----------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------- |
| 雷达在车上的安装位置、安装朝向、FAST_LIO 输出位姿补偿 | [measurement_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/measurement_params_real.yaml#L1)                 | 物理安装单一入口       |
| Livox 雷达网络 IP、端口                               | [MID360_config.json](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/MID360_config.json#L1)                                     | 驱动联通层             |
| FAST_LIO 的 LiDAR-IMU 外参、滤波、建图与发布参数      | [fastlio_mid360_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/fastlio_mid360_real.yaml#L1)                         | 算法层，不是车体安装层 |
| Nav2 / TEB 速度、避障、安全距离、footprint            | [nav2_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/nav2_params_real.yaml#L1)                               | 导航控制层             |
| slam_toolbox 在线建图                                 | [mapper_params_online_async_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/mapper_params_online_async_real.yaml#L1) | 2D 建图层              |
| slam_toolbox 已知地图定位                             | [mapper_params_localization_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/mapper_params_localization_real.yaml#L1) | 2D 定位层              |
| ICP 定位                                              | [icp_registration_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/icp_registration_real.yaml#L1)                     | 点云配准定位层         |
| 地面/障碍物分割                                       | [segmentation_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/segmentation_real.yaml#L1)                             | 感知预处理层           |

## 4. 雷达安装参数：现在统一改哪里

雷达安装相关参数现在统一收敛到：

- [measurement_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/measurement_params_real.yaml#L1)

当前结构：

```yaml
robot_measurements:
  # base_link is defined at the ground-projected center of the vehicle:
  # z = 0 at ground plane, x/y = chassis center.
  vehicle_body:
    size: [0.6852, 0.57, 1.3345]

  base_link_to_livox:
    xyz: [0.04851, 0.285, 0.1845]
    rpy: [0.0, 0.0, 0.0]

  fastlio_vehicle_pose:
    enabled: true
    xyz: [0.31262, 0.0, 0.12925]
    initial_zero_mean:
      enabled: true
      frames: 20
```

### 4.1 `base_link_to_livox`

对应：

- `base_link -> livox_frame` 的 TF
- `robot_state_publisher` 使用的雷达安装位姿
- `ground_segmentation.sensor_height` 的来源

具体字段：

- `xyz`
  - 含义：雷达坐标系原点相对 `base_link` 的平移，单位米
  - 顺序：`[x, y, z]`
  - 常见修改场景：
    - 雷达前后位置变了：改 `x`
    - 雷达左右偏了：改 `y`
    - 雷达高度变了：改 `z`
- `rpy`
  - 含义：雷达相对 `base_link` 的姿态，单位弧度
  - 顺序：`[roll, pitch, yaw]`
  - 常见修改场景：
    - 雷达有俯仰：改 `pitch`
    - 雷达有左右倾斜：改 `roll`
    - 雷达安装方向不正：改 `yaw`

什么时候改这里：

- 换车但雷达型号没变
- 雷达支架位置变了
- 雷达高度、前后、左右位置变了
- RViz 里 `livox_frame` 明显不在车体正确位置

为什么统一到这里：

- 这是真正的物理安装关系
- 启动时会由 [bringup_real.launch.py:31](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/launch/bringup_real.launch.py#L31) 读入，再传给 [sentry_robot_real.xacro](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/urdf/sentry_robot_real.xacro#L1)
- 同时会自动覆盖地面分割的 `sensor_height`
- 不应该再分散写到多个 yaml 或 cpp 里

### 4.2 `vehicle_body`

对应：

- `sentry_robot_real.xacro` 的车体盒体尺寸
- Nav2 local/global costmap 的 `footprint`
- TEB 的 `footprint_model`

具体字段：

- `size`
  - 含义：车体外接盒尺寸，单位米
  - 顺序：`[length_x, width_y, height_z]`

什么时候改这里：

- 换了一台尺寸不同的车
- 车体结构改了，外形包络变了
- 导航避障应该按新的外轮廓重新计算

为什么统一到这里：

- 车体尺寸属于物理测量参数，不应该分别手改 `xacro` 和 `nav2_params_real.yaml`
- 当前由 [bringup_real.launch.py](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/launch/bringup_real.launch.py#L1) 自动同步到车体模型和导航 footprint
- 当前会自动生成：
  - URDF 盒体尺寸：`0.6852 x 0.57 x 1.3345`
  - Nav2 / TEB footprint：`[[0.3426,0.285],[0.3426,-0.285],[-0.3426,-0.285],[-0.3426,0.285]]`

### 4.3 `fastlio_vehicle_pose`

对应：

- FAST_LIO 发布 `/Odometry` 和路径时，对输出参考点做的车体位姿补偿

具体字段：

- `enabled`
  - 是否启用这层补偿
- `xyz`
  - 把 FAST_LIO 输出位姿从传感器参考点补偿到车体参考点
- `initial_zero_mean.enabled`
  - 是否启用启动初期零均值平滑
- `initial_zero_mean.frames`
  - 前多少帧参与求平均

什么时候改这里：

- 你希望 `/Odometry` 代表车体中心，而不是雷达安装点
- 换了车体尺寸或参考点定义，但 FAST_LIO 本身算法参数不想动

什么时候不要改这里：

- 只是雷达支架位置变了，但你只想修正 TF
- 只是 LiDAR 与 IMU 外参变了

为什么也放在这个文件：

- 它和“车体参考点定义”属于同一类物理测量信息
- 比继续散落在 `laserMapping.cpp` 或 `fastlio_mid360_real.yaml` 更容易维护
- 当前由 [bringup_real.launch.py:248](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/launch/bringup_real.launch.py#L248) 注入 FAST_LIO

## 5. FAST_LIO 参数改哪里

文件：

- [fastlio_mid360_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/fastlio_mid360_real.yaml#L1)

这里主要放的是算法层参数，不是车体安装参数。

重点关注：

- `common.lid_topic`
  - 雷达点云输入话题
- `common.imu_topic`
  - IMU 输入话题
- `mapping.extrinsic_T`
  - LiDAR 相对 IMU 的平移外参
- `mapping.extrinsic_R`
  - LiDAR 相对 IMU 的旋转外参
- `filter_size_surf`
  - 点云下采样尺寸
- `filter_size_map`
  - 地图下采样尺寸
- `mapping.det_range`
  - 有效检测范围
- `pcd_save.*`
  - 保存 PCD 地图相关参数

什么时候改这里：

- 你重新标定了 LiDAR-IMU 外参
- 你想改 FAST_LIO 的滤波、建图效果、地图保存行为

什么时候不要改这里：

- 雷达在车上的安装位置变化
- 车体中心定义变化

## 6. Nav2 / TEB 参数改哪里

文件：

- [nav2_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/nav2_params_real.yaml#L1)

这里解决的是“导航行为”问题，不是“传感器安装”问题。

说明：

- 车体几何尺寸现在优先来自 [measurement_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/measurement_params_real.yaml#L1) 的 `vehicle_body.size`
- `bringup_real.launch.py` 会在启动时自动覆盖 Nav2/TEB footprint
- 你当前这台车会自动生成：
  - 半长：`0.3426m`
  - 半宽：`0.285m`
- `nav2_params_real.yaml` 里仍然保留 footprint 字段作为可读默认值，但不再建议手工维护这部分物理尺寸

### 6.1 方形车尺寸迁移

优先改：

- `measurement_params_real.yaml` 的 `vehicle_body.size`

原因：

- 方形车不适合长期只用 `robot_radius`
- costmap 和 TEB 的几何模型必须一致，否则会出现“局部轨迹看起来能走，实车却擦边”的现象
- 现在由 launch 自动同时覆盖两者，避免手工同步出错

### 6.2 平滑迁移建议顺序

先改速度约束：

- `max_vel_x`
- `max_vel_y`
- `max_vel_theta`
- `acc_lim_x`
- `acc_lim_y`
- `acc_lim_theta`

原因：

- 新底盘刚接入时，先稳住动态响应，避免抖动和冲撞

再改安全距离：

- `min_obstacle_dist`
- `inflation_dist`
- local/global costmap 的 `inflation_radius`

原因：

- 先让车“不撞”，再去追求通过性和速度

最后再改到点精度：

- `xy_goal_tolerance`
- `yaw_goal_tolerance`

原因：

- 太早收紧容差，容易出现终点抖动

## 7. Livox 网络参数改哪里

文件：

- [MID360_config.json](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/MID360_config.json#L1)

重点字段：

- `lidar_configs[0].ip`
- `MID360.host_net_info.cmd_data_ip`
- `MID360.host_net_info.point_data_ip`
- `MID360.host_net_info.imu_data_ip`
- 各类 `*_port`

什么时候改这里：

- 雷达连不上
- 换了电脑网卡 IP
- 多雷达或端口冲突

## 8. slam_toolbox / ICP / 地面分割分别改哪里

### 8.1 slam_toolbox 在线建图

- 文件： [mapper_params_online_async_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/mapper_params_online_async_real.yaml#L1)

常改内容：

- 分辨率
- 地图更新频率
- 激光话题与坐标系

### 8.2 slam_toolbox 已知地图定位

- 文件： [mapper_params_localization_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/mapper_params_localization_real.yaml#L1)

常改内容：

- `map_frame`
- `odom_frame`
- `base_frame`
- 激光话题

### 8.3 ICP 定位

- 文件： [icp_registration_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/icp_registration_real.yaml#L1)

常改内容：

- 点云输入话题
- 参考里程计坐标系
- 匹配阈值

### 8.4 地面分割

- 文件： [segmentation_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/segmentation_real.yaml#L1)

常改内容：

- 地面阈值
- 障碍物分割高度
- 发布输出话题

说明：

- `sensor_height` 现在由 [measurement_params_real.yaml](/home/pi/workspace2/fastlio_nav2/src/nav_bringup/config/reality/measurement_params_real.yaml#L1) 中 `base_link_to_livox.xyz[2]` 自动覆盖
- 前提是假设 `base_link` 定义在车体中心投影到地面的点，且 `z=0` 在地面
- 以你当前填写的参数为例，运行时 `sensor_height = 0.1845`
