# fastlio_nav2（实车建图与导航）

本仓库当前定位为“实车链路优先”（Livox Mid360 + FAST_LIO/Point_LIO + Nav2 + TEB），用于：

- 边建图边导航（`mode:=mapping`）
- 已有全局地图导航（`mode:=nav`）

主入口：`src/nav_bringup/launch/bringup_real.launch.py`

## 1. 环境与编译

### 1.1 软件环境

- Ubuntu 22.04
- ROS 2 Humble
- colcon / rosdep

### 1.2 编译

```bash
git clone --recursive https://github.com/L-Anjing/fastlio_nav2.git
cd fastlio_nav2
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
./build.sh
source install/setup.bash
```

### 1.3 Livox SDK2（若本机未安装）

```bash
sudo apt update
sudo apt install -y cmake build-essential

git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

## 2. 核心文件作用（迁移时最常改）

- `build.sh`：工作区编译。
- `save_pcd.sh`：保存 FAST_LIO 点云地图（`.pcd`）。
- `save_grid_map.sh`：保存 slam_toolbox 的 2D 栅格地图（`.yaml/.pgm`）。
- `src/nav_bringup/launch/bringup_real.launch.py`：实车总启动入口，选择 `mode/lio/localization`。
- `src/nav_bringup/config/reality/nav2_params_real.yaml`：Nav2 + TEB 主参数（速度、避障、膨胀、容差等）。
- `src/nav_bringup/config/reality/measurement_params_real.yaml`：`base_link -> livox_frame` 外参（雷达安装位姿）。
- `src/nav_bringup/urdf/sentry_robot_real.xacro`：车体几何/碰撞模型（用于 TF 与可视化描述）。
- `src/nav_bringup/config/reality/fastlio_mid360_real.yaml`：FAST_LIO 参数（含 LiDAR-IMU 外参、PCD 地图路径）。
- `src/nav_bringup/config/reality/MID360_config.json`：Livox 网络参数（雷达 IP、主机网口 IP、端口）。
- `src/navigation/teb_local_planner/README.md`：本工程 TEB 使用与调参说明（本文档同步维护）。
- `docs` : 底盘控制接口以及参数配置说明文件

## 3. 实车流程

### 3.1 边建图边导航（在线建图）

```bash
ros2 launch nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=mapping \
lio:=fastlio \
lio_rviz:=True \
nav_rviz:=False
```

建图完成后保存：

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'YOUR_WORLD_NAME'}}"
./save_pcd.sh
```

### 3.2 已知地图导航（全局地图已存在）

```bash
ros2 launch nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=nav \
lio:=fastlio \
localization:=slam_toolbox \
lio_rviz:=False \
nav_rviz:=True
```

`localization`（仅 `mode:=nav` 有效）支持：

- `slam_toolbox`
- `amcl`
- `icp`

## 4. 方形车适配：必须改哪些参数

车是方形底盘，但尺寸可能不同。建议按下面顺序改。

### 4.1 第一步：改雷达网络（能连上雷达）

文件：`src/nav_bringup/config/reality/MID360_config.json`

必改：

- `lidar_configs[0].ip`
- `MID360.host_net_info.*_ip`

按需改：

- `MID360.*_port`（端口冲突时）

### 4.2 第二步：改雷达安装位姿（点云不歪）

文件：`src/nav_bringup/config/reality/measurement_params_real.yaml`

- `base_link2livox_frame.xyz`
- `base_link2livox_frame.rpy`

说明：

- 这是 `base_link -> livox_frame` 的 TF，`bringup_real.launch.py` 会把它传给 `sentry_robot_real.xacro`。
- 这组参数错了，常见现象是：地图倾斜、定位漂移、绕障异常。

### 4.3 第三步：改车体几何（描述尺寸）

文件：`src/nav_bringup/urdf/sentry_robot_real.xacro`

重点字段：

- `base_link` 视觉/碰撞盒：`<box size="x y z"/>`
- 轮子位置（仅可视化/模型用途）：`base_to_wheel*` 的 `origin xyz`
- 雷达默认安装位姿参数：`<xacro:arg name="xyz" ...>`、`<xacro:arg name="rpy" ...>`

建议：

- 以“外接矩形真实尺寸”定义 `base_link` 盒体。
- `x/y` 尺寸与 Nav2 footprint 保持同源（见 4.4）。

### 4.4 第四步：改 Nav2 碰撞模型（真正影响规划避障）

文件：`src/nav_bringup/config/reality/nav2_params_real.yaml`

当前默认是：

- `local_costmap...robot_radius: 0.2`
- `global_costmap...robot_radius: 0.2`

对于方形车，建议改成 `footprint` 多边形（local/global 都要改）：

1. 注释 `robot_radius`
2. 新增 `footprint`

示例（车长 `L=0.56m`，车宽 `W=0.46m`，安全裕量 `m=0.02m`）：

```yaml
footprint: "[[0.30,0.25],[0.30,-0.25],[-0.30,-0.25],[-0.30,0.25]]"
```

为什么要这样改：

- `robot_radius` 只适合近似圆形底盘。
- 方形车用圆形半径会导致“该过不敢过”或“贴边太近”。

### 4.5 第五步：让 TEB 与 costmap 尺寸一致

文件：`src/nav_bringup/config/reality/nav2_params_real.yaml`
位置：`controller_server.ros__parameters.FollowPath`

补充：

```yaml
footprint_model:
  type: "polygon"
  vertices: [[0.30,0.25],[0.30,-0.25],[-0.30,-0.25],[-0.30,0.25]]
```

为什么：

- costmap 与 TEB 尺寸不一致会导致轨迹看似可行但实车擦障或震荡。

## 5. nav2_params_real.yaml 如何做“平滑迁移”

下面只列最关键参数，并说明“为什么先调它”。

### 5.1 先保守速度（先稳）

位置：`controller_server.ros__parameters.FollowPath`

- `max_vel_x`
- `max_vel_y`
- `max_vel_theta`
- `acc_lim_x`
- `acc_lim_y`
- `acc_lim_theta`

原因：先限制动态性能，避免新车刚接入时因底盘参数不匹配导致抖动或冲撞。

### 5.2 再扩安全距离（先不撞）

位置：

- `FollowPath.min_obstacle_dist`
- `FollowPath.inflation_dist`
- `local_costmap...inflation_layer.inflation_radius`
- `global_costmap...inflation_layer.inflation_radius`

原因：尺寸迁移初期环境误差更大，先提高保守性。

### 5.3 再提控制响应（再变快）

位置：

- `controller_server.controller_frequency`
- `FollowPath.dt_ref`
- `FollowPath.dt_hysteresis`
- `FollowPath.no_inner_iterations`
- `FollowPath.no_outer_iterations`

原因：确保系统稳定后，再优化轨迹平滑与响应速度。

### 5.4 最后收终点精度（再变准）

位置：

- `general_goal_checker.xy_goal_tolerance`
- `general_goal_checker.yaw_goal_tolerance`

原因：过早收紧容差容易导致“临门抖动”或反复修正。

## 6. 雷达位置到底改哪里（结论）

换车但雷达型号不变时，优先级如下：

1. `measurement_params_real.yaml`

- 改 `base_link2livox_frame.xyz/rpy`（最关键）

2. `MID360_config.json`

- 改网络 IP，确保驱动联通

3. `fastlio_mid360_real.yaml`

- `mapping.extrinsic_T/R` 是 LiDAR-IMU 外参，通常不是“换底盘尺寸”要改的参数

## 7. 坐标系与控制链注意事项

当前工程里，Nav2 使用：

- `robot_base_frame: base_link_fake`

并由 `fake_vel_transform` 参与控制链路。如果你后续想改回 `base_link`，必须同步修改：

- `bt_navigator.robot_base_frame`
- `local_costmap.robot_base_frame`
- `global_costmap.robot_base_frame`
- `recoveries_server.robot_base_frame`
- 以及底盘控制节点对应的话题/TF 对齐

## 8. 常用命令

```bash
./build.sh
./save_pcd.sh
./save_grid_map.sh
```

TEB 详细使用和排错请看：`src/navigation/teb_local_planner/README.md`
