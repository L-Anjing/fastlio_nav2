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
- `src/nav_bringup/config/reality/`：实车配置主目录。
- `docs/config_single_source.md`：配置文件职责、参数说明、改哪个文件的总说明。
- `docs/chassis_integration.md`：底盘控制对接说明。
- `src/navigation/teb_local_planner/README.md`：TEB 控制器使用与调参说明。

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

## 4. 坐标系与控制链注意事项

当前工程里，Nav2 使用：

- `robot_base_frame: base_link_fake`

并由 `fake_vel_transform` 参与控制链路。如果你后续想改回 `base_link`，必须同步修改：

- `bt_navigator.robot_base_frame`
- `local_costmap.robot_base_frame`
- `global_costmap.robot_base_frame`
- `recoveries_server.robot_base_frame`
- 以及底盘控制节点对应的话题/TF 对齐

## 5. 常用命令

```bash
./build.sh
./save_pcd.sh
./save_grid_map.sh
```

## 6. 配置与对接文档

- 配置文件职责、参数说明、改哪个文件： [docs/config_single_source.md](/home/pi/workspace2/fastlio_nav2/docs/config_single_source.md#L1)
- 底盘控制对接： [docs/chassis_integration.md](/home/pi/workspace2/fastlio_nav2/docs/chassis_integration.md#L1)
- TEB 详细使用和排错： [src/navigation/teb_local_planner/README.md](/home/pi/workspace2/fastlio_nav2/src/navigation/teb_local_planner/README.md#L1)
