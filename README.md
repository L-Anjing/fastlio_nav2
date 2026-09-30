# fastlio_nav2（实车建图与导航）

FAST-LIO2 实车路径规划链路：Livox Mid360 + FAST-LIO2 + Nav2 全局规划。

启动入口：[`launch_gnome_chain.sh`](launch_gnome_chain.sh)

## 1. 主链路

```text
livox_ros_driver2
  -> /livox/lidar/pointcloud
  -> /livox/imu

fast_lio
  <- /livox/lidar/pointcloud
  <- /livox/imu
  -> /Odometry
  -> /OdometryHighFreq
  -> /cloud_registered_body
  -> TF: odom -> base_link（base_link -> livox_frame -> imu_link 为 YAML 静态外参）

linefit_ground_segmentation_ros
  <- /livox/lidar/pointcloud
  -> /segmentation/ground
  -> /segmentation/obstacle

navigation（仅全局路径规划）
  <- /map
  <- /segmentation/obstacle
  <- /terrain/type      # 由先验 PCD 自动生成的地形类型栅格
  <- /terrain/direction # 由先验 PCD 自动生成的坡道/台阶上行方向
  -> /plan              # TerrainMincoPlanner 输出的 nav_msgs/Path
  -> /terrain_minco/plan_meta # 路径身份与本次发布性质
```

`navigation` 只接收 Nav2 Goal 并发布全局路径 `/plan`，不会启动控制器、速度平滑器或任何底盘接口。
目标由 RViz 的 Nav2 Goal 工具发送；Goal 激活期间，规划器以 5 Hz 结合静态地图和最新
`/segmentation/obstacle` 点云更新全局 costmap。TerrainMincoPlanner 先做地形语义 A*，
再用分段五次 MINCO 生成连续几何路径，由 `planner_server` 直接发布
`/plan`，再由外部控制程序订阅并执行。

`/plan` 始终保持标准 `nav_msgs/msg/Path`。配套的
`/terrain_minco/plan_meta`（`navigation/msg/PlanMeta`）使用与对应 `/plan`
完全相同的 header，包含 `path_id`、`publish_seq`、`replanned`、
`path_start_s` 和 `reason`。只有成功接受一条新的 A* + MINCO 几何路径才递增
`path_id`；安全缓存的裁剪只递增 `publish_seq`，并更新 `path_start_s`。

启动顺序，脚本会各开一个终端；FAST-LIO 自带的 RViz 被关闭，只启动导航 RViz：

1. `fast_lio` 重定位链路
2. `linefit_ground_segmentation_ros`
3. `navigation`（Nav2 全局规划 + RViz）

Livox 驱动不由该脚本启动，需要在外部提供对应点云和 IMU 话题。

## 2. 关键配置文件

| 要改什么 | 文件 |
| --- | --- |
| FAST-LIO2：雷达/IMU 话题、外参、建图参数、地图保存（`pcd_save`） | [`src/fast_lio/config/mapping/mid360.yaml`](src/fast_lio/config/mapping/mid360.yaml) |
| PCD 转栅格、坡道/楼梯自动识别阈值 | [`src/fast_lio/config/reloc/relocalization.yaml`](src/fast_lio/config/reloc/relocalization.yaml) |
| Nav2 全局规划：机器人规划半径、避障、规划器插件 | [`src/navigation/params/nav2_params.yaml`](src/navigation/params/nav2_params.yaml) |
| 地面分割：车体包围盒、`sensor_height`、坡度、输入输出话题 | [`src/linefit_ground_segmentation_ros/launch/segmentation_params.yaml`](src/linefit_ground_segmentation_ros/launch/segmentation_params.yaml) |
| Livox 雷达 IP / 网络端口 | [`src/livox_ros_driver2/config/MID360_config.json`](src/livox_ros_driver2/config/MID360_config.json) |

链路里的 `base_link -> livox_frame` TF 由 `fast_lio` 按 `mid360.yaml` 的 `robot_extrinsic` 发布，不需要 robot_state_publisher。

`planner_server` 使用全局 costmap 进行碰撞检查。当前将矩形安全包络转换为与朝向无关的外接圆：

```text
本体 0.7 × 0.7 m
矩形安全包络 0.8 × 0.8 m
包络角点半径 sqrt(0.4² + 0.4²) ≈ 0.566 m
Nav2 robot_radius = 0.60 m，inflation_radius = 0.65 m
```

换车时修改 `nav2_params.yaml` 的 `global_costmap.robot_radius`。该参数只控制规划碰撞模型，不改变 RViz 中独立的车体外观显示。

地面分割不再使用圆形近距盲区；原始点先转换到 `base_link`，只删除车体
`0.7 × 0.7 × 1.4 m` 包围盒内的回波，盒外近距离障碍仍会进入 costmap。

导航 RViz 显示先验栅格地图 `/map`、叠加实时障碍物的全局 costmap、平滑后的 `/plan`
路径，以及 `Terrain` 分组里的坡道/台阶叠加层；不包含局部 costmap、局部路径或控制器调试显示。

实时障碍物在 RViz 里有三处可见：

- `obstacle`（`PointCloud2`，`/segmentation/obstacle`）：地面分割输出的原始障碍点，
  坐标系是 `livox_frame`，靠 TF 变换到 `map`。这是最直接的"雷达看到了什么"视图。
  显示项固定用 `Color Transformer: FlatColor`（白色球，0.08 m）。注意 RViz2 里
  `Color Transformer: ""` 表示"自动选择"，而自动选择按 score 取最高分：`AxisColor` 是
  255、`FlatColor` 是 0，于是没有 intensity/rgb 字段的点云会被自动染成按 Z 轴的彩虹色；
  想固定成白色必须显式写 `FlatColor`。
- `Global Costmap`（`/global_costmap/costmap`）：静态层 + ObstacleLayer + 膨胀层合并后的结果，
  实时障碍在里面是致命栅格（costmap 调色板下的紫色/青色）。这一路就是规划器真正读到的东西。



## 3. 目录结构

`src/` 下每个 ROS 包直接一层，不再按 driver / localization / navigation / perception 分组：

```text
src/
├── fast_lio/                         FAST-LIO2
├── livox_ros_driver2/                Mid360 驱动
├── linefit_ground_segmentation/      地面分割库
├── linefit_ground_segmentation_ros/  地面分割 ROS 节点 + 配置
└── navigation/                       Nav2 全局规划 + RViz（nav2_params.yaml / behavior_trees / rviz 在这里）
```

## 4. 构建与运行

### 4.1 首次构建

```bash
cd ~/workspace/fastlio_nav2
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**构建类型必须显式指定 Release。** `colcon` 默认不设置 `CMAKE_BUILD_TYPE`，空值等于
`-O0` 且不定义 `NDEBUG`，Eigen 表达式模板在这种配置下会退化到逐函数调用：同一段
MINCO + L-BFGS 规划在 `-O0` 下约 6.3 s，`-O3` 下约 0.06 s（相差约 100 倍），表现为
`planner_server` 报 “Planner loop missed its desired rate of 2.0000 Hz”。`navigation`
包的 `CMakeLists.txt` 已经内置了 Release 默认值，所以直接 `colcon build --symlink-install`
也会走 `-O3`；显式写出该参数是为了避免以后有人改回默认值。

Livox 驱动不在总启动脚本中。实车运行前先启动驱动，并确认雷达和 IMU 有数据：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
```

### 4.2 PCD 自动生成栅格与地形

`fast_lio/pcd_to_occupancy_map` 启动时读取一次先验 PCD，并自动发布几何完全一致的
三张 transient-local 栅格，以及一路只给 RViz 看的彩色叠加点云：

```text
/map                普通占据栅格
/terrain/type       0=平地，1=障碍，2=坡道，3=台阶
/terrain/direction  坡道/台阶的上行方向（0..100 对应 0..2π）
/terrain/colored    仅供 RViz 观察的彩色点云（坡道=琥珀，台阶=洋红）
```

`/terrain/type` 的取值只有 `0..3`，而 RViz 的 Map 显示三种调色板都无法把 `1/2/3`
分开（`map` 调色板下三者都接近白色，`costmap` 调色板下都接近蓝色），所以额外发布
`/terrain/colored`：坡道/台阶逐格一个点，`rgb` 按类别着色，`z` 取局部行驶表面之上
5 cm。`nav2.rviz` 的 `Terrain` 分组已经把这四路都配好（`Terrain Type` 用 costmap
调色板显示地形覆盖范围，`Terrain Classes` 显示类别颜色，`Terrain Direction` 默认关闭
——它的 `-1` 格不透明会盖住底图，需要时手动勾选，灰阶深浅表示方向角）。

算法先建立局部行驶表面高程，因此 L1/L2 平台不会因为绝对高度较高而变成障碍；
连续低残差斜面识别为坡道，重复出现 `0.15 m` 高差、`0.30 m` 间距且宽度足够的
结构识别为楼梯。识别出的坡道和楼梯会从静态障碍层释放，实时障碍仍由
`/segmentation/obstacle` 和 ObstacleLayer 处理。

比赛场地参数已经写入 `relocalization.yaml`。正常使用不需要制作 msgpack、输入
地图原点或运行地形编辑器。更换不同规格的场地时才需要修改 `stair_rise`、
`stair_tread`、坡度范围和表面高度范围。

### 4.3 日常实车启动

```bash
cd ~/workspace/fastlio_nav2
MAP_PCD=/absolute/path/to/scans.pcd ./launch_gnome_chain.sh
```

脚本依次打开 FAST-LIO 重定位、地面分割和 Nav2/RViz 窗口。启动后：

1. 机器人保持静止，等待日志出现“重定位成功”。
2. 确认 `/map`、`/terrain/type`、`/terrain/direction` 和全局 costmap 正常。
3. 在 RViz 使用 **Nav2 Goal** 设置终点。
4. `/plan` 出现后，检查路径是否避开实时障碍，并沿坡道、楼梯方向进入。

#### 启动时间都花在哪

Nav2 本体并不慢：实测从 `component_container_mt` 起来到 `planner_server` 完成
configure（含 static/obstacle/inflation 三个插件和 TerrainMinco 规划器）只用 **约 0.6 s**，
没有任何人为 sleep。感觉慢的是下面两段等待：

| 阶段 | 典型耗时 | 原因 |
| --- | --- | --- |
| Nav2 节点 configure | ~0.6 s | 加载插件、建 costmap、读参数 |
| `planner_server` activate | **8~12 s** | `global_costmap` 在 activate 里阻塞等待 `map → base_link` 的 TF，每 0.5 s 打一条 `Timed out waiting for transform from base_link to map`。这条 TF 要等 `icp_relocalizer` 重定位成功后才由静态广播器发出 |
| `bt_navigator` activate | 紧随其后 | lifecycle_manager 按 `planner_server → bt_navigator` 顺序激活，前一个不 active 就不会往下走 |
| RViz 窗口 | 默认 +12 s | `bringup_navigation.py` 里的 `nav_rviz_delay`（唯一的人为延迟），为了等 TF 就绪、避免 RViz 闪 "Fixed Frame [map] does not exist" |

`icp_relocalizer` 那边的时序是 `startup_delay: 5.0 s` + `accumulate_frames: 30`
（10 Hz 下约 3 s）+ 粗/精配准，也就是**重定位成功大约在启动后 8~10 s**。如果 Nav2 一直
停在 `Timed out waiting for transform`，说明重定位没成功——先去 FAST-LIO 窗口看有没有
“重定位成功”，而不是怀疑 Nav2 启动慢。

想让 RViz 更早出现可以调小这个延迟（代价是开头几秒会提示 Fixed Frame 不存在，TF 一到
就自动恢复）：

```bash
ros2 launch navigation bringup_navigation.py nav_rviz_delay:=2.0
```

快速检查命令：

```bash
ros2 topic echo /terrain/type --once --field info
ros2 topic echo /terrain/direction --once --field info
ros2 lifecycle get /planner_server
ros2 topic echo /plan --once
```

`pcd_to_occupancy_map` 日志会打印四类格子数量。普通平地 PCD 应显示坡道和台阶均为
零或接近零；比赛地图应在对应区域检测到非零结果。如果检测数量明显异常，先调整
`relocalization.yaml` 的检测阈值，不要直接测试跨越。

回放 rosbag 时用仿真时钟，否则 Nav2 的墙钟时间和 bag 里的时间戳对不上：

```bash
ros2 bag play <bag> --clock
USE_SIM_TIME=1 ./launch_gnome_chain.sh
```

脚本默认 source：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/fastlio_nav2/install/setup.bash
```

## 5. 使用规划结果

重定位使用当前 `odom → base_link` 作为唯一初值，不需要人工设置初始位姿。测试时让机器人初始位置距离建图原点不超过约 1.2m，并在启动后的点云累积阶段保持静止。重定位采用多航向粗配准加精配准；日志出现“重定位成功”后，再用 **Nav2 Goal** 设置规划终点。

Goal 激活期间，`bt_navigator` 以 5 Hz 请求 `planner_server`。TerrainMincoPlanner
每次只对当前剩余路径做碰撞和地形方向检查；目标改变、路径被实时障碍阻断或
机器人偏离路径时，才重新运行语义 A* + MINCO。对外发布：

```text
/plan    nav_msgs/msg/Path    连续平滑的完整几何路径
/terrain_minco/plan_meta  navigation/msg/PlanMeta  路径 ID、发布序号和重规划原因
```

实时策略分成两级：路径被阻挡时不等待，立即从机器人当前位置重跑 A* + MINCO；
致命障碍格减少后等待 `0.8 s`，最多 `1 Hz` 先运行 A* 评估候选路线。只有候选路线
预计至少缩短 `8%` 才继续运行 MINCO 并替换 `/plan`，主动优化之间至少间隔 `1.5 s`。
因此行人离开后能够恢复明显更优的路线，同时避免点云闪烁造成路径反复切换。

当安全缓存已经失效而某一帧 costmap 暂时无路时，行为树不会立即终止 Goal：首次规划
失败后以 `200 ms` 间隔补试两次，总瞬态窗口约 `0.4 s`。重试期间不发布已确认不安全
的旧路径；连续三次仍无路才报告 `Goal failed`。此机制只处理点云/清除边缘的瞬态封路，
不包含清空 costmap、旋转或其他恢复动作。

平地碰撞模型使用 0.8 × 0.8 m 安全方形的外接圆，半径
`0.565685 m`；因此动态障碍即使没有压在中心线上，只要进入该通行走廊也会使
缓存失效。坡道和台阶使用沿 `/terrain/direction` 对齐的 0.8 m 方形包络，并继续
检查进出方向。costmap 的 254 致命格用于构造这套硬净空；原有非致命 inflation
只作为 A* 远离障碍的代价偏好，不再从 inflation 边界二次膨胀。

动态障碍链路为：

```text
/livox/lidar/pointcloud（原始实时雷达 PointCloud2）
→ /segmentation/obstacle（有效半径 11 m）
→ global_costmap 的 ObstacleLayer（障碍点即时 marking+clearing、地面点补充 clearing）
→ TerrainMinco 5 Hz 检查剩余路径通行走廊
```

障碍点在 `base_link` 中还要满足离地高度 `0.08~1.5 m`；低于 8 cm 的贴地非地面点
按地面拟合边界噪声处理，避免少量误分割点直接在 costmap 中形成致命小斑块。
ObstacleLayer 不缓存历史观测帧；每帧障碍回波先清除射线路径再标记当前终点，地面回波
继续补充清除，global costmap 以 5 Hz 发布，减少移动目标留下的拖尾。

这里不能沿用局部导航常见的 `3 m` 障碍标记范围：本工程只输出路径、不驱动机器人，
机器人可能一直停在起点，超过 3 m 的路径障碍不会随着机器人前进而进入感知范围。

独立 `smoother_server` 和 `SmoothPath` BT 节点已移除，不会二次平滑，也不再
维护 `_nav2_raw_plan` 中间话题。

### 5.1 自动地形栅格

两个地形话题都使用 `nav_msgs/msg/OccupancyGrid`，坐标系应为 `map`：

- `/terrain/type`：`data` 只使用 `0=平地, 1=障碍, 2=坡道, 3=台阶`。地面至 L1、L1 至 L2 的楼梯以及直接跨层入口都标为 `3`。
- `/terrain/direction`：只在 `type >= 2` 的格子生效，`0..100` 线性编码 `0..2π`，表示地形上行方向；`-1` 表示无方向。
- `/terrain/colored`：`sensor_msgs/msg/PointCloud2`（`xyz` + `rgb`），只含坡道/台阶格，坡道=`255,191,0`、台阶=`255,0,255`，供 RViz 分色观察；规划器不订阅它。

两个地形话题与 `/map` 由同一个 PCD 转换节点同时生成，因此宽高、分辨率和原点
天然一致。若某格被识别为台阶/坡道但没有有效方向，该格按不可通行处理。
规则场地的楼梯每级长 `0.30 m`、宽 `1.00 m`、高 `0.15 m`；检测器会将完整楼梯
水平投影标为 `3`，方向指向上层平台。高度不进入 `/plan`，因为它只输出二维几何路径。

坡道和台阶段采用方向约束：A* 搜索和最终轨迹验收使用硬限制，MINCO 在优化时
持续施加方向对齐代价。路径切线与地形方向需平行或反平行，默认最大偏角为
`15°`。因此把机器人 `+x`
轴（下行时可为 `-x`）沿 `/plan` 的 Pose 朝向放置，即可使车体的一组边与坡道或
楼梯方向平行；这只是几何姿态信息，不会生成速度或底盘控制指令。

所需的 A*、L-BFGS 与 MINCO 已作为裁剪后的内部实现放在 `navigation/terrain_core`
中，并保留原 MIT 许可证。构建和运行均不依赖 `src/HWSentryNav26`，该目录可以删除。

每次 `/plan` 消息都是最新的完整路径。外部控制程序应整体替换旧路径，并自行完成跟踪、速度规划与底盘通信。本仓库的 `navigation` 启动链路不会发布 `/cmd_vel_nav`、`/cmd_vel_chassis` 或 `cmd_chassis`。

快速查看一次最新路径：

```bash
ros2 topic echo /plan --once
```
