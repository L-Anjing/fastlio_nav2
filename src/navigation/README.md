# Navigation：地形感知全局规划

`navigation` 基于 Nav2 的 `planner_server` 与 `bt_navigator`，接收目标点和地图/障碍/地形输入，输出平滑的全局路径 `/plan`。该包不包含局部控制器、速度平滑器或底盘驱动。

## 数据流

```text
/map + /terrain/type + /terrain/direction ─┐
                                           ├→ global_costmap → TerrainMincoPlanner → /plan
/segmentation/obstacle ────────────────────┘                         └→ /terrain_minco/plan_meta
```

`/map`、`/terrain/type`、`/terrain/direction` 由 FAST-LIO 包内的 PCD 转换节点从先验地图生成；实时障碍来自地面分割包。

## 规划器

插件 `navigation/TerrainMincoPlanner` 先在滚动全局 costmap 上运行地形语义 A*，再将路线简化并交给分段五次 MINCO 优化，输出标准 `nav_msgs/msg/Path`。

- 平地碰撞净空使用 `flat_clearance_radius`；默认按 0.8 × 0.8 m 安全包络的外接圆处理。
- 坡道/台阶使用沿 `/terrain/direction` 对齐的矩形包络，并在搜索与轨迹验收时限制路径切线方向；默认最大偏角约 15°，上下行都允许。
- costmap 的 lethal 栅格用于硬碰撞净空，较低 inflation 代价用于 A* 避障偏好。
- `seed_spacing`、`output_spacing`、`validation_spacing` 分别控制初始路线采样、输出路径采样和最终验证采样。

主要参数位于 [`params/nav2_params.yaml`](params/nav2_params.yaml)，参数含义紧邻配置项。更换底盘时重点核对 `global_costmap.robot_radius`、`flat_clearance_radius`、`terrain_half_length` 和 `terrain_half_width`。

## 动态障碍与实时性

global costmap 的障碍层订阅 `/segmentation/obstacle` 进行 marking/clearing，`/segmentation/ground` 只用于 clearing。当前配置以 10 Hz 更新和发布 costmap，行为树也以 10 Hz 请求路径安全检查；射线清除范围为 12 m。障碍点云观测不做历史帧累积，旧栅格依靠后续清除射线经过对应区域后移除。

每次路径请求都会基于当前 costmap 检查缓存路径剩余段。障碍进入安全走廊、目标改变或机器人明显偏离时，缓存失效并运行新的 A* + MINCO。障碍减少后则按 `costmap_stable_time`、`optimization_check_frequency`、`optimization_cooldown` 和 `minimum_length_improvement_ratio` 控制可选的路线优化，避免小幅波动引发频繁换路。

若路径变化过频，检查 `/terrain_minco/plan_meta.reason` 和 planner 日志：`obstacle_entered_corridor` 表示新 costmap 判定路径被挡；`robot_deviated` 表示机器人相对缓存路径偏离；`goal_changed` 表示目标变化。也应在 RViz 同时查看 `/segmentation/obstacle` 与 `/global_costmap/costmap`，区分感知、清除和规划器问题。

## 路径接口

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/plan` | `nav_msgs/msg/Path` | 当前完整全局几何路径；外部控制器应整体替换旧路径 |
| `/terrain_minco/plan_meta` | `navigation/msg/PlanMeta` | 路径身份和本次发布信息 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 规划器实际使用的合成 costmap |

`PlanMeta` 与对应路径共享 header：`path_id` 仅在接受新的几何规划结果时递增；`publish_seq` 每次发布递增；`replanned` 表示是否新规划；`path_start_s` 表示裁剪后路径起点在原路径上的弧长；`reason` 说明复用或重规划原因。

## 行为树与启动

行为树位于 [`behavior_trees/plan_to_pose.xml`](behavior_trees/plan_to_pose.xml)。Goal 激活期间持续检查路径；瞬时规划失败会短暂重试，不执行清图、旋转或速度控制等恢复动作。

```bash
ros2 launch navigation bringup_navigation.py
```

启动参数和 RViz 延时定义在 [`launch/bringup_navigation.py`](launch/bringup_navigation.py)。组合式 Nav2 节点及 lifecycle 管理位于 [`launch/navigation_launch.py`](launch/navigation_launch.py)。独立调试时需确保 `map → base_link` TF、`/map` 和地形栅格已可用。

## RViz

[`rviz/nav2.rviz`](rviz/nav2.rviz) 展示先验地图、全局 costmap、规划路径与地形叠加。排查动态障碍时同时打开障碍点云和全局 costmap：前者验证分割输出，后者验证代价层合成结果。
