# LineFit Ground Segmentation ROS

ROS 2 分割节点把 Livox 点云划分为地面和障碍两路，供导航 costmap 做射线清除与障碍标记。

## 话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | MID-360 原始点云 |
| 输出 | `/segmentation/ground` | `sensor_msgs/msg/PointCloud2` | 地面点，供 costmap 清除旧障碍 |
| 输出 | `/segmentation/obstacle` | `sensor_msgs/msg/PointCloud2` | 非地面点，供 costmap 标记障碍 |

默认配置：[`launch/segmentation_params.yaml`](launch/segmentation_params.yaml)，启动入口：[`launch/segmentation.launch.py`](launch/segmentation.launch.py)。

## 处理顺序

1. 按输入消息时间戳将点变换到 `base_link`，供车体滤除与高度判断使用。
2. 在配置的车体三维包围盒内删除自身回波；包围盒外的近距离物体仍保留。
3. LineFit 按径向 bin 和角度 segment 拟合局部地面线。
4. 地面点输出至 ground 话题；非地面点仅在 `obstacle_min_height..obstacle_max_height` 高度范围内输出。
5. `restamp_output: true` 时以当前 ROS 时间戳发布输出，便于 costmap 的 TF/message filter 接收动态点云。

## 常用参数

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `n_threads` | 4 | 分割算法使用的工作线程数 |
| `r_max` | 11.0 m | 参与处理的最大径向距离 |
| `n_bins` / `n_segments` | 120 / 360 | 径向与角度方向的分区数量 |
| `sensor_height` | 0.30 m | 雷达相对地面的安装高度 |
| `max_dist_to_line` | 0.10 m | 点到拟合地面线的最大距离 |
| `min_slope` / `max_slope` | -0.4 / 0.4 | 地面线坡度判定范围 |
| `obstacle_min_height` / `obstacle_max_height` | 0.08 / 1.5 m | 障碍点的离地高度范围 |
| `self_filter.box_min` / `box_max` | `[-0.4,-0.4,0]` / `[0.4,0.4,1.4]` m | `base_link` 中车体包围盒边界 |
| `restamp_output` | true | 将输出消息时间戳改为当前 ROS 时间 |
| `visualize` | false | 打开算法可视化调试 |

实际参数以 YAML 为准。传感器高度、车体盒和障碍高度范围应按实车尺寸核对；错误包围盒会删除真实障碍或把车体回波送进 costmap。

## 运行检查

```bash
ros2 launch linefit_ground_segmentation_ros segmentation.launch.py
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /segmentation/ground
ros2 topic hz /segmentation/obstacle
```

用 RViz 查看输入与两路输出，并确认点云 frame 能通过 TF 转换到 `base_link`/`map`。若 costmap 留下移动障碍的旧栅格，需进一步检查地面清除点云和 clearing 射线是否覆盖旧位置。
