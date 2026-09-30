# LineFit Ground Segmentation Core

本包提供 linefit 地面分割核心 C++ 算法；ROS 话题、TF、车体滤除和消息时间戳处理由 [`linefit_ground_segmentation_ros`](../linefit_ground_segmentation_ros/README.md) 负责。

## 算法概览

点云按水平距离划分为径向 bin，并按方位角划分为 segment。每个 segment 内按距离顺序分析点列，结合传感器高度、局部拟合线、相邻点距离和高度变化判断地面连续性。算法输出地面/非地面分类，不承担机器人自身滤除或 Nav2 障碍层清除。

## 参数调节方向

ROS 节点通过 [`segmentation_params.yaml`](../linefit_ground_segmentation_ros/launch/segmentation_params.yaml) 配置核心算法。

- `n_bins`、`n_segments` 越大，角向/径向分辨率越高，计算量和稀疏区域不稳定风险也会上升。
- `max_dist_to_line` 越小，地面判定越严格；过小可能把坡地或稀疏地面归为非地面。
- `min_slope`、`max_slope` 限定可跟踪地面的局部坡度范围。
- `long_threshold`、`max_long_height` 控制相邻点较远时对地面连续性的判断。
- `max_start_height` 限制开启新的地面线时允许的高度跳变。
- `line_search_angle` 控制角度邻域中搜索地面线的范围。

所有距离单位为米，角度搜索范围以弧度表示。建议使用记录点云逐项调参，并通过 ROS 包的测试/可视化节点检查分类边界。
