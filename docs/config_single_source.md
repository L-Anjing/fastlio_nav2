# 配置唯一化约定（Single Source of Truth）

本文档说明本工程“到底改哪个配置文件”。

## 1. 目标

将 `src/nav_bringup/config/reality/` 作为实车主链路的唯一配置源。

## 2. 已完成唯一化（下游软链接 -> reality）

以下下游路径已统一软链接到 `src/nav_bringup/config/reality/`：

- FastLIO（ROS2）

  - `src/localization/FAST_LIO/config_ROS2/mapping/mid360.yaml`
    -> `src/nav_bringup/config/reality/fastlio_mid360_real.yaml`
  - `src/localization/FAST_LIO/config_ROS2/reloc/mid360.yaml`
    -> `src/nav_bringup/config/reality/fastlio_mid360_real.yaml`
  - `src/localization/FAST_LIO/config_ROS2/debug_mid360.yaml`
    -> `src/nav_bringup/config/reality/fastlio_mid360_real.yaml`
- ICP Localization

  - `src/localization/icp_registration/config/icp.yaml`
    -> `src/nav_bringup/config/reality/icp_registration_real.yaml`
- Nav2

  - `src/navigation/navigation/params/nav2_params.yaml`
    -> `src/nav_bringup/config/reality/nav2_params_real.yaml`
  - `src/navigation/navigation/params/mapper_params_online_async.yaml`
    -> `src/nav_bringup/config/reality/mapper_params_online_async_real.yaml`
- 地面分割

  - `src/perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/launch/segmentation_params.yaml`
    -> `src/nav_bringup/config/reality/segmentation_real.yaml`
- Livox MID360 驱动

  - `src/driver/livox_ros_driver2/src/config/MID360_config.json`
    -> `src/nav_bringup/config/reality/MID360_config.json`

结论：以上路径改任何一个，实际都在改同一份源文件。

## 3. 为什么这样做

原先配置在多个包里重复，存在以下风险：

- 不知道该改哪份
- 同名参数在不同文件中漂移
- 同一个现象在不同启动方式下表现不一致

软链接后可保证参数定义唯一、修改路径唯一、协作更可控。

## 4. 当前仍保留多份的配置（有意不合并）

以下文件目前不建议强行并到 reality：

- ROS1 配置（`src/localization/FAST_LIO/config_ROS1/**`）
- 不同雷达型号模板（`avia/hap/horizon/ouster64/velodyne/marsim`）
- 与实车主流程无关的上游示例配置

原因：这些文件属于不同运行场景或兼容层，语义不是“同一份配置”。

## 5. 使用约定

- 实车参数只改：`src/nav_bringup/config/reality/`
- 下游路径只作为兼容入口，不直接当源维护
- 变更配置时，优先在 reality 修改并通过主入口验证：
  - `src/nav_bringup/launch/bringup_real.launch.py`

## 6. 提交到仓库后的 clone 行为

如果你把这些改动 commit 并 push：

- Linux/macOS 用户 clone 后会保留软链接语义（推荐）
- Windows 若未启用 symlink 支持，可能检出为普通文本文件（内容为链接目标路径）

如果团队包含 Windows 开发机，建议额外约定：

- 开启 Git symlink 支持，或
- 改用“同步脚本生成链接/拷贝”方案作为兼容兜底
