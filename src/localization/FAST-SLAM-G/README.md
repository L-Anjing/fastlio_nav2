# FAST-SLAM-G

> fastlio_nav2 当前实车主链路说明：本包作为 `fast_lio_sam` 被 `nav_bringup` 启动，负责输出 `/Odometry`、`/scan`、在线 `/map` 和 `map -> odom -> base_link` TF。Nav2 不再使用 ICP、AMCL、slam_toolbox 或 Point-LIO。实车主用配置请优先修改 `src/nav_bringup/config/reality/fast_lio_sam_gridmap_mid360_real.yaml`，不要在多个 FAST-LIO 配置文件之间重复维护。

当前仍需重点排查的问题见仓库根目录 `docs/remaining_issues.md`，尤其是 `/scan` 时间戳/TF message filter 丢包和 `/map` 尺寸频繁变化。

A LiDAR-inertial SLAM system that integrates **FAST-LIO2** as the high-frequency frontend with a **LIO-SAM-style** factor graph backend for global optimization, and further provides a **GridMap** projection pipeline for navigation, supporting **RoboSense LiDARs**, **Unilidar LiDARs**, and compatible with both **ROS1** and **ROS2**.

## 🧩 Contributions

* A SLAM system that integrates FAST-LIO2 with a LIO-SAM-style factor graph backend.

* ROS1 and ROS2 adaptation

* High-frequency odometry via IMU propagation between LiDAR scans

* Manual initial pose setting for relocalization

* Stationary detection and adaptive weight handling between LiDAR update scans and ZUPT

* Online GridMap projection from optimized global point cloud to 2D `OccupancyGrid`

* Support for RoboSense LiDARs, Unilidar LiDARs

## 🛠️ Prerequisites

### Dependency

* [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)

```bash
# Ubuntu 20.04
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
# Ubuntu 22.04
sudo add-apt-repository ppa:borglab/gtsam-release-4.1

sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### ROS1 Build

```bash
mkdir fastlio_sam_ws
cd fastlio_sam_ws

mkdir src && cd src
git clone https://github.com/RightTr/FAST-LIO-SAM.git

cd src/FAST-LIO-SAM
git submodule update --init --recursive

# ROS1 build
./build.sh ROS1

# ROS2 build
./build.sh humble
```

## 🚀 Usage

### LIO-SAM-style Backend

```bash
cd fastlio_ws
source devel/setup.bash
# e.g.
roslaunch fast_lio_sam sam_airy.launch
```

### GridMap Pipeline

For navigation-oriented usage, the recommended entry is the GridMap pipeline. It keeps the optimized 3D map and the exported 2D navigation map in the same SLAM system instead of running another online 2D SLAM node.

```bash
# ROS2
cd fastlio_ws
source install/setup.bash
ros2 launch fast_lio_sam gridmap_mid360.launch.py
```

### Relocalization

The modified system supports relocalization using manually set odometry poses. Once odometry poses are published to the */reloc_topic* (according to the following .yaml file), the system will reset the system and the initial pose according to your input.

Run relocalization mode:

```bash
# e.g.
# ROS1
cd fastlio_ws
source devel/setup.bash
roslaunch fast_lio_sam reloc_mid360.launch
# Publish geometry_msgs::PoseStamped to the /reloc_topic

# ROS2
cd fastlio_ws
source install/setup.bash
ros2 launch fast_lio_sam reloc_mid360.launch.py
# Publish geometry_msgs::msg::PoseStamped to the /reloc_topic
```

## Stationary detection and adaptive weight handling

The system will adjust the confidence of the ZUPT and LiDAR updates based on the detected motion state, using **accelerometer and gyroscope variances** as well as **the EMA of velocity**. When the system detects a stationary state, it will increase the confidence of the ZUPT update and decrease the confidence of the LiDAR update, and vice versa when in motion.

Check the related parameters in the .yaml files.

```yaml
zupt:
    use_zupt: true                    # enable adaptive zero velocity update
    zupt_acc_var_threshold:  0.0001   # (m/s²)² per-axis acc variance → full-confidence threshold
    zupt_gyro_var_threshold: 0.00001  # (rad/s)² per-axis gyro variance → full-confidence threshold

    # Static confidence: exp(-3 * max_normalized_variance), in [0,1]
    zupt_confidence_min: 0.05         # below this no ZUPT is applied

    # Dynamic ZUPT weight: R = clamp(zupt_r_min / eff_confidence, zupt_r_max)
    zupt_r_min: 1.0e-5                # ZUPT measurement noise at full confidence
    zupt_r_max: 1.0                   # ZUPT measurement noise cap (≈ no constraint)

    # Dynamic LiDAR weight: lidar_cov = LASER_POINT_COV * (1 + scale*conf) * max(1, residual/ref)
    lidar_cov_static_scale: 5.0       # LiDAR cov multiplier at full confidence
    lidar_residual_ref: 0.05          # (m) reference residual; above this LiDAR trust reduces

    # Covariance inflation (lock-in prevention)
    cov_inflate_start: 200            # IMU steps of continuous static before inflation kicks in
    cov_inflate_pos:   1.0e-7         # per-step position covariance inflation
    cov_inflate_rot:   1.0e-8         # per-step rotation covariance inflation
```

### High frequency odometry via IMU propagation between LiDAR scans

Subscribe the topic */OdometryHighFreq* to receive high frequency odometry output via IMU propagation between LiDAR scans.

### GridMap

The GridMap node projects the optimized global point cloud to `/map` as a 2D `OccupancyGrid`.

The current design assumes:

* the global point cloud comes from the backend-optimized map

* the 2D navigation map should stay consistent with the 3D map

### Extended LiDAR support

Now, FAST-LIO supports tracking and mapping using the RoboSense LiDARs (e.g., RoboSense Airy) and Unilidar LiDARs (e.g., Unilidar L2). Check the related files in ./config and ./launch folder.

```bash
# e.g.
roslaunch fast_lio_sam mapping_airy.launch
```

## 📝 TODO List

* [x] Full ROS2 adaptation
* [x] ROS2 adaptation Test
* [x] GridMap projection pipeline
* [ ] Incremental GridMap update optimization

## 📚 Related Works

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

[FAST-LIO official repository](https://github.com/hku-mars/FAST_LIO.git)

[FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM.git)

[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)

[robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio.git)

[point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar.git)
