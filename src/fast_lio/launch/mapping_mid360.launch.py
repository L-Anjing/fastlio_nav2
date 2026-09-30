from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def flatten_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    result = {}

    def visit(value, prefix=""):
        for key, child in value.items():
            name = f"{prefix}/{key}" if prefix else key
            if isinstance(child, dict):
                visit(child, name)
            else:
                result[name] = child

    visit(data)
    return result


def generate_launch_description():
    package_dir = Path(get_package_share_directory("fast_lio"))
    params = flatten_yaml(package_dir / "config" / "mapping" / "mid360.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # TF 全部由 fastlio_mapping 按 YAML 外参发布：
        #   odom -> base_link / base_link_hf，base_link -> livox_frame -> imu_link
        # frames.lidar=livox_frame 与 Livox 驱动消息的 frame_id 一致，所以这里不再需要
        # robot_state_publisher / URDF。
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fast_lio",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(package_dir / "rviz_cfg" / "mapping_mid360.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
