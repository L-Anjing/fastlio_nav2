import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _format_xacro_vector_arg(value):
    if isinstance(value, str):
        return value
    return '"{}"'.format(' '.join(str(v) for v in value))


def _as_float_vector(value):
    if isinstance(value, str):
        stripped = value.strip().strip('"').strip("'")
        return [float(v) for v in stripped.split()]
    return [float(v) for v in value]


def _write_temp_yaml(data, prefix):
    temp_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix=prefix, delete=False, encoding='utf-8')
    yaml.safe_dump(data, temp_file, allow_unicode=True, sort_keys=False)
    temp_file.close()
    return temp_file.name


def _flatten_yaml_params(data, parent_key='', sep='/'):
    params = {}
    for key, value in data.items():
        param_key = f'{parent_key}{sep}{key}' if parent_key else key
        if isinstance(value, dict):
            params.update(_flatten_yaml_params(value, param_key, sep))
        else:
            params[param_key] = value
    return params


def _load_fast_lio_sam_params(yaml_file_path):
    with open(yaml_file_path, 'r', encoding='utf-8') as config_file:
        return _flatten_yaml_params(yaml.safe_load(config_file))


def generate_launch_description():
    nav_bringup_dir = get_package_share_directory('nav_bringup')
    navigation2_launch_dir = os.path.join(get_package_share_directory('navigation'), 'launch')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    launch_chassis = LaunchConfiguration('launch_chassis')
    chassis_port_name = LaunchConfiguration('chassis_port_name')
    chassis_robot_model = LaunchConfiguration('chassis_robot_model')
    chassis_cmd_vel_topic = LaunchConfiguration('chassis_cmd_vel_topic')
    chassis_odom_topic = LaunchConfiguration('chassis_odom_topic')

    start_fastlio = IfCondition(PythonExpression(["'", mode, "' in ['mapping', 'fastlio_nav']"]))

    measurement_params = yaml.safe_load(open(os.path.join(
        nav_bringup_dir, 'config', 'reality', 'measurement_params_real.yaml')))
    measurement_root = measurement_params.get('robot_measurements', {})
    base_link_to_livox = measurement_root.get('base_link_to_livox')
    if base_link_to_livox is None:
        base_link_to_livox = measurement_params['base_link2livox_frame']
    base_link_to_livox_xyz_values = _as_float_vector(base_link_to_livox['xyz'])
    base_link_to_livox_rpy_values = _as_float_vector(base_link_to_livox['rpy'])

    vehicle_body = measurement_root.get('vehicle_body', {})
    vehicle_body_size = _as_float_vector(vehicle_body.get('size', [0.6852, 0.57, 1.3345]))
    nav2_safety_margin = float(vehicle_body.get('nav2_safety_margin', 0.10))
    body_half_length = vehicle_body_size[0] / 2.0 + nav2_safety_margin
    body_half_width = vehicle_body_size[1] / 2.0 + nav2_safety_margin
    body_center_z = vehicle_body_size[2] / 2.0
    footprint_vertices = [
        [body_half_length, body_half_width],
        [body_half_length, -body_half_width],
        [-body_half_length, -body_half_width],
        [-body_half_length, body_half_width],
    ]
    nav2_footprint = str(footprint_vertices).replace(' ', '')

    robot_description = Command(['xacro ', os.path.join(
        nav_bringup_dir, 'urdf', 'sentry_robot_real.xacro'),
        ' xyz:=', _format_xacro_vector_arg(base_link_to_livox_xyz_values),
        ' rpy:=', _format_xacro_vector_arg(base_link_to_livox_rpy_values),
        ' body_size:=', _format_xacro_vector_arg(vehicle_body_size),
        ' body_center_z:=', str(body_center_z)])

    segmentation_params_source = os.path.join(nav_bringup_dir, 'config', 'reality', 'segmentation_real.yaml')
    with open(segmentation_params_source, 'r', encoding='utf-8') as segmentation_file:
        segmentation_params = yaml.safe_load(segmentation_file)
    segmentation_params['ground_segmentation']['ros__parameters']['sensor_height'] = base_link_to_livox_xyz_values[2]
    segmentation_params = _write_temp_yaml(segmentation_params, 'segmentation_real_')

    fast_lio_sam_params_source = os.path.join(
        nav_bringup_dir, 'config', 'reality', 'fast_lio_sam_gridmap_mid360_real.yaml')
    fast_lio_sam_params = _load_fast_lio_sam_params(fast_lio_sam_params_source)
    fastlio_rviz_cfg_dir = os.path.join(
        get_package_share_directory('fast_lio_sam'), 'rviz_cfg', 'grid_ros2.rviz')

    nav2_params_source = os.path.join(nav_bringup_dir, 'config', 'reality', 'nav2_params_real.yaml')
    with open(nav2_params_source, 'r', encoding='utf-8') as nav2_file:
        nav2_params = yaml.safe_load(nav2_file)
    nav2_params['controller_server']['ros__parameters']['FollowPath']['footprint_model'] = {
        'type': 'polygon',
        'vertices': nav2_footprint
    }
    nav2_params['local_costmap']['local_costmap']['ros__parameters']['footprint'] = nav2_footprint
    nav2_params['global_costmap']['global_costmap']['ros__parameters']['footprint'] = nav2_footprint
    nav2_params_file_dir = _write_temp_yaml(nav2_params, 'nav2_params_real_')

    livox_ros2_params = [
        {'xfer_format': 4},
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': 20.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': os.path.join(nav_bringup_dir, 'config', 'reality', 'MID360_config.json')},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation clock if true')
    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz', default_value='False', description='Visualize FAST-LIO-SAM-G if true')
    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz', default_value='True', description='Visualize Nav2 if true')
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='test_map', description='Compatibility map name placeholder')
    declare_mode_cmd = DeclareLaunchArgument(
        'mode', default_value='fastlio_nav', choices=['mapping', 'fastlio_nav'],
        description='mapping: FAST-LIO-SAM-G only; fastlio_nav: FAST-LIO-SAM-G + Nav2')
    declare_lio_cmd = DeclareLaunchArgument(
        'lio', default_value='fastlio_sam', choices=['fastlio_sam'],
        description='Compatibility argument; only FAST-LIO-SAM-G is maintained')
    declare_launch_chassis_cmd = DeclareLaunchArgument(
        'launch_chassis', default_value='False', description='Launch AgileX Ranger chassis driver')
    declare_chassis_port_name_cmd = DeclareLaunchArgument(
        'chassis_port_name', default_value='can0', description='AgileX chassis CAN interface')
    declare_chassis_robot_model_cmd = DeclareLaunchArgument(
        'chassis_robot_model', default_value='ranger_mini_v3',
        description='AgileX chassis model')
    declare_chassis_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'chassis_cmd_vel_topic', default_value='/cmd_vel_chassis',
        description='Twist topic consumed by the chassis driver')
    declare_chassis_odom_topic_cmd = DeclareLaunchArgument(
        'chassis_odom_topic', default_value='/chassis/odom',
        description='Odometry topic published by the chassis driver')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        output='screen')

    start_livox_ros_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=start_fastlio)

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[('/imu/data_raw', '/livox/imu')],
        condition=start_fastlio)

    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params],
        condition=start_fastlio)

    start_fast_lio_sam_node = Node(
        package='fast_lio_sam',
        executable='fastlio_mapping',
        name='fast_lio_gridmap',
        parameters=[
            {'sam_enable': True},
            {'feature_extract_enable': False},
            {'point_filter_num': 1},
            {'max_iteration': 3},
            {'filter_size_surf': 0.1},
            {'filter_size_map': 0.1},
            {'cube_side_length': 1000.0},
            {'runtime_pos_log_enable': False},
            fast_lio_sam_params,
            {
                'use_sim_time': use_sim_time,
                'common/imu_topic': '/livox/imu',
                'frontend_scan/topic': '/scan',
            },
        ],
        output='screen',
        condition=start_fastlio)

    start_cloud_to_occupancy_node = Node(
        package='fast_lio_sam',
        executable='cloud_to_occupancy',
        name='cloud_to_occupancy',
        parameters=[fast_lio_sam_params, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=start_fastlio)

    start_lio_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', fastlio_rviz_cfg_dir],
        condition=IfCondition(use_lio_rviz))

    bringup_fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'spin_speed': 0.0}])

    start_staged_navigate_to_pose_node = Node(
        package='nav_bringup',
        executable='staged_navigate_to_pose.py',
        name='staged_navigate_to_pose',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'public_action_name': '/navigate_to_pose',
            'raw_action_name': '/navigate_to_pose_raw',
            'cmd_vel_topic': '/cmd_vel_nav',
            'global_frame': 'map',
            'base_frame': 'base_link',
            'start_yaw_enabled': True,
            'start_yaw_tolerance': 0.20,
            'start_yaw_timeout': 8.0,
            'start_yaw_kp': 1.6,
            'start_yaw_min_vel': 0.20,
            'start_yaw_max_vel': 0.90,
            'start_yaw_rate': 20.0,
            'final_yaw_enabled': True,
            'final_yaw_tolerance': 0.25,
            'final_yaw_timeout': 8.0,
            'final_yaw_kp': 1.6,
            'final_yaw_min_vel': 0.20,
            'final_yaw_max_vel': 0.90,
            'final_yaw_rate': 20.0,
        }],
        condition=LaunchConfigurationEquals('mode', 'fastlio_nav'))

    start_chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ranger_bringup'), 'launch', 'ranger.launch.py'])),
        condition=IfCondition(launch_chassis),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'port_name': chassis_port_name,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'odom_topic_name': chassis_odom_topic,
            'cmd_vel_topic_name': chassis_cmd_vel_topic,
            'publish_odom_tf': 'false',
            'robot_model': chassis_robot_model}.items())

    start_navigation2 = TimerAction(
        period=3.0,
        condition=LaunchConfigurationEquals('mode', 'fastlio_nav'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_navigation.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file_dir,
                    'nav_rviz': use_nav_rviz}.items())
        ])

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lio_cmd)
    ld.add_action(declare_launch_chassis_cmd)
    ld.add_action(declare_chassis_port_name_cmd)
    ld.add_action(declare_chassis_robot_model_cmd)
    ld.add_action(declare_chassis_cmd_vel_topic_cmd)
    ld.add_action(declare_chassis_odom_topic_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_livox_ros_driver2_node)
    ld.add_action(bringup_imu_complementary_filter_node)
    ld.add_action(bringup_linefit_ground_segmentation_node)
    ld.add_action(start_fast_lio_sam_node)
    ld.add_action(start_cloud_to_occupancy_node)
    ld.add_action(start_lio_rviz_node)
    ld.add_action(bringup_fake_vel_transform_node)
    ld.add_action(start_staged_navigate_to_pose_node)
    ld.add_action(start_chassis)
    ld.add_action(start_navigation2)

    return ld
