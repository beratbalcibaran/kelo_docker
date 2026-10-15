import math
import os
import time

import xacro
from launch.actions import (EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from kelo_bringup.common import (SOURCES, config_path, data_dir, load_hardware, nav2_params,
                                 odom_plan, safety_params, sensors, share, write_params)

MODES = ('mapping', 'navigation', 'sensors')
NAV_LIFECYCLE = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                 'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']


def _xyz(m):
    return f"{m['x']} {m['y']} {m['z']}"


def _rpy(m):
    return f"{m['roll']} {m['pitch']} {m['yaw']}"


def sensor_topics(hw, sens):
    tp = hw['topics']
    out = [tp['scan'], tp['odom']]
    if sens['camera']:
        out += [tp['camera_depth'], tp['camera_info']]
    if sens['marvelmind']:
        out.append(tp['marvelmind_raw'])
    return out


def parse_initial_pose(text):
    text = (text or '').strip()
    if not text:
        return None
    parts = [float(v) for v in text.replace(',', ' ').split()]
    if len(parts) != 3:
        raise ValueError(f"initial_pose must be 'x y yaw_deg': {text!r}")
    return parts[0], parts[1], math.radians(parts[2])


def sim_params(plan, sens, opts):
    sim = {'publish_odom_tf': plan['tf_owner'] == 'driver',
           'camera_enabled': sens['camera'], 'marvelmind_enabled': sens['marvelmind'],
           'seed': int(opts.get('sim_seed', 0) or 0)}
    for key in ('x0', 'y0', 'yaw0'):
        val = str(opts.get(f'sim_{key}', '') or '').strip()
        if val:
            sim[key] = float(val)
    if opts.get('sim_odom_model'):
        sim['odom_model'] = opts['sim_odom_model']
    return sim


def build(opts, hw=None):
    hw = hw or load_hardware()
    source, mode = opts['source'], opts['mode']
    if source not in SOURCES:
        raise ValueError(f'source must be one of {SOURCES}: {source}')
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}: {mode}')
    sens = sensors(hw)
    plan = odom_plan(hw, opts['odom_source'])
    fr, tp = hw['frames'], hw['topics']
    replay = source == 'replay'
    acts = [LogInfo(msg=f"[kelo] source={source} mode={mode} odom_source={opts['odom_source']} "
                        f"odom TF owner={plan['tf_owner']} sensors={sens}")]
    if replay:
        if not opts.get('bag'):
            raise ValueError('source:=replay requires bag:=<bag directory>')
        acts.append(SetParameter(name='use_sim_time', value=True))

    safety = write_params(safety_params(hw), 'kelo_safety_')

    urdf = xacro.process_file(os.path.join(share(), 'urdf', 'kelo.urdf.xacro'), mappings={
        'base_frame': fr['base'], 'laser_frame': fr['laser'], 'camera_frame': fr['camera'],
        'camera_optical_frame': fr['camera_optical'], 'hedgehog_frame': fr['hedgehog'],
        'body_l': str(hw['body']['length']), 'body_w': str(hw['body']['width']),
        'body_h': str(hw['body']['height']),
        'laser_xyz': _xyz(hw['mounts']['laser']), 'laser_rpy': _rpy(hw['mounts']['laser']),
        'camera_xyz': _xyz(hw['mounts']['camera']), 'camera_rpy': _rpy(hw['mounts']['camera']),
        'hedgehog_xyz': _xyz(hw['mounts']['hedgehog']), 'hedgehog_rpy': _rpy(hw['mounts']['hedgehog']),
    }).toxml()
    acts.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': urdf}], output='screen'))

    if source == 'hardware':
        acts += _hardware_drivers(hw, sens)
    elif source == 'sim':
        sim = sim_params(plan, sens, opts)
        acts.append(Node(package='kelo_bringup', executable='sim_robot', name='sim_robot',
                         output='screen', parameters=[sim]))
    else:
        topics = sensor_topics(hw, sens) + ['/sim/ground_truth']
        play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', opts['bag'], '--clock', '100',
                 '--rate', str(opts.get('bag_rate', '1.0')), '--topics', *topics],
            output='screen', name='bag_play')
        acts.append(TimerAction(period=float(opts.get('bag_delay', 5.0)), actions=[play]))
        acts.append(RegisterEventHandler(OnProcessExit(
            target_action=play, on_exit=[LogInfo(msg='[kelo] REPLAY_FINISHED: bag playback complete')])))

    oc = hw['odom_correction']
    if plan['tf_owner'] == 'odom_filter':
        acts.append(Node(package='kelo_bringup', executable='odom_filter', name='odom_filter',
                         output='screen', parameters=[{
                             'input_topic': plan['raw_topic'], 'output_topic': plan['topic'],
                             'odom_frame': fr['odom'], 'base_frame': fr['base'],
                             'yaw_k_wz': [float(v) for v in oc['yaw_k']['wz']],
                             'yaw_k_k': [float(v) for v in oc['yaw_k']['k']],
                             'lin_err_v': [float(v) for v in oc['linear_error']['v']],
                             'lin_err_e': [float(v) for v in oc['linear_error']['e']],
                             'max_age_s': float(oc['max_age_s'])}]))
    elif plan['tf_owner'] == 'driver' and replay:
        acts.append(Node(package='kelo_bringup', executable='odom_filter', name='odom_tf_replay',
                         output='screen', parameters=[{
                             'input_topic': plan['raw_topic'], 'output_topic': '/odom_tf_replay',
                             'odom_frame': fr['odom'], 'base_frame': fr['base'],
                             'max_age_s': float(oc['max_age_s'])}]))
    elif plan['tf_owner'] == 'rf2o':
        acts.append(Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
                         output='screen', parameters=[{
                             'laser_scan_topic': tp['scan'], 'odom_topic': plan['topic'],
                             'publish_tf': True, 'base_frame_id': fr['base'],
                             'odom_frame_id': fr['odom'], 'init_pose_from_topic': '',
                             'freq': 10.0}]))

    acts.append(Node(package='kelo_bringup', executable='cmd_vel_gate', name='cmd_vel_gate',
                     output='screen', respawn=True, respawn_delay=1.0,
                     parameters=[safety, {'output_type': hw['robot']['cmd_vel_type'],
                                          'output_topic': tp['cmd_vel']}]))
    acts.append(Node(package='nav2_collision_monitor', executable='collision_monitor',
                     name='collision_monitor_teleop', output='screen', parameters=[safety]))
    acts.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_teleop', output='screen',
                     parameters=[{'autostart': True, 'node_names': ['collision_monitor_teleop']}]))
    if sens['camera']:
        acts.append(Node(package='kelo_bringup', executable='cloud_filter', name='cloud_filter',
                         output='screen', parameters=[safety, {
                             'depth_topic': tp['camera_depth'], 'info_topic': tp['camera_info'],
                             'output_topic': tp['camera_points'], 'clearing_topic': tp['camera_clearing'],
                             'base_frame': fr['base']}]))
    acts.append(Node(package='kelo_bringup', executable='status_node', name='kelo_status',
                     output='screen', parameters=[safety, {
                         'odom_topic': plan['topic'], 'camera_info_topic': tp['camera_info'],
                         'camera_enabled': sens['camera'], 'marvelmind_enabled': sens['marvelmind']}]))
    if sens['marvelmind']:
        acts.append(Node(package='kelo_bringup', executable='marvelmind_bridge', name='marvelmind_bridge',
                         output='screen', parameters=[{
                             'raw_topic': tp['marvelmind_raw'], 'pose_topic': tp['marvelmind_pose'],
                             'map_frame': fr['map'], 'mm_frame': fr['marvelmind_map']}]))

    if mode == 'mapping':
        acts += _mapping(hw, sens, opts, source)
    elif mode == 'navigation':
        acts += _navigation(hw, sens, plan, opts, safety, replay)

    if str(opts.get('use_rviz', 'false')).lower() == 'true':
        acts.append(Node(package='rviz2', executable='rviz2', output='screen', respawn=True,
                         respawn_delay=3.0,
                         arguments=['-d', os.path.join(share(), 'rviz', 'kelo.rviz')]))
    return acts


def _hardware_drivers(hw, sens):
    sr, fr, tp = hw['serial'], hw['frames'], hw['topics']
    acts = [Node(
        package='urg_node', executable='urg_node_driver', name='urg_node', output='screen',
        parameters=[{'serial_port': sr['lidar_port'], 'serial_baud': sr['lidar_baud'],
                     'laser_frame_id': fr['laser'],
                     'angle_min': hw['lidar']['angle_min'], 'angle_max': hw['lidar']['angle_max'],
                     'publish_intensity': False, 'publish_multiecho': False, 'error_limit': 4}],
        remappings=[('scan', tp['scan'])], respawn=True, respawn_delay=3.0)]
    if sens['camera']:
        from ament_index_python.packages import get_package_share_directory
        rs = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        acts.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(rs), launch_arguments={
            'camera_name': 'camera', 'camera_namespace': 'camera',
            'enable_color': 'false', 'enable_depth': 'true', 'enable_infra1': 'false',
            'enable_infra2': 'false', 'enable_gyro': 'false', 'enable_accel': 'false',
            'depth_module.depth_profile': hw['camera']['depth_profile'],
            'pointcloud.enable': 'false',
            'publish_tf': 'false',
            'initial_reset': 'true'}.items()))
    if sens['marvelmind']:
        acts.append(Node(
            package='marvelmind_ros2', executable='marvelmind_ros2', name='marvelmind_ros2',
            output='screen', respawn=True, respawn_delay=3.0,
            parameters=[{'marvelmind_tty_filename': sr['marvelmind_port'],
                         'marvelmind_tty_baudrate': sr['marvelmind_baud'],
                         'marvelmind_publish_rate_in_hz': 20}]))
    return acts


def _mapping(hw, sens, opts, source):
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node',
                         name='slam_toolbox', namespace='', output='screen',
                         parameters=[config_path('slam_toolbox.yaml')])
    acts = [slam,
            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                        transition_id=Transition.TRANSITION_CONFIGURE)),
            RegisterEventHandler(OnStateTransition(
                target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE))]))]
    if sens['marvelmind']:
        acts.append(Node(package='kelo_bringup', executable='mm_recorder', name='mm_recorder',
                         output='screen'))
    record = str(opts.get('record', 'auto')).lower()
    if record == 'true' or (record == 'auto' and source != 'replay'):
        out = os.path.join(data_dir(), 'bags', time.strftime(f'map_{source}_%Y%m%d_%H%M%S'))
        topics = sensor_topics(hw, sens) + ['/tf', '/tf_static']
        if source == 'sim':
            topics.append('/sim/ground_truth')
        acts.append(LogInfo(msg=f'[kelo] mapping recording: {out}'))
        acts.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap',
                                        '--storage-preset-profile', 'zstd_fast', '-o', out,
                                        '--max-bag-duration', '600', *topics],
                                   output='screen', name='bag_record'))
    return acts


def _navigation(hw, sens, plan, opts, safety, replay):
    map_yaml = opts['map']
    params = write_params(nav2_params(hw, plan['topic']), 'kelo_nav2_')
    amcl_extra = {}
    pose = parse_initial_pose(opts.get('initial_pose'))
    if pose is not None:
        amcl_extra = {'set_initial_pose': True, 'always_reset_initial_pose': False,
                      'initial_pose': {'x': pose[0], 'y': pose[1], 'z': 0.0, 'yaw': pose[2]}}
    acts = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'yaml_filename': map_yaml}], respawn=True, respawn_delay=2.0),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, amcl_extra], respawn=True, respawn_delay=2.0),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl'], 'bond_timeout': 10.0}]),
    ]
    if sens['marvelmind']:
        acts.append(Node(package='kelo_bringup', executable='mm_monitor', name='mm_monitor',
                         output='screen', parameters=[safety]))

    def node(pkg, exe, name, remap=None):
        return Node(package=pkg, executable=exe, name=name, output='screen',
                    parameters=[params], remappings=remap or [])

    nav_nodes = [
        node('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_smoother', 'smoother_server', 'smoother_server'),
        node('nav2_planner', 'planner_server', 'planner_server'),
        node('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': NAV_LIFECYCLE, 'bond_timeout': 0.0}]),
    ]
    cmd = ['ros2', 'run', 'kelo_bringup', 'wait_for_tf', str(opts.get('wait_tf_s', '600')),
           hw['frames']['map'], hw['frames']['base']]
    if replay:
        cmd += ['--ros-args', '-p', 'use_sim_time:=true']
    wait = ExecuteProcess(cmd=cmd, output='screen', name='wait_for_tf')
    acts += [wait, RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=nav_nodes))]
    return acts
