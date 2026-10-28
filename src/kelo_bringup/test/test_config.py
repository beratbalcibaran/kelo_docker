import ast
import os

import yaml

CFG = os.path.join(os.path.dirname(__file__), '..', 'config')


def load(name):
    with open(os.path.join(CFG, name), encoding='utf-8') as f:
        return yaml.safe_load(f)


def poly(text):
    return ast.literal_eval(text)


def test_footprints_identical():
    nav = load('nav2_params.yaml')
    saf = load('safety.yaml')
    ref = poly(nav['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    # the global costmap uses the inscribed radius (path validity like the planner), see nav2_params.yaml
    gc = nav['global_costmap']['global_costmap']['ros__parameters']
    assert 'footprint' not in gc
    assert gc['robot_radius'] == min(max(abs(x) for x, _ in ref), max(abs(y) for _, y in ref))
    assert poly(nav['collision_monitor']['ros__parameters']['FootprintApproach']['points']) == ref
    assert poly(saf['collision_monitor_teleop']['ros__parameters']['FootprintApproach']['points']) == ref


def test_speed_limits_chain_consistent():
    nav = load('nav2_params.yaml')
    gate = load('safety.yaml')['cmd_vel_gate']['ros__parameters']
    mppi = nav['controller_server']['ros__parameters']['FollowPath']
    smoother = nav['velocity_smoother']['ros__parameters']
    assert gate['max_vx'] >= smoother['max_velocity'][0] >= mppi['vx_max']
    assert gate['max_vy'] >= smoother['max_velocity'][1] >= mppi['vy_max']
    assert gate['max_wz'] >= smoother['max_velocity'][2] >= mppi['wz_max']
    assert mppi['motion_model'] == 'DiffDrive'
    assert nav['amcl']['ros__parameters']['robot_model_type'] == 'nav2_amcl::OmniMotionModel'


def test_topic_chain():
    nav = load('nav2_params.yaml')['collision_monitor']['ros__parameters']
    gate = load('safety.yaml')['cmd_vel_gate']['ros__parameters']
    assert nav['cmd_vel_out_topic'] == gate['nav_topic']
    tel = load('safety.yaml')['collision_monitor_teleop']['ros__parameters']
    assert tel['cmd_vel_out_topic'] == gate['teleop_topic']


def test_camera_layer_respects_depth_limits():
    nav = load('nav2_params.yaml')
    filt = load('safety.yaml')['cloud_filter']['ros__parameters']
    local = nav['local_costmap']['local_costmap']['ros__parameters']['camera_layer']
    glob = nav['global_costmap']['global_costmap']['ros__parameters']['camera_layer']
    assert local == glob
    assert local['camera']['obstacle_min_range'] == filt['min_range']
    assert local['camera']['min_obstacle_height'] > 0.0


def test_hardware_has_all_frames_and_mounts():
    hw = load('hardware.yaml')
    for key in ('map', 'odom', 'base', 'laser', 'camera', 'hedgehog', 'marvelmind_map'):
        assert key in hw['frames']
    for key in ('laser', 'camera', 'hedgehog'):
        assert set(hw['mounts'][key]) == {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}
    assert hw['robot']['cmd_vel_type'] in ('Twist', 'TwistStamped')
    assert hw['robot']['odom_mode'] in ('driver', 'corrected')


import math

from kelo_bringup.logic import convex_contains


def _cms():
    return (load('nav2_params.yaml')['collision_monitor']['ros__parameters'],
            load('safety.yaml')['collision_monitor_teleop']['ros__parameters'])


def _strip(zone):
    return {k: v for k, v in zone.items() if k != 'polygon_pub_topic'}


def _stop_sub(cm, name):
    return poly(cm['StopZone'][name]['points'])


def test_cm_zones_identical_in_both_monitors():
    nav, tel = _cms()
    assert nav['polygons'] == tel['polygons'] == ['FootprintApproach', 'SlowdownZone', 'StopZone']
    for name in ('SlowdownZone', 'StopZone'):
        assert _strip(nav[name]) == _strip(tel[name])
    assert nav['StopZone']['action_type'] == 'stop'
    assert nav['StopZone']['type'] == 'velocity_polygon' and nav['StopZone']['holonomic']
    assert nav['SlowdownZone']['action_type'] == 'slowdown'
    assert 0.0 < nav['SlowdownZone']['slowdown_ratio'] < 1.0


def test_zones_nested_around_footprint():
    nav, _ = _cms()
    fp = poly(load('nav2_params.yaml')['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    for sub in nav['StopZone']['velocity_polygons']:
        assert convex_contains(_stop_sub(nav, sub), fp), sub
    slow = poly(nav['SlowdownZone']['points'])
    assert convex_contains(slow, _stop_sub(nav, 'forward'))
    r = max(math.hypot(x, y) for x, y in fp)
    rot = _stop_sub(nav, 'rotation')
    inscribed = min(math.hypot(x, y) for x, y in rot) * math.cos(math.pi / len(rot))
    assert inscribed >= r + 0.02


def test_stop_zone_covers_all_directions():
    sz = _cms()[0]['StopZone']
    subs = sz['velocity_polygons']
    assert subs[0] == 'rotation' and subs[-1] == 'fallback'
    assert sz['rotation']['linear_min'] == 0.0 and sz['rotation']['linear_max'] == 0.05

    def matches(name, v, ang):
        p = sz[name]
        if not (p['linear_min'] <= v <= p['linear_max']):
            return False
        a, b = p['direction_start_angle'], p['direction_end_angle']
        return a <= ang <= b if a <= b else (ang >= a or ang <= b)
    for deg in range(-180, 181, 5):
        ang = math.radians(deg)
        first = next(n for n in subs if matches(n, 0.2, ang))
        assert first != 'fallback', deg
    fb = _stop_sub(_cms()[0], 'fallback')
    for sub in subs[:-1]:
        assert convex_contains(fb, _stop_sub(_cms()[0], sub)), sub


def test_cm_sources_lidar_and_camera_fail_safe():
    filt = load('safety.yaml')['cloud_filter']['ros__parameters']
    for cm in _cms():
        assert cm['observation_sources'] == ['scan', 'camera']
        assert cm['camera']['type'] == 'pointcloud'
        assert cm['camera']['min_height'] == filt['min_height'] > 0.0
        assert cm['camera']['max_height'] == filt['max_height']
        assert 0.0 < cm['source_timeout'] <= 1.0


def test_camera_blind_zone_rule():
    hw = load('hardware.yaml')
    cam, hfov = hw['mounts']['camera'], math.radians(hw['camera']['hfov_deg'])
    filt = load('safety.yaml')['cloud_filter']['ros__parameters']
    for cm in _cms():
        stop = _stop_sub(cm, 'forward')
        front = max(p[0] for p in stop)
        half_w = max(abs(p[1]) for p in stop if p[0] == front)
        for y in (-half_w, 0.0, half_w):
            dx, dy = front - cam['x'], y - cam['y']
            rng = math.hypot(dx, dy, 0.0)
            assert rng >= filt['min_range'], (front, y)
            assert rng <= filt['max_range']
            assert abs(math.atan2(dy, dx)) <= hfov / 2.0, (front, y)
        slow = poly(cm['SlowdownZone']['points'])
        assert max(p[0] for p in slow) - cam['x'] <= filt['max_range']


def test_cloud_filter_limits():
    filt = load('safety.yaml')['cloud_filter']['ros__parameters']
    assert filt['min_range'] >= 0.28
    assert filt['min_height'] >= 0.05
    cam = load('nav2_params.yaml')['local_costmap']['local_costmap']['ros__parameters']['camera_layer']['camera']
    assert cam['min_obstacle_height'] == filt['min_height']
    assert cam['max_obstacle_height'] == filt['max_height']


def test_camera_layer_marks_filtered_clears_with_floor():
    cl = load('nav2_params.yaml')['local_costmap']['local_costmap']['ros__parameters']['camera_layer']
    assert cl['observation_sources'].split() == ['camera', 'camera_clearing']
    assert cl['camera']['marking'] and not cl['camera']['clearing']
    assert cl['camera_clearing']['clearing'] and not cl['camera_clearing']['marking']
    assert cl['camera_clearing']['min_obstacle_height'] < 0.0
