import copy
import math
from types import SimpleNamespace as NS

import yaml

from kelo_bringup.common import config_path
from kelo_bringup.robot_check import FAIL, WARN, Stream, evaluate


def hw(**robot):
    with open(config_path('hardware.yaml'), encoding='utf-8') as f:
        h = yaml.safe_load(f)
    h['robot'].update(robot)
    return h


def stream(hz, frame, child=None, offset=0.02, msg=None, seconds=3.0):
    st = Stream()
    n = int(hz * seconds) + 1
    for i in range(n):
        t = 1000.0 + i / hz
        st.add(t, t - offset, frame, child, msg)
    return st


def scan_msg(valid=0.9):
    n = 683
    ranges = [2.0 if i < valid * n else float('inf') for i in range(n)]
    return NS(ranges=ranges, range_min=0.02, range_max=5.6, angle_min=-1.65, angle_max=1.65)


def odom_msg(vx=0.0):
    return NS(twist=NS(twist=NS(linear=NS(x=vx, y=0.0))))


def good_obs(h):
    return {'streams': {'odom': stream(50, 'odom', 'base_link', msg=odom_msg()),
                        'scan': stream(10, 'laser', msg=scan_msg()),
                        'depth': stream(15, 'camera_depth_optical_frame'),
                        'camera_info': stream(15, 'camera_depth_optical_frame'),
                        'marvelmind': Stream()},
            'cmd_pubs': [], 'cmd_subs': [('kelo_driver', 'geometry_msgs/msg/Twist')],
            'tf_pubs': [], 'odom_tf_seen': False, 'map': '/data/maps/current.yaml', 'camera_map': None,
            'domain': '0'}


def levels(results, word):
    return [lv for lv, text in results if word in text]


def test_a_good_robot_passes():
    h = hw()
    res = evaluate(h, good_obs(h))
    assert not [r for r in res if r[0] == FAIL], res


def test_driver_odom_tf_with_corrected_mode_fails():
    h = hw(odom_mode='corrected')
    obs = good_obs(h)
    obs.update(odom_tf_seen=True, tf_pubs=['kelo_driver'])
    assert FAIL in levels(evaluate(h, obs), 'TF is published by kelo_driver')
    h = hw(odom_mode='driver')
    assert FAIL not in [lv for lv, _ in evaluate(h, obs)]
    obs.update(odom_tf_seen=False, tf_pubs=[])
    assert FAIL in levels(evaluate(h, obs), "odom_mode is 'driver' but nobody")


def test_cmd_vel_type_and_extra_publishers():
    h = hw(cmd_vel_type='Twist')
    obs = good_obs(h)
    obs['cmd_subs'] = [('kelo_driver', 'geometry_msgs/msg/TwistStamped')]
    obs['cmd_pubs'] = ['cmd_vel_gate', 'teleop_twist_keyboard']
    res = evaluate(h, obs)
    assert FAIL in levels(res, 'TwistStamped')
    assert FAIL in levels(res, 'teleop_twist_keyboard')
    obs['cmd_subs'] = [('surround_view', 'geometry_msgs/msg/Twist')]   # a view is not the driver
    assert FAIL in levels(evaluate(h, obs), 'nobody subscribes')


def test_slow_odom_wrong_frames_and_clock_offset():
    h = hw()
    obs = good_obs(h)
    obs['streams']['odom'] = stream(5, 'odom', 'base_footprint', offset=0.8, msg=odom_msg())
    res = evaluate(h, obs)
    assert FAIL in levels(res, 'Hz (min 10)')
    assert FAIL in levels(res, 'base_footprint')
    assert FAIL in levels(res, 'chrony')


def test_missing_sensors_and_moving_robot():
    h = hw()
    obs = good_obs(h)
    obs['streams']['scan'] = Stream()
    obs['streams']['depth'] = Stream()
    obs['streams']['odom'] = stream(50, 'odom', 'base_link', msg=odom_msg(0.2))
    res = evaluate(h, obs)
    assert FAIL in levels(res, 'LiDAR /scan: no messages')
    assert FAIL in levels(res, 'depth')
    assert WARN in levels(res, 'reports motion')


def test_marvelmind_only_when_enabled_and_no_map_warns():
    h = hw()
    obs = good_obs(h)
    obs['map'] = None
    res = evaluate(h, obs)
    assert not levels(res, 'Marvelmind')
    assert WARN in levels(res, 'no map yet')
    h2 = copy.deepcopy(h)
    h2['sensors']['marvelmind'] = True
    assert FAIL in levels(evaluate(h2, obs), 'Marvelmind')


def test_stream_rate_and_offset():
    st = stream(20, 'x', offset=0.05)
    assert math.isclose(st.hz(), 20.0, rel_tol=1e-6)
    assert math.isclose(st.offset(), 0.05, abs_tol=1e-9)
    assert Stream().hz() == 0.0 and Stream().offset() is None
