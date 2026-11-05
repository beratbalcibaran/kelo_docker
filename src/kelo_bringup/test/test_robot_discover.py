import yaml

from kelo_bringup.common import config_path
from kelo_bringup.robot_discover import DEFAULT_DOMAINS, FAIL, WARN, parse_domains, propose

TW = 'geometry_msgs/msg/Twist'
TWS = 'geometry_msgs/msg/TwistStamped'
OD = 'nav_msgs/msg/Odometry'


def hw(**robot):
    with open(config_path('hardware.yaml'), encoding='utf-8') as f:
        h = yaml.safe_load(f)
    h['robot'].update(robot)
    return h


def obs(cmd='/cmd_vel', cmd_type=TW, odom='/odom', tf=False, domain=0, nodes=('kelo_driver',), **kw):
    o = {'domain': domain, 'current_domain': 0, 'nodes': list(nodes),
         'topics': {cmd: {'pubs': [], 'subs': [['kelo_driver', cmd_type]]},
                    odom: {'pubs': [['kelo_driver', OD]], 'subs': []},
                    '/rosout': {'pubs': [['kelo_driver', 'rcl_interfaces/msg/Log']], 'subs': []}},
         'odom': {odom: {'hz': 50.0, 'frame': 'odom', 'child': 'base_link', 'offset': 0.01}},
         'tf_pairs': [['odom', 'base_link']] if tf else [],
         'devices': {'lidar': True, 'camera': True, 'marvelmind': None}}
    o.update(kw)
    return o


def settings(res):
    return {'.'.join(p): v for p, v in res['settings']}


def test_matching_robot_needs_no_settings():
    res = propose(hw(odom_mode='corrected'), obs())
    assert res['found'] and settings(res) == {}
    assert not [n for n in res['notes'] if n[0] in (FAIL, WARN)], res['notes']


def test_other_topic_type_domain_and_driver_tf():
    res = propose(hw(odom_mode='corrected'), obs(cmd='/base/cmd_vel', cmd_type=TWS, odom='/base/odom', tf=True,
                                                 domain=7))
    assert res['found']
    assert settings(res) == {'env.ROS_DOMAIN_ID': 7, 'topics.cmd_vel': '/base/cmd_vel',
                             'robot.cmd_vel_type': 'TwistStamped', 'topics.odom': '/base/odom',
                             'robot.odom_mode': 'driver'}


def test_driver_mode_without_driver_tf_goes_corrected():
    assert settings(propose(hw(odom_mode='driver'), obs())) == {'robot.odom_mode': 'corrected'}


def test_running_stack_and_no_driver_fail():
    res = propose(hw(), obs(nodes=('kelo_driver', 'cmd_vel_gate')))
    assert not res['found'] and 'stack is running' in res['notes'][0][1]
    res = propose(hw(), obs(nodes=('surround_view', '_ros2cli_123')))
    assert not res['found'] and 'no robot driver' in res['notes'][0][1]


def test_our_watchers_are_not_the_driver_and_extra_publishers_warn():
    o = obs()
    o['topics']['/cmd_vel_teleop'] = {'pubs': [], 'subs': [['collision_monitor_teleop', TW]]}
    o['topics']['/cmd_vel']['pubs'] = [['teleop_twist_keyboard', TW]]
    res = propose(hw(), o)
    assert settings(res) == {}
    assert any(lv == WARN and 'teleop_twist_keyboard' in t for lv, t in res['notes'])


def test_no_odometry_fails_and_frame_mismatch_warns():
    o = obs()
    o['topics']['/odom']['pubs'] = []
    res = propose(hw(), o)
    assert not res['found'] and any(lv == FAIL and 'no odometry' in t for lv, t in res['notes'])
    o = obs()
    o['odom']['/odom'].update(frame='odom', child='base_footprint')
    o['tf_pairs'] = [['odom', 'base_footprint'], ['base_footprint', 'laser']]
    res = propose(hw(odom_mode='driver'), o)
    assert settings(res) == {}
    texts = ' '.join(t for lv, t in res['notes'] if lv == WARN)
    assert 'base_footprint' in texts and 'laser' in texts


def test_missing_devices_warn():
    res = propose(hw(), obs(devices={'lidar': False, 'camera': True, 'marvelmind': None}))
    assert any(lv == WARN and 'LiDAR not found' in t for lv, t in res['notes'])


def test_parse_domains():
    assert parse_domains('0-3,7,2') == [0, 1, 2, 3, 7]
    assert parse_domains('0-10,42') == DEFAULT_DOMAINS


def test_driver_that_fixes_the_rotation_itself_gets_relay():
    res = propose(hw(odom_mode='corrected'), obs(driver_params={'fix_odom_rotation': True, 'publish_tf': False}))
    assert settings(res) == {'robot.odom_mode': 'relay'}
    assert any('relay' in t for _, t in res['notes'])
    res = propose(hw(odom_mode='relay'), obs(tf=True, driver_params={'fix_odom_rotation': True}))
    assert settings(res) == {'robot.odom_mode': 'driver'}
    res = propose(hw(odom_mode='relay'), obs(driver_params={}))
    assert settings(res) == {'robot.odom_mode': 'corrected'}
    assert any('did not report fix_odom_rotation' in t for _, t in res['notes'])
