import copy

import pytest
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node, SetParameter

from kelo_bringup.common import load_hardware, nav2_params, odom_plan, sensors
from kelo_bringup.stack import build, parse_initial_pose, sim_params

SOURCE_LAYER = {('kelo_bringup', 'sim_robot'), ('urg_node', 'urg_node_driver'),
                ('marvelmind_ros2', 'marvelmind_ros2')}


def opts(**kw):
    base = {'source': 'sim', 'mode': 'navigation', 'odom_source': 'robot', 'map': '/tmp/x.yaml',
            'bag': '/tmp/bag', 'record': 'false', 'use_rviz': 'false', 'initial_pose': ''}
    base.update(kw)
    return base


def walk(actions):
    for a in actions:
        if isinstance(a, Node):
            yield ('node', a.node_package, a.node_executable)
        elif isinstance(a, ExecuteProcess):
            yield ('proc', ' '.join(''.join(getattr(s, 'text', '?') for s in part) for part in a.cmd))
        elif isinstance(a, TimerAction):
            yield from walk(a.actions)
        elif isinstance(a, SetParameter):
            name = ''.join(getattr(x, 'text', '?') for x in a.name)
            yield ('param', name, a.value)


def all_sensors():
    hw = copy.deepcopy(load_hardware())
    hw['sensors'] = {k: True for k in hw['sensors']}
    return hw


def nodes(acts):
    return {(p, e) for kind, p, *rest in walk(acts) if kind == 'node' for e in rest}


def test_common_nodes_identical_across_sources():
    for mode in ('mapping', 'navigation', 'sensors'):
        common = [nodes(build(opts(source=s, mode=mode))) - SOURCE_LAYER
                  for s in ('sim', 'hardware', 'replay')]
        assert common[0] == common[1] == common[2], mode


def test_source_layers():
    sim = nodes(build(opts(source='sim')))
    hw = nodes(build(opts(source='hardware')))
    rp = list(walk(build(opts(source='replay'))))
    assert ('kelo_bringup', 'sim_robot') in sim and ('urg_node', 'urg_node_driver') not in sim
    assert ('urg_node', 'urg_node_driver') in hw and ('kelo_bringup', 'sim_robot') not in hw
    assert ('param', 'use_sim_time', True) in rp
    plays = [r[1] for r in rp if r[0] == 'proc' and 'bag play' in r[1]]
    assert plays and '--clock' in plays[0] and '/tf' not in plays[0].split()


def test_all_sensors_in_every_source():
    for s in ('sim', 'hardware', 'replay'):
        n = nodes(build(opts(source=s), hw=all_sensors()))
        assert ('kelo_bringup', 'cloud_filter') in n
        assert ('kelo_bringup', 'marvelmind_bridge') in n
        assert ('kelo_bringup', 'mm_monitor') in n


def test_camera_layer_kept_in_sim():
    hw = load_hardware()
    cfg = nav2_params(hw, '/odom_corrected')
    lc = cfg['local_costmap']['local_costmap']['ros__parameters']
    assert 'camera_layer' in lc['plugins']
    assert lc['camera_layer']['camera']['topic'] == hw['topics']['camera_points']
    assert lc['camera_layer']['camera_clearing']['topic'] == hw['topics']['camera_clearing']


def test_sensor_disabled_in_config_removes_everywhere():
    hw = copy.deepcopy(load_hardware())
    hw['sensors']['camera'] = False
    cfg = nav2_params(hw, '/odom')
    assert 'camera_layer' not in cfg['local_costmap']['local_costmap']['ros__parameters']['plugins']
    assert 'camera' not in cfg['collision_monitor']['ros__parameters']['observation_sources']
    for s in ('sim', 'hardware'):
        assert ('kelo_bringup', 'cloud_filter') not in nodes(build(opts(source=s), hw=hw))


def test_single_odom_tf_owner():
    hw = copy.deepcopy(load_hardware())
    for mode, odom_src, owner in (('corrected', 'robot', 'odom_filter'), ('driver', 'robot', 'driver'),
                                  ('corrected', 'lidar', 'rf2o'), ('driver', 'lidar', 'rf2o')):
        hw['robot']['odom_mode'] = mode
        assert odom_plan(hw, odom_src)['tf_owner'] == owner
        acts = build(opts(source='sim', odom_source=odom_src), hw=hw)
        n = nodes(acts)
        sim_tf = sim_params(odom_plan(hw, odom_src), sensors(hw), {})['publish_odom_tf']
        publishers = sum([sim_tf, ('kelo_bringup', 'odom_filter') in n,
                          ('rf2o_laser_odometry', 'rf2o_laser_odometry_node') in n])
        assert publishers == 1, (mode, odom_src)


def test_no_fake_flags():
    import inspect
    import kelo_bringup.stack as st
    src = inspect.getsource(st)
    for word in ('fake', 'use_lidar', 'use_camera', 'use_marvelmind'):
        assert word not in src


def test_mapping_records_bag_except_replay():
    def recs(source):
        return [r for r in walk(build(opts(source=source, mode='mapping', record='auto'), hw=all_sensors()))
                if r[0] == 'proc' and 'bag record' in r[1]]
    assert recs('sim') and recs('hardware') and not recs('replay')
    cmd = recs('hardware')[0][1]
    assert '/camera/camera/depth/image_rect_raw' in cmd and '/hedgehog_pos_ang' in cmd
    assert '/camera/points_filtered' not in cmd


def test_initial_pose_parse():
    assert parse_initial_pose('') is None
    x, y, yaw = parse_initial_pose('1 2 90')
    assert (x, y) == (1.0, 2.0) and yaw == pytest.approx(1.5707963, abs=1e-6)
    with pytest.raises(ValueError):
        parse_initial_pose('1 2')


def test_all_nav2_odom_consumers_use_planned_topic():
    hw = load_hardware()
    for odom_src in ('robot', 'lidar'):
        topic = odom_plan(hw, odom_src)['topic']
        cfg = nav2_params(hw, topic)
        for name in ('bt_navigator', 'velocity_smoother', 'controller_server'):
            assert cfg[name]['ros__parameters']['odom_topic'] == topic, (odom_src, name)


def test_localize_keeps_good_localization():
    import math
    from kelo_bringup.localize import already_localized
    good = (0.12, math.radians(6.8))
    assert already_localized('LOCALIZED', *good)
    assert not already_localized('LOCALIZED', *good, force=True)
    for state in ('FAILED', 'LOCALIZING', None, ''):
        assert not already_localized(state, *good)
    assert not already_localized('LOCALIZED', 0.5, math.radians(6.8))
    assert not already_localized('LOCALIZED', 0.12, math.radians(30))


def _params_of(acts, name):
    for a in acts:
        if isinstance(a, Node) and a.node_name == name:
            return a._Node__parameters
    return None


def _global_plugins(hw, camera_map):
    gc = nav2_params(hw, '/odom', camera_map=camera_map)['global_costmap']['global_costmap']['ros__parameters']
    return gc['plugins'], gc


def test_camera_layer_mapping_switch():
    hw = all_sensors()
    hw['mapping'] = {'camera_layer': True}
    assert ('kelo_bringup', 'camera_mapper') in nodes(build(opts(mode='mapping'), hw=hw))
    assert ('kelo_bringup', 'camera_mapper') not in nodes(build(opts(mode='navigation'), hw=hw))
    for off in ({'camera_layer': False}, None):
        h = copy.deepcopy(hw)
        h['mapping'] = off
        assert ('kelo_bringup', 'camera_mapper') not in nodes(build(opts(mode='mapping'), hw=h))
    h = copy.deepcopy(hw)
    h['sensors']['camera'] = False
    assert ('kelo_bringup', 'camera_mapper') not in nodes(build(opts(mode='mapping'), hw=h))


def test_camera_layer_navigation_only_with_file(tmp_path):
    hw = all_sensors()
    hw['mapping'] = {'camera_layer': True}
    lidar = tmp_path / 'room_20261020.yaml'
    lidar.write_text('image: room_20261020.pgm\n')
    link = tmp_path / 'current.yaml'
    link.symlink_to(lidar.name)
    servers = lambda h: sum(1 for k, p, *e in walk(build(opts(map=str(link)), hw=h))
                            if k == 'node' and p == 'nav2_map_server')
    assert servers(hw) == 1
    (tmp_path / 'room_20261020_camera.yaml').write_text('image: room_20261020_camera.pgm\n')
    assert servers(hw) == 2
    for key, val in (('mapping', {'camera_layer': False}), ('sensors', dict(hw['sensors'], camera=False))):
        h = copy.deepcopy(hw)
        h[key] = val
        assert servers(h) == 1, key


def test_camera_static_layer_only_adds_obstacles():
    hw = all_sensors()
    plain, _ = _global_plugins(hw, False)
    with_cam, gc = _global_plugins(hw, True)
    assert 'camera_static_layer' not in plain
    assert plain[0] == 'static_layer'
    assert with_cam == ['static_layer', 'camera_static_layer'] + plain[1:]
    layer = gc['camera_static_layer']
    assert layer['plugin'] == 'nav2_costmap_2d::StaticLayer'
    # StaticLayer reads use_maximum from the costmap (Nav2 Jazzy), not from the layer
    assert gc['use_maximum'] is True and 'use_maximum' not in layer
    assert layer['map_topic'] == hw['topics']['camera_map']
    for camera_map in (False, True):
        lc = nav2_params(hw, '/odom', camera_map=camera_map)['local_costmap']['local_costmap']['ros__parameters']
        assert ('camera_static_layer' in lc['plugins']) is camera_map
        if camera_map:
            assert lc['plugins'][0] == 'camera_static_layer'
            assert lc['camera_static_layer'] == layer


def test_localize_switches_every_map_layer_of_the_local_costmap():
    # localize turns these off during the spins; a map-fixed layer missing here would stop the spin
    from kelo_bringup.localize import LOCAL_MAP_LAYERS
    lc = nav2_params(all_sensors(), '/odom', camera_map=True)['local_costmap']['local_costmap']['ros__parameters']
    fixed = [p for p in lc['plugins'] if lc[p]['plugin'] == 'nav2_costmap_2d::StaticLayer']
    assert fixed and set(fixed) <= set(LOCAL_MAP_LAYERS)


def test_camera_costmap_layer_is_voxel():
    hw = all_sensors()
    cfg = nav2_params(hw, '/odom')
    for name in ('local_costmap', 'global_costmap'):
        layer = cfg[name][name]['ros__parameters']['camera_layer']
        assert layer['plugin'] == 'nav2_costmap_2d::VoxelLayer'
        assert layer['origin_z'] + layer['z_resolution'] * layer['z_voxels'] >= layer['camera']['max_obstacle_height']


def test_camera_layer_rebuild_mode():
    hw = all_sensors()
    hw['mapping'] = {'camera_layer': True}
    acts = build(opts(source='replay', mode='camera_layer', initial_pose='1 2 90'), hw=hw)
    n = nodes(acts)
    for need in (('kelo_bringup', 'camera_mapper'), ('kelo_bringup', 'cloud_filter'),
                 ('nav2_map_server', 'map_server'), ('nav2_amcl', 'amcl')):
        assert need in n, need
    for never in (('slam_toolbox', 'async_slam_toolbox_node'), ('nav2_controller', 'controller_server'),
                  ('nav2_planner', 'planner_server')):
        assert never not in n, never
    procs = [p for k, p, *_ in walk(acts) if k == 'proc']
    assert any(p.startswith('ros2 bag play') for p in procs)
    assert not any('bag record' in p for p in procs)
    with pytest.raises(ValueError):
        build(opts(source='sim', mode='camera_layer'), hw=hw)
    h = copy.deepcopy(hw)
    h['mapping'] = {'camera_layer': False}
    with pytest.raises(ValueError):
        build(opts(source='replay', mode='camera_layer'), hw=h)


def test_localization_params_in_node_sections():
    # dict parameters reach the nodes as a '/**' wildcard that loses against the node sections
    from kelo_bringup.stack import localization_params
    cfg = localization_params(nav2_params(all_sensors(), '/odom'), '/data/maps/x.yaml', (1.0, 2.0, 0.5))
    assert cfg['map_server']['ros__parameters']['yaml_filename'] == '/data/maps/x.yaml'
    amcl = cfg['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is True
    assert amcl['initial_pose'] == {'x': 1.0, 'y': 2.0, 'z': 0.0, 'yaw': 0.5}
    cfg = localization_params(nav2_params(all_sensors(), '/odom'), '/m.yaml', None)
    assert cfg['amcl']['ros__parameters']['set_initial_pose'] is False


def test_camera_layer_rebuild_localizer(tmp_path):
    from kelo_bringup.stack import rebuild_localizer
    hw = all_sensors()
    hw['mapping'] = {'camera_layer': True}
    m = tmp_path / 'room.yaml'
    m.write_text('image: room.pgm\n')
    run = lambda **kw: nodes(build(opts(source='replay', mode='camera_layer', map=str(m), **kw), hw=hw))
    assert rebuild_localizer(str(m))[0] == 'amcl'
    with pytest.raises(ValueError):
        rebuild_localizer(str(m), 'slam')
    assert ('nav2_amcl', 'amcl') in run()
    (tmp_path / 'room.posegraph').write_bytes(b'')
    assert rebuild_localizer(str(m)) == ('slam', str(tmp_path / 'room'))
    n = run()
    assert ('slam_toolbox', 'localization_slam_toolbox_node') in n
    assert ('nav2_amcl', 'amcl') not in n
    assert ('nav2_amcl', 'amcl') in run(rebuild_localizer='amcl')
