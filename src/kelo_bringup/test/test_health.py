from types import SimpleNamespace

from launch_ros.actions import LifecycleNode, Node

from kelo_bringup import health as hl
from kelo_bringup.stack import NAV_LIFECYCLE, build, node_names


def opts(**kw):
    base = {'source': 'sim', 'mode': 'navigation', 'odom_source': 'robot', 'map': '/tmp/x.yaml',
            'bag': '/tmp/bag', 'record': 'false', 'use_rviz': 'false', 'initial_pose': ''}
    base.update(kw)
    return base


def test_node_lifecycle_of_states():
    t = hl.NodeTracker(['amcl', 'controller_server'])
    st, ch, _ = t.step(0.0, set())
    assert st == {'amcl': 'waiting', 'controller_server': 'waiting'} and ch == []
    st, ch, _ = t.step(1.0, {'amcl'})
    assert st['amcl'] == 'ok' and ('amcl', 'waiting', 'ok') in ch
    st, ch, _ = t.step(2.0, set())
    assert st['amcl'] == 'down' and ('amcl', 'ok', 'down') in ch
    st, _, _ = t.step(3.0, {'amcl'})
    assert st['amcl'] == 'ok' and t.restarts['amcl'] == 1
    assert st['controller_server'] == 'waiting'     # Nav2 not started yet is no alarm


def test_quick_restart_is_seen_by_its_new_pid():
    t = hl.NodeTracker(['controller_server'])
    t.step(0.0, {'controller_server'}, {'controller_server': 'active'}, {'controller_server': 10})
    states = {'controller_server': 'active'}       # stale: read before the crash
    st, _, restarted = t.step(1.0, {'controller_server'}, states, {'controller_server': 11})
    assert restarted == ['controller_server'] and t.restarts['controller_server'] == 1
    assert 'controller_server' not in states and st['controller_server'] == 'inactive'
    st, _, _ = t.step(2.0, {'controller_server'}, {'controller_server': 'unconfigured'}, {'controller_server': 11})
    assert st['controller_server'] == 'inactive'


def test_respawned_lifecycle_node_is_activated_again_after_a_delay():
    t = hl.NodeTracker(['controller_server', 'amcl'])
    t.step(0.0, {'controller_server', 'amcl'}, {'controller_server': 'unconfigured'})
    assert t.to_activate(10.0, 3.0, {'controller_server'}) == []      # still starting: not ours
    t.step(1.0, {'controller_server', 'amcl'}, {'controller_server': 'active', 'amcl': 'active'})
    t.step(2.0, {'amcl'}, {'amcl': 'active'})                           # crash
    st, _, _ = t.step(2.5, {'controller_server', 'amcl'}, {'controller_server': 'active', 'amcl': 'active'})
    assert st['controller_server'] == 'inactive'      # back, the stale 'active' is dropped
    st, _, _ = t.step(3.0, {'controller_server', 'amcl'}, {'controller_server': 'unconfigured', 'amcl': 'active'})
    assert st['controller_server'] == 'inactive' and t.restarts['controller_server'] == 1
    assert t.to_activate(4.0, 3.0, {'controller_server'}) == []
    assert t.to_activate(6.0, 3.0, {'controller_server'}) == ['controller_server']
    assert t.to_activate(6.0, 3.0, set()) == []                         # bonded manager handles it
    t.activated('controller_server', 6.0)
    assert t.to_activate(7.0, 3.0, {'controller_server'}) == []
    st, _, _ = t.step(8.0, {'controller_server', 'amcl'}, {'controller_server': 'active', 'amcl': 'active'})
    assert st['controller_server'] == 'ok'


def test_summary_text():
    assert hl.summary({'a': 'ok', 'b': 'waiting'}, {'a': 0, 'b': 0}) == 'all 1 nodes up (1 waiting to start)'
    s = hl.summary({'a': 'down', 'b': 'ok', 'c': 'inactive'}, {'a': 0, 'b': 2, 'c': 0})
    assert s == 'down: a; not active: c; restarted: b 2x'


def test_load_helpers(tmp_path):
    f = tmp_path / 'load'
    f.write_text('437\n')
    assert hl.gpu_load_file([str(tmp_path / 'nothing'), str(f)]) == str(f)
    assert hl.read_gpu(str(f)) == 43.7
    assert hl.read_gpu(None) is None
    e = lambda c: SimpleNamespace(current=c)
    assert hl.max_temperature({'cpu': [e(51.0), e(0.0)], 'gpu': [e(58.5)]}) == 58.5
    assert hl.max_temperature({}) is None
    assert hl.process_label('python3', ['/usr/bin/python3', '/x/cloud_filter', '--ros-args', '-r',
                                        '__node:=cloud_filter']) == 'cloud_filter'
    assert hl.process_label('python3', ['/usr/bin/python3', '/x/objects']) == 'objects'
    assert hl.process_label('rviz2', ['/opt/ros/jazzy/lib/rviz2/rviz2']) == 'rviz2'
    assert hl.top_processes([('a', 5.0), ('b', 50.0), ('c', None), ('d', 20.0), ('e', 1.0)]) == \
        [('b', 50), ('d', 20), ('a', 5)]
    load = {'cpu': 95.0, 'gpu': 30.0, 'ram_used': 7.0, 'ram_total': 7.4, 'temp': 61.0, 'top': [('amcl', 12)]}
    assert hl.load_text(load) == 'CPU 95%  GPU 30%  RAM 7.0/7.4 GB  61°C  top: amcl 12%'
    assert hl.load_level(load) == 'CPU 95%, RAM almost full'
    assert hl.load_level({'cpu': 20.0}) == ''


def walk_nodes(actions):
    from launch.actions import RegisterEventHandler, TimerAction
    for a in actions:
        if isinstance(a, Node):
            yield a
        elif isinstance(a, TimerAction):
            yield from walk_nodes(a.actions)
        elif isinstance(a, RegisterEventHandler):
            yield from walk_nodes(getattr(a.event_handler, '_OnActionEventBase__actions_on_event', []) or [])


def respawns(n):
    return bool(getattr(n, '_ExecuteLocal__respawn', False))


def test_health_monitor_watches_every_node_and_nav2_is_in_it():
    for source in ('sim', 'hardware', 'replay'):
        acts = build(opts(source=source))
        hm = [n for n in walk_nodes(acts) if getattr(n, '_Node__node_name', None) == 'health_monitor']
        assert len(hm) == 1, source
        names = set(node_names(acts))
        assert set(NAV_LIFECYCLE) <= names
        assert {'amcl', 'map_server', 'loc_monitor', 'nav_watch', 'collision_monitor_teleop'} <= names


def test_long_running_nodes_respawn():
    # not respawned: lifecycle managers (autostart on active nodes fails), slam_toolbox (the map would
    # be lost), sim_robot (the simulated robot would jump back to the start)
    keep_out = {'sim_robot', 'slam_toolbox'}
    for mode in ('navigation', 'mapping', 'sensors'):
        for source in ('sim', 'hardware'):
            for n in walk_nodes(build(opts(source=source, mode=mode))):
                name = getattr(n, '_Node__node_name', None) or n.node_executable
                if isinstance(n, LifecycleNode) or name in keep_out or name.startswith('lifecycle_manager'):
                    continue
                assert respawns(n), (mode, source, name)
