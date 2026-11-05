import pytest

from kelo_bringup import nav_logic as nl
from kelo_bringup.nav_logic import (Grid, StuckTimer, clock_offset, first_blocked, readiness,
                                    wait_reason)


def grid_with(cells, w=100, h=40, res=0.05):
    data = [0] * (w * h)
    for x, y, v in cells:
        data[int(y / res) * w + int(x / res)] = v
    return Grid(res, 0.0, 0.0, w, h, data)


PATH = [(0.5 + 0.05 * i, 1.0) for i in range(80)]


def test_obstacle_on_the_path_ahead_is_found():
    g = grid_with([(1.72, 1.01, 100)])
    d, x, y = first_blocked(PATH, (0.5, 1.0), g)
    assert d == pytest.approx(1.2, abs=0.06)
    assert x == pytest.approx(1.7, abs=0.05)


def test_robot_cells_goal_end_and_far_cells_are_ignored():
    g = grid_with([(0.62, 1.01, 100), (3.2, 1.01, 100), (4.3, 1.01, 100)])
    assert first_blocked(PATH, (0.5, 1.0), g) is None
    assert first_blocked(PATH, (3.0, 1.0), g) is None
    assert first_blocked(PATH, (2.0, 1.0), g)[0] == pytest.approx(1.2, abs=0.06)


def test_clock_offset_uses_the_lower_quartile():
    assert clock_offset([0.51, 0.52, 0.9, 1.4, 0.5, 0.55, 0.6, 2.0]) == pytest.approx(0.52)
    assert clock_offset([]) is None


def seen(now, **ages):
    return {k: now - v for k, v in ages.items()}


def test_readiness_names_the_first_missing_piece():
    now = 100.0
    ok_seen = seen(now, odom=0.05, scan=0.1, camera=0.2)
    assert readiness(now, {}, None, False, None, False)[1].startswith('no odometry from the robot yet')
    assert 'for 5 s' in readiness(now, seen(now, odom=5.0, scan=0.1), None, False, 'LOCALIZED', True)[1]
    r = readiness(now, ok_seen, 1.8, False, 'LOCALIZED', True)
    assert not r[0] and 'robot clock is +1.80 s' in r[1] and 'robot.sh time' in r[1]
    assert 'two publishers' in readiness(now, ok_seen, 0.01, True, 'LOCALIZED', True)[1]
    r = readiness(now, seen(now, odom=0.05, scan=0.1), 0.0, False, 'LOCALIZED', True)
    assert r[0] and 'no depth camera' in r[1]
    assert readiness(now, seen(now, odom=0.05, scan=0.1), 0.0, False, 'LOCALIZED', True, camera=False)[0]
    assert readiness(now, ok_seen, 0.0, False, 'LOST', True)[1] == 'not localized (LOST)'
    assert readiness(now, ok_seen, 0.0, False, 'LOCALIZED', True) == (True, 'ready')


def test_wait_reason_priority():
    ready = (True, 'ready')
    assert wait_reason((False, 'not localized (LOST)'), True, 'StopZone', None, None, 1.0, 0) == \
        'not localized (LOST)'
    assert 'person' in wait_reason(ready, True, 'StopZone', None, None, 1.0, 0)
    assert 'StopZone' in wait_reason(ready, False, 'StopZone', (1.0, 2.0, 3.0), None, 1.0, 0)
    assert 'goal is inside an obstacle' in wait_reason(ready, False, '', (1.0, 2.0, 3.0), 100, 1.0, 0)
    assert 'obstacle on the path 1.0 m ahead at (2.00, 3.00)' in wait_reason(ready, False, '', (1.0, 2.0, 3.0), 0, 1.0, 0)
    assert 'no path' in wait_reason(ready, False, '', None, 0, None, 0)
    assert 'recovering' in wait_reason(ready, False, '', None, 0, 1.0, 2)
    assert wait_reason(ready, False, '', None, 0, 1.0, 0) == ''
    assert wait_reason(ready, False, '', None, 0, 9.0, 2, progressing=True) == ''
    assert wait_reason((False, 'not localized (LOST)'), False, '', None, 0, 1.0, 0, progressing=True) == \
        'not localized (LOST)'


def test_stuck_timer():
    t = StuckTimer(60.0, 0.10)
    assert not t.step(0.0, 5.0, (0.0, 0.0))
    assert not t.step(30.0, 5.0, (0.0, 0.0))
    assert not t.step(59.0, 4.95, (0.05, 0.0))
    assert t.step(60.5, 4.95, (0.05, 0.0))
    assert not t.progressing(60.5)
    t.reset(70.0)
    assert not t.step(70.0, 5.0, (0.0, 0.0))
    assert t.progressing(71.0)
    for i in range(1, 20):
        assert not t.step(70.0 + 10 * i, 6.0, (0.2 * i, 0.0))
    t.pause(400.0)
    assert not t.step(440.0, 6.0, (3.8, 0.0))
    assert t.step(461.0, 6.0, (3.8, 0.0))


def test_restarting_navigation_node_means_not_ready():
    h = {'nodes': {'planner_server': ['inactive', 1], 'amcl': ['ok', 0], 'objects': ['down', 0]}}
    assert nl.broken_nav_nodes(h) == ['planner_server']
    assert nl.broken_nav_nodes(None) == []
    seen = {'odom': 9.9, 'scan': 9.9, 'camera': 9.9}
    ok, why = readiness(10.0, seen, 0.0, False, 'LOCALIZED', True, broken=['planner_server'])
    assert not ok and 'planner_server' in why
    assert readiness(10.0, seen, 0.0, False, 'LOCALIZED', True, broken=[]) == (True, 'ready')
