import math

import pytest

from kelo_bringup import route as rt


def square():
    return [rt.station('A', 0, 0), rt.station('B', 2, 0), rt.station('C', 2, 2), rt.station('D', 0, 2)]


def test_station_defaults_and_rounding():
    s = rt.station('A', 1.23456, -2, None, -3, True)
    assert s == {'name': 'A', 'x': 1.235, 'y': -2.0, 'yaw': None, 'wait_s': 0.0, 'stop': True}


def test_clean_route_fills_names_and_auto_yaw():
    r = rt.clean_route({'stations': [{'x': 1, 'y': 2}, {'name': 'K', 'x': 3, 'y': 4, 'yaw': 'auto', 'stop': False}]})
    assert [s['name'] for s in r['stations']] == ['S1', 'K']
    assert r['stations'][1]['yaw'] is None and r['stations'][1]['stop'] is False
    with pytest.raises(ValueError):
        rt.clean_route({'stations': [{'x': 1}]})


def test_routes_file_round_trip(tmp_path):
    path = str(tmp_path / 'routes.yaml')
    assert rt.load_routes(path) == {}
    rt.save_routes({'room': {'stations': square()}}, path)
    assert rt.load_routes(path) == {'room': {'stations': square()}}


def test_auto_heading_is_the_arrival_direction():
    st = square()
    assert rt.goal_yaw(st, 1, loop=False, came_from=(0, 0)) == pytest.approx(0.0)
    assert rt.goal_yaw(st, 2, loop=False, came_from=(2, 0)) == pytest.approx(math.pi / 2)
    assert rt.goal_yaw(st, 0, loop=True, came_from=(0, 2)) == pytest.approx(-math.pi / 2)


def test_without_an_arrival_direction_it_faces_the_next_station():
    st = square()
    assert rt.goal_yaw(st, 0, loop=False) == pytest.approx(0.0)
    assert rt.goal_yaw(st, 0, loop=False, came_from=(0, 0)) == pytest.approx(0.0)
    assert rt.goal_yaw(st, 3, loop=True) == pytest.approx(-math.pi / 2)
    assert rt.goal_yaw([rt.station('A', 1, 1)], 0, loop=False) == 0.0


def test_fixed_yaw_wins():
    st = square()
    st[1]['yaw'] = 1.0
    assert rt.goal_yaw(st, 1, loop=False, came_from=(0, 0)) == 1.0


def test_two_stations_on_the_same_spot_do_not_break_the_heading():
    st = [rt.station('A', 0, 0), rt.station('B', 0, 0), rt.station('C', 1, 0)]
    assert rt.goal_yaw(st, 1, loop=False, came_from=(0, 0)) == pytest.approx(0.0)


def test_plan_one_lap():
    p = rt.Plan(3, laps=1)
    seen = [(p.lap, p.index)]
    while p.advance():
        seen.append((p.lap, p.index))
    assert seen == [(1, 0), (1, 1), (1, 2)]
    assert not p.loop


def test_plan_laps_and_start():
    p = rt.Plan(2, laps=2, start=1)
    seen = [(p.lap, p.index)]
    while p.advance():
        seen.append((p.lap, p.index))
    assert seen == [(1, 1), (2, 0), (2, 1)]
    assert p.loop


def test_plan_until_stopped():
    p = rt.Plan(2, laps=0)
    for _ in range(100):
        assert p.advance()
    assert p.lap == 51 and not p.last_visit()


def test_plan_needs_stations():
    with pytest.raises(ValueError):
        rt.Plan(0)


def test_pass_through_only_where_asked_and_never_at_the_end():
    st = square()
    st[1]['stop'] = False
    st[3]['stop'] = False
    p = rt.Plan(4, laps=1)
    stops = [rt.stops_here(st, p)]
    while p.advance():
        stops.append(rt.stops_here(st, p))
    assert stops == [True, False, True, True]
    st[1]['wait_s'] = 2.0
    p = rt.Plan(4, laps=1, start=1)
    assert rt.stops_here(st, p)


def test_map_check():
    grid = [[0] * 20 for _ in range(20)]
    grid[10][15] = 100
    grid[0][0] = -1
    info = {'resolution': 0.1, 'origin': [0.0, 0.0]}
    assert rt.map_check(grid, info, 0.5, 0.5, 0.3) == 'ok'
    assert rt.map_check(grid, info, 1.35, 1.05, 0.3) == 'blocked'
    assert rt.map_check(grid, info, 0.05, 0.05, 0.3) == 'unknown'
    assert rt.map_check(grid, info, 5.0, 0.5, 0.3) == 'unknown'


def test_map_check_with_numpy_grid():
    import numpy as np
    grid = np.zeros((20, 20), np.int16)
    grid[10, 15] = 100
    info = {'resolution': 0.1, 'origin': [0.0, 0.0]}
    assert rt.map_check(grid, info, 0.5, 0.5, 0.3) == 'ok'
    assert rt.map_check(grid, info, 1.35, 1.05, 0.3) == 'blocked'


def test_through_poses_tree_is_the_same_tree_with_a_path_through_all_stations():
    import os
    bt = os.path.join(os.path.dirname(__file__), '..', 'behavior_trees')
    one = open(os.path.join(bt, 'navigate_wait_then_detour.xml'), encoding='utf-8').read()
    many = open(os.path.join(bt, 'navigate_through_poses_wait_then_detour.xml'), encoding='utf-8').read()
    assert many.count('RemovePassedGoals input_goals="{goals}" output_goals="{goals}"') == 1
    assert many.count('ComputePathThroughPoses goals="{goals}"') == 1
    norm = (many.replace('ComputePathThroughPoses goals="{goals}"', 'ComputePathToPose goal="{goal}"')
            .replace('            <Sequence>\n            <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.5"/>\n', '')
            .replace('            </Fallback>\n            </Sequence>\n          </RateController>',
                     '            </Fallback>\n          </RateController>'))
    assert norm == one


def test_both_trees_are_configured():
    import yaml, os
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')
    bt = yaml.safe_load(open(cfg, encoding='utf-8'))['bt_navigator']['ros__parameters']
    assert bt['default_nav_to_pose_bt_xml'] == 'navigate_wait_then_detour.xml'
    assert bt['default_nav_through_poses_bt_xml'] == 'navigate_through_poses_wait_then_detour.xml'


def test_map_image_and_pixel_round_trip(tmp_path):
    import cv2
    import numpy as np
    img = np.full((40, 60), 254, np.uint8)
    img[5, 10] = 0
    img[30, 50] = 205
    cv2.imwrite(str(tmp_path / 'm.pgm'), img)
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.05\norigin: [-1.0, -0.5, 0]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    gray, grid, info = rt.load_map_image(str(tmp_path / 'm.yaml'))
    assert gray.shape == grid.shape == (40, 60)
    assert grid[40 - 1 - 5, 10] == 100 and grid[40 - 1 - 30, 50] == -1 and grid[0, 0] == 0
    x, y = rt.pixel_to_world(10.5, 5.5, info, 40)
    assert rt.map_check(grid, info, x, y, 0.01) == 'blocked'
    u, v = rt.world_to_pixel(x, y, info, 40, scale=2.0)
    assert (u, v) == pytest.approx((21.0, 11.0))
    assert rt.pixel_to_world(u, v, info, 40, scale=2.0) == pytest.approx((x, y))
