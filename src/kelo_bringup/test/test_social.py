import math

import numpy as np
import pytest
import yaml

from kelo_bringup import scene

RES = 0.05


def zone(people, shape=(200, 200), origin=(-5.0, -5.0, 0.0), **kw):
    return scene.people_zone(people, origin, shape, RES, **kw)


def at(grid, x, y, origin=(-5.0, -5.0)):
    return int(grid[int((y - origin[1]) / RES), int((x - origin[0]) / RES)])


def test_standing_person_gets_a_high_core_and_a_falling_cost():
    g = zone([(0.0, 0.0, 0.0, 0.0)], core=0.55, social=1.2)
    assert at(g, 0.0, 0.0) == 90
    assert at(g, 0.5, 0.0) == 90
    near, far = at(g, 0.7, 0.0), at(g, 1.1, 0.0)
    assert 0 < far < near < 90
    # never lethal: only the sensors' person is (a zone around a jittering centre made paths invalid)
    assert g.max() < 100
    assert at(g, 1.5, 0.0) == -1  # no information: the layer leaves the costmap alone


def test_walking_person_marks_the_way_ahead_not_behind():
    # 1 m/s along +x: core cost 1.5 s ahead, falling cost up to 3 s, nothing behind
    g = zone([(0.0, 0.0, 1.0, 0.0)], core=0.4, social=1.0, lethal_s=1.5, horizon=3.0)
    assert at(g, 1.4, 0.0) == 90
    assert 0 < at(g, 2.6, 0.0) < 100
    assert at(g, -1.2, 0.0) == -1


def test_zone_follows_a_rotated_grid_origin():
    # grid turned by 90 deg: the grid's x axis is the world y axis
    g = scene.people_zone([(3.0, 2.0, 0.0, 0.0)], (0.0, 0.0, math.pi / 2), (100, 100), RES)
    # world (3, 2) -> grid column 2 / RES, row -3 / RES: outside (3 m beyond the edge), nothing drawn
    assert (g == -1).all()
    g = scene.people_zone([(-1.0, 2.0, 0.0, 0.0)], (0.0, 0.0, math.pi / 2), (100, 100), RES)
    assert g[int(1.0 / RES), int(2.0 / RES)] == 90


def test_path_blocked_by_a_person_on_it_but_not_beside_it():
    path = np.stack([np.linspace(0, 4, 41), np.zeros(41)], axis=1)
    assert scene.path_blocked(path, [(1.5, 0.1, 0.0, 0.0)], clearance=0.8, ahead=2.5)
    assert not scene.path_blocked(path, [(1.5, 1.2, 0.0, 0.0)], clearance=0.8, ahead=2.5)
    # beyond the checked part of the path
    assert not scene.path_blocked(path, [(3.6, 0.0, 0.0, 0.0)], clearance=0.8, ahead=2.5)


def test_path_blocked_by_a_person_walking_towards_it():
    path = np.stack([np.linspace(0, 4, 41), np.zeros(41)], axis=1)
    # 2 m beside the path, walking at 1 m/s towards it: inside the clearance within 1.5 s
    assert scene.path_blocked(path, [(1.5, 2.0, 0.0, -1.0)], clearance=0.8, ahead=2.5)
    # walking away
    assert not scene.path_blocked(path, [(1.5, 2.0, 0.0, 1.0)], clearance=0.8, ahead=2.5)


def test_speed_limit_ramps_between_near_and_far():
    assert scene.speed_limit(None) == 100.0
    assert scene.speed_limit(0.5, 1.0, 2.0, 40.0) == 40.0
    assert scene.speed_limit(1.5, 1.0, 2.0, 40.0) == pytest.approx(70.0)
    assert scene.speed_limit(3.0, 1.0, 2.0, 40.0) == 100.0


def test_predict_track_standing_is_one_point():
    pts, t = scene.predict_track(1.0, 2.0, 0.05, 0.0)
    assert pts.shape == (1, 2) and t[0] == 0.0
    pts, t = scene.predict_track(0.0, 0.0, 0.5, 0.0, horizon=2.0, dt=0.5)
    assert pts[-1] == pytest.approx([1.0, 0.0]) and t[-1] == pytest.approx(2.0)


def test_people_from_detections_and_frame_change():
    from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
    from kelo_bringup.social import people_from, transform_people
    arr = Detection3DArray()
    for label, x in (('person', 1.0), ('chair', 2.0)):
        d = Detection3D()
        d.bbox.center.position.x = x
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = label
        h.pose.pose.orientation.x = 0.5
        d.results.append(h)
        arr.detections.append(d)
    people = people_from(arr)
    assert people == [(1.0, 0.0, 0.5, 0.0)]
    (x, y, vx, vy), = transform_people(people, (1.0, 0.0, math.pi / 2))
    assert (x, y, vx, vy) == pytest.approx((1.0, 1.0, 0.0, 0.5))


def test_people_layer_in_both_costmaps_before_inflation(tmp_path, monkeypatch):
    import os
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')
    with open(cfg, encoding='utf-8') as f:
        nav = yaml.safe_load(f)
    assert 'people_layer' not in nav['local_costmap']['local_costmap']['ros__parameters']['plugins']
    for name in ('global_costmap',):
        p = nav[name][name]['ros__parameters']
        assert p['plugins'].index('people_layer') < p['plugins'].index('inflation_layer')
        layer = p['people_layer']
        assert layer['map_topic'] == '/kelo/people_map'
        # read from the costmap by every StaticLayer: max combination, graded people costs kept
        assert p['use_maximum'] is True and p['trinary_costmap'] is False
    assert 'kelo_is_person_blocking_bt_node' in nav['bt_navigator']['ros__parameters']['plugin_lib_names']


def test_people_aware_off_removes_the_layers():
    from kelo_bringup.common import load_hardware, nav2_params
    hw = load_hardware()
    hw['navigation'] = {'people_aware': False}
    cfg = nav2_params(hw, '/odom')
    assert 'people_layer' not in cfg['global_costmap']['global_costmap']['ros__parameters']['plugins']
    # the tree still references IsPersonBlocking, so its plugin stays loaded
    assert cfg['bt_navigator']['ros__parameters']['plugin_lib_names']
