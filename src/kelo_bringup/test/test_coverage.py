import numpy as np
from nav_msgs.msg import MapMetaData

from kelo_bringup.coverage_view import coverage, frontiers, next_target, resample


def info(w, h, res=0.05, ox=0.0, oy=0.0):
    m = MapMetaData()
    m.width, m.height, m.resolution = w, h, res
    m.origin.position.x, m.origin.position.y = ox, oy
    m.origin.orientation.w = 1.0
    return m


def test_frontier_between_free_and_unknown():
    g = np.full((40, 40), -1, np.int8)
    g[:, :20] = 0          # free on the left, unknown on the right
    g[:, 0] = 100          # a wall on the far left
    fr, mask = frontiers(g, min_cells=5)
    assert len(fr) == 1
    assert mask[:, 19].all() and mask.sum() == 40
    # closed area: no frontier
    g[:, 20:] = 100
    assert frontiers(g)[0] == []


def test_small_frontier_gaps_are_ignored():
    g = np.zeros((30, 30), np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[10, 10] = -1  # one unknown cell inside a room: noise, not a place to explore
    assert frontiers(g, min_cells=6)[0] == []


def test_coverage_classes():
    g = np.zeros((100, 100), np.int8)
    cam = np.full((100, 100), -1, np.int8)
    cam[:, :50] = 0
    track = [(50, c) for c in range(0, 30)]
    both, lidar_only, far = coverage(g, cam, track, 0.05, near=3.0)
    assert both[50, 10] and lidar_only[50, 60] and not far[50, 60]
    # 3 m = 60 cells from the end of the track (50, 29)
    assert far[50, 95] and not (both | lidar_only)[50, 95]


def test_resample_onto_a_shifted_grid():
    src = np.zeros((10, 10), np.int8)
    src[2, 3] = 100
    out = resample(src, info(10, 10), info(10, 10, ox=0.05, oy=0.05))
    assert out[1, 2] == 100
    assert out[-1, -1] == -1  # outside the source


def test_next_target_prefers_a_real_frontier_over_a_small_gap():
    x, y = np.meshgrid(np.arange(10) * 1.0, np.arange(10) * 1.0)
    fronts = [(0.0, 1.0, 6), (0.0, 5.0, 40)]  # (row, col, cells): small one 1 m away, big one 5 m
    assert next_target(fronts, x, y, (0.0, 0.0, 0.0), 12)[2] == 5.0
    assert next_target(fronts[:1], x, y, (0.0, 0.0, 0.0), 12)[2] == 1.0
