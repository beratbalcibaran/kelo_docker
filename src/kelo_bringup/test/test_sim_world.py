import math

import numpy as np
import pytest

from kelo_bringup.logic import depth_to_points
from kelo_bringup.sim import world as W

MOUNT0 = {'x': 0.0, 'y': 0.0, 'z': 0.20, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
CAM = {'x': 0.15, 'y': 0.0, 'z': 0.30, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
FOOT = [[0.30, 0.25], [0.30, -0.25], [-0.30, -0.25], [-0.30, 0.25]]


def room(extra=()):
    return W.load_world({'room': {'width': 8.0, 'height': 6.0, 'wall_height': 2.0},
                         'obstacles': list(extra)})


def test_lidar_hits_wall_at_known_distance():
    boxes, cyls = room()
    r = W.lidar_ranges((1.0, 3.0, 0.0), MOUNT0, boxes, cyls, -0.1, 0.1, 3, 10.0)
    assert r[1] == pytest.approx(7.0, abs=1e-6)
    r = W.lidar_ranges((1.0, 3.0, math.pi / 2), MOUNT0, boxes, cyls, 0.0, 0.0, 1, 10.0)
    assert r[0] == pytest.approx(3.0, abs=1e-6)


def test_out_of_range_is_inf():
    boxes, cyls = room()
    r = W.lidar_ranges((1.0, 3.0, 0.0), MOUNT0, boxes, cyls, 0.0, 0.0, 1, 5.6)
    assert math.isinf(r[0])


def test_table_top_invisible_to_lidar_visible_to_camera():
    table = {'type': 'box', 'name': 'tabletop', 'xy': [2.0, 2.5, 2.8, 3.5], 'z': [0.25, 0.80]}
    boxes, cyls = room([table])
    r = W.lidar_ranges((1.0, 3.0, 0.0), MOUNT0, boxes, cyls, 0.0, 0.0, 1, 10.0)
    assert r[0] == pytest.approx(7.0, abs=1e-6)
    intr = W.camera_intrinsics(106, 60, 87.0, 58.0)
    d = W.depth_image((1.0, 3.0, 0.0), CAM, boxes, cyls, 106, 60, intr, 0.28, 6.0)
    assert d[30, 53] == pytest.approx(0.85, abs=0.02)


def test_camera_sees_floor_and_min_depth_zero():
    boxes, cyls = room()
    intr = W.camera_intrinsics(106, 60, 87.0, 58.0)
    d = W.depth_image((1.0, 3.0, 0.0), CAM, boxes, cyls, 106, 60, intr, 0.28, 6.0)
    assert (d[-1] > 0).all()
    near = {'type': 'box', 'xy': [1.2, 2.5, 1.4, 3.5], 'z': [0.0, 1.0]}
    boxes, cyls = room([near])
    d = W.depth_image((1.0, 3.0, 0.0), CAM, boxes, cyls, 106, 60, intr, 0.28, 6.0)
    assert d[30, 53] == 0.0


def test_depth_to_points_filters_floor_and_range():
    boxes, cyls = room([{'type': 'box', 'xy': [2.0, 2.5, 2.8, 3.5], 'z': [0.25, 0.80]}])
    intr = W.camera_intrinsics(106, 60, 87.0, 58.0)
    d = W.depth_image((1.0, 3.0, 0.0), CAM, boxes, cyls, 106, 60, intr, 0.28, 6.0)
    rot = W.R_LINK_OPTICAL
    pts = depth_to_points(d, intr, 1, (0.30, 2.5), (0.06, 1.20), rot, np.array([0.15, 0.0, 0.30]))
    assert len(pts) > 50
    assert pts[:, 2].min() >= 0.06 and pts[:, 2].max() <= 1.20
    rng = np.linalg.norm(pts - np.array([0.15, 0.0, 0.30]), axis=1)
    assert rng.min() >= 0.30 - 1e-6 and rng.max() <= 2.5 + 1e-6
    assert pts[:, 0].min() == pytest.approx(1.0, abs=0.03)


def test_stride_decimation():
    d = np.full((240, 424), 1.0, dtype=np.float32)
    intr = W.camera_intrinsics(424, 240, 87.0, 58.0)
    pts = depth_to_points(d, intr, 4, (0.0, 10.0), (-10, 10), np.eye(3), np.zeros(3))
    assert len(pts) == 60 * 106


def test_collision_detection():
    boxes, cyls = room([{'type': 'box', 'xy': [2.0, 2.5, 2.8, 3.5], 'z': [0.25, 0.80]},
                        {'type': 'box', 'xy': [5.0, 0.0, 5.5, 1.0], 'z': [0.50, 0.80]}])
    assert W.collides(FOOT, (1.5, 3.0, 0.0), 0.40, boxes, cyls) is None
    assert W.collides(FOOT, (1.75, 3.0, 0.0), 0.40, boxes, cyls) is not None
    assert W.collides(FOOT, (5.2, 0.6, 0.0), 0.40, boxes, cyls) is None
    assert W.collides(FOOT, (0.2, 3.0, 0.0), 0.40, boxes, cyls) is not None


def test_moving_cylinder_ping_pong():
    c = W.Cylinder(0, 0, 0.2, 0, 1.7, path=[[0, 0], [2, 0]], speed=1.0, t0=0.0)
    c.update(1.0)
    assert (c.x, c.y) == pytest.approx((1.0, 0.0))
    c.update(3.0)
    assert (c.x, c.y) == pytest.approx((1.0, 0.0))


def test_table_obstacle_type():
    boxes, cyls = W.obstacle_from_dict({'type': 'table', 'xy': [0, 0, 1, 1], 'top': [0.7, 0.75]})
    assert len(boxes) == 1 and len(cyls) == 4 and boxes[0].lo[2] == pytest.approx(0.7)
