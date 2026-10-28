import numpy as np
import pytest

from kelo_bringup.parking_view import classify, nearest, project, project_many

# optical frame (z forward, x right, y down) -> base_link (x forward, y left, z up)
ROT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
TRANS = np.array([0.42, 0.0, 0.5])
K = (50.0, 50.0, 50.0, 30.0)


def test_wall_ahead_is_in_the_corridor():
    depth = np.full((60, 100), 1.5, dtype=np.float32)
    inside, beside, dist, _ = classify(depth, K, ROT, TRANS, front=0.425, half_width=0.365,
                                    length=2.5, rng=(0.3, 4.0), hgt=(0.06, 1.2), side=0.4)
    assert nearest(dist, 4) == pytest.approx(1.92 - 0.425, abs=1e-3)
    # straight ahead: corridor; |y| = 0.6 m: beside it; image edge (|y| = 1.5 m): neither
    assert inside[30, 50] and not beside[30, 50]
    assert beside[30, 30] and not inside[30, 30]
    assert not inside[30, 0] and not beside[30, 0]


def test_free_corridor_and_single_pixel_noise():
    depth = np.zeros((60, 100), dtype=np.float32)
    depth[30, 50] = 1.0
    _, _, dist, _ = classify(depth, K, ROT, TRANS, 0.425, 0.365, 2.5, (0.3, 4.0), (0.06, 1.2))
    assert np.isfinite(dist).sum() == 1
    assert nearest(dist, 4) is None


def test_project_point_ahead_at_camera_height():
    (u, v), behind = project([(2.42, 0.0, 0.5), (0.0, 0.0, 0.5)], K, ROT, TRANS)
    assert (u, v) == pytest.approx((50.0, 30.0))
    assert behind is None


def test_project_many_matches_project():
    pts = np.array([[2.42, 0.3, 0.2], [1.0, -0.2, 0.0], [0.0, 0.0, 0.5]])
    u, v, ok = project_many(pts, K, ROT, TRANS)
    single = project(pts, K, ROT, TRANS)
    assert list(ok) == [True, True, False]
    for i in (0, 1):
        assert (u[i], v[i]) == pytest.approx(single[i])


def test_curved_corridor_follows_the_turn():
    from kelo_bringup import scene
    # a wall 1.5 m ahead: driving straight it is in the corridor, turning hard it is beside it
    depth = np.full((60, 100), 1.5, dtype=np.float32)
    straight = classify(depth, K, ROT, TRANS, 0.425, 0.365, 2.5, (0.3, 4.0), (0.06, 1.2), 0.4,
                        *scene.predict_poses(0.0, 0.0, 0.0, 2.5, ds=0.1))
    assert nearest(straight[2], 4) == pytest.approx(1.92 - 0.425, abs=1e-3)
    turning = classify(depth, K, ROT, TRANS, 0.425, 0.365, 2.5, (0.3, 4.0), (0.06, 1.2), 0.4,
                       *scene.predict_poses(0.3, 0.0, 0.15, 2.5, ds=0.1))
    # radius 2 m around (0, 2): the wall 1.5 m to the left (image column 0) is on the curve
    assert turning[0][30, 0] and not straight[0][30, 0]


def test_draw_path_stays_behind_obstacles():
    from kelo_bringup.parking_view import draw_path
    img = np.zeros((60, 100, 3), np.uint8)
    solid = np.zeros((60, 100), bool)
    solid[:, 50:] = True
    draw_path(img, solid, np.array([(0.0, 0.0), (3.0, 0.0)]), K, ROT, TRANS, 0.0)
    assert not img[:, 50:].any() and img[:, :50].any()
