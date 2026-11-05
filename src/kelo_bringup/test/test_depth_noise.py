import math

import numpy as np
import pytest

from kelo_bringup.depth_noise import (Persistence, TiltEstimator, cell_support, depth_points, fit_floor,
                                      min_height_at, rot_xy, supported_pixels)

W, H = 424, 240
FX = (W / 2) / math.tan(math.radians(87.0) / 2)
FY = (H / 2) / math.tan(math.radians(58.0) / 2)
K = (FX, FY, W / 2, H / 2)
T = np.array([0.42, 0.0, 0.68])
OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def cam_rot(pitch, roll=0.0):
    return rot_xy(roll, pitch) @ OPT


def render_floor(pitch, roll=0.0, boxes=()):
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - K[2]) / K[0], (v - K[3]) / K[1], np.ones_like(u, dtype=float)], axis=-1)
    dirs = rays @ cam_rot(pitch, roll).T
    depth = np.zeros((H, W))
    down = dirs[..., 2] < -1e-6
    t_floor = np.where(down, -T[2] / np.where(down, dirs[..., 2], -1), np.inf)
    t_best = t_floor.copy()
    for x0, x1, y0, y1, h in boxes:
        tx = (x0 - T[0]) / np.where(np.abs(dirs[..., 0]) > 1e-9, dirs[..., 0], 1e-9)
        p = T + dirs * tx[..., None]
        hit = (tx > 0) & (p[..., 1] >= y0) & (p[..., 1] <= y1) & (p[..., 2] >= 0) & (p[..., 2] <= h)
        t_best = np.where(hit & (tx < t_best), tx, t_best)
    ok = np.isfinite(t_best)
    depth[ok] = t_best[ok] * 1.0
    return depth.astype(np.float32)


def obstacles(depth, pitch_cfg, min_h=0.06, slope=0.0, denoise=True):
    pts, dist = depth_points(depth, K, 4, (0.3, 2.5), cam_rot(pitch_cfg), T, denoise=denoise)
    keep = (pts[:, 2] >= min_height_at(dist, min_h, slope)) & (pts[:, 2] <= 1.2)
    return pts[keep]


def test_isolated_pixels_are_dropped_and_surfaces_kept():
    d = np.full((20, 20), 2.0, dtype=np.float32)
    d[5, 5] = 0.8
    d[10, 10:12] = 1.2
    m = supported_pixels(d)
    assert not m[5, 5]
    assert not m[10, 10] and not m[10, 11]
    assert m[15, 15] and m[0, 0]


def test_height_limit_rises_with_distance():
    assert min_height_at(0.8, 0.06, 0.03) == pytest.approx(0.06)
    assert min_height_at(2.0, 0.06, 0.03) == pytest.approx(0.09)


def test_cell_support_drops_lonely_points():
    cluster = np.array([[1.01, 0.01], [1.02, 0.02], [1.03, 0.03]])
    lonely = np.array([[2.0, 0.5]])
    m = cell_support(np.vstack([cluster, lonely]), 0.05, 3)
    assert m.tolist() == [True, True, True, False]


def test_persistence_needs_two_of_three_frames():
    p = Persistence(3, 2, 0.05)
    a = np.array([[1.0, 0.0]])
    b = np.array([[2.0, 0.0]])
    assert not p.step(a).any()
    assert p.step(a).all()
    assert not p.step(b).any()
    assert not p.step(np.zeros((0, 2))).any()


def test_wrong_pitch_floor_band_is_removed_by_rising_limit():
    depth = render_floor(math.radians(12.0))
    assert len(obstacles(depth, math.radians(10.0), slope=0.0)) > 50
    assert len(obstacles(depth, math.radians(10.0), slope=0.0, denoise=True)) > 50
    assert len(obstacles(depth, math.radians(10.0), slope=0.03)) == 0


def test_real_box_survives_the_filter():
    depth = render_floor(math.radians(10.0), boxes=[(1.6, 1.9, -0.15, 0.15, 0.20)])
    pts = obstacles(depth, math.radians(10.0), slope=0.03)
    assert len(pts) > 10
    assert pts[:, 0].min() == pytest.approx(1.6, abs=0.05)
    assert cell_support(pts[:, :2], 0.05, 3).sum() > 10


def test_speckles_on_the_floor_are_removed():
    rng = np.random.default_rng(1)
    depth = render_floor(math.radians(10.0))
    ys, xs = rng.integers(0, H, 60), rng.integers(0, W, 60)
    depth[ys, xs] *= 0.7
    raw = obstacles(depth, math.radians(10.0), slope=0.03, denoise=False)
    clean = obstacles(depth, math.radians(10.0), slope=0.03, denoise=True)
    assert len(raw) > 0
    assert len(clean) == 0


@pytest.mark.parametrize('true_deg,roll_deg', [(12.0, 0.0), (8.5, 0.0), (10.0, 1.5), (11.0, -1.0)])
def test_floor_fit_recovers_the_tilt_error(true_deg, roll_deg):
    depth = render_floor(math.radians(true_deg), math.radians(roll_deg))
    pts, _ = depth_points(depth, K, 2, (0.3, 3.0), cam_rot(math.radians(10.0)), T, denoise=False)
    fit = fit_floor(pts, T[:2])
    assert fit is not None
    roll, pitch, h, share, n = fit
    corr = rot_xy(roll, pitch) @ cam_rot(math.radians(10.0))
    fixed, _ = depth_points(depth, K, 2, (0.3, 2.5), corr, T, denoise=False)
    assert np.abs(fixed[:, 2]).max() < 0.02
    assert share > 0.9


def test_tilt_estimator_waits_for_stable_frames():
    est = TiltEstimator(need=5, min_inliers=10)
    for _ in range(4):
        assert est.add((0.0, 0.035, 0.0, 0.95, 500)) is None
    r, p = est.add((0.0, 0.035, 0.0, 0.95, 500))
    assert p == pytest.approx(0.035)
    est2 = TiltEstimator(need=3, min_inliers=10)
    for _ in range(5):
        assert est2.add((0.0, 0.2, 0.0, 0.95, 500)) is None
