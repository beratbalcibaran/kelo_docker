import math

import numpy as np
import pytest

from kelo_bringup import scan_match as sm
from kelo_bringup.localize import accept, already_localized

RES = 0.05


def room(w=8.0, h=6.0, box=True):
    """Walls of a w x h room (origin 0,0) and a box near one corner: no 180 deg symmetry."""
    grid = np.zeros((int(h / RES), int(w / RES)), dtype=np.int16)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    if box:
        grid[int(1.5 / RES):int(2.5 / RES), int(2.0 / RES):int(3.0 / RES)] = 100
    return grid


def ray_scan(grid, x, y, yaw, n=360, rmax=5.6):
    """Ranges of an ideal 360 deg scanner at (x, y, yaw) in `grid` (simple ray marching)."""
    ranges = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        r = 0.05
        while r < rmax:
            px, py = x + r * math.cos(yaw + a), y + r * math.sin(yaw + a)
            c, rw = int(px / RES), int(py / RES)
            if not (0 <= rw < grid.shape[0] and 0 <= c < grid.shape[1]) or grid[rw, c] >= sm.OCCUPIED:
                break
            r += 0.02
        ranges.append(r)
    return ranges, -math.pi, 2 * math.pi / n


def fit_at(grid, true, guess):
    ranges, amin, inc = ray_scan(grid, *true)
    pts = sm.scan_points(ranges, amin, inc, 0.05, 5.6, step=1)
    return sm.MapFit(grid, RES, (0.0, 0.0)).fit(sm.transform(pts, *guess), origin=guess[:2])


def test_scan_fits_at_the_true_pose():
    g = room()
    assert fit_at(g, (1.0, 1.0, 0.3), (1.0, 1.0, 0.3)) > 0.9


def test_scan_does_not_fit_at_a_wrong_pose():
    g = room()
    true = (1.0, 1.0, 0.3)
    assert fit_at(g, true, (5.5, 3.0, 0.3)) < sm.LOST_FIT
    # the box is behind the robot at this guess: the beams that measured free space cross it
    assert fit_at(g, (4.0, 2.0, 0.0), (2.5, 3.2, 0.0)) < sm.LOST_FIT
    # 0.5 m off: clearly worse than the true pose
    assert fit_at(g, true, (1.5, 1.0, 0.3)) < sm.GOOD_FIT


def test_a_person_in_front_lowers_the_fit_only_a_little():
    g = room()
    with_person = g.copy()
    with_person[int(0.8 / RES):int(1.2 / RES), int(1.6 / RES):int(2.0 / RES)] = 100
    ranges, amin, inc = ray_scan(with_person, 1.0, 1.0, 0.0)
    pts = sm.transform(sm.scan_points(ranges, amin, inc, 0.05, 5.6, step=1), 1.0, 1.0, 0.0)
    assert sm.MapFit(g, RES, (0.0, 0.0)).fit(pts, origin=(1.0, 1.0)) > sm.GOOD_FIT


def test_points_outside_the_map_miss_and_few_points_give_no_answer():
    mf = sm.MapFit(room(), RES, (0.0, 0.0))
    assert mf.fit(np.full((50, 2), -10.0)) == 0.0
    assert mf.fit(np.zeros((5, 2))) is None


def test_scan_points_drop_max_range_and_invalid():
    pts = sm.scan_points([1.0, float('inf'), float('nan'), 5.6, 0.01, 2.0], 0.0, 0.1, 0.02, 5.6, step=1)
    assert len(pts) == 2
    assert pts[0] == pytest.approx((1.0, 0.0))


def test_accept_needs_the_scan_fit_when_known():
    assert accept(0.10, math.radians(5), None)
    assert accept(0.10, math.radians(5), 0.8)
    assert not accept(0.10, math.radians(5), 0.3)        # tight cloud on the wrong place
    assert accept(0.18, math.radians(7), 0.85)           # slightly wide cloud, very good fit
    assert not accept(0.18, math.radians(7), 0.65)
    assert not accept(0.40, math.radians(5), 0.95)


def test_already_localized_rechecks_the_fit():
    assert already_localized('LOCALIZED', 0.1, 0.05, fit=0.8)
    assert not already_localized('LOCALIZED', 0.1, 0.05, fit=0.2)
    assert not already_localized('LOST', 0.1, 0.05, fit=0.8)
    assert not already_localized('LOCALIZED', 0.1, 0.05, force=True, fit=0.8)
    assert already_localized('LOCALIZED', 0.16, 0.05, fit=0.9)     # wider cloud, very good fit


def test_last_pose_roundtrip_and_other_map(tmp_path):
    path = str(tmp_path / 'last_pose.yaml')
    a, b = tmp_path / 'a.yaml', tmp_path / 'b.yaml'
    a.write_text('x'), b.write_text('x')
    (tmp_path / 'current.yaml').symlink_to(a)
    sm.save_last_pose(path, str(tmp_path / 'current.yaml'), 1.234, -2.0, math.radians(45), 0.8)
    x, y, yaw = sm.load_last_pose(path, str(a))
    assert (x, y, math.degrees(yaw)) == pytest.approx((1.234, -2.0, 45.0))
    assert sm.load_last_pose(path, str(b)) is None
    assert sm.load_last_pose(str(tmp_path / 'missing.yaml'), str(a)) is None
    (tmp_path / 'broken.yaml').write_text('x: [')
    assert sm.load_last_pose(str(tmp_path / 'broken.yaml'), str(a)) is None


def test_people_all_around_are_not_lost_but_a_wrong_pose_is():
    g = room()
    crowd = g.copy()
    # people (0.4 m) all around the robot at 0.8 m: most beams end on them
    for a in np.linspace(0, 2 * math.pi, 8, endpoint=False):
        cx, cy = 4.0 + 0.8 * math.cos(a), 3.0 + 0.8 * math.sin(a)
        crowd[int((cy - 0.2) / RES):int((cy + 0.2) / RES), int((cx - 0.2) / RES):int((cx + 0.2) / RES)] = 100
    ranges, amin, inc = ray_scan(crowd, 4.0, 3.0, 0.0)
    pts = sm.transform(sm.scan_points(ranges, amin, inc, 0.05, 5.6, step=1), 4.0, 3.0, 0.0)
    hits, crossings = sm.MapFit(g, RES, (0.0, 0.0)).parts(pts, origin=(4.0, 3.0))
    assert hits - crossings < sm.LOST_FIT
    assert not sm.looks_lost(hits, crossings)
    # the same scan at a wrong pose: beams through the walls
    wrong = sm.transform(sm.scan_points(ranges, amin, inc, 0.05, 5.6, step=1), 1.0, 1.0, 0.0)
    assert sm.looks_lost(*sm.MapFit(g, RES, (0.0, 0.0)).parts(wrong, origin=(1.0, 1.0)))
