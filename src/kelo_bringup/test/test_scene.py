import math

import numpy as np
import pytest

from kelo_bringup import scene


def test_standing_still_gives_the_straight_corridor():
    poses, s = scene.predict_poses(0.0, 0.0, 0.4, 2.0)
    assert poses[-1] == pytest.approx([2.0, 0.0, 0.0])
    assert s[-1] == pytest.approx(2.0)


def test_turning_left_follows_a_circle():
    # 0.3 m/s with 0.3 rad/s: radius 1 m around (0, 1)
    poses, s = scene.predict_poses(0.3, 0.0, 0.3, math.pi / 2, ds=0.01)
    x, y, yaw = poses[-1]
    assert (x, y) == pytest.approx((1.0, 1.0), abs=0.02)
    assert yaw == pytest.approx(math.pi / 2, abs=0.02)
    assert np.allclose(np.hypot(poses[:, 0], poses[:, 1] - 1.0), 1.0, atol=1e-6)


def test_sideways_motion_and_no_spirals():
    poses, _ = scene.predict_poses(0.0, 0.2, 0.0, 1.0)
    assert poses[-1] == pytest.approx([0.0, 1.0, 0.0])
    # slow and turning hard: the prediction stops after max_turn
    poses, s = scene.predict_poses(0.05, 0.0, 1.0, 2.5, max_turn=2.0)
    assert s[-1] == pytest.approx(0.1, abs=0.05) and abs(poses[-1, 2]) <= 2.0 + 1e-6


def test_sweep_distance_straight_matches_the_front_edge():
    poses, s = scene.predict_poses(0.0, 0.0, 0.0, 2.5, ds=0.1)
    pts = [(1.5, 0.0), (1.5, 0.5), (0.425 + 3.0, 0.0)]
    d = scene.sweep_distance(pts, poses, s, front=0.425, rear=0.425, half_width=0.365)
    assert d[0] == pytest.approx(1.5 - 0.425, abs=1e-6)
    assert math.isinf(d[1]) and math.isinf(d[2])


def test_sweep_distance_on_a_curve():
    # turning left: a point on the left of the curve is reached, the same point mirrored is not
    # radius 1.5 m around (0, 1.5): after 0.8 rad the centre is at (1.076, 0.455)
    poses, s = scene.predict_poses(0.3, 0.0, 0.2, 2.5, ds=0.05)
    d = scene.sweep_distance([(1.076, 0.455), (1.076, -0.455)], poses, s, 0.425, 0.425, 0.3)
    assert np.isfinite(d[0]) and math.isinf(d[1])


def test_frames_round_trip():
    pose = (1.0, 2.0, 0.7)
    p = np.array([[0.5, -0.3], [2.0, 1.0]])
    assert scene.to_base(scene.from_base(p, pose), pose) == pytest.approx(p)


def test_remaining_path_starts_at_the_robot_and_is_cut():
    path = np.array([(-1.0, 0.0), (0.0, 0.0), (1.0, 0.0), (5.0, 0.0)])
    rest, total = scene.remaining_path(path, max_length=2.0)
    assert total == pytest.approx(5.0)
    assert rest[0] == pytest.approx([0.0, 0.0])
    assert rest[-1] == pytest.approx([2.0, 0.0])


def test_ribbon_and_chevrons():
    pts, s = scene.resample([(0.0, 0.0), (2.0, 0.0)], 0.1)
    left, right = scene.ribbon(pts, 0.2)
    assert left[:, 1] == pytest.approx(0.2) and right[:, 1] == pytest.approx(-0.2)
    heads = scene.chevrons(pts, s, 0.5, 0.0, 0.1)
    assert len(heads) == 4
    wl, tip, wr = heads[0]
    assert tip[0] > wl[0] and wl[1] > 0 > wr[1]
    # the arrows flow forwards with the phase
    assert scene.chevrons(pts, s, 0.5, 0.2, 0.1)[0][1][0] == pytest.approx(tip[0] + 0.2)


def test_sector_distances():
    pts = [(1.0, 0.0), (0.0, 0.8), (-0.6, 0.0)]
    best, where = scene.sector_distances(pts, 0.425, 0.425, 0.265, sectors=4, max_dist=1.5)
    assert best[0] == pytest.approx(0.575)   # ahead
    assert best[1] == pytest.approx(0.535)   # left
    assert best[2] == pytest.approx(0.175)   # behind
    assert math.isinf(best[3]) and where[3] is None


def test_clusters_split_groups_and_drop_noise():
    a = np.random.default_rng(1).normal((2.0, 0.0), 0.05, (30, 2))
    b = np.random.default_rng(2).normal((2.0, 1.5), 0.05, (30, 2))
    noise = np.array([[4.0, -2.0]])
    groups = scene.clusters(np.vstack([a, b, noise]), min_points=3)
    assert sorted(len(g) for g in groups) == [30, 30]


def test_tracker_keeps_ids_velocity_and_labels():
    tr = scene.Tracker(min_hits=2, label_keep=2.0)
    obj = lambda x, label='': {'x': x, 'y': 0.0, 'size': [0.5, 0.5, 1.7], 'label': label, 'score': 0.9}
    assert tr.update([obj(1.0, 'person')], 0.0) == []
    out = tr.update([obj(1.1)], 0.2)
    assert len(out) == 1 and out[0].label == 'person'
    for i in range(2, 16):
        out = tr.update([obj(1.0 + 0.1 * i)], 0.2 * i)
    assert out[0].id == 1 and out[0].vx > 0.2
    # the label goes after label_keep seconds without a labelled detection
    assert out[0].label == ''
    assert tr.update([], 4.5) == []
