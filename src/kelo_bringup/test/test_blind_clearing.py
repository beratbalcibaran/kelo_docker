import math

import numpy as np

from kelo_bringup.logic import blind_wedge_points, scan_sees_past

W, H = 106, 60
FX = (W / 2) / math.tan(math.radians(87 / 2))
FY = (H / 2) / math.tan(math.radians(58 / 2))
K = (FX, FY, W / 2, H / 2)
P = math.radians(10)
ROT = np.array([[math.cos(P), 0, math.sin(P)], [0, 1, 0], [-math.sin(P), 0, math.cos(P)]]) @ \
    np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
TRANS = np.array([0.42, 0.0, 0.68])


def test_wedge_points_lie_under_the_image_and_in_front_of_the_robot():
    pts = blind_wedge_points(K, W, H, ROT, TRANS, 0.26, 0.45)
    assert len(pts) > 20
    assert np.all(pts[:, 2] == 0.26)
    assert np.all(pts[:, 0] >= 0.45)
    cam = (pts - TRANS) @ ROT
    v = FY * cam[:, 1] / cam[:, 2] + H / 2
    assert np.all(v > H - 1)
    assert pts[:, 0].max() < 0.42 + 0.42 / math.tan(math.radians(39)) + 1e-6


def test_upper_wedge_lies_over_the_image():
    pts = blind_wedge_points(K, W, H, ROT, TRANS, 1.20, 0.45)
    assert len(pts) > 20
    assert np.all(pts[:, 2] == 1.20) and np.all(pts[:, 0] >= 0.45)
    cam = (pts - TRANS) @ ROT
    assert np.all(FY * cam[:, 1] / cam[:, 2] + H / 2 < 0)
    assert pts[:, 0].max() < 0.42 + 0.52 / math.tan(math.radians(19)) + 1e-6
    assert len(blind_wedge_points(K, W, H, ROT, TRANS, 0.68, 0.45)) == 0


def test_scan_sees_past_only_where_the_beam_is_longer():
    angles = np.linspace(-math.pi, math.pi, 361)
    ranges = np.full(361, 4.0)
    ranges[np.abs(angles) < math.radians(5)] = 0.5
    ranges[np.abs(angles - math.radians(30)) < math.radians(2)] = float('inf')
    ranges[np.abs(angles + math.radians(30)) < math.radians(2)] = float('nan')
    laser = (0.45, 0.0)
    pts = np.array([[0.45 + 0.8, 0.0],
                    [0.45 + 0.3, 0.0],
                    [0.45 + 0.8 * math.cos(math.radians(60)), 0.8 * math.sin(math.radians(60))],
                    [0.45 + math.cos(math.radians(30)), math.sin(math.radians(30))],
                    [0.45 + math.cos(math.radians(30)), -math.sin(math.radians(30))]])
    free = scan_sees_past(pts, laser, 0.0, angles[0], angles[1] - angles[0], ranges, 0.05)
    assert free.tolist() == [False, True, True, True, False]
    assert not scan_sees_past(pts, laser, 0.0, angles[0], angles[1] - angles[0], [], 0.05).any()
