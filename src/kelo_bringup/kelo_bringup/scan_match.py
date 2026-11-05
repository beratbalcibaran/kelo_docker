import math
import os

import cv2
import numpy as np
import yaml

OCCUPIED = 65
HIT_TOL_M = 0.15
CROSS_MARGIN_M = 0.30
GOOD_FIT = 0.60
LOST_FIT = 0.35
MIN_POINTS = 30


class MapFit:

    def __init__(self, grid, resolution, origin):
        grid = np.asarray(grid)
        self.res = float(resolution)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.h, self.w = grid.shape
        walls = (grid >= OCCUPIED).astype(np.uint8)
        self.walls = walls.astype(bool)
        self.dist = cv2.distanceTransform(1 - walls, cv2.DIST_L2, 5) * self.res

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return cls(grid, info.resolution, (info.origin.position.x, info.origin.position.y))

    def _cells(self, pts):
        col = np.floor((pts[..., 0] - self.ox) / self.res).astype(int)
        row = np.floor((pts[..., 1] - self.oy) / self.res).astype(int)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        return row.clip(0, self.h - 1), col.clip(0, self.w - 1), inside

    def hits(self, points, tol=HIT_TOL_M):
        row, col, inside = self._cells(points)
        return float((inside & (self.dist[row, col] <= tol)).mean())

    def crossings(self, origin, points, margin=CROSS_MARGIN_M):
        d = points - np.asarray(origin, dtype=float)
        rng = np.hypot(d[:, 0], d[:, 1])
        s = np.arange(0.10, float(rng.max()), self.res * 0.5)
        if len(s) == 0:
            return 0.0
        samples = origin + (d / rng[:, None])[:, None, :] * s[None, :, None]
        row, col, inside = self._cells(samples)
        wall = inside & self.walls[row, col] & (s[None, :] < (rng - margin)[:, None])
        return float(wall.any(axis=1).mean())

    def parts(self, points, origin=None, tol=HIT_TOL_M):
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts) < MIN_POINTS:
            return None
        return self.hits(pts, tol), (self.crossings(origin, pts) if origin is not None else 0.0)

    def fit(self, points, origin=None, tol=HIT_TOL_M):
        p = self.parts(points, origin, tol)
        return None if p is None else p[0] - p[1]


def looks_lost(hits, crossings):
    return hits - crossings < LOST_FIT and (crossings >= 0.05 or hits < 0.05)


def scan_points(ranges, angle_min, angle_inc, range_min, range_max, max_use=5.5, step=2):
    r = np.asarray(ranges, dtype=float)[::step]
    a = angle_min + angle_inc * np.arange(0, len(ranges), step)[:len(r)]
    ok = np.isfinite(r) & (r > max(range_min, 0.05)) & (r < min(range_max, max_use) - 0.05)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    return np.column_stack([c * pts[:, 0] - s * pts[:, 1] + x, s * pts[:, 0] + c * pts[:, 1] + y])


def last_pose_file(data_dir):
    return os.path.join(data_dir, 'last_pose.yaml')


def save_last_pose(path, map_yaml, x, y, yaw, fit):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'map': os.path.realpath(map_yaml) if map_yaml else '', 'x': round(float(x), 3),
                        'y': round(float(y), 3), 'yaw_deg': round(math.degrees(yaw), 2),
                        'fit': round(float(fit), 3)}, f)
    os.replace(tmp, path)


def load_last_pose(path, map_yaml):
    try:
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f) or {}
        if map_yaml and d.get('map') and d['map'] != os.path.realpath(map_yaml):
            return None
        return float(d['x']), float(d['y']), math.radians(float(d['yaw_deg']))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None
