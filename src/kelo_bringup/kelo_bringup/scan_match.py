"""How well the LiDAR scan fits the map at a given pose. AMCL's covariance only says that the particles
agree with each other; in a room with similar corners they can agree on the wrong corner.

fit = share of the scan points on a wall of the map - share of the beams that pass through a wall of the
map (a beam cannot see through a wall: at a mirrored pose the walls fit, but the beams cross the
furniture that is on the other side). People and new objects only shorten beams: they lower the first
term a little and never count as a crossing.
"""
import math
import os

import cv2
import numpy as np
import yaml

OCCUPIED = 65          # OccupancyGrid value (0..100) from which a cell is a wall
HIT_TOL_M = 0.15       # a scan point within this distance of a wall is a hit
CROSS_MARGIN_M = 0.30  # a wall more than this in front of the measured end point is crossed
GOOD_FIT = 0.60        # localization is trusted from this fit on
LOST_FIT = 0.35        # below this the estimate is most likely wrong
MIN_POINTS = 30


class MapFit:
    """Distance to the nearest wall for every cell of an occupancy grid (values 0..100, -1 unknown,
    row 0 at origin y)."""

    def __init__(self, grid, resolution, origin):
        grid = np.asarray(grid)
        self.res = float(resolution)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.h, self.w = grid.shape
        walls = (grid >= OCCUPIED).astype(np.uint8)
        self.walls = walls.astype(bool)
        # distanceTransform measures the distance to the nearest zero pixel: walls -> 0
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
        """Share of the points (N x 2, map frame) within `tol` of a wall; points outside the map miss."""
        row, col, inside = self._cells(points)
        return float((inside & (self.dist[row, col] <= tol)).mean())

    def crossings(self, origin, points, margin=CROSS_MARGIN_M):
        """Share of the beams from `origin` (laser, map frame) to the points that pass through a wall
        more than `margin` before their end point."""
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
        """(hits, crossings) of the end points (N x 2, map frame) seen from the laser at `origin`
        (crossings 0 without it); None with too few points."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts) < MIN_POINTS:
            return None
        return self.hits(pts, tol), (self.crossings(origin, pts) if origin is not None else 0.0)

    def fit(self, points, origin=None, tol=HIT_TOL_M):
        """Scan fit: hits - crossings. None with too few points."""
        p = self.parts(points, origin, tol)
        return None if p is None else p[0] - p[1]


def looks_lost(hits, crossings):
    """Low fit AND evidence against the pose: beams through walls, or almost nothing on a wall. People
    standing around the robot lower the hits but cross no wall: not lost."""
    return hits - crossings < LOST_FIT and (crossings >= 0.05 or hits < 0.05)


def scan_points(ranges, angle_min, angle_inc, range_min, range_max, max_use=5.5, step=2):
    """Scan endpoints (N x 2) in the laser frame; max-range and invalid readings dropped."""
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
    """Written atomically: the stack can be stopped at any moment."""
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'map': os.path.realpath(map_yaml) if map_yaml else '', 'x': round(float(x), 3),
                        'y': round(float(y), 3), 'yaw_deg': round(math.degrees(yaw), 2),
                        'fit': round(float(fit), 3)}, f)
    os.replace(tmp, path)


def load_last_pose(path, map_yaml):
    """(x, y, yaw) saved for this map, or None (no file, other map, broken file)."""
    try:
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f) or {}
        if map_yaml and d.get('map') and d['map'] != os.path.realpath(map_yaml):
            return None
        return float(d['x']), float(d['y']), math.radians(float(d['yaw_deg']))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None
