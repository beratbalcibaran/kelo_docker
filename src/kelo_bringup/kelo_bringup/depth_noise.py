import collections
import math

import numpy as np


def supported_pixels(d, rel_tol=0.05, abs_tol=0.03, min_neighbors=3):
    d = np.asarray(d, dtype=np.float32)
    valid = d > 0.0
    if d.ndim != 2 or min(d.shape) < 3:
        return valid
    pad = np.pad(d, 1, mode='constant', constant_values=0.0)
    tol = abs_tol + rel_tol * d
    count = np.zeros(d.shape, dtype=np.int16)
    h, w = d.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nb = pad[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
            count += ((nb > 0.0) & (np.abs(nb - d) <= tol)).astype(np.int16)
    return valid & (count >= min_neighbors)


def min_height_at(dist_xy, min_height, slope=0.0, start=1.0):
    return min_height + slope * np.maximum(0.0, np.asarray(dist_xy, dtype=float) - start)


def cell_keys(xy, cell):
    ij = np.floor(np.asarray(xy, dtype=float) / cell).astype(np.int64)
    return ij[:, 0] * 1_000_003 + ij[:, 1]


def cell_support(xy, cell=0.05, min_points=3):
    if len(xy) == 0 or min_points <= 1:
        return np.ones(len(xy), dtype=bool)
    keys = cell_keys(xy, cell)
    _, inv, counts = np.unique(keys, return_inverse=True, return_counts=True)
    return counts[inv] >= min_points


class Persistence:

    def __init__(self, n=3, k=2, cell=0.05):
        self.n, self.k, self.cell = max(1, int(n)), max(1, int(k)), float(cell)
        self.frames = collections.deque(maxlen=self.n)

    def reset(self):
        self.frames.clear()

    def step(self, xy_fixed):
        keys = cell_keys(xy_fixed, self.cell) if len(xy_fixed) else np.zeros(0, dtype=np.int64)
        uniq = np.unique(keys)
        self.frames.append(uniq)
        if self.k <= 1 or len(keys) == 0:
            return np.ones(len(keys), dtype=bool)
        seen = np.concatenate(list(self.frames))
        cells, counts = np.unique(seen, return_counts=True)
        good = cells[counts >= self.k]
        return np.isin(keys, good)


def depth_points(depth, k, stride, range_lim, rot, trans, denoise=True, **kw):
    fx, fy, cx, cy = k
    d = np.asarray(depth, dtype=np.float32)[::stride, ::stride]
    d = np.where(np.isfinite(d), d, 0.0)
    keep = supported_pixels(d, **kw) if denoise else d > 0.0
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    u, v = (u * stride)[keep], (v * stride)[keep]
    z = d[keep]
    pts = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=1)
    rng = np.linalg.norm(pts, axis=1)
    sel = (rng >= range_lim[0]) & (rng <= range_lim[1])
    base = pts[sel] @ np.asarray(rot).T + np.asarray(trans)
    dist = np.hypot(base[:, 0] - trans[0], base[:, 1] - trans[1])
    return base, dist


def rot_xy(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def fit_floor(points, cam_xy, z_band=0.12, near=0.6, far=2.0, iters=60, inlier=0.015, seed=0):
    p = np.asarray(points, dtype=float)
    if len(p) == 0:
        return None
    dist = np.hypot(p[:, 0] - cam_xy[0], p[:, 1] - cam_xy[1])
    p = p[(np.abs(p[:, 2]) <= z_band) & (dist >= near) & (dist <= far)]
    if len(p) < 50:
        return None
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[2] < 0:
            n = -n
        if n[2] < math.cos(math.radians(8.0)):
            continue
        cnt = int((np.abs((p - a) @ n) <= inlier).sum())
        if cnt > best_n:
            best, best_n = (n, a), cnt
    if best is None:
        return None
    n, a = best
    ins = p[np.abs((p - a) @ n) <= inlier]
    A = np.c_[ins[:, 0], ins[:, 1], np.ones(len(ins))]
    (ga, gb, gc), *_ = np.linalg.lstsq(A, ins[:, 2], rcond=None)
    pitch = math.atan(ga)
    roll = -math.atan(gb)
    return roll, pitch, float(gc), best_n / len(p), int(best_n)


class TiltEstimator:

    def __init__(self, need=15, max_deg=6.0, min_share=0.6, min_inliers=150):
        self.need, self.max = int(need), math.radians(max_deg)
        self.min_share, self.min_inliers = float(min_share), int(min_inliers)
        self.samples = []
        self.result = None

    def add(self, fit):
        if self.result is not None or fit is None:
            return self.result
        roll, pitch, _h, share, n = fit
        if share < self.min_share or n < self.min_inliers or max(abs(roll), abs(pitch)) > self.max:
            return None
        self.samples.append((roll, pitch))
        if len(self.samples) >= self.need:
            r, p = np.median(np.asarray(self.samples), axis=0)
            spread = np.ptp(np.asarray(self.samples), axis=0).max()
            if spread <= math.radians(1.0):
                self.result = (float(r), float(p))
            else:
                self.samples = self.samples[len(self.samples) // 2:]
        return self.result
