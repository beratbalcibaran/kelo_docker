import math

import cv2
import numpy as np


def predict_poses(vx, vy, wz, length, ds=0.05, min_speed=0.03, max_turn=2.0):
    speed = math.hypot(vx, vy)
    if speed < min_speed or vx < -min_speed:
        s = np.arange(0.0, length + 1e-9, ds)
        return np.stack([s, np.zeros_like(s), np.zeros_like(s)], axis=1), s
    total = min(length, speed * max_turn / abs(wz)) if abs(wz) > 1e-6 else length
    s = np.arange(0.0, total + 1e-9, ds)
    t = s / speed
    yaw = wz * t
    if abs(wz) < 1e-6:
        x, y = vx * t, vy * t
    else:
        sw, cw = np.sin(yaw), np.cos(yaw)
        x = (vx * sw - vy * (1.0 - cw)) / wz
        y = (vx * (1.0 - cw) + vy * sw) / wz
    return np.stack([x, y, yaw], axis=1), s


def to_pose_frames(points_xy, poses):
    p = np.asarray(points_xy, dtype=float)
    dx = p[None, :, 0] - poses[:, None, 0]
    dy = p[None, :, 1] - poses[:, None, 1]
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=-1)


def sweep_distance(points_xy, poses, s, front, rear, half_width):
    p = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    out = np.full(len(p), np.inf)
    if len(p) == 0 or len(poses) == 0:
        return out
    for i0 in range(0, len(p), 20000):
        loc = to_pose_frames(p[i0:i0 + 20000], poses)
        inside = (loc[..., 0] <= front) & (loc[..., 0] >= -rear) & (np.abs(loc[..., 1]) <= half_width)
        hit = inside.any(axis=0)
        first = np.argmax(inside, axis=0)
        idx = np.arange(loc.shape[1])
        d = s[first] - (front - loc[first, idx, 0])
        out[i0:i0 + 20000] = np.where(hit, np.maximum(d, 0.0), np.inf)
    return out


def edge_tracks(poses, front, half_width):
    out = []
    for side in (half_width, -half_width):
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        out.append(np.stack([poses[:, 0] + c * front - s * side, poses[:, 1] + s * front + c * side], axis=1))
    return out


def to_base(points_xy, tf_xy_yaw):
    x, y, yaw = tf_xy_yaw
    p = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[:, 0] - x, p[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def from_base(points_xy, tf_xy_yaw):
    x, y, yaw = tf_xy_yaw
    p = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], axis=1)


def remaining_path(path_xy, max_length=None):
    p = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    if len(p) < 2:
        return p, 0.0
    i0 = int(np.argmin(np.hypot(p[:, 0], p[:, 1])))
    p = p[i0:]
    seg = np.hypot(*np.diff(p, axis=0).T)
    total = float(seg.sum())
    if max_length is not None and total > max_length:
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        k = int(np.searchsorted(cum, max_length))
        a = (max_length - cum[k - 1]) / max(seg[k - 1], 1e-9)
        p = np.vstack([p[:k], p[k - 1] + a * (p[k] - p[k - 1])])
    return p, total


def resample(path_xy, step):
    p = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    if len(p) < 2:
        return p, np.zeros(len(p))
    seg = np.hypot(*np.diff(p, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.arange(0.0, cum[-1] + 1e-9, step)
    if cum[-1] - s[-1] > 1e-6:
        s = np.append(s, cum[-1])
    return np.stack([np.interp(s, cum, p[:, 0]), np.interp(s, cum, p[:, 1])], axis=1), s


def ribbon(path_xy, half_width):
    p = np.asarray(path_xy, dtype=float)
    d = np.gradient(p, axis=0) if len(p) > 1 else np.array([[1.0, 0.0]])
    n = np.stack([-d[:, 1], d[:, 0]], axis=1)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    return p + half_width * n, p - half_width * n


def chevrons(path_xy, s, spacing, phase, size):
    p = np.asarray(path_xy, dtype=float)
    if len(p) < 2:
        return []
    out = []
    for d in np.arange(phase % spacing, s[-1] - size, spacing):
        x, y = np.interp(d, s, p[:, 0]), np.interp(d, s, p[:, 1])
        x2, y2 = np.interp(d + size, s, p[:, 0]), np.interp(d + size, s, p[:, 1])
        h = math.atan2(y2 - y, x2 - x)
        c, sn = math.cos(h), math.sin(h)
        tip = (x + c * size, y + sn * size)
        left = (x - sn * size * 0.8, y + c * size * 0.8)
        right = (x + sn * size * 0.8, y - c * size * 0.8)
        out.append((left, tip, right))
    return out


def footprint_distance(points_xy, front, rear, half_width):
    p = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    dx = np.maximum(np.maximum(p[:, 0] - front, -rear - p[:, 0]), 0.0)
    dy = np.maximum(np.abs(p[:, 1]) - half_width, 0.0)
    return np.hypot(dx, dy), np.arctan2(p[:, 1], p[:, 0])


def sector_distances(points_xy, front, rear, half_width, sectors=16, max_dist=1.5):
    dist, ang = footprint_distance(points_xy, front, rear, half_width)
    width = 2.0 * math.pi / sectors
    idx = np.floor(((ang + width / 2.0) % (2.0 * math.pi)) / width).astype(int) % sectors
    best = np.full(sectors, np.inf)
    where = [None] * sectors
    p = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    for k in range(sectors):
        m = (idx == k) & (dist < max_dist)
        if m.any():
            j = int(np.argmin(np.where(m, dist, np.inf)))
            best[k], where[k] = float(dist[j]), (float(p[j, 0]), float(p[j, 1]))
    return best, where


def clusters(points_xy, cell=0.1, min_points=3, grow=1):
    p = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    if len(p) == 0:
        return []
    ij = np.floor(p / cell).astype(int)
    lo = ij.min(axis=0) - grow - 1
    ij -= lo
    shape = tuple(ij.max(axis=0) + grow + 2)
    grid = np.zeros(shape, np.uint8)
    grid[ij[:, 0], ij[:, 1]] = 1
    if grow > 0:
        grid = cv2.dilate(grid, np.ones((2 * grow + 1, 2 * grow + 1), np.uint8))
    n, labels = cv2.connectedComponents(grid, connectivity=8)
    lab = labels[ij[:, 0], ij[:, 1]]
    out = []
    for k in range(1, n):
        idx = np.nonzero(lab == k)[0]
        if len(idx) >= min_points:
            out.append(idx)
    return out


class Track:
    def __init__(self, tid, obj, t):
        self.id = tid
        self.x, self.y = obj['x'], obj['y']
        self.size = list(obj['size'])
        self.vx = self.vy = 0.0
        self.label, self.score, self.label_t = obj.get('label', ''), obj.get('score', 0.0), t
        if not self.label:
            self.label_t = -1e9
        self.hits, self.seen = 1, t


class Tracker:

    def __init__(self, gate=0.6, alpha=0.5, max_age=1.0, min_hits=2, label_keep=5.0):
        self.gate, self.alpha, self.max_age, self.min_hits, self.label_keep = gate, alpha, max_age, min_hits, label_keep
        self.tracks = []
        self.next_id = 1

    def update(self, objects, t):
        free = list(range(len(self.tracks)))
        for obj in sorted(objects, key=lambda o: -o.get('score', 0.0)):
            best, bd = None, self.gate
            for i in free:
                tr = self.tracks[i]
                dt = max(t - tr.seen, 0.0)
                d = math.hypot(obj['x'] - (tr.x + tr.vx * dt), obj['y'] - (tr.y + tr.vy * dt))
                if d < bd:
                    best, bd = i, d
            if best is None:
                self.tracks.append(Track(self.next_id, obj, t))
                self.next_id += 1
                continue
            free.remove(best)
            tr = self.tracks[best]
            dt = t - tr.seen
            a = self.alpha
            nx, ny = (1 - a) * tr.x + a * obj['x'], (1 - a) * tr.y + a * obj['y']
            if dt > 1e-3:
                tr.vx = 0.7 * tr.vx + 0.3 * (nx - tr.x) / dt
                tr.vy = 0.7 * tr.vy + 0.3 * (ny - tr.y) / dt
            tr.x, tr.y = nx, ny
            tr.size = [(1 - a) * u + a * v for u, v in zip(tr.size, obj['size'])]
            if obj.get('label'):
                tr.label, tr.score, tr.label_t = obj['label'], obj.get('score', 0.0), t
            tr.hits += 1
            tr.seen = t
        self.tracks = [tr for tr in self.tracks if t - tr.seen <= self.max_age]
        for tr in self.tracks:
            if tr.label and t - tr.label_t > self.label_keep:
                tr.label, tr.score = '', 0.0
        return [tr for tr in self.tracks if tr.hits >= self.min_hits]


def predict_track(x, y, vx, vy, horizon=3.0, dt=0.25, min_speed=0.15):
    if math.hypot(vx, vy) < min_speed or horizon <= 0.0:
        return np.array([[x, y]], dtype=float), np.zeros(1)
    t = np.arange(0.0, horizon + 1e-9, dt)
    return np.stack([x + vx * t, y + vy * t], axis=1), t


def people_zone(people, frame_xy_yaw, shape, res, core=0.55, social=1.2, lethal_s=1.5, horizon=3.0,
                dt=0.25, social_value=60, core_value=90):
    grid = np.full(shape, -1, np.int16)
    ox, oy, oyaw = frame_xy_yaw
    c, s = math.cos(oyaw), math.sin(oyaw)
    rcell = int(math.ceil(social / res)) + 1
    for px, py, vx, vy in people:
        pts, ts = predict_track(px, py, vx, vy, horizon, dt)
        for (qx, qy), t in zip(pts, ts):
            dx, dy = qx - ox, qy - oy
            gj, gi = (c * dx + s * dy) / res, (-s * dx + c * dy) / res
            i0, i1 = max(int(gi) - rcell, 0), min(int(gi) + rcell + 1, shape[0])
            j0, j1 = max(int(gj) - rcell, 0), min(int(gj) + rcell + 1, shape[1])
            if i0 >= i1 or j0 >= j1:
                continue
            ii, jj = np.mgrid[i0:i1, j0:j1]
            d = np.hypot(ii + 0.5 - gi, jj + 0.5 - gj) * res
            fall = np.clip((social - d) / max(social - core, 1e-6), 0.0, 1.0)
            weight = 1.0 if t <= lethal_s else max(0.0, 1.0 - (t - lethal_s) / max(horizon - lethal_s, 1e-6))
            val = np.where(d <= core, core_value if t <= lethal_s else int(social_value * weight),
                           np.round(1 + (social_value - 1) * fall * weight))
            val = np.where(d <= social, val, -1).astype(np.int16)
            cell = grid[i0:i1, j0:j1]
            np.maximum(cell, val, out=cell)
    return grid.astype(np.int8)


def path_blocked(path_xy, people, clearance, ahead, lethal_s=1.5, dt=0.25):
    p, _ = remaining_path(path_xy, ahead)
    if len(p) < 2 or not people:
        return False
    pts, _ = resample(p, 0.05)
    for px, py, vx, vy in people:
        track, _ = predict_track(px, py, vx, vy, lethal_s, dt)
        d = np.hypot(pts[:, None, 0] - track[None, :, 0], pts[:, None, 1] - track[None, :, 1])
        if d.min() <= clearance:
            return True
    return False


def speed_limit(dist, near=1.0, far=2.0, slow=40.0):
    if dist is None or dist >= far:
        return 100.0
    if dist <= near:
        return slow
    return slow + (100.0 - slow) * (dist - near) / (far - near)


def pixel_ray(u, v, k):
    fx, fy, cx, cy = k
    d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    return d / np.linalg.norm(d)


def ray_to_floor(origin, direction, max_dist=20.0):
    o, d = np.asarray(origin, dtype=float), np.asarray(direction, dtype=float)
    if d[2] >= -1e-6:
        return None
    t = -o[2] / d[2]
    if t <= 0.0 or t * np.linalg.norm(d) > max_dist:
        return None
    p = o + t * d
    return float(p[0]), float(p[1])


class ObstacleMemory:

    def __init__(self, cell=0.05, max_age=30.0, radius=6.0):
        self.cell, self.max_age, self.radius = cell, max_age, radius
        self.ij = np.zeros((0, 2), np.int64)
        self.t = np.zeros(0)
        self.z = np.zeros(0)
        self.cam = np.zeros(0, bool)

    def __len__(self):
        return len(self.t)

    def add(self, points_xyz, t, from_camera):
        p = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
        if len(p) == 0:
            return
        ij = np.floor(p[:, :2] / self.cell).astype(np.int64)
        self.ij = np.vstack([ij, self.ij])
        self.t = np.concatenate([np.full(len(p), float(t)), self.t])
        self.z = np.concatenate([p[:, 2], self.z])
        self.cam = np.concatenate([np.full(len(p), bool(from_camera)), self.cam])
        key = np.stack([self.ij[:, 0], self.ij[:, 1], self.cam.astype(np.int64)], axis=1)
        _, first = np.unique(key, axis=0, return_index=True)
        self._keep(np.sort(first))

    def _keep(self, idx):
        self.ij, self.t, self.z, self.cam = self.ij[idx], self.t[idx], self.z[idx], self.cam[idx]

    def centres(self):
        return (self.ij + 0.5) * self.cell

    def forget(self, t, pose, lidar=None, camera=None):
        if len(self.t) == 0:
            return
        keep = (t - self.t) <= self.max_age
        b = to_base(self.centres(), pose)
        keep &= np.hypot(b[:, 0], b[:, 1]) <= self.radius
        if lidar is not None:
            sx, sy, syaw, amin, ainc, ranges = lidar
            r = np.asarray(ranges, dtype=float)
            dx, dy = b[:, 0] - sx, b[:, 1] - sy
            ang = (np.arctan2(dy, dx) - syaw + math.pi) % (2 * math.pi) - math.pi
            k = np.round((ang - amin) / ainc).astype(int)
            inside = (k >= 0) & (k < len(r))
            beam = np.where(inside, r[np.clip(k, 0, len(r) - 1)], np.nan)
            beam = np.where(np.isfinite(beam), beam, np.inf)
            seen_through = inside & (np.hypot(dx, dy) < beam - 2.0 * self.cell)
            keep &= ~(seen_through & ~self.cam & (t - self.t > 1e-6))
        if camera is not None:
            sx, sy, syaw, half_fov, rmin, rmax, visible = camera
            dx, dy = b[:, 0] - sx, b[:, 1] - sy
            ang = np.abs((np.arctan2(dy, dx) - syaw + math.pi) % (2 * math.pi) - math.pi)
            rng = np.hypot(dx, dy)
            in_view = (ang <= half_fov) & (rng >= rmin) & (rng <= rmax)
            now = np.zeros(len(self.t), bool)
            if visible is not None and len(visible):
                seen = np.floor(np.asarray(visible, dtype=float)[:, :2] / self.cell).astype(np.int64)
                now = np.isin(_cell_key(self.ij), _cell_key(seen))
            keep &= ~(in_view & self.cam & ~now)
        self._keep(np.nonzero(keep)[0])


def _cell_key(ij):
    ij = np.asarray(ij, dtype=np.int64).reshape(-1, 2)
    return (ij[:, 0] << 32) ^ (ij[:, 1] & 0xFFFFFFFF)
