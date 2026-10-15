import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def kabsch_2d(src, dst):
    p = np.asarray(src, dtype=float)
    q = np.asarray(dst, dtype=float)
    if p.shape != q.shape or p.ndim != 2 or p.shape[1] != 2 or len(p) < 2:
        raise ValueError('src/dst must be (N,2) with N>=2')
    cp, cq = p.mean(axis=0), q.mean(axis=0)
    h = (p - cp).T @ (q - cq)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    if d == 0:
        d = 1.0
    rot = vt.T @ np.diag([1.0, d]) @ u.T
    trans = cq - rot @ cp
    errors = np.linalg.norm((p @ rot.T + trans) - q, axis=1)
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return yaw, float(trans[0]), float(trans[1]), errors


def apply_se2(yaw, tx, ty, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y + tx, s * x + c * y + ty


def path_length(points):
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


class Debounce:
    def __init__(self, on_delay, off_delay):
        self.on_delay = on_delay
        self.off_delay = off_delay
        self.active = False
        self._since = None

    def update(self, cond, now):
        target = bool(cond)
        if target == self.active:
            self._since = None
            return self.active
        if self._since is None:
            self._since = now
        delay = self.on_delay if target else self.off_delay
        if now - self._since >= delay:
            self.active = target
            self._since = None
        return self.active

    def reset(self):
        self.active = False
        self._since = None


def select_command(now, teleop, nav, stop, cmd_timeout, stop_timeout,
                   max_vx, max_vy, max_wz):
    if stop is not None and stop[1] and (now - stop[0]) <= stop_timeout:
        return 0.0, 0.0, 0.0, 'SAFETY_STOP'
    for name, cmd in (('TELEOP', teleop), ('NAV', nav)):
        if cmd is not None and (now - cmd[0]) <= cmd_timeout:
            return (clamp(cmd[1], max_vx), clamp(cmd[2], max_vy),
                    clamp(cmd[3], max_wz), name)
    return 0.0, 0.0, 0.0, 'IDLE'


def table_value(x, xs, ys):
    return float(np.interp(abs(float(x)), xs, ys))


def odom_yaw_rate(wz_true, yaw_k):
    return wz_true / table_value(wz_true, yaw_k['wz'], yaw_k['k'])


def odom_lin_speed(v_true, linear_error):
    return v_true * (1.0 + table_value(v_true, linear_error['v'], linear_error['e']))


def inverse_of(forward, x_max=10.0, n=4001):
    grid = np.linspace(0.0, x_max, n)
    vals = np.array([forward(g) for g in grid])
    if np.any(np.diff(vals) <= 0):
        raise ValueError('odometry table is not monotonic: cannot be inverted')
    return lambda y: math.copysign(float(np.interp(abs(y), vals, grid)), y)


def yaw_rate_corrector(yaw_k):
    return inverse_of(lambda w: odom_yaw_rate(w, yaw_k))


def lin_speed_corrector(linear_error):
    return inverse_of(lambda v: odom_lin_speed(v, linear_error), x_max=3.0)


def true_yaw_rate(wz_odom, yaw_k):
    return yaw_rate_corrector(yaw_k)(wz_odom)


def true_lin_speed(v_odom, linear_error):
    return lin_speed_corrector(linear_error)(v_odom)


def polygon_area(points):
    pts = np.asarray(points, dtype=float)
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def convex_contains(outer, inner, tol=1e-9):
    out = np.asarray(outer, dtype=float)
    if polygon_area(out) < 0:
        out = out[::-1]
    for px, py in np.asarray(inner, dtype=float):
        for (x1, y1), (x2, y2) in zip(out, np.roll(out, -1, axis=0)):
            if (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) < -tol:
                return False
    return True


def polygon_front(points):
    return float(np.asarray(points, dtype=float)[:, 0].max())


def depth_to_points(depth, k, stride, range_lim, height_lim, rot, trans):
    fx, fy, cx, cy = k
    d = np.asarray(depth, dtype=np.float32)[::stride, ::stride]
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    u, v = u * stride, v * stride
    z = d.ravel()
    x = (u.ravel() - cx) / fx * z
    y = (v.ravel() - cy) / fy * z
    pts = np.stack([x, y, z], axis=1)
    rng = np.linalg.norm(pts, axis=1)
    pts = pts[(z > 0.0) & (rng >= range_lim[0]) & (rng <= range_lim[1])]
    base = pts @ np.asarray(rot).T + np.asarray(trans)
    keep = (base[:, 2] >= height_lim[0]) & (base[:, 2] <= height_lim[1])
    return base[keep]
