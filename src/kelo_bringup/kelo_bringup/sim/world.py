import math

import numpy as np

INF = float('inf')


class Box:
    def __init__(self, lo, hi, name=''):
        self.lo = np.asarray(lo, dtype=float)
        self.hi = np.asarray(hi, dtype=float)
        self.name = name


class Cylinder:
    def __init__(self, x, y, r, z1, z2, name='', path=None, speed=0.0, t0=0.0):
        self.x, self.y, self.r, self.z1, self.z2 = float(x), float(y), float(r), float(z1), float(z2)
        self.name = name
        self.path = np.asarray(path, dtype=float) if path is not None and len(path) >= 2 else None
        self.speed = float(speed)
        self.t0 = float(t0)

    def update(self, t):
        if self.path is None or self.speed <= 0.0:
            return
        seg = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        total = float(seg.sum())
        if total <= 0.0:
            return
        s = (self.speed * (t - self.t0)) % (2.0 * total)
        if s > total:
            s = 2.0 * total - s
        for i, length in enumerate(seg):
            if s <= length or i == len(seg) - 1:
                a = min(s / length, 1.0) if length > 0 else 0.0
                self.x, self.y = self.path[i] + a * (self.path[i + 1] - self.path[i])
                return
            s -= length


def _f(v):
    return [float(x) for x in v]


def load_world(cfg):
    room = cfg.get('room', {})
    w, h, wh = float(room.get('width', 8.0)), float(room.get('height', 6.0)), float(room.get('wall_height', 2.0))
    t = 0.1
    boxes = [Box((-t, -t, 0), (w + t, 0, wh), 'wall_s'), Box((-t, h, 0), (w + t, h + t, wh), 'wall_n'),
             Box((-t, 0, 0), (0, h, wh), 'wall_w'), Box((w, 0, 0), (w + t, h, wh), 'wall_e')]
    cyls = []
    for ob in cfg.get('obstacles', []) or []:
        boxes_c, cyls_c = obstacle_from_dict(ob)
        boxes += boxes_c
        cyls += cyls_c
    return boxes, cyls


def obstacle_from_dict(ob, t=0.0):
    kind = ob.get('type', 'box')
    name = ob.get('name', kind)
    z = _f(ob.get('z', [0.0, 1.0]))
    if kind == 'box':
        x1, y1, x2, y2 = _f(ob['xy'])
        return [Box((min(x1, x2), min(y1, y2), z[0]), (max(x1, x2), max(y1, y2), z[1]), name)], []
    if kind == 'cylinder':
        path = ob.get('path')
        x, y = _f(ob['center']) if 'center' in ob else _f(path[0])
        return [], [Cylinder(x, y, float(ob.get('r', 0.2)), z[0], z[1], name, path,
                             float(ob.get('speed', 0.0)), t)]
    if kind == 'table':
        x1, y1, x2, y2 = _f(ob['xy'])
        top = _f(ob.get('top', [0.70, 0.75]))
        leg = float(ob.get('leg_r', 0.02))
        lo_x, hi_x, lo_y, hi_y = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
        legs = [Cylinder(px, py, leg, 0.0, top[0], f'{name}_leg')
                for px in (lo_x + leg, hi_x - leg) for py in (lo_y + leg, hi_y - leg)]
        return [Box((lo_x, lo_y, top[0]), (hi_x, hi_y, top[1]), name)], legs
    raise ValueError(f'unknown obstacle type: {kind}')


def raycast(origin, dirs, boxes, cyls, floor=True):
    o = np.asarray(origin, dtype=float)
    d = np.asarray(dirs, dtype=float)
    best = np.full(len(d), INF)
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = np.where(np.abs(d) > 1e-12, 1.0 / d, np.copysign(1e12, d + 1e-30))
        if boxes:
            lo = np.stack([b.lo for b in boxes])
            hi = np.stack([b.hi for b in boxes])
            t1 = (lo[None, :, :] - o) * inv[:, None, :]
            t2 = (hi[None, :, :] - o) * inv[:, None, :]
            tmin = np.minimum(t1, t2).max(axis=2)
            tmax = np.maximum(t1, t2).min(axis=2)
            hit = (tmax >= np.maximum(tmin, 0.0))
            t = np.where(hit, np.where(tmin > 0.0, tmin, 0.0), INF)
            best = np.minimum(best, t.min(axis=1))
        for c in cyls:
            ox, oy = o[0] - c.x, o[1] - c.y
            a = d[:, 0] ** 2 + d[:, 1] ** 2
            b = 2.0 * (ox * d[:, 0] + oy * d[:, 1])
            cc = ox * ox + oy * oy - c.r * c.r
            disc = b * b - 4.0 * a * cc
            ok = (disc >= 0.0) & (a > 1e-12)
            sq = np.sqrt(np.where(ok, disc, 0.0))
            for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
                zt = o[2] + t * d[:, 2]
                good = ok & (t > 0.0) & (zt >= c.z1) & (zt <= c.z2)
                best = np.where(good & (t < best), t, best)
            for zc in (c.z1, c.z2):
                t = (zc - o[2]) * inv[:, 2]
                px, py = o[0] + t * d[:, 0] - c.x, o[1] + t * d[:, 1] - c.y
                good = (t > 0.0) & (px * px + py * py <= c.r * c.r)
                best = np.where(good & (t < best), t, best)
        if floor:
            t = -o[2] * inv[:, 2]
            good = (d[:, 2] < 0.0) & (t > 0.0)
            best = np.where(good & (t < best), t, best)
    return best


def rot_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


R_LINK_OPTICAL = rot_rpy(-math.pi / 2, 0.0, -math.pi / 2)


def sensor_pose(robot_xy_yaw, mount):
    x, y, yaw = robot_xy_yaw
    r_base = rot_rpy(0.0, 0.0, yaw)
    pos = np.array([x, y, 0.0]) + r_base @ np.array([mount['x'], mount['y'], mount['z']])
    return pos, r_base @ rot_rpy(mount['roll'], mount['pitch'], mount['yaw'])


def lidar_ranges(robot_pose, mount, boxes, cyls, angle_min, angle_max, n, range_max):
    pos, rot = sensor_pose(robot_pose, mount)
    ang = np.linspace(angle_min, angle_max, n)
    local = np.stack([np.cos(ang), np.sin(ang), np.zeros(n)], axis=1)
    dirs = local @ rot.T
    t = raycast(pos, dirs, boxes, cyls, floor=False)
    return np.where(t <= range_max, t, INF)


def camera_intrinsics(width, height, hfov_deg, vfov_deg):
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    return fx, fy, (width - 1) / 2.0, (height - 1) / 2.0


def depth_image(robot_pose, mount, boxes, cyls, width, height, intr, min_depth, max_depth):
    fx, fy, cx, cy = intr
    pos, rot = sensor_pose(robot_pose, mount)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    opt = np.stack([(u.ravel() - cx) / fx, (v.ravel() - cy) / fy, np.ones(u.size)], axis=1)
    dirs = opt @ (rot @ R_LINK_OPTICAL).T
    t = raycast(pos, dirs, boxes, cyls, floor=True)
    t = np.where((t >= min_depth) & (t <= max_depth), t, 0.0)
    return t.reshape(height, width)


PALETTE = np.array([[200, 120, 60], [70, 140, 200], [190, 170, 80], [140, 90, 160], [90, 160, 110],
                    [200, 90, 90]], dtype=float)


def color_image(robot_pose, mount, boxes, cyls, width, height, intr, offset_y=0.0):
    """Simple colour camera (rgb8): checkered floor, light walls, one colour per obstacle, people blue,
    darker with distance. Only for the parking camera view: no sensor model behind it."""
    fx, fy, cx, cy = intr
    pos, rot = sensor_pose(robot_pose, mount)
    pos = pos + rot @ np.array([0.0, offset_y, 0.0])
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    opt = np.stack([(u.ravel() - cx) / fx, (v.ravel() - cy) / fy, np.ones(u.size)], axis=1)
    dirs = opt @ (rot @ R_LINK_OPTICAL).T
    t = raycast(pos, dirs, boxes, cyls, floor=True)
    hit = np.isfinite(t)
    p = pos + np.where(hit, t, 0.0)[:, None] * dirs
    rgb = np.tile(np.array([45.0, 45.0, 50.0]), (len(t), 1))
    eps = 1e-3
    obj = np.full(len(t), -1)
    for i, b in enumerate(boxes):
        inside = hit & (obj < 0) & np.all((p >= b.lo - eps) & (p <= b.hi + eps), axis=1)
        obj[inside] = i
        rgb[inside] = [215, 210, 195] if b.name.startswith('wall') else PALETTE[i % len(PALETTE)]
    for c in cyls:
        on = hit & (obj < 0) & (np.hypot(p[:, 0] - c.x, p[:, 1] - c.y) <= c.r + eps) & \
            (p[:, 2] >= c.z1 - eps) & (p[:, 2] <= c.z2 + eps)
        obj[on] = len(boxes)
        rgb[on] = [60, 90, 200]
    floor = hit & (obj < 0) & (p[:, 2] < 0.01)
    tile = (np.floor(p[:, 0] / 0.5) + np.floor(p[:, 1] / 0.5)) % 2 == 0
    rgb[floor] = np.where(tile[floor, None], [170, 165, 150], [140, 135, 120])
    shade = np.clip(1.15 - 0.08 * np.where(hit, t, 0.0), 0.45, 1.0)
    return np.clip(rgb * shade[:, None], 0, 255).astype(np.uint8).reshape(height, width, 3)


def _poly_world(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(points, dtype=float)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], axis=1)


def _sat_overlap(a, b):
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def _circle_poly_overlap(cx, cy, r, poly):
    p = np.array([cx, cy])
    inside = True
    best = INF
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ab = b - a
        tt = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(a + tt * ab - p)))
        if ab[0] * (p[1] - a[1]) - ab[1] * (p[0] - a[0]) < 0:
            inside = False
    return inside or best <= r


def collides(footprint, pose, robot_height, boxes, cyls):
    fp = _poly_world(footprint, pose)
    for b in boxes:
        if b.lo[2] >= robot_height:
            continue
        poly = np.array([[b.lo[0], b.lo[1]], [b.hi[0], b.lo[1]], [b.hi[0], b.hi[1]], [b.lo[0], b.hi[1]]])
        if _sat_overlap(fp, poly):
            return b.name or 'box'
    for c in cyls:
        if c.z1 < robot_height and _circle_poly_overlap(c.x, c.y, c.r, fp):
            return c.name or 'cylinder'
    return None
