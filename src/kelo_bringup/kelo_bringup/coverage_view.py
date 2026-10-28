"""Mapping coverage: what the map already has and where to drive next (/coverage_view/image, shown in the
KELO Surround window under 'Coverage', and in the web view).

Top view of the map being built (slam_toolbox /map and the camera layer /map_camera):
- green: free floor seen by the LiDAR and the camera (the camera layer has low obstacles there);
- amber: free for the LiDAR only, the camera never looked there (low objects may be missing);
- grey-blue: seen only from further than `near` metres (the LiDAR is less exact there, the camera does
  not reach): drive closer;
- orange: frontiers, where free floor meets unexplored space; the nearest one is marked as the next
  target with its distance;
- the robot's track and pose.
The status lines give the mapped area, the camera coverage, the share seen only from far and the
frontiers. Display only. Rendered only while someone watches.
"""
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup.common import load_hardware
from kelo_bringup.logic import yaw_from_quat

BG, WALL = (30, 32, 38), (235, 236, 240)
BOTH, LIDAR_ONLY, FAR = (60, 150, 90), (200, 160, 60), (80, 95, 120)
FRONTIER, TRACK, ROBOT, WHITE = (255, 140, 0), (60, 150, 255), (255, 255, 255), (255, 255, 255)


def grid_of(msg):
    info = msg.info
    return np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width), info


def cell_centres(info):
    """World x, y of every cell of a grid (rows x cols each)."""
    o = info.origin
    yaw = yaw_from_quat(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
    j, i = np.meshgrid(np.arange(info.width), np.arange(info.height))
    lx, ly = (j + 0.5) * info.resolution, (i + 0.5) * info.resolution
    c, s = math.cos(yaw), math.sin(yaw)
    return o.position.x + c * lx - s * ly, o.position.y + s * lx + c * ly


def to_cells(x, y, info):
    """World points -> (row, col) int arrays and a mask of those inside the grid."""
    o = info.origin
    yaw = yaw_from_quat(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = np.asarray(x, dtype=float) - o.position.x, np.asarray(y, dtype=float) - o.position.y
    j = np.floor((c * dx + s * dy) / info.resolution).astype(int)
    i = np.floor((-s * dx + c * dy) / info.resolution).astype(int)
    return i, j, (i >= 0) & (i < info.height) & (j >= 0) & (j < info.width)


def resample(src, src_info, dst_info):
    """Values of grid `src` at the cell centres of another grid (-1 outside)."""
    x, y = cell_centres(dst_info)
    i, j, ok = to_cells(x, y, src_info)
    out = np.full((dst_info.height, dst_info.width), -1, np.int8)
    out[ok] = src[i[ok], j[ok]]
    return out


def frontiers(grid, min_cells=6):
    """Free cells (0..49) next to unknown cells (-1), grouped: [(row, col centroid, cell count)] and the
    mask of all frontier cells of groups with at least `min_cells` cells."""
    free = (grid >= 0) & (grid < 50)
    unknown = grid < 0
    pad = np.pad(unknown, 1)
    near_unknown = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
    edge = (free & near_unknown).astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(edge, connectivity=8)
    out, mask = [], np.zeros(grid.shape, bool)
    for k in range(1, n):
        if stats[k, cv2.CC_STAT_AREA] >= min_cells:
            out.append((float(cents[k][1]), float(cents[k][0]), int(stats[k, cv2.CC_STAT_AREA])))
            mask |= labels == k
    return out, mask


def next_target(fronts, x, y, pose, min_cells):
    """(distance, row, col) of the frontier to explore next: the nearest one with at least
    `min_cells` cells (a small gap near the robot is not worth the drive), else the nearest."""
    def dist(r, c):
        return math.hypot(x[int(r), int(c)] - pose[0], y[int(r), int(c)] - pose[1])
    big = [(dist(r, c), r, c) for r, c, n in fronts if n >= min_cells]
    return min(big or [(dist(r, c), r, c) for r, c, _ in fronts])


def coverage(grid, camera, track_cells, res, near=3.0):
    """Classes of the free cells: (both, lidar_only, far) masks, from the LiDAR grid, the camera grid
    resampled onto it (or None) and the robot track as (row, col) cells."""
    free = (grid >= 0) & (grid < 50)
    seen_cam = camera >= 0 if camera is not None else np.zeros(grid.shape, bool)
    far = np.zeros(grid.shape, bool)
    if len(track_cells):
        src = np.full(grid.shape, 255, np.uint8)
        r, c = np.asarray(track_cells).T
        ok = (r >= 0) & (r < grid.shape[0]) & (c >= 0) & (c < grid.shape[1])
        src[r[ok], c[ok]] = 0
        dist = cv2.distanceTransform(src, cv2.DIST_L2, 5) * res
        far = free & (dist > near)
    both = free & seen_cam & ~far
    lidar_only = free & ~seen_cam & ~far
    return both, lidar_only, far


class CoverageView(Node):
    def __init__(self):
        super().__init__('coverage_view')
        hw = load_hardware()
        d = self.declare_parameter
        d('width', 960)
        d('height', 720)
        d('rate_hz', 2.0)
        d('near', 3.0)
        d('min_frontier', 0.3)
        g = lambda n: self.get_parameter(n).value
        self.fr = hw['frames']
        self.w, self.h = int(g('width')), int(g('height'))
        self.near, self.min_frontier = float(g('near')), float(g('min_frontier'))
        self.map = self.camera = None
        self.track = []
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), latched)
        self.create_subscription(OccupancyGrid, hw['topics']['camera_map'], lambda m: setattr(self, 'camera', m),
                                 latched)
        self.pub = self.create_publisher(Image, '~/image', 2)
        self.create_timer(0.2, self._follow)
        self.create_timer(1.0 / max(float(g('rate_hz')), 0.2), self._tick)

    def _pose(self):
        frame = (self.map.header.frame_id if self.map is not None else '') or self.fr['map']
        try:
            tf = self.buf.lookup_transform(frame, self.fr['base'], Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w)

    def _follow(self):
        """Robot track in the map frame, a point every 0.1 m."""
        p = self._pose()
        if p is not None and (not self.track or math.hypot(p[0] - self.track[-1][0], p[1] - self.track[-1][1]) > 0.1):
            self.track.append((p[0], p[1]))

    def _tick(self):
        if self.map is None or self.pub.get_subscription_count() == 0:
            return
        grid, info = grid_of(self.map)
        cam = None
        if self.camera is not None:
            cg, ci = grid_of(self.camera)
            cam = resample(cg, ci, info)
        tr = np.asarray(self.track) if self.track else np.zeros((0, 2))
        ti, tj, ok = to_cells(tr[:, 0], tr[:, 1], info)
        cells = np.stack([ti[ok], tj[ok]], axis=1)
        both, lidar_only, far = coverage(grid, cam, cells, info.resolution, self.near)
        fronts, fmask = frontiers(grid, max(2, int(self.min_frontier / info.resolution)))
        img = np.zeros((info.height, info.width, 3), np.uint8)
        img[:] = BG
        img[both], img[lidar_only], img[far] = BOTH, LIDAR_ONLY, FAR
        img[grid >= 50] = WALL
        img[fmask] = FRONTIER
        pose = self._pose()
        out, to_px = self._fit(img, grid)
        if len(cells) > 1:
            pts = np.array([to_px(i, j) for i, j in cells], np.int32)
            cv2.polylines(out, [pts], False, TRACK, 2, cv2.LINE_AA)
        target = None
        if pose is not None:
            pi, pj, inside = to_cells([pose[0]], [pose[1]], info)
            if inside[0]:
                self._robot(out, to_px(pi[0], pj[0]), pose[2])
            if fronts:
                x, y = cell_centres(info)
                target = next_target(fronts, x, y, pose, int(0.6 / info.resolution))
                u, v = to_px(target[1], target[2])
                cv2.circle(out, (u, v), 14, WHITE, 2, cv2.LINE_AA)
                cv2.circle(out, (u, v), 4, WHITE, -1, cv2.LINE_AA)
                cv2.putText(out, f'next {target[0]:.1f} m', (u + 18, v + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            WHITE, 2, cv2.LINE_AA)
        self._stats(out, both, lidar_only, far, fronts, info.resolution, target)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map.header.frame_id or self.fr['map']
        msg.height, msg.width = out.shape[:2]
        msg.encoding, msg.is_bigendian, msg.step = 'rgb8', 0, out.shape[1] * 3
        msg.data = out.tobytes()
        self.pub.publish(msg)

    def _fit(self, img, grid):
        """Known part of the map scaled into the output (map +y up), and cell -> pixel."""
        known = np.argwhere(grid >= 0)
        top = 70
        if len(known) == 0:
            r0, c0, r1, c1 = 0, 0, grid.shape[0], grid.shape[1]
        else:
            (r0, c0), (r1, c1) = known.min(axis=0) - 10, known.max(axis=0) + 11
            r0, c0 = max(r0, 0), max(c0, 0)
        crop = img[r0:r1, c0:c1][::-1]
        bottom = 34
        s = min(self.w / float(crop.shape[1]), (self.h - top - bottom) / float(crop.shape[0]))
        sw, sh = max(1, int(crop.shape[1] * s)), max(1, int(crop.shape[0] * s))
        out = np.zeros((self.h, self.w, 3), np.uint8)
        out[:] = BG
        ox, oy = (self.w - sw) // 2, top + (self.h - top - bottom - sh) // 2
        out[oy:oy + sh, ox:ox + sw] = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_NEAREST)
        rows = r1 - r0

        def to_px(i, j):
            return int(ox + (j - c0 + 0.5) * s), int(oy + (rows - (i - r0) - 0.5) * s)
        return out, to_px

    @staticmethod
    def _robot(img, p, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        tip = (p[0] + int(16 * c), p[1] - int(16 * s))
        left = (p[0] + int(-9 * c - 8 * s), p[1] - int(-9 * s + 8 * c))
        right = (p[0] + int(-9 * c + 8 * s), p[1] - int(-9 * s - 8 * c))
        cv2.fillConvexPoly(img, np.array([tip, left, right], np.int32), ROBOT, cv2.LINE_AA)

    def _stats(self, img, both, lidar_only, far, fronts, res, target):
        cv2.rectangle(img, (0, 0), (self.w, 64), (15, 16, 20), -1)
        cell = res * res
        free = both.sum() + lidar_only.sum() + far.sum()
        area = free * cell
        pct = lambda m: 100.0 * m.sum() / free if free else 0.0
        line1 = f'MAPPED {area:.0f} m2    camera {pct(both):.0f}%    LiDAR only {pct(lidar_only):.0f}%    far {pct(far):.0f}%'
        n_front = len(fronts)
        line2 = (f'{n_front} frontier{"s" if n_front != 1 else ""}'
                 + (f' - next {target[0]:.1f} m' if target else '') if n_front else 'no frontiers: the area is closed')
        cv2.putText(img, line1, (14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv2.LINE_AA)
        cv2.putText(img, line2, (14, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, FRONTIER if n_front else BOTH, 2, cv2.LINE_AA)
        # legend in the bottom bar, below the map
        cv2.rectangle(img, (0, self.h - 30), (self.w, self.h), (15, 16, 20), -1)
        x = 14
        for color, name in ((BOTH, 'LiDAR + camera'), (LIDAR_ONLY, 'LiDAR only'), (FAR, 'only from far'),
                            (FRONTIER, 'frontier'), (TRACK, 'track')):
            cv2.rectangle(img, (x, self.h - 21), (x + 14, self.h - 9), color, -1)
            cv2.putText(img, name, (x + 20, self.h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1, cv2.LINE_AA)
            x += 34 + cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0]


def main():
    rclpy.init()
    node = CoverageView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
