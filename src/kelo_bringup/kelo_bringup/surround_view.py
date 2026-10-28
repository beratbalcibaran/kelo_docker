"""Surround view like a Tesla's driving screen: the robot seen from behind and above, drawn in 3D with
what is around it (/surround_view/image, shown in the KELO Surround window of camera_view).

- the map walls (/map) as light blocks and a floor grid fixed to the map, so the floor moves under
  the robot;
- the live LiDAR scan as glowing points (red when close) and the camera obstacle cloud;
- the objects of the objects node: people as standing figures, other things as boxes, with their
  distance and an arrow when they move;
- park assist: around the footprint, the nearest obstacle in each direction as a coloured arc with
  its distance in cm (LiDAR + camera; behind the LiDAR's field of view the map walls are used and the
  arc is drawn dashed: that side has no live sensor);
- the guide lines of the commanded velocity and, while Nav2 drives to a goal, the plan as a blue road
  with flowing arrows and a pin on the goal;
- obstacles the sensors no longer see (obstacle memory of the objects node, e.g. behind the robot
  where the LiDAR does not look) as grey dots that fade with the time since they were seen; park
  assist uses them (dashed) in the blind directions;
- people: their personal space as a ring and where they walk in the next 3 s as footsteps;
- a status bar: speed, nearest obstacle, distance to the goal, and the people-aware navigation
  (WAITING FOR PERSON, speed limited near people).
Double click on the floor in the view (camera_view, web_view) sends the robot there; right click
cancels the goal. `view:=top` looks straight down instead. Rendered only while someone watches.

    ros2 run kelo_bringup surround_view --ros-args -p view:=chase
"""
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup import scene
from kelo_bringup.cloud_filter import quat_to_rot
from kelo_bringup.common import load_hardware
from kelo_bringup.drive_inputs import DriveInputs
from kelo_bringup.logic import yaw_from_quat
from kelo_bringup.objects import cloud_xyz

BG_TOP, BG_BOTTOM = np.array([28, 30, 36]), np.array([52, 56, 64])
GRID = (70, 76, 88)
WALL_TOP, WALL_SIDE = np.array([200, 204, 212]), np.array([140, 145, 156])
ROBOT_TOP, ROBOT_SIDE, ROBOT_EDGE = np.array([235, 236, 240]), np.array([150, 154, 165]), (90, 95, 110)
WHEEL = (30, 30, 34)
CHASSIS, LIGHT = np.array([62, 66, 76]), np.array([120, 210, 255])
PERSON, OBSTACLE = np.array([235, 235, 245]), np.array([255, 170, 60])
PATH_BLUE, ARROW, WHITE = (40, 140, 255), (200, 235, 255), (255, 255, 255)
RED, ORANGE, YELLOW, GREEN = (240, 50, 50), (255, 140, 0), (245, 210, 0), (60, 210, 90)


def assist_color(d):
    return RED if d < 0.3 else ORANGE if d < 0.6 else YELLOW if d < 1.0 else GREEN


class Camera:
    """Pinhole camera looking from `eye` at `target` (base frame, z up)."""

    def __init__(self, eye, target, width, height, fov_deg):
        self.eye = np.asarray(eye, dtype=float)
        f = np.asarray(target, dtype=float) - self.eye
        f /= np.linalg.norm(f)
        up = np.array([0.0, 0.0, 1.0]) if abs(f[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
        r = np.cross(f, up)
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        self.rot = np.stack([r, -u, f])  # rows: image x, image y (down), depth
        self.w, self.h = width, height
        self.f = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)

    def project(self, pts):
        """N x 3 points -> pixels (N x 2 float), depth (N) and a mask of the points in front."""
        c = (np.asarray(pts, dtype=float).reshape(-1, 3) - self.eye) @ self.rot.T
        ok = c[:, 2] > 0.1
        z = np.where(ok, c[:, 2], 1.0)
        uv = np.stack([self.f * c[:, 0] / z + self.w / 2.0, self.f * c[:, 1] / z + self.h / 2.0], axis=1)
        return uv, c[:, 2], ok & (np.abs(uv) < 1e5).all(axis=1)

    def ray(self, u, v):
        """Direction (base frame) of the ray through pixel (u, v)."""
        d = self.rot.T @ np.array([(u - self.w / 2.0) / self.f, (v - self.h / 2.0) / self.f, 1.0])
        return d / np.linalg.norm(d)

    def scale_at(self, depth):
        """Pixels per metre at a depth."""
        return self.f / max(depth, 0.1)


def box_faces(cx, cy, sx, sy, z0, z1, yaw=0.0):
    """Top and four side faces of a box (each 4 x 3 points) with outward normals."""
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = sx / 2.0, sy / 2.0
    corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    ring = [(cx + c * x - s * y, cy + s * x + c * y) for x, y in corners]
    faces = [(np.array([(x, y, z1) for x, y in ring]), np.array([0.0, 0.0, 1.0]))]
    for i in range(4):
        (ax, ay), (bx, by) = ring[i], ring[(i + 1) % 4]
        n = np.array([by - ay, -(bx - ax), 0.0])
        n /= max(np.linalg.norm(n), 1e-9)
        faces.append((np.array([(ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1)]), n))
    return faces


class Painter:
    """Polygons collected from the whole scene, drawn far to near (painter's algorithm)."""

    def __init__(self, cam):
        self.cam = cam
        self.items = []

    def face(self, pts, color, edge=None, normal=None, alpha=1.0):
        pts = np.asarray(pts, dtype=float)
        if normal is not None and np.dot(normal, self.cam.eye - pts.mean(axis=0)) <= 0.0:
            return  # facing away
        uv, z, ok = self.cam.project(pts)
        if not ok.all():
            return
        self.items.append((float(z.mean()), 'poly', np.round(uv).astype(np.int32), color, edge, alpha))

    def box(self, cx, cy, sx, sy, z0, z1, top, side, edge=None, yaw=0.0, alpha=1.0):
        for i, (f, n) in enumerate(box_faces(cx, cy, sx, sy, z0, z1, yaw)):
            shade = top if i == 0 else side * (0.85 + 0.15 * abs(n[0]))
            self.face(f, tuple(int(v) for v in np.clip(shade, 0, 255)), edge, n, alpha)

    def prism(self, x, y, r, z0, z1, top, side, n=10):
        """Upright n-sided prism (a round shape: body or head of a figure)."""
        a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        ring = np.stack([x + r * np.cos(a), y + r * np.sin(a)], axis=1)
        self.face([(px, py, z1) for px, py in ring], tuple(int(v) for v in np.clip(top, 0, 255)))
        for i in range(n):
            (ax, ay), (bx, by) = ring[i], ring[(i + 1) % n]
            mid = (a[i] + a[(i + 1) % n] + (2 * math.pi if i == n - 1 else 0.0)) / 2.0
            normal = np.array([math.cos(mid), math.sin(mid), 0.0])
            shade = side * (0.75 + 0.25 * max(0.0, normal[1] * 0.5 + 0.5))
            self.face([(ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1)],
                      tuple(int(v) for v in np.clip(shade, 0, 255)), None, normal)

    def disc(self, x, y, z, r, color, n=14):
        a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        self.face(np.stack([x + r * np.cos(a), y + r * np.sin(a), np.full(n, z)], axis=1), color)

    def draw(self, img):
        for _, kind, uv, color, edge, alpha in sorted(self.items, key=lambda it: -it[0]):
            if alpha >= 1.0:
                cv2.fillPoly(img, [uv], color, cv2.LINE_AA)
            else:
                layer = img.copy()
                cv2.fillPoly(layer, [uv], color, cv2.LINE_AA)
                cv2.addWeighted(layer, alpha, img, 1.0 - alpha, 0.0, dst=img)
            if edge is not None:
                cv2.polylines(img, [uv], True, edge, 1, cv2.LINE_AA)


def floor_line(img, cam, pts_xy, color, thick, z=0.0):
    pts = np.column_stack([np.asarray(pts_xy, dtype=float).reshape(-1, 2), np.full(len(pts_xy), z)])
    uv, _, ok = cam.project(pts)
    q = np.round(uv).astype(np.int32)
    for i in range(len(q) - 1):
        if ok[i] and ok[i + 1]:
            cv2.line(img, tuple(q[i]), tuple(q[i + 1]), color, thick, cv2.LINE_AA)


def floor_poly(img, cam, pts_xy, color, z=0.0):
    pts = np.column_stack([np.asarray(pts_xy, dtype=float).reshape(-1, 2), np.full(len(pts_xy), z)])
    uv, _, ok = cam.project(pts)
    if ok.all():
        cv2.fillPoly(img, [np.round(uv).astype(np.int32)], color, cv2.LINE_AA)


def text(img, org, s, color=WHITE, scale=0.55, thick=1, center=False):
    (w, h), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x, y = int(org[0] - (w / 2 if center else 0)), int(org[1])
    x = min(max(x, 4), img.shape[1] - w - 4)
    y = min(max(y, h + 4), img.shape[0] - 6)
    cv2.putText(img, s, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


class SurroundView(Node):
    def __init__(self):
        super().__init__('surround_view')
        hw = load_hardware()
        d = self.declare_parameter
        d('view', 'chase')
        d('width', 960)
        d('height', 720)
        d('rate_hz', 10.0)
        d('radius', 8.0)
        d('wall_height', 0.45)
        d('margin', 0.10)
        g = lambda n: self.get_parameter(n).value
        self.hw = hw
        self.fr = hw['frames']
        self.view = g('view')
        self.w, self.h = int(g('width')), int(g('height'))
        self.radius, self.wall_h = float(g('radius')), float(g('wall_height'))
        body = hw['body']
        self.half_l, self.half_w, self.body_h = body['length'] / 2.0, body['width'] / 2.0, body['height']
        self.guide_w = self.half_w + float(g('margin'))
        lid = hw['lidar']
        yaw = hw['mounts']['laser']['yaw']
        self.lidar_fov = (lid['angle_min'] + yaw, lid['angle_max'] + yaw)
        self.scan = self.cloud = None
        self.walls = None
        self.assist_label = None
        self.tfs = {}
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.inputs = DriveInputs(self, hw, self.buf, memory=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        sd = qos_profile_sensor_data
        self.create_subscription(LaserScan, hw['topics']['scan'], lambda m: setattr(self, 'scan', (self._now(), m)), sd)
        self.create_subscription(PointCloud2, hw['topics']['camera_points'],
                                 lambda m: setattr(self, 'cloud', (self._now(), m)), sd)
        self.create_subscription(OccupancyGrid, '/map', self._map, latched)
        self.pub = self.create_publisher(Image, '~/image', 2)
        self.inputs.on_click(self._click)
        self.bg = self._background()
        self.create_timer(1.0 / max(float(g('rate_hz')), 0.5), self._tick)
        self.get_logger().info(f'surround_view: {self.view} view, {self.w}x{self.h}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _background(self):
        a = np.linspace(0.0, 1.0, self.h)[:, None, None]
        return np.repeat((BG_TOP * (1 - a) + BG_BOTTOM * a).astype(np.uint8), self.w, axis=1)

    def _camera(self):
        if self.view == 'top':
            return Camera((0.6, 0.0, 9.0), (0.6, 0.0, 0.0), self.w, self.h, 60.0)
        return Camera((-3.4, 0.0, 2.6), (1.6, 0.0, 0.0), self.w, self.h, 62.0)

    def _map(self, msg):
        """Occupied cells on a 0.1 m grid, as map-frame cell centres with their free neighbours."""
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        occ = (grid >= 50).astype(np.uint8)
        f = max(1, int(round(0.1 / info.resolution)))
        h, w = occ.shape[0] // f * f, occ.shape[1] // f * f
        occ = occ[:h, :w].reshape(h // f, f, w // f, f).max(axis=(1, 3))
        res = info.resolution * f
        i, j = np.nonzero(occ)
        o = info.origin
        oyaw = yaw_from_quat(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
        lx, ly = (j + 0.5) * res, (i + 0.5) * res
        c, s = math.cos(oyaw), math.sin(oyaw)
        xy = np.stack([o.position.x + c * lx - s * ly, o.position.y + s * lx + c * ly], axis=1)
        pad = np.pad(occ, 1)
        # neighbours along +x, -x, +y, -y in the grid: a side face is drawn only towards a free cell
        free = np.stack([pad[i + 1, j + 2] == 0, pad[i + 1, j] == 0, pad[i + 2, j + 1] == 0,
                         pad[i, j + 1] == 0], axis=1)
        self.walls = (msg.header.frame_id or self.fr['map'], xy, free, res, oyaw)

    def _static_tf(self, frame):
        if frame not in self.tfs:
            try:
                tf = self.buf.lookup_transform(self.fr['base'], frame, Time())
            except TransformException:
                return None
            tr = tf.transform.translation
            self.tfs[frame] = (quat_to_rot(tf.transform.rotation), np.array([tr.x, tr.y, tr.z]))
        return self.tfs[frame]

    def _scan_points(self, now):
        if self.scan is None or now - self.scan[0] > 0.5:
            return np.zeros((0, 3))
        msg = self.scan[1]
        tf = self._static_tf(msg.header.frame_id)
        if tf is None:
            return np.zeros((0, 3))
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.05)) & (r < min(msg.range_max, self.radius))
        p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())], axis=1)
        return p @ tf[0].T + tf[1]

    def _click(self, frame, u, v, is_goal):
        if frame != 'surround':
            return
        if not is_goal:
            self.inputs.cancel_goal()
            return
        cam = self._camera()
        p = scene.ray_to_floor(cam.eye, cam.ray(u, v), max_dist=40.0)
        if p is None:
            self.inputs.say('click on the floor to send a goal')
            return
        self.inputs.send_goal(*p)

    def _tick(self):
        if self.pub.get_subscription_count() == 0:
            return  # nobody watches: no rendering
        now = self._now()
        cam = self._camera()
        img = self.bg.copy()
        map_pose = self.inputs.pose_in(self.walls[0]) if self.walls is not None else None
        self._grid(img, cam, map_pose)
        lidar = self._scan_points(now)
        cloud = cloud_xyz(self.cloud[1]) if self.cloud is not None and now - self.cloud[0] < 0.5 else np.zeros((0, 3))
        walls_base = self._walls_near(map_pose)
        path, left = self.inputs.plan_in_base(12.0)
        if path is not None:
            self._path(img, cam, path, now)
        vx, vy, wz = self.inputs.twist()
        poses, s = scene.predict_poses(vx, vy, wz, 2.5)
        for tr in scene.edge_tracks(poses, self.half_l, self.guide_w):
            floor_line(img, cam, tr, (230, 230, 240), 2, z=0.01)
        memory = self.inputs.memory_in_base()
        objects = self.inputs.objects_in_base()
        self._people_ahead(img, cam, objects)
        nearest = self._park_assist(img, cam, lidar, cloud, walls_base, memory)
        painter = Painter(cam)
        self._walls(painter, walls_base)
        self._objects(painter, objects)
        self._robot(painter)
        if path is not None and left <= 12.0:
            self._pin(painter, path[-1])
        painter.draw(img)
        self._memory(img, cam, memory)
        self._points(img, cam, lidar, cloud)
        self._labels(img, cam, objects, path, left)
        self._hud(img, math.hypot(vx, vy), wz, nearest, path, left, len(objects))
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.fr['base']
        out.height, out.width = img.shape[:2]
        out.encoding, out.is_bigendian, out.step = 'rgb8', 0, img.shape[1] * 3
        out.data = img.tobytes()
        self.pub.publish(out)

    def _grid(self, img, cam, map_pose):
        """1 m floor grid fixed to the map (to odom without a map), so it slides when the robot moves."""
        pose = map_pose or self.inputs.pose_in(self.fr['odom'])
        if pose is None:
            return
        r = self.radius
        x0, y0 = math.floor(pose[0] - r), math.floor(pose[1] - r)
        for k in range(int(2 * r) + 2):
            for a, b in ((((x0 + k), pose[1] - r), ((x0 + k), pose[1] + r)),
                         ((pose[0] - r, (y0 + k)), (pose[0] + r, (y0 + k)))):
                seg = scene.to_base(np.linspace(a, b, 12), pose)
                floor_line(img, cam, seg[np.hypot(seg[:, 0], seg[:, 1]) <= r], GRID, 1)

    def _walls_near(self, pose):
        """Occupied map cells within the radius in the base frame, with their free sides turned into
        the base frame (+x, -x, +y, -y of the map grid -> outward normals)."""
        if pose is None or self.walls is None:
            return None
        _, xy, free, res, oyaw = self.walls
        b = scene.to_base(xy, pose)
        keep = np.hypot(b[:, 0], b[:, 1]) <= self.radius
        return b[keep], free[keep], res, oyaw - pose[2]

    def _walls(self, painter, walls):
        if walls is None:
            return
        xy, free, res, yaw = walls
        c, s = math.cos(yaw), math.sin(yaw)
        normals = [np.array([c, s, 0.0]), np.array([-c, -s, 0.0]), np.array([-s, c, 0.0]), np.array([s, -c, 0.0])]
        h = res / 2.0
        top = tuple(int(v) for v in WALL_TOP)
        for (x, y), fr in zip(xy, free):
            corners = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((h, h), (-h, h), (-h, -h), (h, -h))]
            painter.face([(px, py, self.wall_h) for px, py in corners], top)
            # sides: +x between corners 3,0; -x 1,2; +y 0,1; -y 2,3
            for side, (i0, i1) in enumerate(((3, 0), (1, 2), (0, 1), (2, 3))):
                if not fr[side]:
                    continue
                n = normals[side]
                shade = WALL_SIDE * (0.8 + 0.2 * abs(n[0]))
                (ax, ay), (bx, by) = corners[i0], corners[i1]
                painter.face([(ax, ay, 0.0), (bx, by, 0.0), (bx, by, self.wall_h), (ax, ay, self.wall_h)],
                             tuple(int(v) for v in shade), None, n)

    def _robot(self, painter):
        """KELO: dark chassis on four wheels, white body on top, a light strip at the front and the
        camera head on the front edge."""
        l, w, hb = self.half_l, self.half_w, self.body_h
        for sx in (1, -1):
            for sy in (1, -1):
                painter.box(sx * (l - 0.14), sy * (w - 0.04), 0.16, 0.08, 0.0, 0.1, np.array(WHEEL), np.array(WHEEL))
        painter.box(0.0, 0.0, 2 * l, 2 * w, 0.05, 0.26, CHASSIS * 1.3, CHASSIS, (30, 32, 38))
        painter.box(-0.02, 0.0, 2 * l - 0.08, 2 * w - 0.04, 0.26, hb - 0.04, ROBOT_TOP, ROBOT_SIDE, ROBOT_EDGE)
        painter.box(l - 0.005, 0.0, 0.012, 2 * w - 0.12, 0.17, 0.21, LIGHT, LIGHT)
        painter.box(l - 0.08, 0.0, 0.07, 0.2, hb - 0.04, hb, CHASSIS * 1.2, CHASSIS)

    def _objects(self, painter, objects):
        for _, label, _, x, y, sx, sy, sz, _, _ in objects:
            if label == 'person':
                r = max(0.15, min(max(sx, sy) / 2.0, 0.3))
                hgt = max(sz, 1.4)
                # legs, torso and head as round prisms (an upright figure, like a game avatar)
                painter.prism(x, y, r * 0.55, 0.0, hgt * 0.48, PERSON * 0.7, PERSON * 0.62)
                painter.prism(x, y, r * 0.75, hgt * 0.48, hgt * 0.82, PERSON, PERSON * 0.85)
                painter.prism(x, y, 0.11, hgt * 0.85, hgt, PERSON, PERSON * 0.9, n=12)
            else:
                painter.box(x, y, max(sx, 0.1), max(sy, 0.1), 0.0, max(sz, 0.2), OBSTACLE, OBSTACLE * 0.75,
                            (120, 70, 20))

    def _pin(self, painter, xy):
        x, y = xy
        painter.box(x, y, 0.05, 0.05, 0.0, 1.0, np.array([235, 235, 245]), np.array([190, 190, 200]))
        painter.box(x, y, 0.22, 0.22, 1.0, 1.22, np.array(PATH_BLUE), np.array(PATH_BLUE) * 0.8)

    def _path(self, img, cam, path, now):
        pts, s = scene.resample(path, 0.1)
        if len(pts) < 2:
            return
        left, right = scene.ribbon(pts, 0.28)
        layer = img.copy()
        for i in range(len(pts) - 1):
            floor_poly(layer, cam, [left[i], left[i + 1], right[i + 1], right[i]], PATH_BLUE, z=0.005)
        cv2.addWeighted(layer, 0.55, img, 0.45, 0.0, dst=img)
        floor_line(img, cam, left, PATH_BLUE, 2, z=0.006)
        floor_line(img, cam, right, PATH_BLUE, 2, z=0.006)
        for wing_l, tip, wing_r in scene.chevrons(pts, s, 0.7, 0.8 * now, 0.2):
            floor_line(img, cam, [wing_l, tip, wing_r], ARROW, 3, z=0.01)

    def _park_assist(self, img, cam, lidar, cloud, walls, memory=None):
        """Coloured arcs around the footprint at the nearest obstacle of each direction. Directions
        outside the LiDAR field of view fall back to the obstacle memory and the map walls (dashed:
        not a live sensor)."""
        live = np.vstack([lidar[:, :2], cloud[:, :2]]) if len(lidar) + len(cloud) else np.zeros((0, 2))
        sectors = 24
        best, _ = scene.sector_distances(live, self.half_l, self.half_l, self.half_w, sectors, 1.2)
        width = 2.0 * math.pi / sectors
        centres = np.arange(sectors) * width
        centres = np.where(centres > math.pi, centres - 2 * math.pi, centres)
        lo, hi = self.lidar_fov
        blind = (centres < lo + width / 2) | (centres > hi - width / 2)
        from_map = np.zeros(sectors, bool)
        remembered = [walls[0]] if walls is not None and len(walls[0]) else []
        if memory is not None and len(memory):
            remembered.append(memory[:, :2])
        if remembered:
            mb, _ = scene.sector_distances(np.vstack(remembered), self.half_l, self.half_l, self.half_w,
                                           sectors, 1.2)
            from_map = blind & ~np.isfinite(best) & np.isfinite(mb)
            from_map &= mb < 0.8
            best = np.where(from_map, mb, best)
        nearest = float(best.min()) if np.isfinite(best).any() else None
        for k in range(sectors):
            d = best[k]
            if not np.isfinite(d):
                continue
            a = np.linspace(centres[k] - width * 0.42, centres[k] + width * 0.42, 6)
            ring = self._ring(a, max(d - 0.03, 0.03))
            color = assist_color(d)
            if from_map[k]:
                for i in range(0, len(ring) - 1, 2):
                    floor_line(img, cam, ring[i:i + 2], color, 3, z=0.02)
            else:
                floor_line(img, cam, ring, color, 6, z=0.02)
        self.assist_label = None
        if nearest is not None and nearest < 1.2:
            k = int(np.argmin(best))
            p = self._ring(np.array([centres[k]]), nearest * 0.5)[0]
            self.assist_label = ((p[0], p[1], 0.3), f'{nearest * 100:.0f} cm', assist_color(nearest))
        return nearest

    def _ring(self, angles, offset):
        """Points at `offset` metres outside the footprint rectangle in the given directions."""
        l, w = self.half_l + offset, self.half_w + offset
        c, s = np.cos(angles), np.sin(angles)
        t = np.minimum(l / np.maximum(np.abs(c), 1e-9), w / np.maximum(np.abs(s), 1e-9))
        return np.stack([c * t, s * t], axis=1)

    def _people_ahead(self, img, cam, objects):
        """Personal space ring and predicted footsteps (next 3 s) of every person, on the floor."""
        for _, label, _, x, y, _, _, _, vx, vy in objects:
            if label != 'person':
                continue
            a = np.linspace(0.0, 2.0 * math.pi, 40)
            floor_line(img, cam, np.stack([x + 0.55 * np.cos(a), y + 0.55 * np.sin(a)], axis=1),
                       (150, 200, 255), 1, z=0.01)
            track, t = scene.predict_track(x, y, vx, vy, 3.0, 0.3)
            if len(track) < 2:
                continue
            h = math.atan2(vy, vx)
            side = np.array([-math.sin(h), math.cos(h)]) * 0.09
            for k, ((px, py), tk) in enumerate(zip(track[1:], t[1:])):
                q = np.array([px, py]) + (side if k % 2 else -side)
                fade = 1.0 - tk / 3.5
                color = tuple(int(c * fade + 40 * (1 - fade)) for c in (255, 220, 110))
                foot = [q + np.array([math.cos(h), math.sin(h)]) * 0.09 * e + side * 0.35 * f
                        for e, f in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
                floor_poly(img, cam, foot, color, z=0.012)

    def _memory(self, img, cam, memory):
        """Remembered obstacles, fading from light grey to the background over 30 s."""
        if len(memory) == 0:
            return
        p = np.column_stack([memory[:, :2], np.full(len(memory), 0.08)])
        uv, _, ok = cam.project(p)
        fade = np.clip(1.0 - memory[:, 3] / 30.0, 0.15, 1.0)
        for (u, v), f, good in zip(np.round(uv).astype(int), fade, ok):
            if good and 0 <= u < self.w and 0 <= v < self.h:
                g = int(70 + 120 * f)
                cv2.circle(img, (u, v), 2, (g, g, g + 10), -1, cv2.LINE_AA)

    def _points(self, img, cam, lidar, cloud):
        """LiDAR points as glowing dots a little above the floor; the camera cloud flattened onto the
        floor and thinned out (a footprint of what the camera sees, not a wall of dots)."""
        cloud = cloud[::3]
        for pts, base_color, size, z in ((cloud, (90, 140, 190), 1, 0.02), (lidar, (90, 230, 255), 3, 0.15)):
            if len(pts) == 0:
                continue
            p = pts.copy()
            p[:, 2] = z
            uv, _, ok = cam.project(p)
            dist, _ = scene.footprint_distance(pts[:, :2], self.half_l, self.half_l, self.half_w)
            for (u, v), d, good in zip(np.round(uv).astype(int), dist, ok):
                if good and 0 <= u < self.w and 0 <= v < self.h:
                    cv2.circle(img, (u, v), size, RED if d < 0.3 else ORANGE if d < 0.6 else base_color, -1, cv2.LINE_AA)

    def _labels(self, img, cam, objects, path, left):
        if self.assist_label is not None:
            p, label, color = self.assist_label
            uv, _, ok = cam.project([p])
            if ok[0]:
                text(img, uv[0], label, color, 0.8, 2, center=True)
        for _, label, _, x, y, sx, sy, sz, ovx, ovy in objects:
            hgt = max(sz, 1.4) if label == 'person' else max(sz, 0.2)
            uv, _, ok = cam.project([(x, y, hgt + 0.15)])
            if not ok[0]:
                continue
            dist, _ = scene.footprint_distance([(x, y)], self.half_l, self.half_l, self.half_w)
            name = label if label != 'obstacle' else 'object'
            text(img, uv[0], f'{name} {dist[0]:.1f} m', WHITE if label == 'person' else (255, 200, 120), 0.5,
                 center=True)
            if math.hypot(ovx, ovy) > 0.15:
                floor_line(img, cam, [(x, y), (x + ovx * 1.5, y + ovy * 1.5)], (255, 230, 120), 3, z=0.03)
        if path is not None:
            uv, _, ok = cam.project([(path[-1][0], path[-1][1], 1.45)])
            if ok[0]:
                text(img, uv[0], f'GOAL {left:.1f} m', PATH_BLUE, 0.6, 2, center=True)

    def _hud(self, img, speed, wz, nearest, path, left, n_objects):
        h, w = img.shape[:2]
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 64), (15, 16, 20), -1)
        cv2.addWeighted(overlay, 0.75, img, 0.25, 0.0, dst=img)
        text(img, (18, 46), f'{speed:.2f}', WHITE, 1.2, 2)
        text(img, (18 + 92, 46), 'm/s', (170, 175, 185), 0.6)
        if abs(wz) > 0.05:
            text(img, (180, 46), f'{"L" if wz > 0 else "R"} {abs(math.degrees(wz)):.0f} deg/s', (170, 175, 185), 0.6)
        if path is not None:
            text(img, (w // 2, 40), f'GOAL  {left:.1f} m', PATH_BLUE, 0.9, 2, center=True)
        else:
            text(img, (w // 2, 40), 'NO GOAL', (130, 135, 145), 0.7, 1, center=True)
        if nearest is None:
            text(img, (w - 190, 40), 'CLEAR', GREEN, 0.8, 2)
        else:
            text(img, (w - 230, 40), f'NEAR {nearest * 100:.0f} cm', assist_color(nearest), 0.8, 2)
        if n_objects:
            text(img, (18, h - 16), f'{n_objects} object{"s" if n_objects > 1 else ""} around', (170, 175, 185), 0.5)
        blocking, limit = self.inputs.social_state()
        if blocking:
            text(img, (w // 2, 100), 'WAITING FOR PERSON', (150, 200, 255), 0.9, 2, center=True)
        elif limit is not None:
            text(img, (w // 2, 100), f'PERSON NEAR  -  SPEED {limit:.0f}%', (150, 200, 255), 0.7, 2, center=True)
        loc = self.inputs.loc_state
        if loc in ('LOST', 'FAILED'):
            text(img, (w // 2, 140), f'LOCALIZATION {loc}  -  PRESS LOCALIZE', RED, 0.8, 2, center=True)
        elif loc == 'LOCALIZING':
            text(img, (w // 2, 140), 'LOCALIZING ...', YELLOW, 0.8, 2, center=True)
        note = self.inputs.note()
        if note:
            text(img, (w // 2, h - 16), note, (255, 230, 120), 0.6, 2, center=True)


def main():
    rclpy.init()
    node = SurroundView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
