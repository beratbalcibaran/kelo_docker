"""Front camera view like a car's parking camera, for the camera window (camera_view) and RViz
(/parking_view/image, /parking_view/color_image).

The D435 depth image is drawn in grey (near = bright). On top of it:
- guide lines on the floor at the robot width plus `margin` on each side, from the front edge forward,
  with cross bars every 0.5 m (red up to 0.5 m, yellow up to 1.0 m, green beyond). They follow the
  commanded velocity (cmd_vel) like a car's guide lines follow the steering wheel: the lines show
  where the front corners go in the next `length` metres; standing still they point straight ahead;
- every pixel whose 3D point is an obstacle (height `min_height`..`max_height` above the floor, the
  same band as cloud_filter) is coloured: red inside the corridor swept by the robot along the guide
  lines, amber up to `side` metres beside it (walls further away stay grey);
- while Nav2 drives to a goal, the plan as a blue strip on the floor with arrows flowing towards the
  goal, hidden behind obstacles like a road in a game, and a pin on the goal when it is in view;
- on the colour image, the objects the detector names (objects node) with their distance;
- the distance from the front edge to the nearest obstacle in the corridor, also published on
  /parking_view/nearest (m, -1 when the corridor is free), the speed and the distance left to the goal.
With `camera.color: true` the same is drawn on the colour image (/parking_view/color_image): the floor
drawings are projected with the colour camera, the obstacle points found in the depth image are
projected into it. Display only: the costmaps and the collision monitor do not use it.
- people: where they walk in the next 3 s as a dotted line on the floor; WAITING FOR PERSON / the
  speed limit near people in the status bar (social node).
Double click on the floor in an image (camera_view, web_view) sends the robot there, right click
cancels the goal. Rendered only while someone watches.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup import scene
from kelo_bringup.cloud_filter import quat_to_rot
from kelo_bringup.common import load_hardware
from kelo_bringup.drive_inputs import DriveInputs

RED, YELLOW, GREEN, AMBER, WHITE = (230, 40, 40), (240, 200, 0), (40, 200, 60), (255, 150, 0), (255, 255, 255)
PATH_BLUE, ARROW = (40, 140, 255), (200, 235, 255)
FLOOR_Z = 0.04


def classify(depth, k, rot, trans, front, half_width, length, rng, hgt, side=0.4, poses=None, s=None,
             rear=None):
    """Obstacle masks of a depth image (m): (inside the corridor, up to `side` beside it), the
    distance from the front edge to each corridor pixel (inf elsewhere) and the pixels in the base
    frame. Without `poses` the corridor is straight ahead; with the predicted poses (and the distance
    `s` travelled at each, scene.predict_poses) it is the area the footprint (front edge `front`,
    back edge `rear`) sweeps along them."""
    fx, fy, cx, cy = k
    v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
    z = depth
    pts = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=-1)
    base = pts @ np.asarray(rot).T + np.asarray(trans)
    r = np.linalg.norm(pts, axis=-1)
    ob = (z > 0.0) & (r >= rng[0]) & (r <= rng[1]) & (base[..., 2] >= hgt[0]) & (base[..., 2] <= hgt[1])
    if poses is None:
        ahead = base[..., 0] - front
        along = (ahead >= -0.05) & (ahead <= length)
        inside = ob & along & (np.abs(base[..., 1]) <= half_width)
        beside = ob & along & ~inside & (np.abs(base[..., 1]) <= half_width + side)
        dist = np.where(inside, np.maximum(ahead, 0.0), np.inf)
        return inside, beside, dist, base
    rear = front if rear is None else rear
    xy = base[ob][:, :2]
    reach = length + front + half_width + side
    near = np.hypot(xy[:, 0], xy[:, 1]) <= reach
    d_in = np.full(len(xy), np.inf)
    d_wide = np.full(len(xy), np.inf)
    d_in[near] = scene.sweep_distance(xy[near], poses, s, front, rear, half_width)
    d_wide[near] = scene.sweep_distance(xy[near], poses, s, front, rear, half_width + side)
    ins = np.isfinite(d_in) & (d_in <= length)
    bes = ~ins & np.isfinite(d_wide) & (d_wide <= length)
    inside, beside = np.zeros_like(ob), np.zeros_like(ob)
    inside[ob], beside[ob] = ins, bes
    dist = np.full(depth.shape, np.inf)
    dist[ob] = np.where(ins, d_in, np.inf)
    return inside, beside, dist, base


def nearest(dist, min_pixels):
    """Distance of the min_pixels-th nearest corridor pixel (one noisy pixel is not an obstacle)."""
    d = dist[np.isfinite(dist)]
    if d.size < min_pixels:
        return None
    return float(np.partition(d, min_pixels - 1)[min_pixels - 1])


def project(points, k, rot, trans):
    """Base frame points -> pixel coordinates (None for points behind the camera)."""
    fx, fy, cx, cy = k
    out = []
    for p in points:
        c = np.asarray(rot).T @ (np.asarray(p, dtype=float) - np.asarray(trans))
        out.append(None if c[2] < 0.05 else (fx * c[0] / c[2] + cx, fy * c[1] / c[2] + cy))
    return out


def project_many(points, k, rot, trans):
    """Base frame points (N x 3) -> pixel columns, rows and a mask of the points in front of the camera."""
    fx, fy, cx, cy = k
    c = (np.asarray(points, dtype=float) - np.asarray(trans)) @ np.asarray(rot)
    ok = c[:, 2] > 0.05
    z = np.where(ok, c[:, 2], 1.0)
    return fx * c[:, 0] / z + cx, fy * c[:, 1] / z + cy, ok


def floor_pixels(xy, k, rot, trans):
    """Floor points (N x 2, base frame) -> int pixel array N x 2 and the mask of the usable ones."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    u, v, ok = project_many(np.column_stack([xy, np.zeros(len(xy))]), k, rot, trans)
    ok &= (np.abs(u) < 1e5) & (np.abs(v) < 1e5)
    return np.column_stack([np.round(u), np.round(v)]).astype(np.int32), ok


def decode_depth(msg):
    """Depth image in metres (0 = no measurement), or None for an unsupported encoding."""
    if msg.encoding == '16UC1':
        dt = np.dtype('>u2' if msg.is_bigendian else '<u2')
        depth = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return depth.astype(np.float32) / 1000.0
    if msg.encoding == '32FC1':
        dt = np.dtype('>f4' if msg.is_bigendian else '<f4')
        depth = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0)
    return None


def decode_color(msg):
    """rgb8 image from an rgb8/bgr8 message, or None."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        return None
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    return img[..., ::-1] if msg.encoding == 'bgr8' else img


def band_color(d):
    return RED if d < 0.5 else YELLOW if d < 1.0 else GREEN


def draw_path(img, solid, path, k, rot, trans, t, half_width=0.22, speed=0.8):
    """Nav2 plan as a translucent blue strip on the floor with arrows flowing along it; pixels in
    `solid` (an obstacle seen in front of the floor) are left alone, so the strip passes behind them."""
    pts, s = scene.resample(path, 0.1)
    if len(pts) < 2:
        return
    left, right = scene.ribbon(pts, half_width)
    pl, okl = floor_pixels(left, k, rot, trans)
    pr, okr = floor_pixels(right, k, rot, trans)
    layer = np.zeros(img.shape[:2], np.uint8)
    for i in range(len(pts) - 1):
        if okl[i] and okl[i + 1] and okr[i] and okr[i + 1]:
            cv2.fillConvexPoly(layer, np.array([pl[i], pl[i + 1], pr[i + 1], pr[i]]), 1, cv2.LINE_AA)
    arrows = np.zeros(img.shape[:2], np.uint8)
    for wing_l, tip, wing_r in scene.chevrons(pts, s, 0.6, speed * t, 0.18):
        q, ok = floor_pixels([wing_l, tip, wing_r], k, rot, trans)
        if ok.all():
            cv2.polylines(arrows, [q], False, 1, max(2, int(img.shape[1] / 280)), cv2.LINE_AA)
    free = ~solid if solid is not None else np.ones(img.shape[:2], bool)
    strip = (layer > 0) & free
    img[strip] = (0.5 * img[strip] + 0.5 * np.array(PATH_BLUE)).astype(np.uint8)
    edge = cv2.morphologyEx(layer, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    edge &= free
    img[edge] = (0.3 * img[edge] + 0.7 * np.array(PATH_BLUE)).astype(np.uint8)
    head = (arrows > 0) & free
    img[head] = ARROW


def draw_pin(img, xy, k, rot, trans, text):
    """Goal pin: a post from the floor to 1 m with a ball and a label."""
    a, b = project([(xy[0], xy[1], 0.0), (xy[0], xy[1], 1.0)], k, rot, trans)
    if a is None or b is None or not (0 <= a[0] < img.shape[1] and 0 <= a[1] < img.shape[0]):
        return
    a, b = tuple(int(round(c)) for c in a), tuple(int(round(c)) for c in b)
    cv2.ellipse(img, a, (max(6, abs(a[1] - b[1]) // 5), max(3, abs(a[1] - b[1]) // 14)), 0, 0, 360,
                PATH_BLUE, 2, cv2.LINE_AA)
    cv2.line(img, a, b, WHITE, 3, cv2.LINE_AA)
    r = max(6, abs(a[1] - b[1]) // 10)
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    right = b[0] + r + 6 + tw + 10 < img.shape[1]
    label(img, (b[0] + r + 6 if right else b[0] - r - 16 - tw, b[1] - 12), text, PATH_BLUE)
    cv2.circle(img, b, r, PATH_BLUE, -1, cv2.LINE_AA)
    cv2.circle(img, b, r, WHITE, 2, cv2.LINE_AA)


def label(img, org, text, color, scale=0.55):
    """Text on a dark chip with a coloured edge."""
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = int(org[0]), int(org[1])
    x = min(max(x, 0), max(img.shape[1] - w - 10, 0))
    y = min(max(y, 0), max(img.shape[0] - h - base - 8, 0))
    cv2.rectangle(img, (x, y), (x + w + 10, y + h + base + 8), (20, 20, 25), -1)
    cv2.rectangle(img, (x, y), (x + w + 10, y + h + base + 8), color, 1)
    cv2.putText(img, text, (x + 5, y + h + 4), cv2.FONT_HERSHEY_SIMPLEX, scale, WHITE, 1, cv2.LINE_AA)


def draw_box(img, box, color, text):
    """Detection box drawn as corner brackets with a label above it."""
    x0, y0, x1, y1 = (int(round(c)) for c in box)
    n = max(8, min(x1 - x0, y1 - y0) // 4)
    for (px, py), (dx, dy) in (((x0, y0), (1, 1)), ((x1, y0), (-1, 1)), ((x0, y1), (1, -1)), ((x1, y1), (-1, -1))):
        cv2.line(img, (px, py), (px + dx * n, py), color, 3, cv2.LINE_AA)
        cv2.line(img, (px, py), (px, py + dy * n), color, 3, cv2.LINE_AA)
    label(img, (x0, y0 - 28), text, color)


class ParkingView(Node):
    def __init__(self):
        super().__init__('parking_view')
        hw = load_hardware()
        d = self.declare_parameter
        d('depth_topic', hw['topics']['camera_depth'])
        d('info_topic', hw['topics']['camera_info'])
        d('color', bool(hw['camera'].get('color', False)))
        d('color_topic', hw['topics'].get('camera_color', '/camera/camera/color/image_raw'))
        d('color_info_topic', hw['topics'].get('camera_color_info', '/camera/camera/color/camera_info'))
        d('base_frame', hw['frames']['base'])
        d('margin', 0.10)
        d('side', 0.4)
        d('length', 2.5)
        d('min_range', 0.30)
        d('max_range', 4.0)
        d('min_height', 0.06)
        d('max_height', 1.20)
        d('min_pixels', 4)
        d('width', 848)
        d('rate_hz', 5.0)
        d('path_length', 8.0)
        g = lambda n: self.get_parameter(n).value
        self.base = g('base_frame')
        self.front = hw['body']['length'] / 2.0
        self.body_half_w = hw['body']['width'] / 2.0
        self.half_w = self.body_half_w + float(g('margin'))
        self.length = float(g('length'))
        self.side = float(g('side'))
        self.rng = (float(g('min_range')), float(g('max_range')))
        self.hgt = (float(g('min_height')), float(g('max_height')))
        self.min_pixels = int(g('min_pixels'))
        self.out_w = int(g('width'))
        self.period = 1.0 / max(float(g('rate_hz')), 0.1)
        self.path_length = float(g('path_length'))
        self.k = None
        self.k_color = None
        self.color = None
        self.tfs = {}
        self.last = 0.0
        self.models = {}  # pane -> (k of the published image, rot, trans): for clicks
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.inputs = DriveInputs(self, hw, self.buf)
        self.inputs.on_click(self._click)
        self.pub = self.create_publisher(Image, '~/image', 10)
        self.color_pub = self.create_publisher(Image, '~/color_image', 10)
        self.near_pub = self.create_publisher(Float32, '~/nearest', 10)
        self.create_subscription(CameraInfo, g('info_topic'), self._info, qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic'), self._depth, qos_profile_sensor_data)
        if g('color'):
            self.create_subscription(CameraInfo, g('color_info_topic'), self._color_info,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, g('color_topic'), self._color, qos_profile_sensor_data)
        self.get_logger().info(f'parking_view: corridor {2 * self.half_w:.2f} m wide, '
                               f'{self.length:.1f} m ahead of the front edge, '
                               f'colour image {"on" if g("color") else "off"}')

    def _info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _color_info(self, msg):
        self.k_color = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _color(self, msg):
        self.color = msg

    def _transform(self, frame):
        if frame not in self.tfs:
            try:
                tf = self.buf.lookup_transform(self.base, frame, Time())
            except TransformException as exc:
                self.get_logger().warn(f'{self.base}<-{frame} TF missing: {exc}', throttle_duration_sec=5.0)
                return None
            tr = tf.transform.translation
            self.tfs[frame] = (quat_to_rot(tf.transform.rotation), np.array([tr.x, tr.y, tr.z]))
        return self.tfs[frame]

    def _publish(self, pub, header, out):
        img = Image()
        img.header = header
        img.height, img.width = out.shape[:2]
        img.encoding, img.is_bigendian, img.step = 'rgb8', 0, out.shape[1] * 3
        img.data = out.tobytes()
        pub.publish(img)

    def _click(self, frame, u, v, is_goal):
        if frame not in ('color', 'depth'):
            return
        if not is_goal:
            self.inputs.cancel_goal()
            return
        if frame not in self.models:
            self.inputs.say('no camera image yet')
            return
        k, rot, trans = self.models[frame]
        p = scene.ray_to_floor(trans, np.asarray(rot) @ scene.pixel_ray(u, v, k))
        if p is None:
            self.inputs.say('click on the floor to send a goal')
            return
        self.inputs.send_goal(*p)

    def _watched(self):
        return any(p.get_subscription_count() > 0 for p in (self.pub, self.color_pub, self.near_pub))

    def _depth(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.k is None or now - self.last < self.period or not self._watched():
            return
        tf = self._transform(msg.header.frame_id)
        if tf is None:
            return
        self.last = now
        depth = decode_depth(msg)
        if depth is None:
            self.get_logger().error(f'unsupported depth encoding: {msg.encoding}', throttle_duration_sec=10.0)
            return
        vx, vy, wz = self.inputs.twist()
        straight = abs(vy) < 0.02 and abs(wz) < 0.05
        poses, s = scene.predict_poses(vx, vy, wz, self.length, ds=0.05)
        coarse = None if straight else scene.predict_poses(vx, vy, wz, self.length, ds=0.1)
        inside, beside, dist, base = classify(depth, self.k, tf[0], tf[1], self.front, self.half_w,
                                              self.length, self.rng, self.hgt, self.side,
                                              *(coarse or (None, None)))
        near = nearest(dist, self.min_pixels)
        self.near_pub.publish(Float32(data=-1.0 if near is None else near))
        solid = (depth > 0.0) & (base[..., 2] > FLOOR_Z)
        path, left = self.inputs.plan_in_base(self.path_length)
        scene_ = {'near': near, 'poses': poses, 's': s, 'path': path, 'left': left, 't': now,
                  'speed': math.hypot(vx, vy)}
        self._publish(self.pub, msg.header, self._render_depth(depth, inside, beside, solid, tf, scene_))
        if self.color is not None and self.k_color is not None:
            ctf = self._transform(self.color.header.frame_id)
            rgb = decode_color(self.color)
            if ctf is not None and rgb is not None:
                out = self._render_color(rgb, base[beside], base[inside], base[solid], ctf, scene_)
                self._publish(self.color_pub, self.color.header, out)

    def _scaled(self, img, k):
        s = self.out_w / float(img.shape[1])
        size = (self.out_w, int(round(img.shape[0] * s)))
        out = np.ascontiguousarray(cv2.resize(img, size, interpolation=cv2.INTER_NEAREST))
        return out, tuple(v * s for v in k), s

    def _render_depth(self, depth, inside, beside, solid, tf, sc):
        grey = np.where(depth > 0.0, np.clip(255.0 * (1.0 - depth / self.rng[1]), 40, 255), 0).astype(np.uint8)
        img = np.repeat(grey[..., None], 3, axis=2)
        for mask, color in ((beside, AMBER), (inside, RED)):
            img[mask] = (0.35 * img[mask] + 0.65 * np.array(color)).astype(np.uint8)
        img, k, _ = self._scaled(img, self.k)
        self.models['depth'] = (k, tf[0], tf[1])
        solid = cv2.resize(solid.astype(np.uint8), (img.shape[1], img.shape[0]),
                           interpolation=cv2.INTER_NEAREST).astype(bool)
        self._overlays(img, k, tf, solid, sc)
        return img

    def _project_mask(self, shape, pts, k, tf, kernel):
        u, v, ok = project_many(pts, k, tf[0], tf[1])
        u, v = np.round(u[ok]).astype(int), np.round(v[ok]).astype(int)
        keep = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
        mask = np.zeros(shape[:2], np.uint8)
        mask[v[keep], u[keep]] = 1
        return cv2.dilate(mask, kernel).astype(bool)

    def _render_color(self, rgb, beside_pts, inside_pts, solid_pts, tf, sc):
        img, k, s = self._scaled(rgb, self.k_color)
        self.models['color'] = (k, tf[0], tf[1])
        # one depth pixel covers about this many colour pixels on screen
        r = max(1, int(round(s * self.k_color[0] / max(self.k[0], 1e-6))))
        kernel = np.ones((2 * r + 1, 2 * r + 1), np.uint8)
        for pts, color in ((beside_pts, AMBER), (inside_pts, RED)):
            if len(pts) == 0:
                continue
            mask = self._project_mask(img.shape, pts, k, tf, kernel)
            img[mask] = (0.45 * img[mask] + 0.55 * np.array(color)).astype(np.uint8)
        solid = self._project_mask(img.shape, solid_pts, k, tf, kernel) if len(solid_pts) else None
        self._overlays(img, k, tf, solid, sc)
        boxes = []
        for lab, score, x0, y0, x1, y1, xy in self.inputs.detections():
            boxes.append((x0 * s, y0 * s, x1 * s, y1 * s))
            self._box(img, boxes[-1], lab, xy)
        # named objects the detector does not see in this image (tracked by the LiDAR, or the
        # simulator's people): their 3D box projected into the image
        for _, lab, _, x, y, sx, sy, sz, _, _ in self.inputs.objects_in_base():
            if lab == 'obstacle':
                continue
            corners = [(x + dx * sx / 2, y + dy * sy / 2, z) for dx in (-1, 1) for dy in (-1, 1) for z in (0.0, sz)]
            u, v, ok = project_many(corners, k, tf[0], tf[1])
            if not ok.all():
                continue
            box = (max(u.min(), 0), max(v.min(), 0), min(u.max(), img.shape[1] - 1), min(v.max(), img.shape[0] - 1))
            cu, cv_ = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            if box[2] - box[0] < 8 or box[3] - box[1] < 8 or any(b[0] <= cu <= b[2] and b[1] <= cv_ <= b[3] for b in boxes):
                continue
            self._box(img, box, lab, (x, y))
        return img

    def _box(self, img, box, lab, xy):
        text = lab.upper()
        if xy is not None:
            dist, _ = scene.footprint_distance([xy], self.front, self.front, self.body_half_w)
            text += f'  {dist[0]:.1f} m'
        draw_box(img, box, WHITE if lab == 'person' else AMBER, text)

    def _overlays(self, img, k, tf, solid, sc):
        if sc['path'] is not None:
            draw_path(img, solid, sc['path'], k, tf[0], tf[1], sc['t'])
            end = sc['path'][-1]
            if sc['left'] <= self.path_length + 1e-6:
                draw_pin(img, end, k, tf[0], tf[1], f'GOAL {sc["left"]:.1f} m')
        self._guides(img, k, tf, sc['poses'], sc['s'], sc['near'])
        self._people_paths(img, k, tf)
        self._hud(img, sc)

    def _guides(self, img, k, tf, poses, s, near):
        """Paths of the two front corners (s = distance the front edge has travelled), a dark
        outline first and the distance colours on top."""
        tracks = [floor_pixels(tr, k, tf[0], tf[1]) for tr in scene.edge_tracks(poses, self.front, self.half_w)]
        for thick, colored in ((7, False), (4, True)):
            for i in range(len(s) - 1):
                for q, ok in tracks:
                    if ok[i] and ok[i + 1]:
                        color = band_color(s[i]) if colored else (0, 0, 0)
                        cv2.line(img, tuple(q[i]), tuple(q[i + 1]), color, thick, cv2.LINE_AA)
        tracks = scene.edge_tracks(poses, self.front, self.half_w)
        for mark in np.arange(0.5, self.length + 1e-6, 0.5):
            i = int(np.argmin(np.abs(s - mark)))
            q, ok = floor_pixels([tracks[0][i], tracks[1][i]], k, tf[0], tf[1])
            if not ok.all():
                continue
            color = band_color(mark - 0.01)
            cv2.line(img, tuple(q[0]), tuple(q[1]), (0, 0, 0), 5, cv2.LINE_AA)
            cv2.line(img, tuple(q[0]), tuple(q[1]), color, 2, cv2.LINE_AA)
            if abs(mark % 1.0) < 1e-6:
                cv2.putText(img, f'{mark:.0f} m', (int(q[1][0]) + 6, int(q[1][1]) + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)

    def _people_paths(self, img, k, tf):
        """Where each person walks in the next 3 s: dots on the floor, fading with time."""
        for _, lab, _, x, y, _, _, _, vx, vy in self.inputs.objects_in_base():
            if lab != 'person':
                continue
            track, t = scene.predict_track(x, y, vx, vy, 3.0, 0.2)
            if len(track) < 2:
                continue
            q, ok = floor_pixels(track[1:], k, tf[0], tf[1])
            for (u, v), good, tk in zip(q, ok, t[1:]):
                if good and 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                    cv2.circle(img, (int(u), int(v)), max(2, int(6 - tk)), (255, 220, 110), -1, cv2.LINE_AA)

    def _hud(self, img, sc):
        near = sc['near']
        if near is None:
            text, color = 'CLEAR', GREEN
        else:
            text, color = f'OBSTACLE {near:.2f} m', band_color(near)
        h, w = img.shape[:2]
        cv2.rectangle(img, (0, h - 30), (w, h), (0, 0, 0), -1)
        cv2.putText(img, text, (10, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        blocking, limit = self.inputs.social_state()
        top = 'WAITING FOR PERSON' if blocking else (f'PERSON NEAR - SPEED {limit:.0f}%' if limit else None)
        note = self.inputs.note()
        # top right: the detection labels sit at the top left of their boxes
        for i, line in enumerate(t for t in (top, note) if t):
            (tw, _), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            label(img, (w - tw - 20, 10 + 34 * i), line, (150, 200, 255) if line == top else (255, 230, 120))
        right = f'{sc["speed"]:.2f} m/s'
        if sc['path'] is not None:
            right = f'GOAL {sc["left"]:.1f} m   ' + right
        (tw, _), _ = cv2.getTextSize(right, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.putText(img, right, (w - tw - 10, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    PATH_BLUE if sc['path'] is not None else WHITE, 1, cv2.LINE_AA)


def main():
    rclpy.init()
    node = ParkingView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
