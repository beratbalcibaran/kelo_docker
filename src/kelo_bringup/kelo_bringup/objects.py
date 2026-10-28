"""Objects around the robot for the driver views (surround_view, parking_view): what is there, where,
and what it is.

- Obstacle points: the LiDAR scan and the filtered camera cloud (/camera/points_filtered), in the base
  frame. Points on the walls of the map (/map, occupied cells grown by `static_margin`) are left out, so
  only things that are not in the map become objects; an unnamed group that touches a wall is left out
  too (the end of a wall the map has a few cm off).
- The points are grouped on a 0.1 m grid; every group up to `max_size` metres is one object.
- YOLOX on the colour image names them (person, chair, ...): the camera points that fall inside a
  detection box give its position, and the nearest group takes the name. A detection without a group
  (e.g. above the LiDAR) is an object of its own.
- A tracker in the odom frame smooths the objects and keeps the name while the camera no longer sees
  them (label_keep).
- In the simulator the people come from /sim/people (sim_robot) instead of YOLOX: the simple colour
  rendering has no people to recognise.
- The detector runs in the detector service when one answers (perception.service: on the Jetson the
  kelo-detector container with TensorRT, docker/detector/), otherwise here on the CPU (OpenCV DNN).
  The service is tried again every `service_retry` seconds.
- Obstacle memory: the LiDAR and camera points are kept in the odom frame after the sensors no longer
  see them (scene.ObstacleMemory), so the driver views can show what is behind the robot, where the
  LiDAR does not look.

Outputs (the people in /kelo/objects are used by the social node in navigation; the rest is display):
  /kelo/objects            vision_msgs/Detection3DArray, odom frame, id = track id, class 'obstacle'
                           when unnamed
  /kelo/detections         vision_msgs/Detection2DArray, colour image pixels, results[0].pose = the
                           position in the base frame (NaN when no camera point fell inside the box)
  /kelo/objects/markers    visualization_msgs/MarkerArray for RViz
  /kelo/obstacle_memory    sensor_msgs/PointCloud2, odom frame, fields x y z age (s since last seen)
"""
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import (Detection2D, Detection2DArray, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray

from kelo_bringup import scene
from kelo_bringup.cloud_filter import quat_to_rot
from kelo_bringup.common import load_hardware
from kelo_bringup.logic import yaw_from_quat
from kelo_bringup.parking_view import decode_color, project_many

# typical sizes (x, y, z in metres) of a named object without camera height information
SIZES = {'person': (0.5, 0.5, 1.7), 'chair': (0.5, 0.5, 0.9), 'couch': (0.9, 1.8, 0.8),
         'bench': (0.5, 1.5, 0.5), 'dining table': (0.9, 1.4, 0.75), 'potted plant': (0.4, 0.4, 0.8),
         'dog': (0.7, 0.3, 0.6), 'cat': (0.5, 0.2, 0.3), 'suitcase': (0.3, 0.5, 0.7),
         'bicycle': (1.7, 0.5, 1.0), 'backpack': (0.3, 0.3, 0.5)}
DEFAULT_SIZE = (0.4, 0.4, 0.6)
TOUCH_M = 0.2
DEFAULT_CLASSES = ['person', 'chair', 'couch', 'bench', 'dining table', 'potted plant', 'dog', 'cat',
                   'suitcase', 'backpack', 'bicycle', 'tv', 'laptop', 'refrigerator', 'bed', 'toilet']


def points_in_box(u, v, ok, box, shrink=0.2):
    """Mask of projected points inside a detection box shrunk by `shrink` (box edges are background)."""
    x0, y0, x1, y1 = box
    dx, dy = (x1 - x0) * shrink / 2.0, (y1 - y0) * shrink / 2.0
    return ok & (u >= x0 + dx) & (u <= x1 - dx) & (v >= y0 + dy) & (v <= y1 - dy)


def cloud_xyz(msg):
    """x, y, z float32 fields of a PointCloud2 (cloud_filter layout: 12-byte points)."""
    if msg.point_step != 12 or msg.width * msg.height == 0:
        return np.zeros((0, 3))
    return np.frombuffer(msg.data, dtype='<f4').reshape(-1, 3).astype(float)


class Objects(Node):
    def __init__(self):
        super().__init__('objects')
        hw = load_hardware()
        tp, fr = hw['topics'], hw['frames']
        per = hw.get('perception', {}) or {}
        d = self.declare_parameter
        d('rate_hz', 5.0)
        d('max_range', 6.0)
        d('max_size', 2.0)
        d('min_points', 6)
        # map walls grown by this much: covers the localization error, so walls do not become objects
        d('static_margin', 0.25)
        d('detector', bool(per.get('detector', True)))
        d('model', per.get('model', '/opt/kelo/models/yolox_tiny.onnx'))
        d('detector_hz', float(per.get('detector_hz', 3.0)))
        d('score_min', float(per.get('score_min', 0.45)))
        d('classes', list(per.get('classes', DEFAULT_CLASSES)))
        d('label_keep', 5.0)
        svc = per.get('service', {}) or {}
        d('service', bool(svc.get('enabled', True)))
        d('service_host', str(svc.get('host', '127.0.0.1')))
        d('service_port', int(svc.get('port', 5810)))
        d('service_retry', 10.0)
        d('memory_age', 30.0)
        g = lambda n: self.get_parameter(n).value
        self.fr = fr
        self.front, self.half_l = hw['body']['length'] / 2.0, hw['body']['length'] / 2.0
        self.half_w = hw['body']['width'] / 2.0
        self.max_range, self.max_size = float(g('max_range')), float(g('max_size'))
        self.min_points, self.static_margin = int(g('min_points')), float(g('static_margin'))
        self.period = 1.0 / max(float(g('rate_hz')), 0.1)
        self.det_period = 1.0 / max(float(g('detector_hz')), 0.1)
        self.tracker = scene.Tracker(label_keep=float(g('label_keep')))
        self.scan = self.cloud = self.color = self.people = None
        self.k_color = None
        self.static = None
        self.dets = []
        self.last_det = 0.0
        self.tfs = {}
        self.yolo = self.service = None
        self.score_min = float(g('score_min'))
        self.service_retry, self.service_down = float(g('service_retry')), -1e9
        self.service_used = None
        if g('detector') and hw['camera'].get('color', False):
            try:
                from kelo_bringup.detector import Yolox
                self.yolo = Yolox(g('model'), score_min=self.score_min, classes=list(g('classes')))
            except Exception as exc:
                self.get_logger().warn(f'local object detection off: model not loaded ({exc})')
            if g('service'):
                from kelo_bringup.detector_service import RemoteDetector
                self.service = RemoteDetector(g('service_host'), g('service_port'), timeout=1.0,
                                              classes=list(g('classes')))
        lid = hw['lidar']
        self.lidar_fov = (lid['angle_min'], lid['angle_max'])
        cm = hw['mounts']['camera']
        self.cam_mount = (cm['x'], cm['y'], cm['yaw'], math.radians(hw['camera'].get('hfov_deg', 87.0)) / 2.0 - 0.08)
        self.memory = scene.ObstacleMemory(max_age=float(g('memory_age')), radius=float(g('max_range')))
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        sd = qos_profile_sensor_data
        self.create_subscription(LaserScan, tp['scan'], lambda m: setattr(self, 'scan', (self._now(), m)), sd)
        self.create_subscription(PointCloud2, tp['camera_points'],
                                 lambda m: setattr(self, 'cloud', (self._now(), m)), sd)
        self.create_subscription(Image, tp['camera_color'], lambda m: setattr(self, 'color', (self._now(), m)), sd)
        self.create_subscription(CameraInfo, tp['camera_color_info'], self._color_info, sd)
        self.create_subscription(OccupancyGrid, '/map', self._map, latched)
        self.create_subscription(PoseArray, '/sim/people', lambda m: setattr(self, 'people', (self._now(), m)), 10)
        self.pub = self.create_publisher(Detection3DArray, '/kelo/objects', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/kelo/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/kelo/objects/markers', 10)
        self.memory_pub = self.create_publisher(PointCloud2, '/kelo/obstacle_memory', 2)
        self.create_timer(self.period, self._tick)
        where = 'off'
        if self.yolo or self.service:
            where = (f'service {g("service_host")}:{g("service_port")}, ' if self.service else '') + \
                    (f'local YOLOX {os.path.basename(g("model"))}' if self.yolo else 'no local model')
        self.get_logger().info(f'objects: detector {where}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _color_info(self, msg):
        self.k_color = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _map(self, msg):
        import cv2
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        occ = (grid >= 50).astype(np.uint8)
        grown = []
        for margin in (self.static_margin, self.static_margin + TOUCH_M):
            r = max(1, int(math.ceil(margin / info.resolution)))
            grown.append(cv2.dilate(occ, np.ones((2 * r + 1, 2 * r + 1), np.uint8)).astype(bool))
        o = info.origin
        self.static = (np.stack(grown), info.resolution, o.position.x, o.position.y,
                       yaw_from_quat(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
                       msg.header.frame_id or self.fr['map'])

    def _static_tf(self, frame):
        if frame not in self.tfs:
            try:
                tf = self.buf.lookup_transform(self.fr['base'], frame, Time())
            except TransformException:
                return None
            tr = tf.transform.translation
            self.tfs[frame] = (quat_to_rot(tf.transform.rotation), np.array([tr.x, tr.y, tr.z]))
        return self.tfs[frame]

    def _pose(self, parent):
        """Base pose (x, y, yaw) in `parent`, or None."""
        try:
            tf = self.buf.lookup_transform(parent, self.fr['base'], Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w)

    def _scan_points(self, now):
        if self.scan is None or now - self.scan[0] > 0.5:
            return np.zeros((0, 3))
        msg = self.scan[1]
        tf = self._static_tf(msg.header.frame_id)
        if tf is None:
            return np.zeros((0, 3))
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.05)) & (r < min(msg.range_max, self.max_range))
        p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())], axis=1)
        return p @ tf[0].T + tf[1]

    def _camera_points(self, now):
        if self.cloud is None or now - self.cloud[0] > 0.5:
            return np.zeros((0, 3))
        return cloud_xyz(self.cloud[1])

    def _static(self, xy):
        """Per point: (on a map wall, within TOUCH_M of one); all False without a map."""
        none = np.zeros((2, len(xy)), bool)
        if self.static is None or len(xy) == 0:
            return none
        occ, res, ox, oy, oyaw, frame = self.static
        pose = self._pose(frame)
        if pose is None:
            return none
        m = scene.from_base(xy, pose)
        c, s = math.cos(oyaw), math.sin(oyaw)
        dx, dy = m[:, 0] - ox, m[:, 1] - oy
        i = np.floor((-s * dx + c * dy) / res).astype(int)
        j = np.floor((c * dx + s * dy) / res).astype(int)
        inside = (i >= 0) & (i < occ.shape[1]) & (j >= 0) & (j < occ.shape[2])
        hit = none.copy()
        hit[:, inside] = occ[:, i[inside], j[inside]]
        return hit

    def _run_detector(self, rgb, now):
        """Detections from the service when it answers, else from the local model."""
        if self.service is not None and now - self.service_down >= self.service_retry:
            try:
                dets = self.service.detect(rgb)
                if self.service_used is not True:
                    self.get_logger().info(f'object detection: using the detector service ({self.service.backend})')
                    self.service_used = True
                return [d for d in dets if d[1] >= self.score_min]
            except Exception as exc:
                self.service_down = now
                if self.service_used is not False:
                    self.get_logger().info(f'detector service not available ({exc}); '
                                           f'{"local CPU model" if self.yolo else "no detection"} until it answers')
                    self.service_used = False
        return self.yolo.detect(rgb) if self.yolo is not None else []

    def _detect(self, now, cam):
        """YOLOX on the latest colour image: [(label, score, box, base xyz or None)]."""
        if (self.yolo is None and self.service is None) or self.color is None or self.k_color is None or now - self.color[0] > 0.5:
            return None
        if now - self.last_det < self.det_period:
            return self.dets
        self.last_det = now
        msg = self.color[1]
        rgb = decode_color(msg)
        tf = self._static_tf(msg.header.frame_id)
        if rgb is None or tf is None:
            return None
        out = []
        u = v = ok = None
        if len(cam):
            u, v, ok = project_many(cam, self.k_color, tf[0], tf[1])
        for label, score, *box in self._run_detector(rgb, now):
            pos = None
            if u is not None:
                m = points_in_box(u, v, ok, box)
                if m.sum() >= 5:
                    pts = cam[m]
                    # the nearest part of what is inside the box: the background behind is not the object
                    rng = np.hypot(pts[:, 0], pts[:, 1])
                    near = pts[rng <= np.percentile(rng, 40)]
                    pos = np.median(near, axis=0)
            out.append((label, score, box, pos))
        self.dets = out
        self._publish_2d(msg.header, out)
        return out

    def _tick(self):
        now = self._now()
        lidar, cam = self._scan_points(now), self._camera_points(now)
        odom = self._pose(self.fr['odom'])
        if odom is not None:
            self._remember(now, odom, lidar, cam)
        pts = np.vstack([lidar, cam])
        is_cam = np.concatenate([np.zeros(len(lidar), bool), np.ones(len(cam), bool)])
        touch = np.zeros(len(pts), bool)
        if len(pts):
            dist, _ = scene.footprint_distance(pts[:, :2], self.front, self.half_l, self.half_w)
            keep = (dist > 0.05) & (np.hypot(pts[:, 0], pts[:, 1]) <= self.max_range)
            on_wall, touch = self._static(pts[:, :2])
            keep &= ~on_wall
            pts, is_cam, touch = pts[keep], is_cam[keep], touch[keep]
        objs = []
        for idx in scene.clusters(pts[:, :2], min_points=self.min_points):
            p = pts[idx]
            lo, hi = p[:, :2].min(axis=0), p[:, :2].max(axis=0)
            if np.any(hi - lo > self.max_size):
                continue
            zc = p[is_cam[idx], 2]
            h = float(zc.max()) if len(zc) else DEFAULT_SIZE[2]
            c = (lo + hi) / 2.0
            objs.append({'x': float(c[0]), 'y': float(c[1]), 'label': '', 'score': 0.0,
                         'size': [float(hi[0] - lo[0]) + 0.1, float(hi[1] - lo[1]) + 0.1, max(h, 0.2)],
                         'touch': bool(touch[idx].any())})
        labels = []
        dets = self._detect(now, cam)
        for label, score, _, pos in dets or []:
            if pos is not None:
                labels.append((label, score, float(pos[0]), float(pos[1]), 0.6))
        if self.people is not None and now - self.people[0] < 1.0:
            labels = [('person', 1.0, p.position.x, p.position.y, 0.6) for p in self.people[1].poses]
        for label, score, x, y, gate in labels:
            best = min(objs, key=lambda o: math.hypot(o['x'] - x, o['y'] - y), default=None)
            if best is not None and math.hypot(best['x'] - x, best['y'] - y) < gate and score > best['score']:
                best['label'], best['score'] = label, score
                if label in SIZES:
                    best['size'][2] = max(best['size'][2], SIZES[label][2] * 0.8)
            elif best is None or math.hypot(best['x'] - x, best['y'] - y) >= gate:
                objs.append({'x': x, 'y': y, 'label': label, 'score': score,
                             'size': list(SIZES.get(label, DEFAULT_SIZE))})
        # an unnamed group touching a map wall is most likely the end of that wall (map and scan a few
        # cm apart): not shown as an object; named ones (a person by the wall) are
        objs = [o for o in objs if o['label'] or not o.get('touch')]
        if odom is None:
            return
        for o in objs:
            o['x'], o['y'] = (float(v) for v in scene.from_base([(o['x'], o['y'])], odom)[0])
        self._publish(self.tracker.update(objs, now))

    def _remember(self, now, odom, lidar, cam):
        def fixed(p):
            return np.column_stack([scene.from_base(p[:, :2], odom), p[:, 2]]) if len(p) else np.zeros((0, 3))
        mem = self.memory
        cam_fixed = fixed(cam)
        mem.add(fixed(lidar), now, False)
        mem.add(cam_fixed, now, True)
        beams = None
        if self.scan is not None and now - self.scan[0] <= 0.5:
            msg = self.scan[1]
            tf = self._static_tf(msg.header.frame_id)
            if tf is not None:
                rot, tr = tf
                beams = (tr[0], tr[1], math.atan2(rot[1, 0], rot[0, 0]), msg.angle_min, msg.angle_increment,
                         msg.ranges)
        camera = None
        if self.cloud is not None and now - self.cloud[0] <= 0.5:
            x, y, yaw, half = self.cam_mount
            camera = (x, y, yaw, half, 0.5, 2.0, cam_fixed)
        mem.forget(now, odom, beams, camera)
        if self.memory_pub.get_subscription_count() == 0:
            return
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.fr['odom']
        msg.height, msg.width = 1, len(mem)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z', 'age'))]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 16, True
        msg.row_step = 16 * len(mem)
        data = np.column_stack([mem.centres(), mem.z, now - mem.t]).astype('<f4')
        msg.data = data.tobytes()
        self.memory_pub.publish(msg)

    def _publish(self, tracks):
        stamp = self.get_clock().now().to_msg()
        arr = Detection3DArray()
        arr.header.stamp, arr.header.frame_id = stamp, self.fr['odom']
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for tr in tracks:
            det = Detection3D()
            det.header = arr.header
            det.id = str(tr.id)
            det.bbox.center.position.x, det.bbox.center.position.y = tr.x, tr.y
            det.bbox.center.position.z = tr.size[2] / 2.0
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = tr.size
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = tr.label or 'obstacle'
            hyp.hypothesis.score = float(tr.score)
            hyp.pose.pose.position = det.bbox.center.position
            # velocity in the odom frame (m/s), for motion arrows
            hyp.pose.pose.orientation.x, hyp.pose.pose.orientation.y = tr.vx, tr.vy
            det.results.append(hyp)
            arr.detections.append(det)
            person = tr.label == 'person'
            for kind, mid in (('body', 2 * tr.id), ('text', 2 * tr.id + 1)):
                m = Marker()
                m.header = arr.header
                m.ns, m.id = 'objects', mid
                m.pose.position.x, m.pose.position.y = tr.x, tr.y
                m.pose.orientation.w = 1.0
                if kind == 'body':
                    m.type = Marker.CYLINDER if person else Marker.CUBE
                    m.pose.position.z = tr.size[2] / 2.0
                    m.scale.x, m.scale.y, m.scale.z = max(tr.size[0], 0.1), max(tr.size[1], 0.1), tr.size[2]
                    m.color.r, m.color.g, m.color.b, m.color.a = ((0.95, 0.95, 1.0, 0.8) if person
                                                                  else (1.0, 0.6, 0.1, 0.6))
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.pose.position.z = tr.size[2] + 0.25
                    m.scale.z = 0.25
                    m.text = tr.label or 'obstacle'
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                markers.markers.append(m)
        self.pub.publish(arr)
        self.marker_pub.publish(markers)

    def _publish_2d(self, header, dets):
        arr = Detection2DArray()
        arr.header = header
        for label, score, (x0, y0, x1, y1), pos in dets:
            det = Detection2D()
            det.header = header
            det.bbox.center.position.x, det.bbox.center.position.y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            det.bbox.size_x, det.bbox.size_y = x1 - x0, y1 - y0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = label, float(score)
            p = pos if pos is not None else (math.nan, math.nan, math.nan)
            hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = (float(c) for c in p)
            det.results.append(hyp)
            arr.detections.append(det)
        self.det_pub.publish(arr)


def main():
    rclpy.init()
    node = Objects()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
