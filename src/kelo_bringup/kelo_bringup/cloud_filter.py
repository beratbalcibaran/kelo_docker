import math
import os
import sys

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

from kelo_bringup import depth_noise as dn
from kelo_bringup.logic import blind_wedge_points, clearing_rays, depth_to_points, scan_sees_past


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class CloudFilter(Node):
    def __init__(self):
        super().__init__('cloud_filter')
        d = self.declare_parameter
        d('depth_topic', '/camera/camera/depth/image_rect_raw')
        d('info_topic', '/camera/camera/depth/camera_info')
        d('output_topic', '/camera/points_filtered')
        d('safety_topic', '/camera/points_safety')
        d('safety_max_range', 2.5)
        d('clearing_topic', '/camera/points_clearing')
        d('clearing_stride', 2)
        d('map_clearing_topic', '')
        d('map_clearing_range', 4.0)
        d('base_frame', 'base_link')
        d('target_width', 106)
        d('min_range', 0.30)
        d('max_range', 2.5)
        d('min_height', 0.06)
        d('max_height', 1.20)
        d('blind_clearing', True)
        d('scan_topic', '/scan')
        d('front_x', 0.45)
        d('blind_clear_above_lidar', 0.02)
        d('blind_clear_top', 1.26)
        d('denoise', True)
        d('denoise_min_neighbors', 3)
        d('denoise_rel_tol', 0.05)
        d('denoise_abs_tol', 0.03)
        d('min_height_slope', 0.03)
        d('min_height_slope_start', 1.0)
        d('cell_size', 0.05)
        d('cell_min_points', 3)
        d('persistence_frames', 3)
        d('persistence_min', 2)
        d('odom_frame', 'odom')
        d('auto_tilt', True)
        d('tilt_file', '/data/camera_tilt.yaml')
        d('tilt_max_deg', 6.0)
        g = lambda n: self.get_parameter(n).value
        self.base = g('base_frame')
        self.target_w = int(g('target_width'))
        self.rng = (float(g('min_range')), float(g('max_range')))
        self.hgt = (float(g('min_height')), float(g('max_height')))
        self.k = None
        self.img_size = None
        self.tf_cache = {}
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PointCloud2, g('output_topic'), 10)
        self.safety_rng = (float(g('min_range')), float(g('safety_max_range')))
        self.safety_pub = (self.create_publisher(PointCloud2, g('safety_topic'), 10) if g('safety_topic') else None)
        self.clear_pub = self.create_publisher(PointCloud2, g('clearing_topic'), 10)
        self.clear_stride = max(1, int(g('clearing_stride')))
        self.map_clear_range = float(g('map_clearing_range'))
        self.map_clear_pub = (self.create_publisher(PointCloud2, g('map_clearing_topic'), 10)
                              if g('map_clearing_topic') else None)
        self.create_subscription(CameraInfo, g('info_topic'), self._info, qos_profile_sensor_data)
        self.scan = None
        self.front_x = float(g('front_x'))
        self.blind_above = float(g('blind_clear_above_lidar'))
        self.blind_top = float(g('blind_clear_top'))
        if g('blind_clearing'):
            self.create_subscription(LaserScan, g('scan_topic'), self._scan, qos_profile_sensor_data)
        self.denoise = bool(g('denoise'))
        self.dn_kw = {'min_neighbors': int(g('denoise_min_neighbors')), 'rel_tol': float(g('denoise_rel_tol')),
                      'abs_tol': float(g('denoise_abs_tol'))}
        self.slope, self.slope_start = float(g('min_height_slope')), float(g('min_height_slope_start'))
        self.cell, self.cell_min = float(g('cell_size')), int(g('cell_min_points'))
        self.persist = dn.Persistence(int(g('persistence_frames')), int(g('persistence_min')), self.cell)
        self.odom_frame = g('odom_frame')
        self.tilt_file = g('tilt_file')
        self.corr = np.eye(3)
        self.tilt = self._load_tilt()
        if self.tilt is not None:
            self.corr = dn.rot_xy(*self.tilt)
        self.tilt_est = dn.TiltEstimator(max_deg=float(g('tilt_max_deg'))) if g('auto_tilt') else None
        self.tilt_pose = None
        self.tilt_pub = self.create_publisher(String, '/kelo/camera_tilt', LATCHED)
        self._publish_tilt('saved' if self.tilt is not None else 'mount')
        self.create_subscription(Image, g('depth_topic'), self._depth, qos_profile_sensor_data)
        self.get_logger().info(
            f'cloud_filter: range {self.rng} m, height {self.hgt} m (+{self.slope} per m from '
            f'{self.slope_start} m), target width {self.target_w}, denoise {self.denoise}, '
            f'cell {self.cell} m x {self.cell_min}, persistence {self.persist.k}/{self.persist.n}')

    def _load_tilt(self):
        try:
            with open(self.tilt_file) as f:
                t = yaml.safe_load(f) or {}
            return float(t['roll']), float(t['pitch'])
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
            return None

    def _publish_tilt(self, source):
        r, p = self.tilt or (0.0, 0.0)
        self.tilt_pub.publish(String(data=f'{source} roll {math.degrees(r):+.2f} deg pitch {math.degrees(p):+.2f} deg'))

    def _still(self):
        try:
            t = self.buf.lookup_transform(self.odom_frame, self.base, Time()).transform
        except TransformException:
            return False
        pose = np.array([t.translation.x, t.translation.y, t.rotation.z, t.rotation.w])
        prev, self.tilt_pose = self.tilt_pose, pose
        return prev is not None and np.abs(pose - prev).max() < 0.004

    def _estimate_tilt(self, depth, stride, tf):
        if self.tilt_est is None or self.tilt_est.result is not None or not self._still():
            return
        pts, _ = dn.depth_points(depth, self.k, max(1, stride // 2), (self.rng[0], 3.0), tf[0], tf[1],
                                 denoise=False)
        res = self.tilt_est.add(dn.fit_floor(pts, tf[1][:2]))
        if res is None:
            return
        self.tilt = res
        self.corr = dn.rot_xy(*res)
        self.persist.reset()
        self._publish_tilt('measured')
        self.get_logger().info(f'camera tilt from the floor: roll {math.degrees(res[0]):+.2f} deg, pitch '
                               f'{math.degrees(res[1]):+.2f} deg (applied; saved to {self.tilt_file})')
        try:
            os.makedirs(os.path.dirname(self.tilt_file) or '.', exist_ok=True)
            with open(self.tilt_file, 'w') as f:
                yaml.safe_dump({'roll': res[0], 'pitch': res[1],
                                'note': 'correction in front of the camera mount (cloud_filter)'}, f)
        except OSError as exc:
            self.get_logger().warn(f'could not save the camera tilt: {exc}')

    def _odom_xy(self, pts):
        try:
            t = self.buf.lookup_transform(self.odom_frame, self.base, Time()).transform
        except TransformException:
            return None
        yaw = 2.0 * math.atan2(t.rotation.z, t.rotation.w)
        c, s_ = math.cos(yaw), math.sin(yaw)
        return np.c_[c * pts[:, 0] - s_ * pts[:, 1] + t.translation.x,
                     s_ * pts[:, 0] + c * pts[:, 1] + t.translation.y]

    def obstacle_points(self, depth, stride, tf):
        rot = self.corr @ tf[0]
        pts, dist = dn.depth_points(depth, self.k, stride, self.rng, rot, tf[1], denoise=self.denoise,
                                    **self.dn_kw)
        lim = dn.min_height_at(dist, self.hgt[0], self.slope, self.slope_start)
        pts = pts[(pts[:, 2] >= lim) & (pts[:, 2] <= self.hgt[1])]
        if len(pts) and self.cell_min > 1:
            pts = pts[dn.cell_support(pts[:, :2], self.cell, self.cell_min)]
        if self.persist.k > 1:
            fixed = self._odom_xy(pts) if len(pts) else np.zeros((0, 2))
            if fixed is not None:
                pts = pts[self.persist.step(fixed)]
        return pts

    def _info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.img_size = (msg.width, msg.height)

    def _scan(self, msg):
        self.scan = msg

    def _blind_clearing(self, stamp, tf):
        s = self.scan
        if s is None or abs((Time.from_msg(stamp) - Time.from_msg(s.header.stamp)).nanoseconds) > 0.5e9:
            return np.zeros((0, 3))
        laser = self._transform(s.header.frame_id)
        if laser is None:
            return np.zeros((0, 3))
        lrot, ltrans = laser
        pts = np.concatenate([
            blind_wedge_points(self.k, *self.img_size, tf[0], tf[1], ltrans[2] + self.blind_above, self.front_x),
            blind_wedge_points(self.k, *self.img_size, tf[0], tf[1], self.blind_top, self.front_x)])
        if len(pts) == 0:
            return pts
        yaw = float(np.arctan2(lrot[1, 0], lrot[0, 0]))
        free = scan_sees_past(pts[:, :2], ltrans[:2], yaw, s.angle_min, s.angle_increment, s.ranges, s.range_min)
        return pts[free]

    def _transform(self, frame):
        if frame in self.tf_cache:
            return self.tf_cache[frame]
        try:
            tf = self.buf.lookup_transform(self.base, frame, Time())
        except TransformException as exc:
            self.get_logger().warn(f'{self.base}<-{frame} TF missing: {exc}', throttle_duration_sec=5.0)
            return None
        tr = tf.transform.translation
        out = (quat_to_rot(tf.transform.rotation), np.array([tr.x, tr.y, tr.z]))
        self.tf_cache[frame] = out
        return out

    def _depth(self, msg):
        if self.k is None:
            return
        tf = self._transform(msg.header.frame_id)
        if tf is None:
            return
        if msg.encoding == '16UC1':
            dt = np.dtype('>u2' if msg.is_bigendian else '<u2')
            depth = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.step // 2)[:, :msg.width]
            depth = depth.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            dt = np.dtype('>f4' if msg.is_bigendian else '<f4')
            depth = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.step // 4)[:, :msg.width]
        else:
            self.get_logger().error(f'unsupported depth encoding: {msg.encoding}',
                                    throttle_duration_sec=10.0)
            return
        stride = max(1, int(round(msg.width / float(self.target_w))))
        depth = np.where(np.isfinite(depth), depth, 0.0)
        self._estimate_tilt(depth, stride, tf)
        if self.safety_pub is not None:
            safe = depth_to_points(depth, self.k, stride, self.safety_rng, self.hgt, self.corr @ tf[0], tf[1])
            self.safety_pub.publish(self._cloud(msg.header.stamp, safe))
        pts = self.obstacle_points(depth, stride, tf)
        self.pub.publish(self._cloud(msg.header.stamp, pts))
        tf = (self.corr @ tf[0], tf[1])
        clear = clearing_rays(depth, self.k, stride * self.clear_stride, self.rng[0], self.rng[1],
                              tf[0], tf[1])
        clear = clear[clear[:, 2] <= self.hgt[1]]
        if self.scan is not None:
            clear = np.concatenate([clear, self._blind_clearing(msg.header.stamp, tf)])
        self.clear_pub.publish(self._cloud(msg.header.stamp, clear))
        if self.map_clear_pub is not None:
            rays = clearing_rays(depth, self.k, stride * self.clear_stride, self.rng[0],
                                 self.map_clear_range, tf[0], tf[1])
            self.map_clear_pub.publish(self._cloud(msg.header.stamp, rays))

    def _cloud(self, stamp, pts):
        out = PointCloud2()
        out.header.stamp = stamp
        out.header.frame_id = self.base
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        out.is_bigendian, out.point_step, out.row_step, out.is_dense = False, 12, 12 * len(pts), True
        out.data = pts.astype('<f4').tobytes()
        return out


def main():
    rclpy.init(args=sys.argv)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
