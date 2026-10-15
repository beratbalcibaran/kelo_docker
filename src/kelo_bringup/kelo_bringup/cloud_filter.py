import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup.logic import depth_to_points


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
        d('clearing_topic', '/camera/points_clearing')
        d('clearing_stride', 2)
        d('base_frame', 'base_link')
        d('target_width', 106)
        d('min_range', 0.30)
        d('max_range', 2.5)
        d('min_height', 0.06)
        d('max_height', 1.20)
        g = lambda n: self.get_parameter(n).value
        self.base = g('base_frame')
        self.target_w = int(g('target_width'))
        self.rng = (float(g('min_range')), float(g('max_range')))
        self.hgt = (float(g('min_height')), float(g('max_height')))
        self.k = None
        self.tf_cache = {}
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PointCloud2, g('output_topic'), 10)
        self.clear_pub = self.create_publisher(PointCloud2, g('clearing_topic'), 10)
        self.clear_stride = max(1, int(g('clearing_stride')))
        self.create_subscription(CameraInfo, g('info_topic'), self._info, qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic'), self._depth, qos_profile_sensor_data)
        self.get_logger().info(
            f'cloud_filter: range {self.rng} m, height {self.hgt} m, target width {self.target_w}')

    def _info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

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
        pts = depth_to_points(depth, self.k, stride, self.rng, self.hgt, tf[0], tf[1])
        self.pub.publish(self._cloud(msg.header.stamp, pts))
        clear = depth_to_points(depth, self.k, stride * self.clear_stride, self.rng,
                                (-1e3, self.hgt[1]), tf[0], tf[1])
        self.clear_pub.publish(self._cloud(msg.header.stamp, clear))

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
