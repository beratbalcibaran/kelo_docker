import math
import statistics
import sys
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener, TransformException

from kelo_bringup.logic import Debounce


class MmMonitor(Node):
    def __init__(self):
        super().__init__('mm_monitor')
        d = self.declare_parameter
        d('pose_topic', '/marvelmind/pose')
        d('map_frame', 'map')
        d('hedgehog_frame', 'hedgehog_link')
        d('mm_frame', 'marvelmind_map')
        d('window', 15)
        d('min_samples', 5)
        d('warn_distance_m', 0.5)
        d('warn_time_s', 3.0)
        d('stop_distance_m', 1.0)
        d('stop_time_s', 2.0)
        d('clear_time_s', 3.0)
        d('mm_timeout_s', 2.0)
        d('stop_enabled', True)
        d('auto_initial_pose', True)
        d('initial_pose_xy_var', 0.25)
        d('initial_pose_yaw_var', 10.0)
        d('initial_pose_retry_s', 5.0)
        d('initial_pose_max_tries', 12)
        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.hedgehog = g('hedgehog_frame')
        self.mm_frame = g('mm_frame')
        self.min_samples = int(g('min_samples'))
        self.warn_d, self.stop_d = float(g('warn_distance_m')), float(g('stop_distance_m'))
        self.mm_timeout = float(g('mm_timeout_s'))
        self.stop_enabled = bool(g('stop_enabled'))
        self.auto_init = bool(g('auto_initial_pose'))
        self.xy_var, self.yaw_var = float(g('initial_pose_xy_var')), float(g('initial_pose_yaw_var'))
        self.init_retry = float(g('initial_pose_retry_s'))
        self.init_max = int(g('initial_pose_max_tries'))

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.dists = deque(maxlen=int(g('window')))
        self.recent_mm = deque(maxlen=int(g('window')))
        self.warn = Debounce(float(g('warn_time_s')), float(g('clear_time_s')))
        self.stop = Debounce(float(g('stop_time_s')), float(g('clear_time_s')))
        self.last_mm = None
        self.alignment_ok = False
        self.amcl_seen = False
        self.init_tries = 0
        self.last_init = -1e9
        self.state = ''

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.stop_pub = self.create_publisher(Bool, '/kelo/safety_stop', 10)
        self.state_pub = self.create_publisher(String, '/kelo/mm_state', latched)
        self.dist_pub = self.create_publisher(Float32, '/kelo/mm_divergence_m', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseStamped, g('pose_topic'), self._mm_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, latched)
        self.create_timer(0.2, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _amcl_cb(self, _msg):
        self.amcl_seen = True

    def _mm_in_map(self, msg):
        tf = self.buf.lookup_transform(self.map_frame, self.mm_frame, Time())
        yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = msg.pose.position.x, msg.pose.position.y
        return (c * x - s * y + tf.transform.translation.x,
                s * x + c * y + tf.transform.translation.y)

    def _mm_cb(self, msg):
        self.last_mm = self._now()
        try:
            mx, my = self._mm_in_map(msg)
        except TransformException:
            self.alignment_ok = False
            return
        self.alignment_ok = True
        self.recent_mm.append((mx, my))
        try:
            tf = self.buf.lookup_transform(self.map_frame, self.hedgehog, Time(),
                                           Duration(seconds=0.1))
        except TransformException:
            return
        ax, ay = tf.transform.translation.x, tf.transform.translation.y
        self.dists.append(math.hypot(mx - ax, my - ay))

    def _tick(self):
        now = self._now()
        status = 'OK'
        stop_cmd = False
        if self.last_mm is None or now - self.last_mm > self.mm_timeout:
            status = 'MM_LOST'
            self.dists.clear()
            self.warn.reset()
            self.stop.reset()
        elif not self.alignment_ok:
            status = 'NO_ALIGNMENT'
        elif len(self.dists) < self.min_samples:
            status = 'NO_LOCALIZATION'
        else:
            div = statistics.median(self.dists)
            self.dist_pub.publish(Float32(data=float(div)))
            warn = self.warn.update(div > self.warn_d, now)
            diverged = self.stop.update(div > self.stop_d, now)
            if diverged:
                status = 'DIVERGED'
                stop_cmd = self.stop_enabled
            elif warn:
                status = 'WARN'
        if status != self.state:
            self.state = status
            self.state_pub.publish(String(data=status))
            log = self.get_logger().error if status == 'DIVERGED' else self.get_logger().info
            log(f'mm_state: {status}')
        self.stop_pub.publish(Bool(data=stop_cmd))
        self._maybe_initial_pose(now)

    def _maybe_initial_pose(self, now):
        if (not self.auto_init or self.amcl_seen or self.init_tries >= self.init_max
                or not self.alignment_ok or len(self.recent_mm) < self.min_samples
                or now - self.last_init < self.init_retry):
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = statistics.median(p[0] for p in self.recent_mm)
        msg.pose.pose.position.y = statistics.median(p[1] for p in self.recent_mm)
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = self.xy_var
        msg.pose.covariance[7] = self.xy_var
        msg.pose.covariance[35] = self.yaw_var
        self.init_pub.publish(msg)
        self.init_tries += 1
        self.last_init = now
        self.get_logger().info(
            f'Marvelmind start pose sent ({self.init_tries}/{self.init_max}); '
            'heading UNKNOWN: correct it with RViz 2D Pose Estimate or the GUI.')


def main():
    rclpy.init(args=sys.argv)
    node = MmMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_pub.publish(Bool(data=False))
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
