"""Watches the localization while navigation runs.

- /kelo/localization_fit (Float32, 1 Hz): share of the LiDAR points that lie on a wall of the map at the
  AMCL pose (scan_match). AMCL's covariance alone can be small on a wrong pose.
- LOCALIZED but the fit stays below LOST_FIT for `lost_time_s`: state LOST, the running goal is cancelled
  (no driving on a wrong map pose); "Localize" again.
- While LOCALIZED with a good fit the pose is saved to /data/last_pose.yaml. At the next start (no
  Marvelmind, no initial_pose) that pose is given to AMCL and kept only if the scan fits there: a robot
  that was switched off where it stood is localized again without turning.
"""
import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup import scan_match as sm
from kelo_bringup.common import data_dir
from kelo_bringup.logic import quat_from_yaw, yaw_from_quat

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw(q):
    return yaw_from_quat(q.x, q.y, q.z, q.w)


class LocMonitor(Node):
    def __init__(self):
        super().__init__('loc_monitor')
        d = self.declare_parameter
        d('map_yaml', '')
        d('map_frame', 'map')
        d('scan_topic', '/scan')
        d('restore_last_pose', True)
        d('lost_time_s', 8.0)
        d('cancel_goal_when_lost', True)
        d('save_period_s', 2.0)
        g = lambda n: self.get_parameter(n).value
        self.map_yaml, self.map_frame = g('map_yaml'), g('map_frame')
        self.restore = bool(g('restore_last_pose'))
        self.lost_time, self.cancel_lost = float(g('lost_time_s')), bool(g('cancel_goal_when_lost'))
        self.save_period = float(g('save_period_s'))
        self.path = sm.last_pose_file(data_dir())
        self.mapfit = None
        self.scan = None
        self.state = None
        self.amcl_t = 0.0
        self.bad_since = None
        self.saved = None
        self.saved_t = 0.0
        self.restoring = None
        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)
        self.create_subscription(OccupancyGrid, '/map', self._map, LATCHED)
        self.create_subscription(LaserScan, g('scan_topic'), self._scan, qos_profile_sensor_data)
        self.create_subscription(String, '/kelo/localization_state', self._state, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, LATCHED)
        self.fit_pub = self.create_publisher(Float32, '/kelo/localization_fit', 10)
        self.state_pub = self.create_publisher(String, '/kelo/localization_state_set', LATCHED)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cancel_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.t0 = time.time()
        self.create_timer(1.0, self._tick)

    def _map(self, m):
        self.mapfit = sm.MapFit.from_msg(m)

    def _scan(self, m):
        self.scan = m

    def _state(self, m):
        self.state = m.data

    def _amcl(self, m):
        self.amcl_t = time.time()

    def fit(self):
        """((hits, crossings), (x, y, yaw) of base_link in the map) or (None, None)."""
        if self.mapfit is None or self.scan is None:
            return None, None
        try:
            laser = self.buf.lookup_transform(self.map_frame, self.scan.header.frame_id, Time())
            base = self.buf.lookup_transform(self.map_frame, 'base_link', Time())
        except TransformException:
            return None, None
        s = self.scan
        pts = sm.scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        t = laser.transform
        f = self.mapfit.parts(sm.transform(pts, t.translation.x, t.translation.y, _yaw(t.rotation)),
                              origin=(t.translation.x, t.translation.y))
        b = base.transform
        return f, (b.translation.x, b.translation.y, _yaw(b.rotation))

    def set_state(self, text, why):
        self.state = text
        self.state_pub.publish(String(data=text))
        if text == 'LOST':
            self.get_logger().warn(f'localization {text}: {why}')
        else:
            self.get_logger().info(f'localization {text}: {why}')

    def _tick(self):
        parts, pose = self.fit()
        f = None if parts is None else parts[0] - parts[1]
        if f is not None:
            self.fit_pub.publish(Float32(data=f))
        self._restore_step(f)
        if self.state != 'LOCALIZED' or f is None:
            self.bad_since = None
            return
        now = time.time()
        if sm.looks_lost(*parts):
            self.bad_since = self.bad_since or now
            if now - self.bad_since >= self.lost_time:
                self.set_state('LOST', f'the LiDAR scan fits the map only {f:.0%} at the AMCL pose for '
                               f'{self.lost_time:.0f} s: press "Localize"')
                self.bad_since = None
                if self.cancel_lost and self.cancel_cli.service_is_ready():
                    self.cancel_cli.call_async(CancelGoal.Request())
            return
        self.bad_since = None
        if f >= sm.GOOD_FIT and now - self.saved_t >= self.save_period:
            if self.saved is None or math.hypot(pose[0] - self.saved[0], pose[1] - self.saved[1]) > 0.03 \
                    or abs(math.atan2(math.sin(pose[2] - self.saved[2]), math.cos(pose[2] - self.saved[2]))) > 0.05:
                try:
                    sm.save_last_pose(self.path, self.map_yaml, *pose, f)
                    self.saved = pose
                except OSError as exc:
                    self.get_logger().warn(f'could not save the last pose: {exc}')
            self.saved_t = now

    def _restore_step(self, f):
        """Once, at start: give AMCL the saved pose and keep it if the scan fits there."""
        if not self.restore:
            return
        if self.restoring is None:
            if self.state not in (None, '', 'UNKNOWN') or time.time() - self.t0 > 120.0:
                self.restore = False
                return
            pose = sm.load_last_pose(self.path, self.map_yaml)
            if pose is None:
                self.restore = False
                return
            if self.mapfit is None or self.scan is None or self.init_pub.get_subscription_count() == 0:
                return
            m = PoseWithCovarianceStamped()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
            _, _, m.pose.pose.orientation.z, m.pose.pose.orientation.w = quat_from_yaw(pose[2])
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.01
            cov[35] = 0.005
            m.pose.covariance = cov
            self.init_pub.publish(m)
            self.restoring = (time.time(), pose)
            self.get_logger().info(f'last pose given to AMCL: x={pose[0]:.2f} y={pose[1]:.2f} '
                                   f'yaw={math.degrees(pose[2]):.0f} deg (checking the scan fit)')
            return
        t, pose = self.restoring
        if time.time() - t < 3.0:
            return
        self.restore = False
        if self.state not in (None, '', 'UNKNOWN'):
            return
        if f is not None and f >= sm.GOOD_FIT:
            self.set_state('LOCALIZED', f'last pose restored, scan fit {f:.0%}')
        else:
            self.get_logger().warn(
                f'last saved pose does not match the scan (fit {"-" if f is None else f"{f:.0%}"}): '
                'the robot was moved; press "Localize"')


def main():
    rclpy.init()
    node = LocMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
