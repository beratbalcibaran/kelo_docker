import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from kelo_bringup.nav_logic import clock_offset
from kelo_bringup.logic import (lin_speed_corrector, quat_from_yaw, wrap, yaw_from_quat,
                                yaw_rate_corrector)


class OdomFilter(Node):
    def __init__(self):
        super().__init__('odom_filter')
        d = self.declare_parameter
        d('input_topic', '/odom')
        d('output_topic', '/odom_corrected')
        d('odom_frame', 'odom')
        d('base_frame', 'base_link')
        d('yaw_k_wz', [0.0])
        d('yaw_k_k', [1.0])
        d('lin_err_v', [0.0])
        d('lin_err_e', [0.0])
        d('max_age_s', 0.5)
        d('publish_tf', True)
        d('restamp', True)
        d('restamp_above_s', 0.1)
        g = lambda n: self.get_parameter(n).value
        yaw_k = {'wz': list(g('yaw_k_wz')), 'k': list(g('yaw_k_k'))}
        lin = {'v': list(g('lin_err_v')), 'e': list(g('lin_err_e'))}
        self.yaw_corr = yaw_rate_corrector(yaw_k)
        self.lin_corr = lin_speed_corrector(lin)
        self.scale = (1.0, 1.0)
        self.max_age = float(g('max_age_s'))
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.publish_tf = bool(g('publish_tf'))
        self.prev = None
        self.x = self.y = self.yaw = 0.0
        self.restamp, self.restamp_above = bool(g('restamp')), float(g('restamp_above_s'))
        self.offsets = []
        self.offset = 0.0
        self.sent = None
        self.conflict = False
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, g('output_topic'), 10)
        self.conflict_pub = self.create_publisher(Bool, '/kelo/odom_conflict', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.conflict_pub.publish(Bool(data=False))
        self.create_subscription(Odometry, g('input_topic'), self._cb, qos_profile_sensor_data)
        if self.publish_tf:
            self.create_subscription(TFMessage, '/tf', self._tf_cb, 50)
        self.get_logger().info(f'odom_filter: yaw k table {yaw_k}, linear error table {lin}')

    def _tf_cb(self, msg):
        if self.conflict or self.sent is None:
            return
        for t in msg.transforms:
            if t.header.frame_id != self.odom_frame or t.child_frame_id != self.base_frame:
                continue
            tr = t.transform.translation
            if all(abs(tr.x - x) > 1e-4 or abs(tr.y - y) > 1e-4 for x, y in self.sent):
                self.conflict = True
                self.conflict_pub.publish(Bool(data=True))
                self.get_logger().error(
                    'another node publishes odom -> base_link (the robot driver?): odom_filter stops its own TF. '
                    'Set robot.odom_mode to "driver" in /data/hardware.yaml (panel: Find robot -> Apply) or start '
                    'the driver with publish_tf:=false')
                return

    def _stamp(self, msg):
        raw = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.restamp or raw <= 0.0:
            return raw
        self.offsets.append(now - raw)
        del self.offsets[:-60]
        if len(self.offsets) >= 10:
            est = clock_offset(self.offsets)
            new = est if abs(est) > self.restamp_above else 0.0
            if abs(new - self.offset) > 0.05:
                if new:
                    self.get_logger().warn(f'robot clock {new:+.2f} s off this computer: odometry restamped '
                                           '(scripts/robot.sh time fixes the robot clock)')
                else:
                    self.get_logger().info('robot clock in sync: odometry stamps kept')
                self.offset = new
        return raw + self.offset

    def _cb(self, msg):
        stamp = self._stamp(msg)
        raw = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > self.max_age:
            self.get_logger().warn(f'stale odom dropped ({age:.2f} s)', throttle_duration_sec=2.0)
            return
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        if self.prev is not None:
            px, py, pyaw, pt = self.prev
            dt = raw - pt
            if dt <= 0.0:
                return
            dx, dy = p.position.x - px, p.position.y - py
            dyaw_o = wrap(yaw - pyaw)
            dyaw = self.yaw_corr(dyaw_o / dt) * dt
            c0, s0 = math.cos(-pyaw), math.sin(-pyaw)
            lx, ly = c0 * dx - s0 * dy, s0 * dx + c0 * dy
            v_o = math.hypot(lx, ly) / dt
            lin = self.lin_corr(v_o) / v_o if v_o > 1e-6 else 1.0
            lx, ly = lx * lin, ly * lin
            mid = self.yaw + dyaw / 2.0
            self.x += math.cos(mid) * lx - math.sin(mid) * ly
            self.y += math.sin(mid) * lx + math.cos(mid) * ly
            self.yaw = wrap(self.yaw + dyaw)
            self.scale = (lin, dyaw / dyaw_o if abs(dyaw_o) > 1e-9 else self.scale[1])
        self.prev = (p.position.x, p.position.y, yaw, raw)

        out = Odometry()
        sec = math.floor(stamp)
        out.header.stamp.sec, out.header.stamp.nanosec = int(sec), min(int((stamp - sec) * 1e9), 999999999)
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y = self.x, self.y
        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = qz, qw
        out.twist = msg.twist
        out.twist.twist.linear.x *= self.scale[0]
        out.twist.twist.linear.y *= self.scale[0]
        out.twist.twist.angular.z *= self.scale[1]
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.1
        out.pose.covariance = cov
        out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)
        if self.publish_tf and not self.conflict:
            self.sent = (self.sent or ())[-9:] + ((self.x, self.y),)
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf.sendTransform(t)


def main():
    rclpy.init(args=sys.argv)
    node = OdomFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
