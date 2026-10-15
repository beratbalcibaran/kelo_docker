import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

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
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, g('output_topic'), 10)
        self.create_subscription(Odometry, g('input_topic'), self._cb, qos_profile_sensor_data)
        self.get_logger().info(f'odom_filter: yaw k table {yaw_k}, linear error table {lin}')

    def _cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > self.max_age:
            self.get_logger().warn(f'stale odom dropped ({age:.2f} s)', throttle_duration_sec=2.0)
            return
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        if self.prev is not None:
            px, py, pyaw, pt = self.prev
            dt = stamp - pt
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
        self.prev = (p.position.x, p.position.y, yaw, stamp)

        out = Odometry()
        out.header.stamp = msg.header.stamp
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
        if self.publish_tf:
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
