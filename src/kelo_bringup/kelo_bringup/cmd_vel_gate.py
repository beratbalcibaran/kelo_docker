import signal
import sys

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String

from kelo_bringup.logic import select_command


class CmdVelGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_gate')
        d = self.declare_parameter
        d('nav_topic', 'cmd_vel_safe')
        d('teleop_topic', '/cmd_vel_teleop')
        d('output_topic', '/cmd_vel')
        d('output_type', 'Twist')
        d('base_frame', 'base_link')
        d('stop_topic', '/kelo/safety_stop')
        d('state_topic', '/kelo/gate_state')
        d('publish_rate_hz', 20.0)
        d('command_timeout_s', 0.5)
        d('stop_timeout_s', 1.0)
        d('max_vx', 0.35)
        d('max_vy', 0.35)
        d('max_wz', 0.9)
        d('publish_zero_when_idle', True)
        g = lambda n: self.get_parameter(n).value

        self.out_type = str(g('output_type'))
        if self.out_type not in ('Twist', 'TwistStamped'):
            raise ValueError("output_type must be 'Twist' or 'TwistStamped'")
        self.base_frame = g('base_frame')
        self.cmd_timeout = float(g('command_timeout_s'))
        self.stop_timeout = float(g('stop_timeout_s'))
        self.limits = (float(g('max_vx')), float(g('max_vy')), float(g('max_wz')))
        self.zero_idle = bool(g('publish_zero_when_idle'))

        self.teleop = None
        self.nav = None
        self.stop = None
        self.state = None

        msg_cls = TwistStamped if self.out_type == 'TwistStamped' else Twist
        self.pub = self.create_publisher(msg_cls, g('output_topic'), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.state_pub = self.create_publisher(String, g('state_topic'), latched)
        self.create_subscription(Twist, g('nav_topic'), self._nav_cb, 10)
        self.create_subscription(Twist, g('teleop_topic'), self._teleop_cb, 10)
        self.create_subscription(Bool, g('stop_topic'), self._stop_cb, 10)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self._tick)
        self.get_logger().info(
            f"cmd_vel_gate: {g('nav_topic')} + {g('teleop_topic')} -> {g('output_topic')} "
            f"[{self.out_type}] limits vx={self.limits[0]} vy={self.limits[1]} wz={self.limits[2]}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _nav_cb(self, m):
        self.nav = (self._now(), m.linear.x, m.linear.y, m.angular.z)

    def _teleop_cb(self, m):
        self.teleop = (self._now(), m.linear.x, m.linear.y, m.angular.z)

    def _stop_cb(self, m):
        self.stop = (self._now(), bool(m.data))

    def _publish(self, vx, vy, wz):
        if self.out_type == 'TwistStamped':
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        else:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.pub.publish(msg)

    def _tick(self):
        vx, vy, wz, state = select_command(
            self._now(), self.teleop, self.nav, self.stop,
            self.cmd_timeout, self.stop_timeout, *self.limits)
        if state != self.state:
            self.state = state
            self.state_pub.publish(String(data=state))
            self.get_logger().info(f'gate state: {state}')
        if state != 'IDLE' or self.zero_idle:
            self._publish(vx, vy, wz)

    def send_zero(self):
        try:
            for _ in range(3):
                self._publish(0.0, 0.0, 0.0)
        except Exception:
            pass


def _sigterm(_signum, _frame):
    raise KeyboardInterrupt


def main():
    rclpy.init(args=sys.argv)
    signal.signal(signal.SIGTERM, _sigterm)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
