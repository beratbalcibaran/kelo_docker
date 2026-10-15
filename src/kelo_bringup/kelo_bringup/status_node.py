import sys

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class StatusNode(Node):
    def __init__(self):
        super().__init__('kelo_status')
        d = self.declare_parameter
        d('scan_topic', '/scan')
        d('odom_topic', '/odom')
        d('camera_info_topic', '/camera/camera/depth/camera_info')
        d('marvelmind_topic', '/marvelmind/pose')
        d('max_age_scan_s', 0.5)
        d('max_age_odom_s', 0.5)
        d('max_age_camera_s', 2.0)
        d('max_age_marvelmind_s', 2.0)
        d('summary_period_s', 10.0)
        d('camera_enabled', True)
        d('marvelmind_enabled', True)
        g = lambda n: self.get_parameter(n).value
        self.max_age = {'scan': g('max_age_scan_s'), 'odom': g('max_age_odom_s'),
                        'camera': g('max_age_camera_s'), 'marvelmind': g('max_age_marvelmind_s')}
        for key in ('camera', 'marvelmind'):
            if not g(f'{key}_enabled'):
                del self.max_age[key]
        self.last = {k: None for k in self.max_age}
        self.counts = {k: 0 for k in self.max_age}
        self.gate = ''
        self.mm_state = ''

        self.create_subscription(LaserScan, g('scan_topic'), lambda m: self._seen('scan'),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic'), lambda m: self._seen('odom'),
                                 qos_profile_sensor_data)
        if 'camera' in self.max_age:
            self.create_subscription(CameraInfo, g('camera_info_topic'),
                                     lambda m: self._seen('camera'), qos_profile_sensor_data)
        if 'marvelmind' in self.max_age:
            self.create_subscription(PoseStamped, g('marvelmind_topic'),
                                     lambda m: self._seen('marvelmind'), 10)
        self.create_subscription(String, '/kelo/gate_state', self._gate_cb, 10)
        self.loc_state = ''
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.loc_pub = self.create_publisher(String, '/kelo/localization_state', latched)
        self.create_subscription(String, '/kelo/localization_state_set', self._loc_cb, latched)
        self.create_subscription(String, '/kelo/mm_state', self._mm_cb, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._publish_diag)
        self.create_timer(float(g('summary_period_s')), self._summary)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _seen(self, key):
        self.last[key] = self._now()
        self.counts[key] += 1

    def _loc_cb(self, msg):
        if msg.data != self.loc_state:
            self.loc_state = msg.data
            self.loc_pub.publish(String(data=msg.data))

    def _gate_cb(self, msg):
        self.gate = msg.data

    def _mm_cb(self, msg):
        self.mm_state = msg.data

    def _age(self, key):
        return None if self.last[key] is None else self._now() - self.last[key]

    def _level(self, key):
        age = self._age(key)
        if age is None:
            return DiagnosticStatus.STALE, 'no data received yet'
        if age > self.max_age[key]:
            return DiagnosticStatus.ERROR, f'stale data ({age:.1f} s)'
        return DiagnosticStatus.OK, 'OK'

    def _publish_diag(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for key in self.max_age:
            level, text = self._level(key)
            if key == 'marvelmind' and level != DiagnosticStatus.OK:
                level = DiagnosticStatus.WARN
            st = DiagnosticStatus(level=level, name=f'kelo/{key}', message=text,
                                  hardware_id='kelo')
            age = self._age(key)
            st.values = [KeyValue(key='age_s', value='-' if age is None else f'{age:.2f}'),
                         KeyValue(key='count', value=str(self.counts[key]))]
            arr.status.append(st)
        arr.status.append(DiagnosticStatus(
            level=DiagnosticStatus.OK, name='kelo/gate', message=self.gate or '-', hardware_id='kelo'))
        arr.status.append(DiagnosticStatus(
            level=DiagnosticStatus.OK, name='kelo/marvelmind_cross_check',
            message=self.mm_state or '-', hardware_id='kelo'))
        self.diag_pub.publish(arr)

    def _summary(self):
        parts = []
        for key in self.max_age:
            level, text = self._level(key)
            parts.append(f"{key}={'OK' if level == DiagnosticStatus.OK else text}")
        self.get_logger().info(
            'STATUS ' + ' '.join(parts) + f" gate={self.gate or '-'} mm={self.mm_state or '-'} "
            f"lok={self.loc_state or '-'}")


def main():
    rclpy.init(args=sys.argv)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
