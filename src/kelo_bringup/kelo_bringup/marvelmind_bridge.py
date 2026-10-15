import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from marvelmind_ros2_msgs.msg import HedgePositionAngle
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster

from kelo_bringup.common import load_alignment
from kelo_bringup.logic import quat_from_yaw

FLAG_NO_DATA = 1 << 0


class MarvelmindBridge(Node):
    def __init__(self):
        super().__init__('marvelmind_bridge')
        self.declare_parameter('raw_topic', '/hedgehog_pos_ang')
        self.declare_parameter('pose_topic', '/marvelmind/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mm_frame', 'marvelmind_map')
        self.declare_parameter('hedgehog_address', 0)
        g = self.get_parameter
        self.mm_frame = g('mm_frame').value
        self.address = int(g('hedgehog_address').value)
        self.last_ts = None
        self.dropped = 0
        self.accepted = 0

        self.pub = self.create_publisher(PoseStamped, g('pose_topic').value, 10)
        self.create_subscription(HedgePositionAngle, g('raw_topic').value,
                                 self._cb, qos_profile_sensor_data)
        self._publish_static_tf(g('map_frame').value)
        self.create_timer(30.0, self._report)

    def _publish_static_tf(self, map_frame):
        align = load_alignment()
        if align is None:
            self.get_logger().warn(
                'No Marvelmind alignment file: map->marvelmind_map TF is NOT published. '
                'Create it while mapping with mm_recorder + mm_calibrate.')
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = map_frame
        t.child_frame_id = self.mm_frame
        t.transform.translation.x = float(align['x'])
        t.transform.translation.y = float(align['y'])
        qx, qy, qz, qw = quat_from_yaw(float(align['yaw']))
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(t)
        self.get_logger().info(
            f"Alignment loaded: x={align['x']:.3f} y={align['y']:.3f} yaw={align['yaw']:.4f} rad")

    def _cb(self, msg):
        if self.address and msg.address != self.address:
            return
        if msg.flags & FLAG_NO_DATA or msg.timestamp_ms == self.last_ts:
            self.dropped += 1
            return
        self.last_ts = msg.timestamp_ms
        self.accepted += 1
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.mm_frame
        out.pose.position.x = float(msg.x_m)
        out.pose.position.y = float(msg.y_m)
        out.pose.orientation.w = 1.0
        self.pub.publish(out)

    def _report(self):
        self.get_logger().info(f'Marvelmind: accepted={self.accepted} dropped={self.dropped}')


def main():
    rclpy.init(args=sys.argv)
    node = MarvelmindBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
