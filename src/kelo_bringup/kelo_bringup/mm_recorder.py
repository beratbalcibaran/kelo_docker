import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from kelo_bringup.common import data_dir


class MmRecorder(Node):
    def __init__(self):
        super().__init__('mm_recorder')
        self.declare_parameter('pose_topic', '/marvelmind/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('hedgehog_frame', 'hedgehog_link')
        self.declare_parameter('min_step_m', 0.05)
        self.declare_parameter('tf_wait_s', 0.2)
        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.hedgehog = g('hedgehog_frame').value
        self.min_step = float(g('min_step_m').value)
        self.tf_wait = float(g('tf_wait_s').value)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.path = os.path.join(data_dir(), time.strftime('mm_samples_%Y%m%d_%H%M%S.csv'))
        self.file = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t', 'mm_x', 'mm_y', 'map_x', 'map_y'])
        self.last = None
        self.count = 0
        self.create_subscription(PoseStamped, g('pose_topic').value, self._cb, 10)
        self.get_logger().info(f'Recording file: {self.path}')

    def _cb(self, msg):
        mx, my = msg.pose.position.x, msg.pose.position.y
        if self.last and math.hypot(mx - self.last[0], my - self.last[1]) < self.min_step:
            return
        try:
            tf = self.buf.lookup_transform(
                self.map_frame, self.hedgehog, Time.from_msg(msg.header.stamp),
                Duration(seconds=self.tf_wait))
        except TransformException:
            return
        self.last = (mx, my)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.writer.writerow([f'{t:.3f}', f'{mx:.4f}', f'{my:.4f}',
                              f'{tf.transform.translation.x:.4f}',
                              f'{tf.transform.translation.y:.4f}'])
        self.file.flush()
        self.count += 1

    def close(self):
        self.file.close()
        self.get_logger().info(f'{self.count} samples saved: {self.path}')


def main():
    rclpy.init(args=sys.argv)
    node = MmRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
