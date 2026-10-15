import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    target = sys.argv[2] if len(sys.argv) > 2 else 'map'
    source = sys.argv[3] if len(sys.argv) > 3 else 'base_link'
    rclpy.init()
    node = Node('wait_for_tf')
    buf = Buffer()
    TransformListener(buf, node)
    t0 = time.time()
    last = 0.0
    ok = False
    while rclpy.ok() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform(target, source, Time()):
            ok = True
            break
        if time.time() - last > 5.0:
            last = time.time()
            node.get_logger().warn(
                f'{target}->{source} TF missing. Waiting for a start pose: GUI "Localize" / '
                f'"Set Initial Pose", RViz "2D Pose Estimate" or initial_pose:= '
                f'({int(time.time() - t0)} s)')
    if ok:
        node.get_logger().info(f'{target}->{source} TF ready; starting Nav2.')
    else:
        node.get_logger().error('Timeout: no TF, starting Nav2 anyway (it may not become active).')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
