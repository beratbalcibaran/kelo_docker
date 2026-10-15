DESCRIPTION = """Named waypoint management and navigation.

usage:
  ros2 run kelo_bringup waypoint_tool add <name>
  ros2 run kelo_bringup waypoint_tool add-click <name>
  ros2 run kelo_bringup waypoint_tool list
  ros2 run kelo_bringup waypoint_tool remove <name>
  ros2 run kelo_bringup waypoint_tool go <name> [<name> ...]
  ros2 run kelo_bringup waypoint_tool loop <name> [...] [--count N]
"""
import argparse
import math
import os
import sys

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from kelo_bringup.common import waypoints_file
from kelo_bringup.logic import quat_from_yaw, yaw_from_quat


def load():
    path = waypoints_file()
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as f:
        return (yaml.safe_load(f) or {}).get('waypoints', {}) or {}


def save(wps):
    with open(waypoints_file(), 'w', encoding='utf-8') as f:
        yaml.safe_dump({'frame': 'map', 'waypoints': wps}, f, sort_keys=True, allow_unicode=True)


class Helper(Node):
    def __init__(self):
        super().__init__('waypoint_tool')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.clicked = None
        self.create_subscription(PoseStamped, '/goal_pose', self._click, 10)

    def _click(self, msg):
        self.clicked = msg

    def current_pose(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.buf.lookup_transform('map', 'base_link', Time())
            except TransformException:
                continue
            q = t.transform.rotation
            return {'x': round(t.transform.translation.x, 3),
                    'y': round(t.transform.translation.y, 3),
                    'yaw': round(yaw_from_quat(q.x, q.y, q.z, q.w), 4)}
        raise RuntimeError('map->base_link TF missing (is localization running?)')

    def wait_click(self, timeout=120.0):
        print('Click the target point in RViz with "2D Goal Pose" (timeout 120 s)...')
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.clicked is None and self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.clicked is None:
            raise RuntimeError('no click received')
        p = self.clicked.pose
        return {'x': round(p.position.x, 3), 'y': round(p.position.y, 3),
                'yaw': round(yaw_from_quat(p.orientation.x, p.orientation.y,
                                           p.orientation.z, p.orientation.w), 4)}


def navigate(names, count=1):
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    wps = load()
    missing = [n for n in names if n not in wps]
    if missing:
        sys.exit(f'Unknown waypoint: {missing}. Available: {sorted(wps)}')
    from kelo_bringup.send_goal import localization_ok
    nav = BasicNavigator()
    if not localization_ok(nav):
        sys.exit(5)
    nav.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')
    for round_no in range(count):
        for name in names:
            w = wps[name]
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = nav.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = float(w['x']), float(w['y'])
            qx, qy, qz, qw = quat_from_yaw(float(w['yaw']))
            goal.pose.orientation.z, goal.pose.orientation.w = qz, qw
            print(f'[{round_no + 1}/{count}] -> {name} ({w["x"]}, {w["y"]})')
            nav.goToPose(goal)
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb:
                    print(f'   remaining {fb.distance_remaining:.2f} m', end='\r')
            result = nav.getResult()
            print(f'\n   result: {result.name if hasattr(result, "name") else result}')
            if result != TaskResult.SUCCEEDED:
                print('Task failed, stopping.')
                return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for c in ('add', 'add-click', 'remove'):
        sub.add_parser(c).add_argument('name')
    sub.add_parser('list')
    sub.add_parser('go').add_argument('names', nargs='+')
    lp = sub.add_parser('loop')
    lp.add_argument('names', nargs='+')
    lp.add_argument('--count', type=int, default=2)
    args = ap.parse_args()

    if args.cmd == 'list':
        for n, w in sorted(load().items()):
            print(f"{n:20s} x={w['x']:.3f} y={w['y']:.3f} yaw={math.degrees(w['yaw']):.1f} deg")
        return 0
    if args.cmd == 'remove':
        wps = load()
        if wps.pop(args.name, None) is None:
            sys.exit(f'Not found: {args.name}')
        save(wps)
        print(f'Removed: {args.name}')
        return 0

    rclpy.init()
    try:
        if args.cmd in ('add', 'add-click'):
            node = Helper()
            pose = node.current_pose() if args.cmd == 'add' else node.wait_click()
            wps = load()
            wps[args.name] = pose
            save(wps)
            print(f'Saved: {args.name} = {pose}  ->  {waypoints_file()}')
            node.destroy_node()
            return 0
        names = args.names
        return navigate(names, getattr(args, 'count', 1))
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
