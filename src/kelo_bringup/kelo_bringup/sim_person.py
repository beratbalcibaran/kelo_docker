"""Simulator test helper: a person steps onto the planned path while the robot navigates.

For each navigation goal, once the robot is driving, a person (cylinder, r 0.25 m, 1.7 m tall) is added
on the global plan `ahead` metres in front of the robot. `mode:=stand` keeps the person standing on the
path, `mode:=cross` lets them walk once across it (2.4 m, then they are gone), `mode:=block` puts a wall of `block_width`
across the path instead (no way around: the robot has to wait). The person is removed when the robot has passed
them, when the goal ends, or after `max_time` seconds. Uses /sim/obstacles, so it works in the
simulator only.

    ros2 run kelo_bringup sim_person --ros-args -p mode:=stand -p ahead:=1.8
"""
import math
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String, UInt32
from tf2_ros import Buffer, TransformListener

from kelo_bringup.logic import wrap, yaw_from_quat


def point_ahead(path, x, y, ahead):
    """Point on the path `ahead` metres along it from the path point closest to (x, y), or None."""
    if len(path) < 2:
        return None
    i0 = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
    s = 0.0
    for i in range(i0, len(path) - 1):
        (ax, ay), (bx, by) = path[i], path[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg > 0.0 and s + seg >= ahead:
            a = (ahead - s) / seg
            return ax + a * (bx - ax), ay + a * (by - ay), math.atan2(by - ay, bx - ax)
        s += seg
    return None


def _yaw(q):
    return yaw_from_quat(q.x, q.y, q.z, q.w)


class SimPerson(Node):
    def __init__(self):
        super().__init__('sim_person')
        d = self.declare_parameter
        d('mode', 'stand')
        d('ahead', 1.8)
        d('min_remaining', 1.0)
        d('radius', 0.25)
        d('cross_speed', 0.3)
        d('start_delay', 2.0)
        d('max_time', 60.0)
        d('block_width', 6.0)
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        g = lambda n: self.get_parameter(n).value
        self.mode, self.ahead, self.r = g('mode'), float(g('ahead')), float(g('radius'))
        self.min_remaining, self.cross_speed = float(g('min_remaining')), float(g('cross_speed'))
        self.start_delay, self.max_time = float(g('start_delay')), float(g('max_time'))
        self.block_width = float(g('block_width'))
        self.map_frame, self.base_frame = g('map_frame'), g('base_frame')
        self.gt = None
        self.speed = 0.0
        self.plan = []
        self.goal = None
        self.served = set()
        self.moving_since = None
        self.person = None
        self.t_person = 0.0
        self.closest = math.inf
        self.contacts = self.contacts0 = 0
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(String, '/sim/obstacles', 10)
        self.create_subscription(Odometry, '/sim/ground_truth', self._gt, 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UInt32, '/sim/contacts', self._contacts, latched)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._status, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'sim_person: mode {self.mode}, {self.ahead:.1f} m ahead on the plan')

    def _gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, _yaw(p.orientation))
        t = msg.twist.twist
        self.speed = math.hypot(t.linear.x, t.linear.y)

    def _contacts(self, msg):
        self.contacts = msg.data

    def _plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _status(self, msg):
        active = [s for s in msg.status_list if s.status == GoalStatus.STATUS_EXECUTING]
        goal = bytes(active[-1].goal_info.goal_id.uuid) if active else None
        if goal != self.goal:
            self.get_logger().info('navigation goal active' if goal else 'navigation goal ended')
        self.goal = goal

    def _send(self, req):
        self.pub.publish(String(data=yaml.safe_dump(req)))

    def _clear(self, why):
        self._send({'clear': True})
        self.get_logger().info(f'person removed ({why}), closest distance to the robot '
                               f'{self.closest:.2f} m, collisions {self.contacts - self.contacts0}')
        self.person = None

    def _tick(self):
        if self.gt is None:
            return
        if self.person is not None:
            px, py = self.person
            d = math.hypot(self.gt[0] - px, self.gt[1] - py)
            self.closest = min(self.closest, d)
            if self.goal is None:
                self._clear('goal ended')
            elif self.closest < 1.5 and d > self.closest + 1.0:
                self._clear('robot passed')
            elif self.mode == 'cross' and time.time() - self.t_person > 2.4 / self.cross_speed:
                self._clear('crossed the path')
            elif time.time() - self.t_person > self.max_time:
                self._clear('time limit')
            return
        if self.goal is None or self.goal in self.served:
            self.moving_since = None
            return
        if self.speed < 0.08:
            self.moving_since = None
            return
        if self.moving_since is None:
            self.moving_since = time.time()
        if time.time() - self.moving_since < self.start_delay:
            return
        try:
            tf = self.buf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return
        mx, my = tf.transform.translation.x, tf.transform.translation.y
        target = point_ahead(self.plan, mx, my, self.ahead)
        if target is None or point_ahead(self.plan, mx, my, self.ahead + self.min_remaining) is None:
            return
        # map -> simulator world, from the robot pose in both frames
        dyaw = wrap(self.gt[2] - _yaw(tf.transform.rotation))
        c, s = math.cos(dyaw), math.sin(dyaw)
        dx, dy = target[0] - mx, target[1] - my
        px, py = self.gt[0] + c * dx - s * dy, self.gt[1] + s * dx + c * dy
        heading = target[2] + dyaw
        hx, hy = math.cos(heading), math.sin(heading)
        ob = {'type': 'cylinder', 'name': 'person', 'r': self.r, 'z': [0.0, 1.7]}
        if self.mode == 'block':
            ob = {'type': 'box', 'ahead': self.ahead, 'size': [0.3, self.block_width], 'z': [0.0, 1.0]}
        elif self.mode == 'cross':
            ob.update(path=[[px + 1.2 * hy, py - 1.2 * hx], [px - 1.2 * hy, py + 1.2 * hx]],
                      speed=self.cross_speed)
        else:
            ob['center'] = [px, py]
        self._send({'add': ob})
        self.served.add(self.goal)
        self.person = (px, py)
        self.t_person = time.time()
        self.closest = math.inf
        self.contacts0 = self.contacts
        self.get_logger().info(f'person on the path ({self.mode}) at world {px:.2f}, {py:.2f}, '
                               f'{math.hypot(dx, dy):.2f} m in front of the robot')


def main():
    rclpy.init()
    node = SimPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
