"""People-aware navigation: Nav2 treats a person differently from a box.

From the people of the objects node (/kelo/objects, class 'person', with their velocity) it publishes:
- /kelo/people_map (nav_msgs/OccupancyGrid, same geometry as /map): a personal space around every
  person and around where they walk in the next seconds (scene.people_zone): a high cost (not
  lethal) within `core` metres, a falling cost out to `social` metres, so planned paths pass people
  wide where there is room and still close by in a narrow passage, as before the people layer; the
  predicted positions up to `horizon` seconds ahead get the falling cost too, so the robot does not
  plan into a walking person's way. The
  global costmap reads it with a StaticLayer (people_layer in nav2_params.yaml); the local costmap
  has the person from the LiDAR and the camera.
- /speed_limit (nav2_msgs/SpeedLimit, percent): slower near people, `slow_percent` within
  `slow_near` metres of the footprint, back to full speed at `slow_far`. Not while the collision
  monitor slows the robot down itself (its slowdown zone): both together made 0.3 * 0.4 * 0.4 m/s,
  a crawl of 2 cm/s past a person.
- /kelo/person_blocking (std_msgs/Bool): true while a person (or their predicted way) is on the next
  `check_ahead` metres of the Nav2 plan. The behaviour tree (IsPersonBlocking, kelo_bt_nodes) then
  waits for the person to step aside instead of planning around them at once, as it does for a box;
  only a person who stays longer than the tree's person wait is passed, with the wide berth above.
- /kelo/people/markers: personal space rings and predicted paths for RViz.

The people layers only add cost: when the detector misses a person, Nav2 still sees an obstacle in the
LiDAR and camera layers and handles it like a box.
"""
import array
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point
from nav2_msgs.msg import CollisionMonitorState, SpeedLimit
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from kelo_bringup import scene
from kelo_bringup.common import load_hardware
from kelo_bringup.logic import yaw_from_quat

STALE_S = 1.0


def people_from(msg, label='person'):
    """[(x, y, vx, vy)] of the detections of class `label` (objects node: velocity in the orientation
    x/y of the hypothesis pose), in the message frame."""
    out = []
    for d in msg.detections:
        if not d.results or d.results[0].hypothesis.class_id != label:
            continue
        p, h = d.bbox.center.position, d.results[0].pose.pose.orientation
        out.append((p.x, p.y, h.x, h.y))
    return out


def transform_people(people, pose):
    """People (x, y, vx, vy) in a frame whose pose in the target frame is `pose` (x, y, yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(pose[0] + c * x - s * y, pose[1] + s * x + c * y, c * vx - s * vy, s * vx + c * vy)
            for x, y, vx, vy in people]


class Social(Node):
    def __init__(self):
        super().__init__('social')
        hw = load_hardware()
        d = self.declare_parameter
        d('core', 0.55)
        d('social', 1.2)
        d('lethal_s', 1.5)
        d('horizon', 3.0)
        d('check_ahead', 2.5)
        d('slow_near', 1.0)
        d('slow_far', 2.0)
        d('slow_percent', 40.0)
        d('collision_monitor_state', '/collision_monitor_state')
        d('core_value', 90)
        d('rate_hz', 5.0)
        d('map_rate_hz', 2.0)
        g = lambda n: self.get_parameter(n).value
        self.core, self.social = float(g('core')), float(g('social'))
        self.lethal_s, self.horizon = float(g('lethal_s')), float(g('horizon'))
        self.core_value = int(g('core_value'))
        self.check_ahead = float(g('check_ahead'))
        self.slow = (float(g('slow_near')), float(g('slow_far')), float(g('slow_percent')))
        self.fr = hw['frames']
        body = hw['body']
        self.half_l, self.half_w = body['length'] / 2.0, body['width'] / 2.0
        # a path point closer than this to a person means the robot body would enter their core zone
        self.clearance = self.core + self.half_w
        self.map_period = 1.0 / max(float(g('map_rate_hz')), 0.1)
        self.objects = None
        self.map_info = None
        self.plan = None
        self.nav_active = False
        self.last_map_t = -1e9
        self.map_had_people = True
        self.last_limit = None
        self.cm_slowing = False
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Detection3DArray, '/kelo/objects', self._objects, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map, latched)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._status, 10)
        self.create_subscription(CollisionMonitorState, g('collision_monitor_state'), self._cm_state, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/kelo/people_map', latched)
        self.limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.block_pub = self.create_publisher(Bool, '/kelo/person_blocking', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/kelo/people/markers', 10)
        self.create_timer(1.0 / max(float(g('rate_hz')), 0.5), self._tick)
        self.get_logger().info(f'social: personal space {self.core:.2f} m (lethal) / {self.social:.2f} m, '
                               f'prediction {self.horizon:.1f} s, slow to {self.slow[2]:.0f}% within '
                               f'{self.slow[0]:.1f} m')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _objects(self, msg):
        self.objects = (self._now(), msg)

    def _map(self, msg):
        self.map_info = (msg.header.frame_id or self.fr['map'], msg.info)
        self.map_had_people = True  # publish once for the new geometry

    def _plan(self, msg):
        if msg.poses:
            self.plan = (msg.header.frame_id or self.fr['map'],
                         np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses]))

    def _cm_state(self, msg):
        # sent on every change of the collision monitor's action
        self.cm_slowing = msg.action_type in (CollisionMonitorState.SLOWDOWN, CollisionMonitorState.STOP)

    def _status(self, msg):
        self.nav_active = any(s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list)

    def _pose(self, target, source):
        try:
            tf = self.buf.lookup_transform(target, source, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w)

    def _people(self, frame, now):
        """People in `frame`, [] when there are none or the objects are stale; None without TF."""
        if self.objects is None or now - self.objects[0] > STALE_S:
            return []
        msg = self.objects[1]
        people = people_from(msg)
        if not people:
            return []
        pose = self._pose(frame, msg.header.frame_id or self.fr['odom'])
        return None if pose is None else transform_people(people, pose)

    def _tick(self):
        now = self._now()
        base_people = self._people(self.fr['base'], now) or []
        self._speed_limit(base_people, now)
        self.block_pub.publish(Bool(data=self._blocking(base_people)))
        if self.map_info is not None and now - self.last_map_t >= self.map_period:
            self.last_map_t = now
            self._publish_map()
        self._markers(now)

    def _speed_limit(self, people, now):
        dist = None
        if people:
            d, _ = scene.footprint_distance([(p[0], p[1]) for p in people], self.half_l, self.half_l, self.half_w)
            dist = max(float(d.min()) - 0.25, 0.0)  # person centre -> body
        # steps of 10 %: few changes while a person moves (each one re-initialises MPPI)
        limit = round(scene.speed_limit(dist, *self.slow) / 10.0) * 10.0
        if self.cm_slowing:
            limit = 100.0  # the collision monitor is slowing down already
        # only on a change: every message makes MPPI re-initialise its motion model, which (sent
        # every second) left it creeping at 2 mm/s on the final turn at a goal
        if limit != self.last_limit:
            msg = SpeedLimit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = True
            # 0 = no limit for the controller server
            msg.speed_limit = 0.0 if limit >= 100.0 else float(limit)
            self.limit_pub.publish(msg)
            self.get_logger().info('near a person: speed limited to %d%%' % limit if limit < 100
                                   else 'no person near: full speed')
            self.last_limit = limit

    def _blocking(self, people):
        if not self.nav_active or self.plan is None or not people:
            return False
        pose = self._pose(self.plan[0], self.fr['base'])
        if pose is None:
            return False
        path = scene.to_base(self.plan[1], pose)
        return scene.path_blocked(path, people, self.clearance, self.check_ahead, self.lethal_s)

    def _publish_map(self):
        frame, info = self.map_info
        people = self._people(frame, self._now())
        if people is None:
            return
        if not people and not self.map_had_people:
            return  # nothing changed: the last grid was already empty
        o = info.origin
        oyaw = yaw_from_quat(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
        grid = scene.people_zone(people, (o.position.x, o.position.y, oyaw), (info.height, info.width),
                                 info.resolution, self.core, self.social, self.lethal_s, self.horizon,
                                 core_value=self.core_value)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.info = info
        msg.data = array.array('b', grid.tobytes())
        self.grid_pub.publish(msg)
        self.map_had_people = bool(people)

    def _markers(self, now):
        people = self._people(self.fr['odom'], now) or []
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for k, (x, y, vx, vy) in enumerate(people):
            for j, (r, a) in enumerate(((self.core, 0.5), (self.social, 0.2))):
                m = Marker()
                m.header.stamp, m.header.frame_id = stamp, self.fr['odom']
                m.ns, m.id, m.type = 'personal_space', 2 * k + j, Marker.LINE_STRIP
                m.pose.orientation.w = 1.0
                m.scale.x = 0.03
                m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.8, 1.0, a + 0.3
                for t in np.linspace(0.0, 2 * math.pi, 33):
                    m.points.append(Point(x=x + r * math.cos(t), y=y + r * math.sin(t), z=0.02))
                arr.markers.append(m)
            track, _ = scene.predict_track(x, y, vx, vy, self.horizon)
            if len(track) > 1:
                m = Marker()
                m.header.stamp, m.header.frame_id = stamp, self.fr['odom']
                m.ns, m.id, m.type = 'predicted_path', k, Marker.LINE_STRIP
                m.pose.orientation.w = 1.0
                m.scale.x = 0.06
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.3, 0.9
                m.points = [Point(x=float(px), y=float(py), z=0.03) for px, py in track]
                arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = Social()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
