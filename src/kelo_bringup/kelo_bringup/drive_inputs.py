import math

import numpy as np
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import TransformException
from vision_msgs.msg import Detection2DArray, Detection3DArray

from kelo_bringup import scene
from kelo_bringup.logic import yaw_from_quat

CMD_TIMEOUT_S = 0.5
OBJECTS_TIMEOUT_S = 1.0
DETECTIONS_TIMEOUT_S = 1.0
SOCIAL_TIMEOUT_S = 1.0
MAX_CLICK_GOAL_M = 15.0


def cloud_fields(msg, names):
    offs = {f.name: f.offset for f in msg.fields}
    if msg.width * msg.height == 0 or any(n not in offs for n in names):
        return np.zeros((0, len(names)))
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    return np.stack([raw[:, offs[n]:offs[n] + 4].copy().view('<f4')[:, 0] for n in names], axis=1).astype(float)


class DriveInputs:
    def __init__(self, node, hw, tf_buffer, memory=False):
        self.node, self.buf = node, tf_buffer
        self.base, self.map = hw['frames']['base'], hw['frames']['map']
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = -1e9
        self.plan, self.plan_t = None, -1e9
        self.nav_active, self.nav_seen = False, False
        self.objects, self.objects_t = None, -1e9
        self.dets, self.dets_t = None, -1e9
        self.memory, self.memory_t = None, -1e9
        self.blocking, self.blocking_t = False, -1e9
        self.limit = None
        self.goal_client = self.cancel_client = None
        self.goal_pub = None
        self.goal_note = None
        self.loc_state = None
        node.create_subscription(String, '/kelo/localization_state', self._loc, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        topic = hw['topics']['cmd_vel']
        if hw['robot']['cmd_vel_type'] == 'TwistStamped':
            node.create_subscription(TwistStamped, topic, lambda m: self._cmd(m.twist), 10)
        else:
            node.create_subscription(Twist, topic, self._cmd, 10)
        node.create_subscription(Path, '/plan', self._plan, 10)
        node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._status, 10)
        node.create_subscription(Detection3DArray, '/kelo/objects', self._objects, 10)
        node.create_subscription(Detection2DArray, '/kelo/detections', self._dets, 10)
        if memory:
            node.create_subscription(PointCloud2, '/kelo/obstacle_memory', self._memory, 2)
        node.create_subscription(Bool, '/kelo/person_blocking', self._blocking, 10)
        node.create_subscription(SpeedLimit, '/speed_limit', self._limit, 10)

    def on_click(self, handler):
        self.node.create_subscription(
            PointStamped, '/kelo/view_click',
            lambda m: handler(m.header.frame_id, m.point.x, m.point.y, m.point.z > 0.5), 10)

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _loc(self, m):
        self.loc_state = m.data

    def _cmd(self, m):
        new = (m.linear.x, m.linear.y, m.angular.z)
        a = 0.35 if self._now() - self.cmd_t < CMD_TIMEOUT_S else 1.0
        self.cmd = tuple((1 - a) * o + a * n for o, n in zip(self.cmd, new))
        self.cmd_t = self._now()

    def _plan(self, m):
        if m.poses:
            self.plan = (m.header.frame_id or self.map,
                         np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses]))
            self.plan_t = self._now()

    def _status(self, m):
        self.nav_seen = True
        self.nav_active = any(s.status == GoalStatus.STATUS_EXECUTING for s in m.status_list)

    def _objects(self, m):
        self.objects, self.objects_t = m, self._now()

    def _dets(self, m):
        self.dets, self.dets_t = m, self._now()

    def _memory(self, m):
        self.memory, self.memory_t = m, self._now()

    def _blocking(self, m):
        self.blocking, self.blocking_t = m.data, self._now()

    def _limit(self, m):
        self.limit = m.speed_limit if m.percentage and 0.0 < m.speed_limit < 100.0 else None

    def social_state(self):
        now = self._now()
        alive = now - self.blocking_t < SOCIAL_TIMEOUT_S
        return self.blocking and alive and self.nav_active, self.limit if alive else None

    def memory_in_base(self, min_age=0.3):
        if self.memory is None or self._now() - self.memory_t > 2.0:
            return np.zeros((0, 4))
        pts = cloud_fields(self.memory, ('x', 'y', 'z', 'age'))
        pts = pts[pts[:, 3] >= min_age]
        pose = self.pose_in(self.memory.header.frame_id)
        if pose is None or len(pts) == 0:
            return np.zeros((0, 4))
        return np.column_stack([scene.to_base(pts[:, :2], pose), pts[:, 2:]])

    def send_goal(self, x, y):
        dist = math.hypot(x, y)
        if dist < 0.3:
            return self.say('goal too close to the robot')
        if dist > MAX_CLICK_GOAL_M:
            return self.say(f'goal further than {MAX_CLICK_GOAL_M:.0f} m: click closer')
        if self.loc_state is not None and self.loc_state != 'LOCALIZED':
            return self.say(f'not localized ({self.loc_state}): press Localize first')
        pose = self.pose_in(self.map)
        if pose is None:
            return self.say('no map pose (navigation not running or not localized)')
        if self.goal_client is None:
            self.goal_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        if not self.goal_client.server_is_ready():
            return self.say('navigation is not running: no goal sent')
        (mx, my), = scene.from_base([(x, y)], pose)
        yaw = pose[2] + math.atan2(y, x)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(mx), float(my)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        if self.goal_pub is None:
            self.goal_pub = self.node.create_publisher(PoseStamped, '/kelo/nav_goal', QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.goal_pub.publish(goal.pose)
        self.goal_client.send_goal_async(goal)
        self.node.get_logger().info(f'clicked goal: map {mx:.2f}, {my:.2f}, {math.degrees(yaw):.0f} deg')
        return self.say(f'goal sent: {dist:.1f} m away')

    def cancel_goal(self):
        if self.goal_client is None:
            self.goal_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        if not self.goal_client.server_is_ready():
            return self.say('navigation is not running')
        if self.cancel_client is None:
            self.cancel_client = self.node.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.cancel_client.call_async(CancelGoal.Request())
        return self.say('goal cancelled')

    def say(self, text):
        self.goal_note = (self._now(), text)
        self.node.get_logger().info(f'view click: {text}')
        return text

    def note(self, keep=3.0):
        if self.goal_note is None or self._now() - self.goal_note[0] > keep:
            return None
        return self.goal_note[1]

    def twist(self):
        return self.cmd if self._now() - self.cmd_t < CMD_TIMEOUT_S else (0.0, 0.0, 0.0)

    def pose_in(self, frame):
        try:
            tf = self.buf.lookup_transform(frame, self.base, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w)

    def plan_in_base(self, max_length=None):
        if self.plan is None:
            return None, 0.0
        if self.nav_seen and not self.nav_active:
            return None, 0.0
        if not self.nav_seen and self._now() - self.plan_t > 5.0:
            return None, 0.0
        pose = self.pose_in(self.plan[0])
        if pose is None:
            return None, 0.0
        path, total = scene.remaining_path(scene.to_base(self.plan[1], pose), max_length)
        if total < 0.05:
            return None, 0.0
        return path, total

    def objects_in_base(self):
        if self.objects is None or self._now() - self.objects_t > OBJECTS_TIMEOUT_S:
            return []
        pose = self.pose_in(self.objects.header.frame_id)
        if pose is None:
            return []
        c, s = math.cos(pose[2]), math.sin(pose[2])
        out = []
        for d in self.objects.detections:
            p = d.bbox.center.position
            (x, y), = scene.to_base([(p.x, p.y)], pose)
            hyp = d.results[0] if d.results else None
            label = hyp.hypothesis.class_id if hyp else 'obstacle'
            score = hyp.hypothesis.score if hyp else 0.0
            vx = hyp.pose.pose.orientation.x if hyp else 0.0
            vy = hyp.pose.pose.orientation.y if hyp else 0.0
            out.append((d.id, label, score, float(x), float(y), d.bbox.size.x, d.bbox.size.y, d.bbox.size.z,
                        c * vx + s * vy, -s * vx + c * vy))
        return out

    def detections(self):
        if self.dets is None or self._now() - self.dets_t > DETECTIONS_TIMEOUT_S:
            return []
        out = []
        for d in self.dets.detections:
            if not d.results:
                continue
            h = d.results[0]
            cx, cy = d.bbox.center.position.x, d.bbox.center.position.y
            p = h.pose.pose.position
            xy = None if math.isnan(p.x) else (p.x, p.y)
            out.append((h.hypothesis.class_id, h.hypothesis.score, cx - d.bbox.size_x / 2.0,
                        cy - d.bbox.size_y / 2.0, cx + d.bbox.size_x / 2.0, cy + d.bbox.size_y / 2.0, xy))
        return out
