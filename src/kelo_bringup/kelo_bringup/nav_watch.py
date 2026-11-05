import json
import math
import os
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup import nav_logic as nl
from kelo_bringup import scan_match as sm
from kelo_bringup.common import data_dir

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
FeedbackMessage = NavigateToPose.Impl.FeedbackMessage


def _yaw(q):
    return 2.0 * math.atan2(q.z, q.w)


class NavWatch(Node):
    def __init__(self):
        super().__init__('nav_watch')
        d = self.declare_parameter
        d('odom_topic', '/odom')
        d('scan_topic', '/scan')
        d('camera_info_topic', '/camera/camera/depth/camera_info')
        d('camera_enabled', True)
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        d('map_yaml', '')
        d('stuck_timeout_s', 60.0)
        d('refuse_goals_when_not_ready', True)
        d('auto_localize', True)
        d('auto_localize_after_s', 15.0)
        d('auto_relocalize', True)
        d('max_relocalize_per_goal', 2)
        d('incident_after_s', 20.0)
        g = lambda n: self.get_parameter(n).value
        self.map_frame, self.base_frame, self.map_yaml = g('map_frame'), g('base_frame'), g('map_yaml')
        self.camera = bool(g('camera_enabled'))
        self.refuse = bool(g('refuse_goals_when_not_ready'))
        self.auto_loc, self.auto_loc_after = bool(g('auto_localize')), float(g('auto_localize_after_s'))
        self.auto_reloc, self.max_reloc = bool(g('auto_relocalize')), int(g('max_relocalize_per_goal'))
        self.incident_after = float(g('incident_after_s'))
        self.stuck = nl.StuckTimer(float(g('stuck_timeout_s')))

        self.t0 = time.time()
        self.seen = {}
        self.offsets = []
        self.odom_conflict = False
        self.loc_state = None
        self.loc_changed = time.time()
        self.person = False
        self.cm_stop = ''
        self.grid = None
        self.path, self.plan_t = None, None
        self.remaining, self.recoveries = None, 0
        self.goal_id = None
        self.goal_t = None
        self.goal_pose = None
        self.last_goal_msg = (0.0, None)
        self.reloc_count = 0
        self.resume_pose = None
        self.resume_t = 0.0
        self.loc_proc, self.loc_kind = None, None
        self.auto_loc_done = False
        self.incident_goal = None
        self.incident_t = 0.0
        self.status_text, self.status_t = None, 0.0
        self.ready = (False, 'starting')
        self.health = None
        self.wait_since, self.wait_text = None, ''

        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)
        self.create_subscription(Odometry, g('odom_topic'), self._odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('scan_topic'), lambda m: self._seen('scan'), qos_profile_sensor_data)
        if self.camera:
            self.create_subscription(CameraInfo, g('camera_info_topic'), lambda m: self._seen('camera'),
                                     qos_profile_sensor_data)
        self.create_subscription(Bool, '/kelo/odom_conflict', self._conflict, LATCHED)
        self.create_subscription(String, '/kelo/localization_state', self._loc, LATCHED)
        self.create_subscription(Bool, '/kelo/person_blocking', lambda m: setattr(self, 'person', bool(m.data)), 10)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self._cm, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._costmap, 1)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._status, 10)
        self.create_subscription(FeedbackMessage, '/navigate_to_pose/_action/feedback', self._feedback, 10)
        self.create_subscription(PoseStamped, '/kelo/nav_goal', self._goal_msg, LATCHED)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_msg, 10)
        self.create_subscription(String, '/kelo/health', self._health, LATCHED)
        self.ready_pub = self.create_publisher(Bool, '/kelo/ready', LATCHED)
        self.status_pub = self.create_publisher(String, '/kelo/nav_status', LATCHED)
        self.goal_pub = self.create_publisher(PoseStamped, '/kelo/nav_goal', LATCHED)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        from rosbag2_interfaces.srv import Snapshot
        self.snap_cli = self.create_client(Snapshot, '/rosbag2_recorder/snapshot')
        self._Snapshot = Snapshot
        self.create_timer(1.0, self._safe_tick)
        self.get_logger().info(f'nav_watch: stuck timeout {self.stuck.timeout:.0f} s, auto_localize '
                               f'{self.auto_loc}, auto_relocalize {self.auto_reloc}')

    def _seen(self, key):
        self.seen[key] = time.time()

    def _odom(self, m):
        now = time.time()
        self.seen['odom'] = now
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st > 0:
            self.offsets.append(self.get_clock().now().nanoseconds * 1e-9 - st)
            del self.offsets[:-100]

    def _conflict(self, m):
        self.odom_conflict = bool(m.data)

    def _loc(self, m):
        if m.data != self.loc_state:
            prev, self.loc_state = self.loc_state, m.data
            self.loc_changed = time.time()
            if m.data == 'LOST':
                self._on_lost(prev)

    def _cm(self, m):
        self.cm_stop = m.polygon_name if m.action_type == CollisionMonitorState.STOP else ''

    def _costmap(self, m):
        i = m.info
        self.grid = nl.Grid(i.resolution, i.origin.position.x, i.origin.position.y, i.width, i.height, m.data)

    def _plan(self, m):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        self.plan_t = time.time()

    def _goal_msg(self, m):
        self.last_goal_msg = (time.time(), m)

    def _feedback(self, m):
        if self.goal_id is not None and bytes(m.goal_id.uuid) != self.goal_id:
            return
        self.remaining = float(m.feedback.distance_remaining)
        self.recoveries = int(m.feedback.number_of_recoveries)

    def _status(self, m):
        active = [s for s in m.status_list if s.status in ACTIVE]
        if not active:
            if self.goal_id is not None:
                self._goal_ended()
            return
        gid = bytes(active[-1].goal_info.goal_id.uuid)
        if gid != self.goal_id:
            self._new_goal(gid)

    def _new_goal(self, gid):
        now = time.time()
        t, msg = self.last_goal_msg
        resumed = now - self.resume_t < 5.0
        self.goal_id, self.goal_t = gid, now
        self.goal_pose = msg if (msg is not None and now - t < 3.0) else None
        self.remaining, self.recoveries, self.path, self.plan_t = None, 0, None, None
        self.stuck.reset(now)
        self.wait_since, self.wait_text = None, ''
        if not resumed:
            self.reloc_count = 0
        self.resume_pose = None
        if self.refuse:
            ready = self._readiness(now)
            if not ready[0]:
                self._cancel(f'goal refused: {ready[1]}')
                return
        if self.goal_pose is not None:
            p = self.goal_pose.pose
            self.get_logger().info(f'goal {"resumed" if resumed else "accepted"}: x={p.position.x:.2f} '
                                   f'y={p.position.y:.2f} yaw={math.degrees(_yaw(p.orientation)):.0f} deg')

    def _goal_ended(self):
        self.goal_id = None
        self.wait_since, self.wait_text = None, ''

    def _cancel(self, why):
        self.get_logger().warn(why)
        self._set_status(why, force=True)
        if self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(CancelGoal.Request())

    def _send(self, pose):
        if not self.nav.server_is_ready():
            self.get_logger().warn('navigation not running: goal not sent again')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.resume_t = time.time()
        self.goal_pub.publish(pose)
        self.last_goal_msg = (time.time(), pose)
        self.nav.send_goal_async(goal)
        return True

    def _on_lost(self, prev):
        self._incident('lost')
        if not self.auto_reloc or self.loc_proc is not None:
            return
        pose = self.goal_pose if self.goal_id is not None else None
        if self.goal_id is not None and self.reloc_count >= self.max_reloc:
            self._set_status(f'LOST {self.reloc_count + 1}x on this goal: press "Localize" and send the goal again',
                             force=True)
            return
        self.reloc_count += 1 if self.goal_id is not None else 0
        self.resume_pose = pose
        self._run_localize('relocalize')

    def _run_localize(self, kind):
        cmd = ['ros2', 'run', 'kelo_bringup', 'localize']
        last = sm.load_last_pose(sm.last_pose_file(data_dir()), self.map_yaml) if kind == 'relocalize' else None
        if last is not None:
            cmd += ['--force', '--x', f'{last[0]:.3f}', '--y', f'{last[1]:.3f}', '--yaw', f'{math.degrees(last[2]):.1f}']
        elif kind == 'relocalize':
            cmd += ['--force']
        self.get_logger().info(f'{kind}: {" ".join(cmd[3:])}')
        self._set_status('localizing by itself' + (' (from the last good pose)' if last else ''), force=True)
        self.loc_proc = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr, start_new_session=True)
        self.loc_kind = (kind, last is not None)

    def _poll_localize(self):
        if self.loc_proc is None:
            return
        rc = self.loc_proc.poll()
        if rc is None:
            return
        kind, from_last = self.loc_kind
        self.loc_proc, self.loc_kind = None, None
        if rc in (0, 3):
            self.get_logger().info(f'{kind}: localized')
            if self.resume_pose is not None:
                self.get_logger().info('sending the interrupted goal again')
                self._send(self.resume_pose)
            return
        if from_last:
            self.get_logger().warn(f'{kind}: the last good pose did not fit, searching the whole map')
            cmd = ['ros2', 'run', 'kelo_bringup', 'localize', '--force', '--global']
            self.loc_proc = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr, start_new_session=True)
            self.loc_kind = (kind, False)
            return
        self.resume_pose = None
        self._set_status('could not localize by itself: give the pose with "Set Initial Pose", then "Localize"',
                         force=True)

    def _incident(self, why):
        now = time.time()
        if now - self.incident_t < 60.0 or not self.snap_cli.service_is_ready():
            return
        self.incident_t = now
        self.get_logger().warn(f'incident ({why}): last minutes of sensor data saved to /data/incidents')
        self.snap_cli.call_async(self._Snapshot.Request())

    def _set_status(self, text, force=False):
        now = time.time()
        if text != self.status_text or force or now - self.status_t > 5.0:
            if text.split(' (')[0] != (self.status_text or '').split(' (')[0]:
                self.get_logger().info(f'STATUS {text}')
            self.status_text, self.status_t = text, now
            self.status_pub.publish(String(data=text))

    def _robot_xy(self):
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame, Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y

    def _safe_tick(self):
        try:
            self._tick()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'nav_watch tick failed: {type(exc).__name__}: {exc}', throttle_duration_sec=10.0)

    def _tick(self):
        now = time.time()
        self._poll_localize()
        nav_up = self.nav.server_is_ready()
        ready = self._readiness(now)
        if ready != self.ready:
            if ready[0] != self.ready[0]:
                self.ready_pub.publish(Bool(data=ready[0]))
                if ready[0]:
                    self.get_logger().info('READY: goals are accepted')
                else:
                    self.get_logger().warn(f'NOT READY: {ready[1]}')
            self.ready = ready
        self._maybe_auto_localize(now, nav_up)
        if self.goal_id is None:
            if self.loc_proc is None:
                self._set_status(('READY' + ('' if ready[1] == 'ready' else f' ({ready[1]})')) if ready[0]
                                 else f'NOT READY: {ready[1]}')
            return
        xy = self._robot_xy()
        blocked = nl.first_blocked(self.path, xy, self.grid) if (xy and self.path) else None
        goal_cost = None
        if self.goal_pose is not None and self.grid is not None:
            p = self.goal_pose.pose.position
            goal_cost = self.grid.cost(p.x, p.y)
        if self.plan_t is not None or (self.goal_t is not None and now - self.goal_t < 5.0):
            plan_age = 0.0
        else:
            plan_age = None
        if self.loc_proc is not None:
            self.stuck.pause(now)
            stuck = False
        else:
            stuck = self.stuck.step(now, self.remaining, xy)
        why = nl.wait_reason(ready, self.person, self.cm_stop, blocked, goal_cost, plan_age, self.recoveries,
                             progressing=self.stuck.progressing(now))
        moving = self.remaining is not None and not why
        if why:
            if self.wait_text != why:
                self.wait_text = why
                self.wait_since = self.wait_since or now
            if self.wait_since and now - self.wait_since > self.incident_after and self.incident_goal != self.goal_id:
                self.incident_goal = self.goal_id
                self._incident(f'waiting {now - self.wait_since:.0f} s: {why}')
            self._set_status(f'WAITING: {why}')
        else:
            self.wait_since, self.wait_text = None, ''
            self._set_status('DRIVING' + (f' ({self.remaining:.1f} m left)' if moving else ''))
        if stuck:
            self._incident('stuck')
            self._cancel(f'goal given up after {self.stuck.timeout:.0f} s without progress: '
                         f'{why or "the robot did not move"}')

    def _health(self, msg):
        try:
            self.health = json.loads(msg.data)
        except ValueError:
            pass

    def _readiness(self, now):
        off = nl.clock_offset(self.offsets[-50:])
        loc = 'LOCALIZING' if self.loc_proc is not None else self.loc_state
        return nl.readiness(now, self.seen, off, self.odom_conflict, loc, self.nav.server_is_ready(), self.camera,
                            nl.broken_nav_nodes(self.health))

    def _maybe_auto_localize(self, now, nav_up):
        if not self.auto_loc or self.auto_loc_done or self.loc_proc is not None:
            return
        if self.loc_state not in (None, '', 'UNKNOWN'):
            self.auto_loc_done = True
            return
        fresh = all(k in self.seen and now - self.seen[k] < 1.0 for k in ('odom', 'scan'))
        if not fresh or now - self.loc_changed < self.auto_loc_after or now - self.t0 < self.auto_loc_after:
            return
        self.auto_loc_done = True
        self.get_logger().info('nobody localized the robot: localizing by itself')
        self._run_localize('auto localize')


def main():
    rclpy.init(args=sys.argv)
    node = NavWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.loc_proc is not None and node.loc_proc.poll() is None:
            try:
                os.killpg(node.loc_proc.pid, 2)
            except OSError:
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
