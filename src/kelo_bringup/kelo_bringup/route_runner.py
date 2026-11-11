import copy
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from kelo_bringup import route as rt
from kelo_bringup.logic import quat_from_yaw, yaw_from_quat

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
FINISHED = ('IDLE', 'PAUSED', 'DONE', 'DONE_WITH_SKIPS', 'FAILED')


class RouteRunner(Node):
    """Visits the stations of a route one after the other. Stations the robot passes through
    without stopping are joined with the next stop into one continuous path (NavigateThroughPoses),
    so the robot takes the corner in one smooth movement; a single stop is a NavigateToPose goal
    (supervised by nav_watch like any other goal)."""

    def __init__(self):
        super().__init__('route_runner')
        d = self.declare_parameter
        d('routes_file', '')
        d('pass_radius', 0.5)
        d('retries', 2)
        d('retry_wait_s', 3.0)
        d('map_frame', 'map')
        d('no_path_s', 15.0)
        d('stuck_s', 60.0)
        d('clearance', 0.30)
        d('arrive_radius', 0.35)
        d('settle_s', 3.0)
        g = lambda n: self.get_parameter(n).value
        self.path = g('routes_file') or rt.routes_file()
        self.pass_radius, self.retries = float(g('pass_radius')), int(g('retries'))
        self.retry_wait, self.frame = float(g('retry_wait_s')), g('map_frame')
        self.no_path_s, self.stuck_s = float(g('no_path_s')), float(g('stuck_s'))
        self.clearance = float(g('clearance'))
        self.arrive_radius, self.settle_s = float(g('arrive_radius')), float(g('settle_s'))
        self.history = []
        self.to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.state_pub = self.create_publisher(String, '/kelo/route/state', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/kelo/route/markers', LATCHED)
        self.create_subscription(String, '/kelo/route/command', self._command, 10)
        self.create_subscription(String, '/kelo/localization_state', self._loc, LATCHED)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        for name in ('navigate_to_pose', 'navigate_through_poses'):
            self.create_subscription(GoalStatusArray, f'/{name}/_action/status', self._goal_status, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map, LATCHED)
        self.create_subscription(String, '/kelo/nav_status', self._nav_status, LATCHED)
        self.localized, self.pose, self.map = None, None, None
        self.no_path_since = None
        self.own_ids, self.send_no = set(), 0
        self.foreign_t = 0.0
        self._reset('IDLE', 'no route running')
        self.create_timer(0.2, self._tick)
        self._publish()
        self.get_logger().info(f'route runner ready, routes in {self.path}')

    # ---------------------------------------------------------------- state
    def _reset(self, status, message):
        self.status, self.message = status, message
        self.name, self.stations, self.plan = '', [], None
        self.goal_handle, self.goal_id, self.goal_sent_t = None, None, 0.0
        self.result, self.remaining, self.leg = None, None, []
        self.tries, self.wait_until, self.failed_in_row = 0, 0.0, 0
        self.visits, self.skipped, self.bad = [], [], {}
        self.started_t, self.came_from = None, None
        self.best_remaining, self.progress_t = None, 0.0

    def _here(self):
        return self.stations[self.plan.index]['name'] if self.plan else ''

    # ------------------------------------------------------------ callbacks
    def _map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.map = (grid, {'resolution': msg.info.resolution,
                           'origin': [msg.info.origin.position.x, msg.info.origin.position.y]})

    def _nav_status(self, msg):
        if msg.data.startswith('WAITING: no path'):
            self.no_path_since = self.no_path_since or time.time()
        else:
            self.no_path_since = None

    def _loc(self, msg):
        self.localized = msg.data == 'LOCALIZED'

    def _update_pose(self):
        try:
            t = self.buf.lookup_transform(self.frame, 'base_link', Time())
        except TransformException:
            return
        q = t.transform.rotation
        self.pose = (t.transform.translation.x, t.transform.translation.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        now = time.time()
        self.history.append((now, self.pose))
        self.history = [h for h in self.history if now - h[0] <= self.settle_s + 1.0]

    def _standing(self, now):
        old = [h for h in self.history if now - h[0] >= self.settle_s]
        if not old or now - self.goal_sent_t < self.settle_s:
            return False
        x0, y0, a0 = old[-1][1]
        return all(math.hypot(p[0] - x0, p[1] - y0) < 0.03 and abs(math.remainder(p[2] - a0, 2 * math.pi)) < 0.08
                   for t, p in self.history if t >= old[-1][0])

    def _close_enough(self, now):
        if len(self.leg) != 1 or self.pose is None:
            return False
        s = self.stations[self.plan.index]
        return math.hypot(self.pose[0] - s['x'], self.pose[1] - s['y']) <= self.arrive_radius and self._standing(now)

    def _goal_status(self, msg):
        foreign = any(s.status in ACTIVE and bytes(s.goal_info.goal_id.uuid) not in self.own_ids
                      and time.time() - self.goal_sent_t > 1.0 for s in msg.status_list)
        if not foreign:
            return
        self.foreign_t = time.time()
        if self.status in ('RUNNING', 'DWELL', 'WAITING') and self.plan is not None:
            self._pause_for_other_goal()

    def _pause_for_other_goal(self):
        self.get_logger().warn('another navigation goal was sent: route paused')
        self.goal_handle = self.goal_id = None
        self.send_no += 1
        self.result = None
        self.status, self.message = 'PAUSED', 'paused: another goal was sent (Resume continues the route)'
        self._publish()

    # ------------------------------------------------------------- commands
    def _command(self, msg):
        self._update_pose()
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            cmd = {'cmd': msg.data.strip()}
        c = cmd.get('cmd', '')
        if c == 'start':
            self._start(cmd)
        elif c == 'pause' and self.status in ('RUNNING', 'DWELL', 'WAITING'):
            self._cancel()
            self.status, self.message = 'PAUSED', 'paused'
            self.get_logger().info('route paused')
        elif c == 'resume' and self.status == 'PAUSED' and self.plan is not None:
            self.status, self.message, self.tries = 'RUNNING', 'resumed', 0
            self.get_logger().info('route resumed')
        elif c == 'skip' and self.plan is not None and self.status in ('RUNNING', 'DWELL', 'PAUSED', 'WAITING'):
            self._cancel()
            self.skipped.append(self._here())
            self.get_logger().info(f'station "{self._here()}" skipped by the operator')
            self._next()
        elif c == 'stop':
            self._cancel()
            name, visits, skipped = self.name, self.visits, self.skipped
            self._reset('IDLE', 'stopped by the operator')
            self.name, self.visits, self.skipped = name, visits, skipped
            self.get_logger().info('route stopped by the operator')
        else:
            self.get_logger().warn(f'route command ignored in state {self.status}: {msg.data}')
        self._publish()

    def _start(self, cmd):
        self._cancel()
        name = str(cmd.get('route', ''))
        try:
            routes = rt.load_routes(self.path)
            if name not in routes:
                raise ValueError(f'unknown route "{name}" (known: {", ".join(sorted(routes)) or "none"})')
            stations = routes[name]['stations']
            plan = rt.Plan(len(stations), cmd.get('laps', 1), cmd.get('start', 0))
        except (ValueError, OSError) as exc:
            self._reset('FAILED', str(exc))
            self.get_logger().error(f'route not started: {exc}')
            return
        self._reset('RUNNING', 'started')
        self.name, self.stations, self.plan = name, stations, plan
        self.started_t = time.time()
        self.came_from = self.pose[:2] if self.pose else None
        if self.map is not None:
            for k, s in enumerate(stations):
                where = rt.map_check(self.map[0], self.map[1], s['x'], s['y'], self.clearance)
                if where != 'ok':
                    self.bad[k] = 'too close to an obstacle' if where == 'blocked' else 'not on free space of the map'
                    self.get_logger().warn(f'station {k + 1} "{s["name"]}" is {self.bad[k]}: it will be skipped')
        laps = 'until stopped' if plan.laps == 0 else f'{plan.laps} lap(s)'
        self.get_logger().info(f'route "{name}" started: {len(stations)} stations, {laps}')
        self._markers()

    # ---------------------------------------------------------------- goals
    def _cancel(self):
        self.send_no += 1
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = self.goal_id = None
        self.result = None

    def _leg(self):
        """Stations of the next goal: the current one and, while it is a pass-through station,
        the following ones up to and including the next stop."""
        plan = copy.copy(self.plan)
        leg = [plan.index]
        while not rt.stops_here(self.stations, plan) and plan.advance() and plan.index not in self.bad:
            leg.append(plan.index)
        return leg

    def _pose_msg(self, index, prev_xy):
        s = self.stations[index]
        yaw = rt.goal_yaw(self.stations, index, self.plan.loop, prev_xy)
        p = PoseStamped()
        p.header.frame_id = self.frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = s['x'], s['y']
        _, _, p.pose.orientation.z, p.pose.orientation.w = quat_from_yaw(yaw)
        return p, yaw

    def _send(self):
        self.leg = self._leg()
        client = self.to_pose if len(self.leg) == 1 else self.through
        if not client.server_is_ready():
            self.message = 'waiting for Nav2'
            return
        poses, prev, desc = [], self.came_from, []
        for k in self.leg:
            p, yaw = self._pose_msg(k, prev)
            poses.append(p)
            s = self.stations[k]
            prev = (s['x'], s['y'])
            desc.append(f'"{s["name"]}" ({s["x"]:.2f}, {s["y"]:.2f}, {math.degrees(yaw):.0f} deg)')
        if len(poses) == 1:
            goal = NavigateToPose.Goal()
            goal.pose = poses[0]
        else:
            goal = NavigateThroughPoses.Goal()
            goal.poses = poses
        self.result, self.remaining, self.best_remaining = None, None, None
        self.goal_sent_t = self.progress_t = time.time()
        self.goal_id = b'pending'
        self.send_no += 1
        token = self.send_no
        fut = client.send_goal_async(goal, feedback_callback=lambda m: self._feedback(token, m))
        fut.add_done_callback(lambda f: self._accepted(token, f))
        i = self.plan.index
        self.message = f'to {self.stations[i]["name"]} ({i + 1}/{len(self.stations)})'
        how = 'stop' if len(self.leg) == 1 else 'through ' + ', '.join(desc[:-1]) + ' without stopping, stop at'
        self.get_logger().info(f'lap {self.plan.lap}: {how} {desc[-1]}')

    def _accepted(self, token, fut):
        handle = fut.result()
        if handle is not None and handle.accepted:
            self.own_ids.add(bytes(handle.goal_id.uuid))
            if len(self.own_ids) > 500:
                self.own_ids = {bytes(handle.goal_id.uuid)}
        if token != self.send_no:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.goal_id = None
            self.result = GoalStatus.STATUS_ABORTED
            return
        self.goal_handle = handle
        self.goal_id = bytes(handle.goal_id.uuid)
        handle.get_result_async().add_done_callback(lambda f: self._done(token, f))

    def _feedback(self, token, msg):
        if token != self.send_no:
            return
        fb = msg.feedback
        self.remaining = float(fb.distance_remaining)
        if self.best_remaining is None or self.remaining < self.best_remaining - 0.2:
            self.best_remaining, self.progress_t = self.remaining, time.time()

    def _done(self, token, fut):
        if token != self.send_no:
            return
        res = fut.result()
        self.result = res.status if res is not None else GoalStatus.STATUS_ABORTED

    # ------------------------------------------------------------- progress
    def _arrived(self):
        s = self.stations[self.plan.index]
        err = None
        if self.pose is not None:
            err = math.hypot(self.pose[0] - s['x'], self.pose[1] - s['y'])
        self.visits.append({'lap': self.plan.lap, 'index': self.plan.index, 'name': s['name'],
                            't': round(time.time() - self.started_t, 1), 'stop': len(self.leg) <= 1,
                            'error_m': None if err is None else round(err, 3)})
        self.get_logger().info(f'station "{s["name"]}" {"reached" if len(self.leg) <= 1 else "passed"}'
                               + ('' if err is None else f' ({err * 100:.0f} cm from the station)'))
        self.failed_in_row = 0

    def _next(self):
        s = self.stations[self.plan.index]
        self.came_from = (s['x'], s['y'])
        self.tries = 0
        if not self.plan.advance():
            dur = time.time() - self.started_t
            skipped = f', skipped: {", ".join(self.skipped)}' if self.skipped else ''
            self.status = 'DONE' if not self.skipped else 'DONE_WITH_SKIPS'
            self.message = f'route finished in {dur:.0f} s{skipped}'
            self.goal_handle = self.goal_id = None
            self.get_logger().info(f'route "{self.name}" finished in {dur:.0f} s{skipped}')
            return False
        self.status = 'RUNNING'
        i = self.plan.index
        self.message = f'to {self.stations[i]["name"]} ({i + 1}/{len(self.stations)})'
        return True

    def _passed_on_the_way(self):
        """In a continuous leg, a pass-through station counts as visited when the robot comes
        within pass_radius of it (the same radius the behaviour tree uses to drop it)."""
        if len(self.leg) <= 1 or self.pose is None:
            return
        while len(self.leg) > 1:
            s = self.stations[self.leg[0]]
            if math.hypot(self.pose[0] - s['x'], self.pose[1] - s['y']) > self.pass_radius:
                return
            self._arrived()
            self.leg.pop(0)
            self._next()

    def _tick(self):
        self._update_pose()
        if self.plan is None or self.status in FINISHED:
            self._publish()
            return
        now = time.time()
        if self.status == 'DWELL':
            if now >= self.wait_until:
                self._next()
            self._publish()
            return
        if self.localized is False:
            if self.status != 'WAITING':
                self._cancel()
                self.get_logger().warn('robot not localized: route waits')
            self.status, self.message = 'WAITING', 'waiting for localization'
            self._publish()
            return
        if self.status == 'WAITING':
            self.status = 'RUNNING'
        if now < self.wait_until:
            self._publish()
            return
        if self.goal_id is None:
            if self.plan.index in self.bad:
                self._skip_bad()
            else:
                self._send()
        elif self.result is None:
            self._passed_on_the_way()
            if self._close_enough(now):
                self.get_logger().info(f'standing {self.settle_s:.0f} s close to "{self._here()}": counted as reached')
                self._cancel()
                self.result = GoalStatus.STATUS_SUCCEEDED
                self.goal_id = b'settled'
            if self.result == GoalStatus.STATUS_SUCCEEDED:
                pass
            elif self._stalled(now):
                self._cancel()
                self._failed(now)
        elif self.result == GoalStatus.STATUS_SUCCEEDED:
            self.goal_handle = self.goal_id = None
            while len(self.leg) > 1:
                self._arrived()
                self.leg.pop(0)
                self._next()
            self._arrived()
            wait = float(self.stations[self.plan.index].get('wait_s', 0.0))
            if wait > 0.0:
                self.status, self.wait_until = 'DWELL', now + wait
                self.message = f'waiting {wait:.0f} s at {self._here()}'
            else:
                self._next()
        else:
            self.goal_handle = self.goal_id = None
            if now - self.foreign_t < 3.0:
                self._pause_for_other_goal()
            else:
                self._failed(now)
        self._publish()

    def _stalled(self, now):
        if self.no_path_since is not None and len(self.leg) == 1 and now - self.no_path_since > self.no_path_s \
                and now - self.goal_sent_t > self.no_path_s:
            self.get_logger().warn(f'no path to "{self._here()}" for {self.no_path_s:.0f} s')
            self.no_path_since = None
            return True
        if now - self.progress_t > self.stuck_s and now - self.goal_sent_t > self.stuck_s:
            self.get_logger().warn(f'no progress towards "{self._here()}" for {self.stuck_s:.0f} s')
            return True
        return False

    def _skip_bad(self):
        name = self._here()
        self.skipped.append(name)
        self.get_logger().warn(f'station "{name}" skipped: {self.bad[self.plan.index]}')
        self.failed_in_row += 1
        if self.failed_in_row >= len(self.stations):
            self._stop_failed('no station can be reached')
        else:
            self._next()

    def _failed(self, now):
        name = self._here()
        self.result = None
        if self.tries < self.retries:
            self.tries += 1
            self.wait_until = now + self.retry_wait
            self.message = f'{name} not reached, trying again ({self.tries}/{self.retries})'
            self.get_logger().warn(f'station "{name}" not reached, trying again ({self.tries}/{self.retries})')
            return
        self.skipped.append(name)
        self.failed_in_row += 1
        self.get_logger().error(f'station "{name}" not reached after {self.retries + 1} tries: skipped')
        if self.failed_in_row >= len(self.stations):
            self._stop_failed(f'no station could be reached ({", ".join(self.skipped[-len(self.stations):])})')
            return
        self._next()

    def _stop_failed(self, message):
        name, visits, skipped = self.name, self.visits, self.skipped
        self._reset('FAILED', message)
        self.name, self.visits, self.skipped = name, visits, skipped
        self.get_logger().error(f'route stopped: {message}')

    # --------------------------------------------------------------- output
    def _publish(self):
        st = {'status': self.status, 'message': self.message, 'route': self.name,
              'stations': len(self.stations), 'visits': self.visits[-50:], 'skipped': self.skipped}
        if self.plan is not None:
            st.update({'index': self.plan.index, 'lap': self.plan.lap, 'laps': self.plan.laps,
                       'station': self._here(), 'remaining_m': self.remaining,
                       'invalid': {self.stations[k]['name']: why for k, why in self.bad.items()}})
        text = json.dumps(st)
        if text != getattr(self, '_last_state', None):
            self._last_state = text
            self.state_pub.publish(String(data=text))

    def _markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id, line.header.stamp = self.frame, stamp
        line.ns, line.id, line.type = 'route', 0, Marker.LINE_STRIP
        line.pose.orientation.w, line.scale.x = 1.0, 0.03
        line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 0.6, 1.0, 0.8
        for k, s in enumerate(self.stations):
            line.points.append(Point(x=s['x'], y=s['y'], z=0.05))
            m = Marker()
            m.header.frame_id, m.header.stamp = self.frame, stamp
            m.ns, m.id, m.type = 'route_text', k + 1, Marker.TEXT_VIEW_FACING
            m.pose.position.x, m.pose.position.y, m.pose.position.z = s['x'], s['y'], 0.4
            m.pose.orientation.w, m.scale.z = 1.0, 0.25
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = f'{k + 1} {s["name"]}'
            arr.markers.append(m)
        if self.plan is not None and self.plan.loop and len(self.stations) > 1:
            s = self.stations[0]
            line.points.append(Point(x=s['x'], y=s['y'], z=0.05))
        arr.markers.append(line)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = RouteRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node._cancel()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
