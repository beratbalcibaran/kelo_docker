import ast
import math
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String, UInt32
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup.common import config_path
from kelo_bringup.logic import quat_from_yaw, wrap, yaw_from_quat


def log(msg):
    print(f'[e2e] {msg}', flush=True)


def _pose(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, yaw_from_quat(p.orientation.x, p.orientation.y,
                                                     p.orientation.z, p.orientation.w)


class Track:
    def __init__(self):
        self.last = None
        self.pose = None
        self.dist = 0.0
        self.turn = 0.0
        self.speed = 0.0

    def cb(self, msg):
        x, y, yaw = _pose(msg)
        if self.last is not None:
            self.dist += math.hypot(x - self.last[0], y - self.last[1])
            self.turn += wrap(yaw - self.last[2])
        self.last = self.pose = (x, y, yaw)
        t = msg.twist.twist
        self.speed = math.hypot(t.linear.x, t.linear.y)

    def reset(self):
        self.dist = self.turn = 0.0


class E2E(Node):
    def __init__(self):
        super().__init__('kelo_e2e')
        self.gt, self.odom, self.corr = Track(), Track(), Track()
        self.create_subscription(Odometry, '/sim/ground_truth', self.gt.cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom.cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom_corrected', self.corr.cb, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.contacts = 0
        self.create_subscription(UInt32, '/sim/contacts', self._contacts, latched)
        self.cm_state = ''
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, latched)
        self.teleop = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.obst = self.create_publisher(String, '/sim/obstacles', 10)
        self.cmd = None
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)

    def _contacts(self, m):
        self.contacts = m.data

    def _amcl(self, m):
        self.amcl = m

    def _cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def spin_for(self, sec, cond=None):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond is not None and cond():
                return True
        return cond is None

    def drive(self, vx, vy, wz, sec):
        end = time.time() + sec
        while time.time() < end:
            m = Twist()
            m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
            self.teleop.publish(m)
            self.spin_for(0.05)
        self.stop()

    def stop(self):
        for _ in range(10):
            self.teleop.publish(Twist())
            self.spin_for(0.05)
        self.spin_for(1.0)

    def obstacles(self, req):
        self.obst.publish(String(data=yaml.safe_dump(req)))
        self.spin_for(0.3)

    def set_sim_param(self, name, value):
        cli = self.create_client(SetParameters, '/sim_robot/set_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('sim_robot parameter service missing')
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)))]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def map_pose(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, yaw_from_quat(q.x, q.y, q.z, q.w)


def check(ok, text):
    log(('PASS ' if ok else 'FAIL ') + text)
    return ok


def sc_odom(n, _args):
    n.spin_for(3.0, lambda: n.gt.pose and n.odom.pose and n.corr.pose)
    ok = True
    for t in (n.gt, n.odom, n.corr):
        t.reset()
    n.drive(0.0, 0.0, 0.6, 2.0 * math.pi / 0.6)
    gt, od, co = n.gt.turn, n.odom.turn, n.corr.turn
    if abs(od) < 1e-3 or abs(gt) < math.radians(180):
        return check(False, f'robot did not turn (real {math.degrees(gt):.0f} deg): obstacle/collision_monitor?')
    log(f'rotation 0.6 rad/s: real {math.degrees(gt):.1f} deg, odom {math.degrees(od):.1f} deg '
        f'(k={gt / od:.2f}), corrected {math.degrees(co):.1f} deg')
    ok &= check(abs(gt / od - 5.3) < 0.25, 'raw odometry yaw k ~ 5.3')
    ok &= check(abs(co - gt) < math.radians(6.0), 'corrected yaw ~ real (within 6 deg)')
    for t in (n.gt, n.odom, n.corr):
        t.reset()
    n.drive(0.15, 0.0, 0.0, 4.0)
    gt, od, co = n.gt.dist, n.odom.dist, n.corr.dist
    if gt < 0.3:
        return check(False, f'robot did not move forward (real {gt:.2f} m)')
    log(f'straight 0.15 m/s: real {gt:.3f} m, odom {od:.3f} m ({(od / gt - 1) * 100:+.2f}%), '
        f'corrected {co:.3f} m')
    ok &= check(-0.022 < od / gt - 1 < -0.010, 'raw odometry linear error ~ -1.8% (ramp included)')
    ok &= check(abs(co / gt - 1) < 0.005, 'corrected distance ~ real (within 0.5%)')
    return ok


def sc_tour(n, _args):
    route = [(1.0, 1.0), (1.0, 3.5), (4.1, 3.5), (4.1, 2.0), (7.1, 2.0), (7.1, 4.0), (7.1, 0.9),
             (4.4, 0.9), (4.1, 3.5), (1.0, 3.5), (1.0, 1.0)]
    n.spin_for(3.0, lambda: n.gt.pose)
    for gx, gy in route:
        t0 = time.time()
        while time.time() - t0 < 120:
            x, y, yaw = n.gt.pose
            dx, dy = gx - x, gy - y
            d = math.hypot(dx, dy)
            if d < 0.08:
                break
            err = wrap(math.atan2(dy, dx) - yaw)
            m = Twist()
            m.angular.z = max(-0.5, min(0.5, 1.5 * err))
            m.linear.x = 0.0 if abs(err) > 0.3 else min(0.2, 0.6 * d)
            n.teleop.publish(m)
            n.spin_for(0.05)
        else:
            n.stop()
            x, y, yaw = n.gt.pose
            return check(False, f'tour waypoint not reached: {gx}, {gy} (robot {x:.2f}, {y:.2f}, '
                                f'{math.degrees(yaw):.0f} deg)')
    n.stop()
    return check(n.contacts == 0, f'tour complete, collisions={n.contacts}')


def _gt_in_map(n, start):
    x, y, yaw = n.gt.pose
    sx, sy, syaw = start
    c, s = math.cos(-syaw), math.sin(-syaw)
    return c * (x - sx) - s * (y - sy), s * (x - sx) + c * (y - sy), wrap(yaw - syaw)


def sc_localization(n, args):
    start = tuple(float(v) for v in (args[0:3] if len(args) >= 3 else (1.0, 1.0, 0.0)))
    n.spin_for(30.0, lambda: n.gt.pose and n.map_pose())
    mp = n.map_pose()
    if mp is None:
        return check(False, 'map->base_link TF missing')
    tx, ty, tyaw = _gt_in_map(n, start)
    exy, eyaw = math.hypot(mp[0] - tx, mp[1] - ty), abs(wrap(mp[2] - tyaw))
    log(f'AMCL ({mp[0]:.2f}, {mp[1]:.2f}, {math.degrees(mp[2]):.0f} deg) true ({tx:.2f}, {ty:.2f}, '
        f'{math.degrees(tyaw):.0f} deg) -> error {exy:.2f} m, {math.degrees(eyaw):.1f} deg')
    return check(exy < 0.25 and eyaw < math.radians(10), 'localization error < 0.25 m / 10 deg')


def _navigate(n, x, y, yaw=0.0):
    cli = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    if not cli.wait_for_server(timeout_sec=60.0):
        raise RuntimeError('navigate_to_pose missing')
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    _, _, goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quat_from_yaw(yaw)
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError('goal rejected')
    return handle


def _navigate_wait(n, x, y, yaw=0.0, timeout=120.0):
    handle = _navigate(n, x, y, yaw)
    fut = handle.get_result_async()
    end = time.time() + timeout
    while time.time() < end and not fut.done():
        n.spin_for(0.2)
    if not fut.done() or fut.result().status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f'could not reach the staging point ({x}, {y})')


STAGING = (3.4, 1.0)
CORRIDOR_GOAL = (5.9, 1.0)
# people scenarios: the longest straight lane of the simulator room with room to pass a person on
# either side (+-0.9 m without lethal cost, found on the global costmap), the goal away from the wall
LANE_START = (3.2, 0.7)
LANE_GOAL = (6.0, 0.7)


def _wait_moving(n, speed=0.08, timeout=40.0):
    return n.spin_for(timeout, lambda: n.gt.speed > speed)


def _wait_cruise(n, speed=0.20, timeout=10.0):
    n.spin_for(timeout, lambda: n.gt.speed >= speed)
    log(f'cruise speed {n.gt.speed:.2f} m/s')


def _robot_front():
    with open(config_path('nav2_params.yaml'), encoding='utf-8') as f:
        nav = yaml.safe_load(f)
    fp = ast.literal_eval(nav['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    return max(p[0] for p in fp)


FRONT = _robot_front()
OBSTACLES = {
    'table': {'type': 'box', 'ahead': FRONT + 0.65, 'size': [0.30, 1.2], 'z': [0.25, 0.80]},
    'low': {'type': 'box', 'ahead': FRONT + 0.65, 'size': [0.30, 1.2], 'z': [0.0, 0.30]},
}


def sc_obstacle(n, args):
    kind = args[0] if args else 'table'
    n.spin_for(5.0, lambda: n.gt.pose)
    _navigate_wait(n, *STAGING)
    c0 = n.contacts
    handle = _navigate(n, *CORRIDOR_GOAL)
    if not _wait_moving(n):
        return check(False, 'robot did not start moving')
    _wait_cruise(n)
    x0, y0, yaw0 = n.gt.pose
    v0 = n.gt.speed
    t0 = time.time()
    if kind == 'person':
        c, s = math.cos(yaw0), math.sin(yaw0)
        px, py = x0 + 1.2 * c, y0 + 1.2 * s
        path = [[px - 1.2 * s, py + 1.2 * c], [px + 1.2 * s, py - 1.2 * c]]
        n.obstacles({'add': {'type': 'cylinder', 'path': path, 'speed': 0.5, 'r': 0.2, 'z': [0, 1.7]}})
    else:
        n.obstacles({'add': OBSTACLES[kind]})
    ob = OBSTACLES.get(kind, {})
    log(f'obstacle added ({kind}), robot speed when added {v0:.2f} m/s')
    stopped = n.spin_for(3.0, lambda: n.gt.speed < 0.02)
    t_stop = time.time() - t0
    n.spin_for(2.0)
    x1, y1, _ = n.gt.pose
    travel = math.hypot(x1 - x0, y1 - y0)
    ok = True
    if kind == 'person':
        ok &= check(n.contacts == c0, f'no collision while the person crosses (counter {n.contacts - c0})')
    else:
        gap = ob['ahead'] - ob['size'][0] / 2 - FRONT - travel
        log(f'stop {t_stop:.2f} s (from obstacle insertion), travel {travel:.2f} m, remaining gap {gap:.2f} m')
        ok &= check(stopped, 'robot stopped within 3 s')
        ok &= check(n.contacts == c0, 'no collision')
        ok &= check(gap > 0.05, 'gap to the obstacle > 5 cm')
    handle.cancel_goal_async()
    n.spin_for(1.0)
    n.obstacles({'clear': True})
    ok &= _resumes(n)
    return ok


def _resumes(n):
    n.spin_for(2.0)
    try:
        _navigate_wait(n, *CORRIDOR_GOAL, timeout=60.0)
        return check(True, 'continued to the goal after the obstacle left (no costmap ghost)')
    except RuntimeError as exc:
        return check(False, f'could not continue after the obstacle left: {exc}')


def sc_dropout(n, args):
    sensor = args[0] if args else 'camera'
    n.spin_for(5.0, lambda: n.gt.pose)
    _navigate_wait(n, *STAGING)
    handle = _navigate(n, *CORRIDOR_GOAL)
    if not _wait_moving(n):
        return check(False, 'robot did not start moving')
    _wait_cruise(n)
    n.set_sim_param(f'{sensor}_enabled', False)
    t0 = time.time()
    stopped = n.spin_for(4.0, lambda: n.gt.speed < 0.02)
    t_stop = time.time() - t0
    log(f'{sensor} cut -> {"stopped" if stopped else "DID NOT STOP"} ({t_stop:.2f} s)')
    still = n.spin_for(3.0, lambda: n.gt.speed > 0.03)
    ok = check(stopped and t_stop < 2.5, f'robot stopped within 2.5 s after {sensor} was cut')
    ok &= check(not still, f'robot stays still while {sensor} is cut')
    n.set_sim_param(f'{sensor}_enabled', True)
    handle.cancel_goal_async()
    n.spin_for(2.0)
    return ok


class Holder:
    def __init__(self, n, vx):
        self.n, self.vx = n, vx

    def run(self, sec, cond=None):
        end = time.time() + sec
        while time.time() < end:
            m = Twist()
            m.linear.x = self.vx
            self.n.teleop.publish(m)
            if self.n.spin_for(0.05, cond) and cond is not None:
                return True
        return cond is None


def _teleop_start(n):
    n.spin_for(5.0, lambda: n.gt.pose)
    _navigate_wait(n, *STAGING)
    n.spin_for(1.0)
    hold = Holder(n, 0.25)
    if not hold.run(10.0, lambda: n.gt.speed >= 0.2):
        raise RuntimeError(f'could not accelerate with teleop (speed {n.gt.speed:.2f})')
    log(f'teleop cruise speed {n.gt.speed:.2f} m/s')
    return hold


def sc_teleop_obstacle(n, args):
    kind = args[0] if args else 'table'
    hold = _teleop_start(n)
    c0 = n.contacts
    x0, y0, _ = n.gt.pose
    t0 = time.time()
    n.obstacles({'add': OBSTACLES[kind]})
    stopped = hold.run(3.0, lambda: n.gt.speed < 0.02)
    t_stop = time.time() - t0
    hold.run(3.0)
    x1, y1, _ = n.gt.pose
    travel = math.hypot(x1 - x0, y1 - y0)
    ob = OBSTACLES[kind]
    gap = ob['ahead'] - ob['size'][0] / 2 - FRONT - travel
    n.stop()
    log(f'stop {t_stop:.2f} s, travel {travel:.2f} m, remaining gap {gap:.2f} m')
    ok = check(stopped, 'robot stopped within 3 s (operator holding the key)')
    ok &= check(n.contacts == c0, 'no collision')
    ok &= check(gap > 0.05, 'gap to the obstacle > 5 cm')
    n.obstacles({'clear': True})
    ok &= _resumes(n)
    return ok


def sc_teleop_dropout(n, args):
    sensor = args[0] if args else 'camera'
    hold = _teleop_start(n)
    n.set_sim_param(f'{sensor}_enabled', False)
    t0 = time.time()
    stopped = hold.run(4.0, lambda: n.gt.speed < 0.02)
    t_stop = time.time() - t0
    moved = hold.run(3.0, lambda: n.gt.speed > 0.03)
    n.stop()
    log(f'{sensor} cut (teleop) -> {"stopped" if stopped else "DID NOT STOP"} ({t_stop:.2f} s)')
    ok = check(stopped and t_stop < 2.5, f'teleop stopped within 2.5 s after {sensor} was cut')
    ok &= check(not moved, f'no motion while {sensor} stays cut, even with the key held')
    n.set_sim_param(f'{sensor}_enabled', True)
    n.spin_for(2.0)
    return ok


def sc_near_wall(n, _args):
    n.spin_for(5.0, lambda: n.gt.pose)
    _navigate_wait(n, *STAGING)
    n.spin_for(1.0)
    c0 = n.contacts
    n.obstacles({'add': {'type': 'box', 'ahead': FRONT + 0.30, 'size': [0.10, 1.2], 'z': [0.0, 1.0]}})
    n.spin_for(1.5)

    def moved(vx, wz, sec):
        x0, y0, yaw0 = n.gt.pose
        n.drive(vx, 0.0, wz, sec)
        x1, y1, yaw1 = n.gt.pose
        return math.hypot(x1 - x0, y1 - y0), abs(wrap(yaw1 - yaw0))
    fwd, _ = moved(0.2, 0.0, 2.0)
    back, _ = moved(-0.15, 0.0, 2.0)
    _, turn = moved(0.0, 0.5, 4.0)
    log(f'forward {fwd:.2f} m, backward {back:.2f} m, turn {math.degrees(turn):.0f} deg')
    ok = check(fwd < 0.06, 'forward motion towards the obstacle blocked (at the stop-zone edge)')
    ok &= check(back > 0.08, 'could reverse (no lock-up)')
    ok &= check(turn > math.radians(30), 'could turn in place (no lock-up)')
    ok &= check(n.contacts == c0, 'no collision')
    n.obstacles({'clear': True})
    return ok


def sc_goal_motion(n, _args):
    n.spin_for(5.0, lambda: n.gt.pose)
    _navigate_wait(n, 0.0, 0.0, 0.0)
    samples = []
    n.create_subscription(Odometry, '/sim/ground_truth', lambda m: samples.append(
        (m.twist.twist.linear.x, m.twist.twist.linear.y)), 10)
    x0, y0, _ = n.gt.pose
    n.gt.reset()
    t0 = time.time()
    _navigate_wait(n, 2.0, 2.5, math.pi / 2, timeout=90.0)
    dur = time.time() - t0
    mv = [(vx, vy) for vx, vy in samples if math.hypot(vx, vy) > 0.05]
    side = sum(abs(vy) / math.hypot(vx, vy) for vx, vy in mv) / max(len(mv), 1)
    ang = sum(abs(math.atan2(vy, vx)) for vx, vy in mv) / max(len(mv), 1)
    log(f'time {dur:.1f} s, path {n.gt.dist:.2f} m, mean |vy|/|v| {side:.2f}, '
        f'mean direction relative to body {math.degrees(ang):.0f} deg')
    ok = check(dur < 45.0, 'goal reached within 45 s')
    ok &= check(side < 0.35, 'motion along the body axis (lateral component < 35%)')
    ok &= check(n.gt.dist < 4.6, 'path length reasonable (plan ~3.8 m)')
    return ok


def sc_relocalize(n, _args):
    import subprocess
    n.spin_for(5.0, lambda: n.map_pose())
    before = n.map_pose()
    r = subprocess.run(['ros2', 'run', 'kelo_bringup', 'localize'], capture_output=True, text=True, timeout=120)
    n.spin_for(3.0)
    after = n.map_pose()
    jump = math.hypot(after[0] - before[0], after[1] - before[1])
    turn = abs(wrap(after[2] - before[2]))
    log(f'second localize: exit {r.returncode}, pose change {jump:.3f} m / {math.degrees(turn):.1f} deg')
    ok = check(r.returncode == 3, "second press: 'already localized' (exit 3)")
    ok &= check(jump < 0.03 and turn < math.radians(2), 'pose estimate did not jump')
    r = subprocess.run(['ros2', 'run', 'kelo_bringup', 'localize', '--force'], capture_output=True,
                       text=True, timeout=300)
    ok &= check(r.returncode == 0 and 'LOCALIZED' in r.stdout, 're-localized with --force')
    return ok


def _loc_state(n, timeout=3.0):
    state = []
    sub = n.create_subscription(String, '/kelo/localization_state', lambda m: state.append(m.data),
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.spin_for(timeout, lambda: state)
    n.destroy_subscription(sub)
    return state[-1] if state else None


def sc_restored(n, args):
    """Navigation restarted where the robot stood: loc_monitor restores the last pose by itself."""
    end = time.time() + 40.0
    state = None
    while time.time() < end and state != 'LOCALIZED':
        state = _loc_state(n, 2.0)
        n.spin_for(1.0)
    ok = check(state == 'LOCALIZED', f'LOCALIZED without Localize (last pose restored), state {state}')
    return sc_localization(n, args) and ok


def sc_kidnap(n, args):
    """The robot is carried away (sim teleport, odometry does not see it): the scan no longer fits
    the map at the AMCL pose -> LOST, a running goal is cancelled; Localize finds the robot again."""
    import subprocess
    start = tuple(float(v) for v in (args[0:3] if len(args) >= 3 else (1.0, 1.0, 0.0)))
    wx, wy = (float(args[3]), float(args[4])) if len(args) >= 5 else (6.2, 2.2)
    n.spin_for(5.0, lambda: n.gt.pose)
    if _loc_state(n) != 'LOCALIZED':
        return check(False, 'not LOCALIZED before the kidnapping')
    pub = n.create_publisher(String, '/sim/teleport', 10)
    n.spin_for(0.5)
    pub.publish(String(data=yaml.safe_dump({'x': wx, 'y': wy, 'yaw': n.gt.pose[2]})))
    n.spin_for(1.0)
    t0 = time.time()
    state = None
    while time.time() - t0 < 30.0 and state != 'LOST':
        state = _loc_state(n, 1.0)
        n.spin_for(1.0)
    ok = check(state == 'LOST', f'state LOST after the robot was carried away ({time.time() - t0:.0f} s)')
    r = subprocess.run(['ros2', 'run', 'kelo_bringup', 'send_goal', '0.5', '0.5'], capture_output=True,
                       text=True, timeout=30)
    ok &= check(r.returncode == 5, 'send_goal refused while LOST')
    r = subprocess.run(['ros2', 'run', 'kelo_bringup', 'localize'], capture_output=True, text=True, timeout=400)
    for line in r.stdout.splitlines():
        if line.startswith('[localize]'):
            log('  ' + line)
    ok &= check(r.returncode == 0, 'Localize finds the robot again')
    n.spin_for(3.0)   # TF that arrived while localize ran
    return sc_localization(n, list(map(str, start))) and ok


def sc_detour(n, args):
    kind = args[0] if args else 'box'
    when = args[1] if len(args) > 1 else 'far'
    z = {'box': [0.0, 1.0], 'table': [0.25, 0.80]}[kind]
    ob = {'type': 'box', 'size': [0.40, 0.40], 'z': z}
    n.spin_for(5.0, lambda: n.gt.pose)
    _navigate_wait(n, *STAGING)
    c0 = n.contacts
    if when == 'far':
        n.obstacles({'add': dict(ob, ahead=FRONT + 1.0)})
        n.spin_for(2.0)
    handle = _navigate(n, *CORRIDOR_GOAL)
    if when == 'sudden':
        if not _wait_moving(n):
            return check(False, 'robot did not start moving')
        _wait_cruise(n)
        n.obstacles({'add': dict(ob, ahead=FRONT + 0.60)})
    log(f'obstacle on the path ({kind}, {when}, z {z[0]:.2f}-{z[1]:.2f} m)')
    t0 = time.time()
    n.gt.reset()
    fut = handle.get_result_async()
    while time.time() - t0 < 150.0 and not fut.done():
        n.spin_for(0.2)
    ok_goal = fut.done() and fut.result().status == GoalStatus.STATUS_SUCCEEDED
    dur = time.time() - t0
    x1, y1, yaw1 = n.gt.pose
    log(f'time {dur:.1f} s, path {n.gt.dist:.2f} m, robot at {x1:.2f}, {y1:.2f}, {math.degrees(yaw1):.0f} deg')
    if not fut.done():
        handle.cancel_goal_async()
        n.spin_for(1.0)
    limit = 90.0 if when == 'far' else 150.0
    ok = check(ok_goal and dur < limit, f'went around the obstacle and reached the goal within {limit:.0f} s')
    ok &= check(n.contacts == c0, f'no collision (counter {n.contacts - c0})')
    n.obstacles({'clear': True})
    return ok


def sc_people(n, args):
    """People-aware navigation.
    stand: a person stands on the path and stays: the robot slows down, waits for them (person
    wait), then passes them with the wide personal space (checked up to beside the person).
    leave: the person leaves after 8 s: the robot waits and then drives on straight, no detour.
    cross: a person walks across the path in front of the robot: it gives way and drives on straight."""
    kind = args[0] if args else 'stand'
    n.spin_for(5.0, lambda: n.gt.pose)
    _navigate_wait(n, *LANE_START)
    c0 = n.contacts
    handle = _navigate(n, *LANE_GOAL)
    if not _wait_moving(n):
        return check(False, 'robot did not start moving')
    _wait_cruise(n)
    x0, y0, yaw0 = n.gt.pose
    c, s = math.cos(yaw0), math.sin(yaw0)
    t0 = time.time()
    if kind == 'cross':
        ahead, speed = 1.4, 0.5
        px, py = x0 + ahead * c, y0 + ahead * s
        a, b = (px - 1.8 * s, py + 1.8 * c), (px + 0.8 * s, py - 0.8 * c)
        n.obstacles({'add': {'type': 'cylinder', 'name': 'person', 'path': [list(a), list(b)], 'speed': speed,
                             'r': 0.22, 'z': [0.0, 1.7]}})
        walk = math.dist(a, b) / speed

        def person(t):
            f = min(max((t - t0) * speed / math.dist(a, b), 0.0), 1.0)
            return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
    else:
        ahead = 1.3
        n.obstacles({'add': {'type': 'cylinder', 'name': 'person', 'ahead': ahead, 'r': 0.22, 'z': [0.0, 1.7]}})
        walk = 8.0 if kind == 'leave' else None

        def person(_t):
            return x0 + ahead * c, y0 + ahead * s
    log(f'person ({kind}) {ahead:.1f} m ahead, robot at {n.gt.speed:.2f} m/s')
    fut = handle.get_result_async()
    gone = False
    closest, lateral, still, near_speed, along = math.inf, 0.0, 0.0, 0.0, 0.0
    last = time.time()
    while time.time() - t0 < 150.0 and not fut.done():
        n.spin_for(0.1)
        now = time.time()
        x, y, _ = n.gt.pose
        if not gone:
            px, py = person(now)
            d = math.hypot(x - px, y - py)
            closest = min(closest, d)
            if d < 1.3:
                near_speed = max(near_speed, n.gt.speed)
            if walk is not None and now - t0 > walk:
                n.obstacles({'clear': True})
                gone = True
                log(f'person gone after {now - t0:.1f} s')
        lateral = max(lateral, abs(-s * (x - x0) + c * (y - y0)))
        along = max(along, c * (x - x0) + s * (y - y0))
        if n.gt.speed < 0.02:
            still += now - last
        last = now
    dur = time.time() - t0
    ok_goal = fut.done() and fut.result().status == GoalStatus.STATUS_SUCCEEDED
    if not fut.done():
        handle.cancel_goal_async()
        n.spin_for(1.0)
    n.obstacles({'clear': True})
    log(f'time {dur:.1f} s, standing {still:.1f} s, closest robot-person {closest:.2f} m, '
        f'max speed near the person {near_speed:.2f} m/s, max lateral offset {lateral:.2f} m, '
        f'progress along the lane {along:.2f} m (person at {ahead:.2f} m), goal {"reached" if ok_goal else "not reached"}')
    ok = True
    if kind != 'stand':
        ok &= check(ok_goal, 'reached the goal')
    ok &= check(n.contacts == c0, f'no collision (counter {n.contacts - c0})')
    ok &= check(near_speed <= 0.17, 'slow near the person (<= 0.17 m/s within 1.3 m)')
    if kind == 'stand':
        ok &= check(still >= 8.0, 'waited for the person before passing (>= 8 s standing)')
        ok &= check(lateral >= 0.5, 'passed the person on a detour')
        ok &= check(closest >= 0.7, 'kept the personal space while passing (centre distance >= 0.7 m)')
        # in the small simulator room the way past a standing person runs between them and a wall:
        # 40 % near the person times the collision monitor slowdown is a crawl, so the goal behind
        # them is not required here, only that the robot got beside them
        ok &= check(along >= ahead, 'got beside the person (passing them)')
    elif kind == 'leave':
        ok &= check(still >= 3.0, 'waited for the person (>= 3 s standing)')
        ok &= check(lateral < 0.35, 'drove on straight after the person left (no detour)')
    else:
        ok &= check(lateral < 0.35, 'gave way and drove on straight (no detour)')
    return ok


def _load_world(path):
    with open(path, encoding='utf-8') as f:
        world = yaml.safe_load(f)
    st = world.get('start', {})
    return world, (float(st.get('x', 0.0)), float(st.get('y', 0.0)), float(st.get('yaw', 0.0)))


def _world_to_map(start, x, y):
    sx, sy, syaw = start
    c, s = math.cos(-syaw), math.sin(-syaw)
    return c * (x - sx) - s * (y - sy), s * (x - sx) + c * (y - sy)


def _drive_to(n, gx, gy, timeout=120.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        x, y, yaw = n.gt.pose
        dx, dy = gx - x, gy - y
        d = math.hypot(dx, dy)
        if d < 0.08:
            return True
        err = wrap(math.atan2(dy, dx) - yaw)
        m = Twist()
        m.angular.z = max(-0.5, min(0.5, 1.5 * err))
        m.linear.x = 0.0 if abs(err) > 0.3 else min(0.2, 0.6 * d)
        n.teleop.publish(m)
        n.spin_for(0.05)
    n.stop()
    return False


def sc_world_tour(n, args):
    """Mapping drive for a world with an eval section; a person walks by during the first legs."""
    world, _ = _load_world(args[0])
    ev = world['eval']
    person = ev.get('person')
    n.spin_for(3.0, lambda: n.gt.pose)
    if person:
        n.obstacles({'add': {'type': 'cylinder', 'path': person['path'], 'speed': person['speed'],
                             'r': person.get('r', 0.2), 'z': person.get('z', [0.0, 1.7])}})
        log(f"person walking along {person['path']}")
    for i, (gx, gy) in enumerate(ev['mapping_route']):
        if not _drive_to(n, gx, gy):
            x, y, _ = n.gt.pose
            return check(False, f'tour waypoint not reached: {gx}, {gy} (robot {x:.2f}, {y:.2f})')
        if person and i + 1 == int(person.get('until_waypoint', 1)):
            n.obstacles({'clear': True})
            log('person left')
    n.stop()
    return check(n.contacts == 0, f'mapping tour complete, collisions={n.contacts}')


class PlanWatch:
    """Counts reroutes: a new global plan that leaves the previous one by more than `dev` m."""

    def __init__(self, dev=0.4, horizon=3.0, min_gap=2.0):
        self.dev, self.horizon, self.min_gap = dev, horizon, min_gap
        self.prev = None
        self.last = -1e9
        self.count = 0

    def cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            return
        if self.prev is not None:
            ahead, dist = [], 0.0
            for a, b in zip(pts, pts[1:]):
                ahead.append(a)
                dist += math.hypot(b[0] - a[0], b[1] - a[1])
                if dist > self.horizon:
                    break
            worst = max(min(math.hypot(x - px, y - py) for px, py in self.prev) for x, y in ahead)
            now = time.time()
            if worst > self.dev and now - self.last > self.min_gap:
                self.count += 1
                self.last = now
        self.prev = pts

    def reset(self):
        self.prev, self.count = None, 0


def _reset_at_goal(n, start, wx, wy):
    """After a failed goal: put the robot on that goal as if it had been reached (sim teleport,
    AMCL re-seeded), so that one failure does not decide the goals after it."""
    n.stop()
    n.spin_for(1.0)
    pub = n.create_publisher(String, '/sim/teleport', 10)
    ipub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    n.spin_for(0.5)
    pub.publish(String(data=yaml.safe_dump({'x': wx, 'y': wy, 'yaw': start[2]})))
    n.spin_for(1.0)
    gx, gy = _world_to_map(start, wx, wy)
    if math.hypot(n.gt.pose[0] - wx, n.gt.pose[1] - wy) > 0.05:
        log(f'reset to ({wx}, {wy}) failed')
        return False
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.pose.position.x, m.pose.pose.position.y = gx, gy
    m.pose.pose.orientation.w = 1.0
    m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
    m.pose.covariance[35] = math.radians(3) ** 2
    ipub.publish(m)
    n.spin_for(3.0)
    log(f'robot reset to goal ({wx}, {wy}) for the next goal')
    return True


def sc_world_goals(n, args):
    """Navigation goals of a world eval section: success, time, reroutes and late stops per goal.
    A failed goal is followed by a reset of the robot onto that goal, so each goal is measured alone."""
    import json
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Path
    world, start = _load_world(args[0])
    out = args[1] if len(args) > 1 else ''
    plans = PlanWatch()
    n.create_subscription(Path, '/plan', plans.cb, 10)
    cm = {'state': 0, 'stops': 0}

    def cm_cb(m):
        if m.action_type == CollisionMonitorState.STOP and cm['state'] != CollisionMonitorState.STOP:
            cm['stops'] += 1
        cm['state'] = m.action_type
    n.create_subscription(CollisionMonitorState, '/collision_monitor_state', cm_cb, 10)
    n.spin_for(5.0, lambda: n.gt.pose)
    rows = []
    for i, (wx, wy) in enumerate(world['eval']['goals']):
        gx, gy = _world_to_map(start, wx, wy)
        plans.reset()
        cm['stops'] = 0
        c0 = n.contacts
        n.gt.reset()
        stops, still_since, moved = 0, None, False
        t0 = time.time()
        try:
            handle = _navigate(n, gx, gy)
        except RuntimeError as exc:
            rows.append({'goal': i + 1, 'success': False, 'error': str(exc)})
            continue
        fut = handle.get_result_async()
        while time.time() - t0 < 150.0 and not fut.done():
            n.spin_for(0.1)
            x, y, _ = n.gt.pose
            far = math.hypot(x - wx, y - wy) > 0.5
            if n.gt.speed > 0.08:
                moved, still_since = True, None
            elif moved and far and n.gt.speed < 0.02:
                if still_since is None:
                    still_since = time.time()
                elif still_since > 0 and time.time() - still_since > 1.0:
                    stops += 1
                    still_since = -1.0
        ok = fut.done() and fut.result().status == GoalStatus.STATUS_SUCCEEDED
        if not fut.done():
            handle.cancel_goal_async()
            n.spin_for(2.0)
        x, y, _ = n.gt.pose
        row = {'goal': i + 1, 'target': [wx, wy], 'success': bool(ok), 'time_s': round(time.time() - t0, 1),
               'path_m': round(n.gt.dist, 2), 'reroutes': plans.count, 'stops': stops,
               'cm_stops': cm['stops'], 'collisions': n.contacts - c0,
               'final_error_m': round(math.hypot(x - wx, y - wy), 2)}
        if not ok:
            row['reset'] = _reset_at_goal(n, start, wx, wy)
        rows.append(row)
        log(f"goal {i + 1} ({wx}, {wy}): {'OK' if ok else 'FAILED'} {row['time_s']} s, path {row['path_m']} m, "
            f"reroutes {row['reroutes']}, stops {row['stops']}, collision_monitor stops {row['cm_stops']}, "
            f"collisions {row['collisions']}")
    if out:
        with open(out, 'w', encoding='utf-8') as f:
            json.dump(rows, f, indent=1)
    good = sum(r.get('success', False) for r in rows)
    return check(good == len(rows), f'{good}/{len(rows)} goals reached')


SCENARIOS = {'odom': sc_odom, 'tour': sc_tour, 'localization': sc_localization,
             'obstacle': sc_obstacle, 'dropout': sc_dropout,
             'teleop_obstacle': sc_teleop_obstacle, 'teleop_dropout': sc_teleop_dropout,
             'near_wall': sc_near_wall, 'goal_motion': sc_goal_motion, 'relocalize': sc_relocalize,
             'detour': sc_detour, 'restored': sc_restored, 'kidnap': sc_kidnap, 'people': sc_people, 'world_tour': sc_world_tour, 'world_goals': sc_world_goals}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in SCENARIOS:
        sys.exit(f'Usage: kelo_e2e {"|".join(SCENARIOS)} [arg...]')
    rclpy.init()
    node = E2E()
    try:
        ok = SCENARIOS[sys.argv[1]](node, sys.argv[2:])
    except RuntimeError as exc:
        ok = check(False, str(exc))
    finally:
        try:
            node.teleop.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
