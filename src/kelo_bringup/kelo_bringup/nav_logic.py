import math
import statistics

LETHAL = 99
CLOCK_MAX_S = 0.3
ODOM_MAX_AGE_S = 1.0
SCAN_MAX_AGE_S = 1.0
CAMERA_MAX_AGE_S = 2.0
NAV_NODES = ('map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
             'bt_navigator', 'velocity_smoother', 'collision_monitor', 'cmd_vel_gate')


def broken_nav_nodes(health):
    nodes = (health or {}).get('nodes', {})
    return [n for n in NAV_NODES if n in nodes and nodes[n][0] in ('down', 'inactive')]


class Grid:

    def __init__(self, res, ox, oy, width, height, data):
        self.res, self.ox, self.oy, self.w, self.h = float(res), float(ox), float(oy), int(width), int(height)
        self.data = data

    def cost(self, x, y):
        i, j = int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))
        if not (0 <= i < self.w and 0 <= j < self.h):
            return -1
        return self.data[j * self.w + i]


def closest_index(path, x, y):
    best, bi = float('inf'), 0
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best:
            best, bi = d, i
    return bi


def first_blocked(path, robot_xy, grid, lookahead=2.0, skip=0.4, end_skip=0.5, lethal=LETHAL):
    if not path or grid is None:
        return None
    start = closest_index(path, *robot_xy)
    total = sum(math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
                for k in range(start, len(path) - 1))
    s = 0.0
    for k in range(start, len(path)):
        if k > start:
            s += math.hypot(path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1])
        if s > lookahead:
            break
        if s < skip or total - s < end_skip:
            continue
        if grid.cost(*path[k]) >= lethal:
            return s, path[k][0], path[k][1]
    return None


def clock_offset(offsets):
    if not offsets:
        return None
    s = sorted(offsets)
    return s[len(s) // 4]


def readiness(now, seen, clock_off, odom_conflict, loc_state, nav_up, camera=True, broken=()):
    def age(k):
        t = seen.get(k)
        return None if t is None else now - t
    a = age('odom')
    if a is None:
        return False, 'no odometry from the robot yet (cable connected? robot driver started?)'
    if a > ODOM_MAX_AGE_S:
        return False, f'no odometry from the robot for {a:.0f} s (cable? robot driver stopped?)'
    if clock_off is not None and abs(clock_off) > CLOCK_MAX_S:
        return False, (f'robot clock is {clock_off:+.2f} s off the Jetson '
                       f'(odometry {"late" if clock_off > 0 else "from the future"}): run scripts/robot.sh time')
    if odom_conflict:
        return False, ('two publishers of odom -> base_link (the robot driver and odom_filter): odom_mode in '
                       '/data/hardware.yaml must be "driver" (Find robot -> Apply)')
    a = age('scan')
    if a is None or a > SCAN_MAX_AGE_S:
        return False, 'no LiDAR scan' + ('' if a is None else f' for {a:.0f} s')
    if loc_state != 'LOCALIZED':
        return False, f'not localized ({loc_state or "no pose yet"})'
    if broken:
        return False, f'navigation node {", ".join(broken)} stopped: it is coming back by itself'
    if not nav_up:
        return False, 'navigation starting'
    if camera:
        a = age('camera')
        if a is None or a > CAMERA_MAX_AGE_S:
            return True, ('ready, but no depth camera images' + ('' if a is None else f' for {a:.0f} s')
                          + ': the collision monitor will stop the robot')
    return True, 'ready'


def wait_reason(ready, person_blocking, cm_stop, blocked, goal_cost, plan_age, recoveries, progressing=False):
    ok, why = ready
    if not ok:
        return why
    if progressing:
        return ''
    if person_blocking:
        return 'waiting for a person on the path to step aside'
    if cm_stop:
        return f'collision monitor stop ({cm_stop}): something is inside the stop zone'
    if goal_cost is not None and goal_cost >= LETHAL:
        return 'the goal is inside an obstacle or too close to a wall: choose another goal'
    if blocked is not None:
        d, x, y = blocked
        return f'obstacle on the path {d:.1f} m ahead at ({x:.2f}, {y:.2f}): waiting, then driving around it'
    if plan_age is None or plan_age > 6.0:
        return 'no path to the goal (the way is closed): waiting until a path is free'
    if recoveries:
        return f'recovering after a controller failure ({recoveries}x)'
    return ''


class StuckTimer:

    def __init__(self, timeout=60.0, min_progress=0.10):
        self.timeout, self.min_progress = float(timeout), float(min_progress)
        self.reset(None)

    def reset(self, now):
        self.best, self.since, self.xy = None, now, None

    def pause(self, now):
        self.since = now

    def progressing(self, now, within=3.0):
        return self.since is not None and self.best is not None and now - self.since <= within

    def step(self, now, remaining, xy=None):
        if self.since is None:
            self.since = now
        moved = (xy is not None and self.xy is not None
                 and math.hypot(xy[0] - self.xy[0], xy[1] - self.xy[1]) >= self.min_progress)
        if xy is not None and (self.xy is None or moved):
            self.xy = xy
        if remaining is not None and remaining > 0.0 and (self.best is None or remaining < self.best - self.min_progress):
            self.best = remaining
            moved = True
        if moved:
            self.since = now
            return False
        return self.timeout > 0 and now - self.since >= self.timeout


def median(values):
    return statistics.median(values) if values else None
