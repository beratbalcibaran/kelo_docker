"""Decisions of nav_watch without ROS: why the robot is not driving, whether it is ready, stuck goals.

The behaviour tree (navigate_wait_then_detour.xml) waits and retries for ever (RecoveryNode 100000)
and only logs "Running wait". nav_watch looks at the same inputs (odometry, clock, localization, people,
collision monitor, plan and global costmap) and names the first thing that stops the robot.
"""
import math
import statistics

LETHAL = 99          # OccupancyGrid view of the costmap: 100 lethal, 99 inscribed
CLOCK_MAX_S = 0.3    # |receive time - header stamp| of the robot's odometry
ODOM_MAX_AGE_S = 1.0
SCAN_MAX_AGE_S = 1.0
CAMERA_MAX_AGE_S = 2.0
# nodes a goal needs: one of them down or not active (restarting after a crash, health_monitor) -> not ready
NAV_NODES = ('map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
             'bt_navigator', 'velocity_smoother', 'collision_monitor', 'cmd_vel_gate')


def broken_nav_nodes(health):
    """Names of NAV_NODES that /kelo/health reports down or not active."""
    nodes = (health or {}).get('nodes', {})
    return [n for n in NAV_NODES if n in nodes and nodes[n][0] in ('down', 'inactive')]


class Grid:
    """Minimal OccupancyGrid view: data row-major, row 0 at origin y."""

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
    """(distance along the path, x, y) of the first lethal/inscribed cell on the next `lookahead` m of
    the path, without the first `skip` m (the robot's own cells) and the last `end_skip` m (a goal near a
    wall), or None. Same window as the behaviour tree's IsPathValid check."""
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
    """Clock offset estimate from (receive - stamp) samples: the lower quartile (transport latency only
    adds, so the smallest values are closest to the true offset)."""
    if not offsets:
        return None
    s = sorted(offsets)
    return s[len(s) // 4]


def readiness(now, seen, clock_off, odom_conflict, loc_state, nav_up, camera=True, broken=()):
    """(ready, reason). `seen` maps 'odom', 'scan', 'camera' to the last receive time (or None);
    broken: navigation nodes down or restarting (broken_nav_nodes)."""
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
        # not a reason to refuse goals: without camera images the collision monitor stops the robot
        a = age('camera')
        if a is None or a > CAMERA_MAX_AGE_S:
            return True, ('ready, but no depth camera images' + ('' if a is None else f' for {a:.0f} s')
                          + ': the collision monitor will stop the robot')
    return True, 'ready'


def wait_reason(ready, person_blocking, cm_stop, blocked, goal_cost, plan_age, recoveries, progressing=False):
    """Why an active goal does not drive, most important first. blocked = first_blocked(...).
    progressing: the robot moved or got closer in the last seconds (then it is driving, e.g. around an
    obstacle on a new path, whatever the counters say)."""
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
    """A goal is stuck when, for `timeout` s, the remaining distance did not shrink by `min_progress` and
    the robot did not move `min_progress` either (a detour makes the remaining distance grow while the
    robot drives: that is progress)."""

    def __init__(self, timeout=60.0, min_progress=0.10):
        self.timeout, self.min_progress = float(timeout), float(min_progress)
        self.reset(None)

    def reset(self, now):
        self.best, self.since, self.xy = None, now, None

    def pause(self, now):
        self.since = now

    def progressing(self, now, within=3.0):
        """True when the last progress is at most `within` s old."""
        return self.since is not None and self.best is not None and now - self.since <= within

    def step(self, now, remaining, xy=None):
        """True once the goal has been stuck for `timeout` seconds."""
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
