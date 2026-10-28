DESCRIPTION = """Set the initial pose and verify AMCL localization.

usage:
  ros2 run kelo_bringup localize
  ros2 run kelo_bringup localize --yaw 90
  ros2 run kelo_bringup localize --x 1.0 --y 2.0 --yaw 90
  ros2 run kelo_bringup localize --global
  ros2 run kelo_bringup localize --force

exit codes: 0 localized, 1 failed, 3 already localized
"""
import argparse
import math
import os
import statistics
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup import scan_match as sm
from kelo_bringup.common import data_dir, load_hardware
from kelo_bringup.logic import quat_from_yaw, yaw_from_quat

XY_STD_OK = 0.15
YAW_STD_OK = math.radians(8.0)
# a very good scan fit also accepts a slightly wider particle cloud
XY_STD_FIT = 0.25
YAW_STD_FIT = math.radians(12.0)
FIT_SURE = 0.75
GLOBAL_ROUNDS = 3
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# map-fixed layers of the local costmap: they follow the AMCL pose, so while it is still wrong their
# cells land under the robot and the spin stops with 'Collision Ahead'. Off during the turns.
LOCAL_COSTMAP = '/local_costmap/local_costmap'
LOCAL_MAP_LAYERS = ('camera_static_layer',)
EXIT_ALREADY = 3


def already_localized(state, xy_std, yaw_std, force=False, fit=None):
    return not force and state == 'LOCALIZED' and accept(xy_std, yaw_std, fit)


def log(msg):
    print(f'[localize] {msg}', flush=True)


def accept(xy_std, yaw_std, fit):
    """AMCL has converged AND the scan lies on the walls of the map there. fit None = no map/scan
    (cannot check): covariance only."""
    tight = xy_std < XY_STD_OK and yaw_std < YAW_STD_OK
    if fit is None:
        return tight
    return (tight and fit >= sm.GOOD_FIT) or (xy_std < XY_STD_FIT and yaw_std < YAW_STD_FIT and fit >= FIT_SURE)


def amcl_std(msg):
    c = msg.pose.covariance
    return math.sqrt(max(c[0], c[7])), math.sqrt(max(c[35], 0.0))


class Localizer(Node):
    def __init__(self):
        super().__init__('kelo_localize')
        hw = load_hardware()
        self.fr = hw['frames']
        self.mm_topic = hw['topics']['marvelmind_pose']
        self.amcl = None
        self.amcl_t = 0.0
        self.mm = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, LATCHED)
        self.prev_state = None
        self.create_subscription(String, '/kelo/localization_state', self._state_cb, LATCHED)
        self.create_subscription(PoseStamped, self.mm_topic, self._mm, 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.state_pub = self.create_publisher(String, '/kelo/localization_state_set', LATCHED)
        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)
        self.mapfit = None
        self.scan = None
        self.spin_cli = None
        self.create_subscription(OccupancyGrid, '/map', self._map, LATCHED)
        self.create_subscription(LaserScan, hw['topics']['scan'], self._scan, qos_profile_sensor_data)

    def _map(self, m):
        self.mapfit = sm.MapFit.from_msg(m)

    def _scan(self, m):
        self.scan = m

    def fit(self):
        """Share of the current scan on the walls of the map at the AMCL pose (None: cannot tell)."""
        if self.mapfit is None or self.scan is None:
            return None
        try:
            t = self.buf.lookup_transform(self.fr['map'], self.scan.header.frame_id, Time()).transform
        except TransformException:
            return None
        s = self.scan
        pts = sm.scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        return self.mapfit.fit(sm.transform(pts, t.translation.x, t.translation.y, yaw_from_quat(
            t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)), origin=(t.translation.x, t.translation.y))

    def _state_cb(self, m):
        if self.prev_state is None:
            self.prev_state = m.data

    def _amcl(self, m):
        self.amcl, self.amcl_t = m, time.time()

    def _mm(self, m):
        try:
            tf = self.buf.lookup_transform(self.fr['map'], m.header.frame_id, Time())
        except TransformException:
            return
        yaw = yaw_from_quat(tf.transform.rotation.x, tf.transform.rotation.y,
                            tf.transform.rotation.z, tf.transform.rotation.w)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = m.pose.position.x, m.pose.position.y
        self.mm.append((c * x - s * y + tf.transform.translation.x,
                        s * x + c * y + tf.transform.translation.y))

    def spin_for(self, sec, cond=None):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond is not None and cond():
                return True
        return cond is None

    def state(self, text):
        self.state_pub.publish(String(data=text))
        log(f'state: {text}')

    def marvelmind_xy(self, n=10, timeout=15.0):
        self.mm.clear()
        if not self.spin_for(3.0, lambda: self.buf.can_transform(
                self.fr['map'], self.fr['marvelmind_map'], Time())):
            return None
        if not self.spin_for(timeout, lambda: len(self.mm) >= n):
            return None
        return statistics.median(p[0] for p in self.mm), statistics.median(p[1] for p in self.mm)

    def send_initial(self, x, y, yaw, yaw_known):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.fr['map']
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        _, _, m.pose.pose.orientation.z, m.pose.pose.orientation.w = quat_from_yaw(yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.10
        cov[35] = 0.07 if yaw_known else 10.0
        m.pose.covariance = cov
        t0 = time.time()
        for _ in range(3):
            self.init_pub.publish(m)
            self.spin_for(0.2)
        self.spin_for(5.0, lambda: self.amcl_t > t0)

    def global_init(self):
        cli = self.create_client(Empty, '/reinitialize_global_localization')
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('AMCL /reinitialize_global_localization service missing')
        fut = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

    def spin_server(self):
        if self.spin_cli is None:
            self.spin_cli = ActionClient(self, Spin, 'spin')
        if not self.spin_cli.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("Nav2 'spin' behavior missing (is navigation running?)")
        return self.spin_cli

    def rotate(self, angle):
        cli = self.spin_server()
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance.sec = 60
        handle = None
        end = time.time() + 60.0
        while time.time() < end:
            fut = cli.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            handle = fut.result()
            if handle is not None and handle.accepted:
                break
            log('Nav2 not active yet, retrying the turn in 2 s...')
            self.spin_for(2.0)
        if handle is None or not handle.accepted:
            raise RuntimeError('spin rejected (Nav2 did not become active)')
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=70.0)
        if res.result() is None or res.result().status != GoalStatus.STATUS_SUCCEEDED:
            log('the turn stopped early (obstacle close to the robot?): AMCL saw less of the room')
            return False
        return True

    def map_layers(self, enabled):
        """Switch the map-fixed layers of the local costmap; returns the layers switched."""
        get = self.create_client(GetParameters, f'{LOCAL_COSTMAP}/get_parameters')
        put = self.create_client(SetParameters, f'{LOCAL_COSTMAP}/set_parameters')
        if not (get.wait_for_service(timeout_sec=5.0) and put.wait_for_service(timeout_sec=5.0)):
            return []
        fut = get.call_async(GetParameters.Request(names=['plugins']))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        plugins = list(res.values[0].string_array_value) if res is not None and res.values else []
        layers = [p for p in plugins if p in LOCAL_MAP_LAYERS]
        if layers:
            value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(enabled))
            fut = put.call_async(SetParameters.Request(
                parameters=[Parameter(name=f'{p}.enabled', value=value) for p in layers]))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is None or not all(r.successful for r in fut.result().results):
                raise RuntimeError(f'could not switch the local costmap layers {layers}')
        return layers

    def converged(self):
        """(accepted, (xy std, yaw std), scan fit)"""
        if self.amcl is None:
            return False, (float('inf'), float('inf')), None
        self.spin_for(1.0)
        xy, yaw = amcl_std(self.amcl)
        f = self.fit()
        return accept(xy, yaw, f), (xy, yaw), f

    def try_last_pose(self, map_yaml):
        """The pose saved by loc_monitor while localized: kept when the scan fits there."""
        pose = sm.load_last_pose(sm.last_pose_file(data_dir()), map_yaml)
        if pose is None:
            return False
        log(f'trying the last saved pose: x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.0f} deg')
        self.send_initial(*pose, True)
        self.spin_for(2.0)
        ok, (sxy, syaw), f = self.converged()
        log(f'scan fit {fmt_fit(f)}, AMCL uncertainty xy {sxy:.2f} m, yaw {math.degrees(syaw):.1f} deg')
        if ok and f is not None:
            return True
        log('the robot is not at the last saved pose (the scan does not fit there)')
        return False


def fmt_fit(f):
    return '-' if f is None else f'{f:.0%}'


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--yaw', type=float, help='degrees; if omitted the heading is treated as unknown')
    ap.add_argument('--global', dest='global_', action='store_true',
                    help='no Marvelmind: AMCL global localization (whole map)')
    ap.add_argument('--no-spin', action='store_true', help='only set the pose, do not turn in place')
    ap.add_argument('--force', action='store_true',
                    help='re-localize from scratch even if already localized')
    ap.add_argument('--map', default=os.environ.get('KELO_MAP', '/data/maps/current.yaml'),
                    help='map of the running navigation (for the last saved pose)')
    args = ap.parse_args()
    if (args.x is None) != (args.y is None):
        sys.exit('--x and --y must be given together')

    rclpy.init()
    node = Localizer()
    ok = False
    switched = None
    try:
        node.spin_for(2.0, lambda: node.prev_state is not None and node.amcl is not None)
        sxy, syaw = amcl_std(node.amcl) if node.amcl is not None else (float('inf'),) * 2
        # map, scan and the map -> laser TF of a fresh listener: up to 4 s
        if node.amcl is not None:
            node.spin_for(4.0, lambda: node.fit() is not None)
        f0 = node.fit() if node.amcl is not None else None
        if already_localized(node.prev_state, sxy, syaw, args.force, f0):
            log(f'Already localized (AMCL uncertainty xy {sxy:.2f} m, yaw {math.degrees(syaw):.1f} deg, '
                f'scan fit {fmt_fit(f0)}): nothing changed. Use --force / GUI confirmation to re-localize '
                'from scratch.')
            ok = True
            sys.exit(EXIT_ALREADY)
        if node.prev_state == 'LOCALIZED':
            log('State LOCALIZED but ' + ('--force requested' if args.force else
                f'uncertainty high (xy {sxy:.2f} m, yaw {math.degrees(syaw):.1f} deg) or the scan does not '
                f'fit the map ({fmt_fit(f0)})')
                + ': re-localizing.')
        node.state('LOCALIZING')
        yaw_known = args.yaw is not None
        yaw = math.radians(args.yaw) if yaw_known else 0.0
        use_global = args.global_
        xy = None
        if not use_global:
            if args.x is not None:
                xy = (args.x, args.y)
            else:
                xy = node.marvelmind_xy()
                if xy is None:
                    log('No Marvelmind position (no alignment file or no signal): '
                        'trying GLOBAL mode (AMCL searches the whole map + turn in place).')
                    use_global = True
                else:
                    log(f'Marvelmind position (map): x={xy[0]:.2f} y={xy[1]:.2f}')
        if use_global and not args.global_ and not args.force and node.try_last_pose(args.map):
            ok = True
        else:
            tries = 0 if args.no_spin else (3 if use_global else 2)
            for rnd in range(GLOBAL_ROUNDS if use_global else 1):
                if use_global:
                    log('AMCL global localization (particles spread over the whole map)'
                        + (f', round {rnd + 1}/{GLOBAL_ROUNDS}' if rnd else ''))
                    node.global_init()
                    node.amcl = None
                    node.spin_for(3.0, lambda: node.amcl is not None)
                else:
                    x, y = xy
                    log(f"start pose: x={x:.2f} y={y:.2f} yaw={'%.0f deg' % args.yaw if yaw_known else 'UNKNOWN'}")
                    node.send_initial(x, y, yaw, yaw_known)
                for i in range(tries + 1):
                    ok, (sxy, syaw), f = node.converged()
                    log(f'AMCL uncertainty: xy {sxy:.2f} m, yaw {math.degrees(syaw):.1f} deg, scan fit {fmt_fit(f)}')
                    if ok or i == tries:
                        break
                    if switched is None:
                        # Nav2 starts only once AMCL has a pose (map->odom): switch the layers now
                        node.spin_server()
                        switched = node.map_layers(False)
                        if switched:
                            log(f'local costmap map layers off during the turns: {", ".join(switched)}')
                    log('turning in place (360 deg, safety chain active)...')
                    node.rotate(2.0 * math.pi)
                    node.spin_for(2.0)
                if ok:
                    break
                if use_global and sxy < XY_STD_FIT:
                    log(f'AMCL settled where the scan does not fit the map ({fmt_fit(f)}): wrong place, '
                        'searching again')
        node.state('LOCALIZED' if ok else 'FAILED')
        if not ok:
            log('Did not converge: give the position/heading with "Set Initial Pose" (x, y, yaw) '
                'or RViz "2D Pose Estimate", then Localize again.')
    except RuntimeError as exc:
        log(f'ERROR: {exc}')
        node.state('FAILED')
    finally:
        if switched:
            try:
                node.map_layers(True)
                log('local costmap map layers back on')
            except RuntimeError as exc:
                log(f'ERROR: {exc}: restart navigation')
        node.spin_for(0.5)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
