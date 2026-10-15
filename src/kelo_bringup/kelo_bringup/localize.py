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
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup.common import load_hardware
from kelo_bringup.logic import quat_from_yaw, yaw_from_quat

XY_STD_OK = 0.15
YAW_STD_OK = math.radians(8.0)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
EXIT_ALREADY = 3


def already_localized(state, xy_std, yaw_std, force=False):
    return (not force and state == 'LOCALIZED'
            and xy_std < XY_STD_OK and yaw_std < YAW_STD_OK)


def log(msg):
    print(f'[localize] {msg}', flush=True)


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

    def rotate(self, angle):
        cli = ActionClient(self, Spin, 'spin')
        if not cli.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("Nav2 'spin' behavior missing (is navigation running?)")
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

    def converged(self):
        if self.amcl is None:
            return False, (float('inf'), float('inf'))
        xy, yaw = amcl_std(self.amcl)
        return xy < XY_STD_OK and yaw < YAW_STD_OK, (xy, yaw)


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
    args = ap.parse_args()
    if (args.x is None) != (args.y is None):
        sys.exit('--x and --y must be given together')

    rclpy.init()
    node = Localizer()
    ok = False
    try:
        node.spin_for(2.0, lambda: node.prev_state is not None and node.amcl is not None)
        sxy, syaw = amcl_std(node.amcl) if node.amcl is not None else (float('inf'),) * 2
        if already_localized(node.prev_state, sxy, syaw, args.force):
            log(f'Already localized (AMCL uncertainty xy {sxy:.2f} m, yaw {math.degrees(syaw):.1f} deg): '
                'nothing changed. Use --force / GUI confirmation to re-localize from scratch.')
            ok = True
            sys.exit(EXIT_ALREADY)
        if node.prev_state == 'LOCALIZED':
            log('State LOCALIZED but ' + ('--force requested' if args.force else
                f'uncertainty high (xy {sxy:.2f} m, yaw {math.degrees(syaw):.1f} deg)')
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
        if use_global:
            log('AMCL global localization (particles spread over the whole map)')
            node.global_init()
            node.amcl = None
        else:
            x, y = xy
            log(f"start pose: x={x:.2f} y={y:.2f} yaw={'%.0f deg' % args.yaw if yaw_known else 'UNKNOWN'}")
            node.send_initial(x, y, yaw, yaw_known)
        tries = 0 if args.no_spin else (4 if use_global else 2)
        for i in range(tries + 1):
            ok, (sxy, syaw) = node.converged()
            log(f'AMCL uncertainty: xy {sxy:.2f} m, yaw {math.degrees(syaw):.1f} deg')
            if ok or i == tries:
                break
            log('turning in place (360 deg, safety chain active)...')
            node.rotate(2.0 * math.pi)
            node.spin_for(2.0)
        node.state('LOCALIZED' if ok else 'FAILED')
        if not ok:
            log('Did not converge: give the heading with RViz "2D Pose Estimate" or retry with --yaw.')
    except RuntimeError as exc:
        log(f'ERROR: {exc}')
        node.state('FAILED')
    finally:
        node.spin_for(0.5)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
