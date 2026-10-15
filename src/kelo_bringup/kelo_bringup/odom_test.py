DESCRIPTION = """Odometry characterisation. The robot moves.

usage:
  ros2 run kelo_bringup odom_test straight --speed 0.1 --distance 2.0
  ros2 run kelo_bringup odom_test rotate --rate 0.3 --turns 1
  ros2 run kelo_bringup odom_test summary
  ros2 run kelo_bringup odom_test calibrate-rotate --rate 1.5 --turns 3

calibrate-rotate bypasses cmd_vel_gate and is for calibration only.
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from kelo_bringup.common import data_dir, load_hardware
from kelo_bringup.logic import wrap, yaw_from_quat

HARD_MAX_SPEED = 0.25
HARD_MAX_RATE = 0.8
MAX_DURATION_S = 180.0
CAL_MAX_RATE = 1.6
CAL_MAX_TURNS = 3.0
CAL_CONFIRM = 'CALIBRATE'
CAL_BANNER = """
================================================================================
 !!! CALIBRATION ONLY - DO NOT USE IN PRODUCTION !!!
 This mode BYPASSES cmd_vel_gate (speed limit, watchdog, safety stop) and collision_monitor;
 it writes the command directly to the robot driver. The robot turns IN PLACE (no linear speed).
 - At least 1 m of free space around the robot, nobody nearby
 - Hardware EMERGENCY STOP in your hand
 - KELO stack (cmd_vel_gate) OFF (this tool refuses to run otherwise)
================================================================================
"""
FIELDS = ['time', 'test', 'cmd', 'odom_value', 'measured', 'ratio']


class OdomTester(Node):
    def __init__(self, topic, cmd_topic):
        super().__init__('odom_test')
        self.pose = None
        self.yaw_sum = 0.0
        self._last_yaw = None
        self.create_subscription(Odometry, topic, self._cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, cmd_topic, 10)

    def _cb(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        if self._last_yaw is not None:
            self.yaw_sum += wrap(yaw - self._last_yaw)
        self._last_yaw = yaw
        self.pose = (p.position.x, p.position.y, yaw)

    def wait_odom(self, timeout=5.0):
        end = time.time() + timeout
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            raise RuntimeError('no odom received')

    def other_publishers(self, topic, wait_s=2.0):
        end = time.time() + wait_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.count_publishers(topic)

    def drive(self, vx, wz, finished):
        t0 = time.time()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                elapsed = time.time() - t0
                if finished(self, elapsed) or elapsed > MAX_DURATION_S:
                    break
                msg = Twist()
                msg.linear.x, msg.angular.z = vx, wz
                self.pub.publish(msg)
                time.sleep(0.05)
        finally:
            for _ in range(10):
                self.pub.publish(Twist())
                time.sleep(0.02)


class DirectDriver(Node):
    def __init__(self, topic, msg_type, base_frame):
        super().__init__('odom_test_calibration')
        self.stamped = msg_type == 'TwistStamped'
        self.base_frame = base_frame
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, topic, 10)

    def send(self, wz):
        tw = Twist()
        tw.angular.z = wz
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.twist = tw
            self.pub.publish(msg)
        else:
            self.pub.publish(tw)


def calibrate_rotate(args):
    hw = load_hardware()
    topic, msg_type = hw['topics']['cmd_vel'], hw['robot']['cmd_vel_type']
    rate, turns = abs(args.rate), abs(args.turns)
    if rate > CAL_MAX_RATE or turns > CAL_MAX_TURNS or rate <= 0.0:
        sys.exit(f'Calibration limit: 0 < rate <= {CAL_MAX_RATE} rad/s, turns <= {CAL_MAX_TURNS}')
    print(CAL_BANNER)
    rclpy.init()
    tester = OdomTester(args.odom, '/odom_test_unused')
    try:
        n = tester.other_publishers(topic)
        if n > 0:
            sys.exit(f'REFUSED: {topic} has {n} publisher(s) (cmd_vel_gate / stack is running). '
                     'Calibration only runs with the stack OFF and the robot driver on its own.')
        tester.wait_odom()
        answer = input(f"Type '{CAL_CONFIRM}' to continue: ").strip()
        if answer != CAL_CONFIRM:
            sys.exit('Cancelled.')
        drv = DirectDriver(topic, msg_type, hw['frames']['base'])
        t_total = 2.0 * math.pi * turns / rate
        tester.yaw_sum = 0.0
        t0 = time.time()
        try:
            while rclpy.ok() and time.time() - t0 < min(t_total, MAX_DURATION_S):
                rclpy.spin_once(tester, timeout_sec=0.0)
                drv.send(rate)
                time.sleep(0.05)
        finally:
            for _ in range(20):
                drv.send(0.0)
                time.sleep(0.02)
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(tester, timeout_sec=0.05)
        odom_turns = abs(tester.yaw_sum) / (2.0 * math.pi)
        if args.measured_turns is not None:
            measured = float(args.measured_turns)
        else:
            measured = float(input(f'Commanded {turns} turns ({rate} rad/s), odom {odom_turns:.3f} turns. '
                                   'REAL number of turns: '))
        print(f'  k = real/odom = {measured / odom_turns:.2f}  (wz = {rate:.2f} rad/s)')
        print('  Note: k is lower during the acceleration/deceleration ramp, so the measurement reads k '
              'slightly LOW (sim: 1 turn 5%, 3 turns 2%). Use >= 3 turns at high rates.')
        record('rotate', f'{rate:.2f}', odom_turns, measured)
    finally:
        tester.destroy_node()
        rclpy.try_shutdown()


def record(test, cmd, odom_value, measured):
    path = os.path.join(data_dir(), 'odom_tests.csv')
    new = not os.path.isfile(path)
    ratio = measured / odom_value if odom_value else float('nan')
    with open(path, 'a', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        if new:
            w.writerow(FIELDS)
        w.writerow([time.strftime('%F %T'), test, cmd, f'{odom_value:.4f}',
                    f'{measured:.4f}', f'{ratio:.4f}'])
    print(f'  ratio (real/odom) = {ratio:.3f}   saved: {path}')


def summary():
    path = os.path.join(data_dir(), 'odom_tests.csv')
    if not os.path.isfile(path):
        sys.exit('No records.')
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    for kind in ('straight', 'rotate'):
        groups = {}
        for r in rows:
            if r['test'] == kind:
                groups.setdefault(r['cmd'], []).append(float(r['ratio']))
        if groups:
            print(f'\n[{kind}] speed -> mean ratio (real/odom)')
            for cmd, vals in sorted(groups.items()):
                print(f'  {cmd:>6s}: {sum(vals) / len(vals):.3f}  (n={len(vals)})')
    print('\nFill in the hardware.yaml odom_correction tables (robot.odom_mode: corrected):\n'
          '  yaw_k: wz = [rotation rates], k = [rotate ratios]   (real_angle = k * odom_angle)\n'
          '  linear_error: v = [speeds], e = [1/ratio - 1]        (odom = real * (1 + e))')


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('straight', 'rotate'):
        p = sub.add_parser(name)
        p.add_argument('--odom', default='/odom')
        p.add_argument('--cmd-topic', default='/cmd_vel_teleop')
        if name == 'straight':
            p.add_argument('--speed', type=float, default=0.1)
            p.add_argument('--distance', type=float, default=2.0)
        else:
            p.add_argument('--rate', type=float, default=0.3)
            p.add_argument('--turns', type=float, default=1.0)
    sub.add_parser('summary')
    cal = sub.add_parser('calibrate-rotate',
                         help='CALIBRATION ONLY: bypasses the gate, writes directly to the driver')
    cal.add_argument('--odom', default='/odom')
    cal.add_argument('--rate', type=float, required=True)
    cal.add_argument('--turns', type=float, default=3.0)
    cal.add_argument('--measured-turns', type=float, default=None,
                     help='real number of turns (asked if omitted)')
    args = ap.parse_args()
    if args.cmd == 'summary':
        return summary()
    if args.cmd == 'calibrate-rotate':
        return calibrate_rotate(args)

    speed = abs(getattr(args, 'speed', 0.0))
    rate = abs(getattr(args, 'rate', 0.0))
    if speed > HARD_MAX_SPEED or rate > HARD_MAX_RATE:
        sys.exit(f'Safety limit: speed <= {HARD_MAX_SPEED} m/s, rotation <= {HARD_MAX_RATE} rad/s')
    input('Open area, emergency stop IN YOUR HAND, nothing in front of the robot. Press ENTER to start: ')

    rclpy.init()
    node = OdomTester(args.odom, args.cmd_topic)
    try:
        node.wait_odom()
        if args.cmd == 'straight':
            x0, y0 = node.pose[0], node.pose[1]
            node.drive(speed, 0.0, lambda n, t: math.hypot(
                n.pose[0] - x0, n.pose[1] - y0) >= args.distance)
            odom_d = math.hypot(node.pose[0] - x0, node.pose[1] - y0)
            measured = float(input(f'odom distance = {odom_d:.3f} m. Distance MEASURED on the floor (m): '))
            record('straight', f'{speed:.2f}', odom_d, measured)
        else:
            t_total = 2.0 * math.pi * args.turns / rate
            node.yaw_sum = 0.0
            node.drive(0.0, rate, lambda n, t: t >= t_total)
            odom_turns = abs(node.yaw_sum) / (2.0 * math.pi)
            measured = float(input(f'Commanded {args.turns} turns, odom {odom_turns:.3f} turns. '
                                   'REAL number of turns: '))
            record('rotate', f'{rate:.2f}', odom_turns, measured)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
