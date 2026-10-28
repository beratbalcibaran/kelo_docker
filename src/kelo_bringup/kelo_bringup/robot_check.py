DESCRIPTION = """First-connection check of the robot interface and the sensors (does not move the robot).

Listens for a few seconds and checks what hardware.yaml expects: the robot's /odom (rate, frames, clock),
who publishes odom -> base_link, /cmd_vel (type, who subscribes, who publishes), the LiDAR, the depth
camera, Marvelmind, and the map. Run it with the robot driver on and the KELO stack off, or with the
stack in sensors mode:

  ros2 run kelo_bringup robot_check
  ros2 run kelo_bringup robot_check --time 10

exit code: 0 no FAIL, 1 at least one FAIL
"""
import argparse
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from kelo_bringup.common import load_hardware, maps_dir, sensors

OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'
MIN_HZ = {'odom': 10.0, 'scan': 5.0, 'depth': 5.0, 'marvelmind': 2.0}
MAX_CLOCK_OFFSET_S = 0.2
OUR_CMD_PUBLISHERS = ('cmd_vel_gate',)
OUR_ODOM_TF = ('odom_filter', 'rf2o_laser_odometry', 'sim_robot')
# KELO nodes that only watch /cmd_vel (driver views): not the robot driver
OUR_CMD_WATCHERS = ('surround_view', 'parking_view', 'web_view', 'objects', 'social', 'video_recorder',
                    'coverage_view', 'camera_view', 'map_side_view', 'kelo_e2e', 'kelo_robot_check')


class Stream:
    """Message count, rate, frames and the clock offset (receive time - header stamp) of one topic."""

    def __init__(self):
        self.n = 0
        self.t0 = self.t1 = None
        self.frame = self.child = None
        self.offsets = []
        self.last = None

    def add(self, now, stamp_s, frame, child=None, msg=None):
        self.n += 1
        self.t0 = now if self.t0 is None else self.t0
        self.t1 = now
        self.frame, self.child, self.last = frame, child, msg
        if stamp_s > 0:
            self.offsets.append(now - stamp_s)

    def hz(self):
        if self.n < 2 or self.t1 <= self.t0:
            return 0.0
        return (self.n - 1) / (self.t1 - self.t0)

    def offset(self):
        if not self.offsets:
            return None
        s = sorted(self.offsets)
        return s[len(s) // 2]


def _rate(name, st, min_hz):
    if st.n == 0:
        return FAIL, f'{name}: no messages'
    hz = st.hz()
    return (OK if hz >= min_hz else FAIL), f'{name}: {hz:.1f} Hz (min {min_hz:.0f})'


def _clock(name, st):
    off = st.offset()
    if off is None:
        return None
    if abs(off) > MAX_CLOCK_OFFSET_S:
        return FAIL, (f'{name}: header stamps are {off:+.2f} s off this computer\'s clock (latency or clocks '
                      'not synchronised: chrony on both computers)')
    return OK, f'{name}: clock offset {off * 1000:+.0f} ms'


def evaluate(hw, obs):
    """Check results [(level, text)] from what was observed:
    obs = {'streams': {key: Stream}, 'cmd_pubs': [node names], 'cmd_subs': [(node, type)],
           'tf_pubs': [node names], 'odom_tf_seen': bool, 'map': path or None, 'camera_map': path or None,
           'domain': str}"""
    fr, sens, out = hw['frames'], sensors(hw), []
    st = obs['streams']
    out.append((OK, f"ROS_DOMAIN_ID={obs.get('domain') or '0 (default)'}"))

    o = st['odom']
    out.append(_rate(f"odom {hw['topics']['odom']}", o, MIN_HZ['odom']))
    if o.n:
        good = o.frame == fr['odom'] and o.child == fr['base']
        out.append((OK if good else FAIL, f"odom frames {o.frame} -> {o.child} (expected {fr['odom']} -> {fr['base']})"))
        c = _clock('odom', o)
        if c:
            out.append(c)
        tw = o.last.twist.twist if o.last is not None else None
        if tw is not None and math.hypot(tw.linear.x, tw.linear.y) > 0.02:
            out.append((WARN, f'odom reports motion ({tw.linear.x:.2f}, {tw.linear.y:.2f} m/s) while the robot should stand'))

    mode = hw['robot']['odom_mode']
    ours = [n for n in obs['tf_pubs'] if any(n.startswith(p) for p in OUR_ODOM_TF)]
    others = [n for n in obs['tf_pubs'] if n not in ours]
    if obs['odom_tf_seen'] and mode == 'corrected' and not ours:
        out.append((FAIL, f"{fr['odom']} -> {fr['base']} TF is published by {', '.join(others) or 'another node'}, "
                          "but robot.odom_mode is 'corrected' (odom_filter owns this TF): switch it off in the "
                          "driver or set odom_mode: driver"))
    elif obs['odom_tf_seen'] and mode == 'corrected' and others:
        out.append((WARN, f"the KELO stack publishes {fr['odom']} -> {fr['base']}; whether the driver does too can "
                          'only be checked with the stack off'))
    elif not obs['odom_tf_seen'] and mode == 'driver':
        out.append((FAIL, f"robot.odom_mode is 'driver' but nobody publishes {fr['odom']} -> {fr['base']}"))
    else:
        out.append((OK, f"{fr['odom']} -> {fr['base']} TF ownership matches odom_mode '{mode}'"))

    want = hw['robot']['cmd_vel_type']
    subs = [(n, t) for n, t in obs['cmd_subs'] if n not in OUR_CMD_WATCHERS]
    if not subs:
        out.append((FAIL, f"{hw['topics']['cmd_vel']}: nobody subscribes (robot driver not running or other topic name)"))
    for node, typ in subs:
        short = typ.split('/')[-1]
        out.append((OK if short == want else FAIL,
                    f"{hw['topics']['cmd_vel']} subscriber {node}: {short} (robot.cmd_vel_type {want})"))
    others = [n for n in obs['cmd_pubs'] if n not in OUR_CMD_PUBLISHERS]
    if others:
        out.append((FAIL, f"{hw['topics']['cmd_vel']} has other publishers: {', '.join(others)} (only cmd_vel_gate "
                          'may command the robot)'))

    s = st['scan']
    out.append(_rate(f"LiDAR {hw['topics']['scan']}", s, MIN_HZ['scan']))
    if s.n:
        out.append((OK if s.frame == fr['laser'] else FAIL, f"LiDAR frame {s.frame} (expected {fr['laser']})"))
        m = s.last
        valid = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
        share = len(valid) / max(1, len(m.ranges))
        out.append((OK, f'LiDAR valid readings {share:.0%}') if share >= 0.3 else
                   (WARN, f'LiDAR valid readings only {share:.0%} (dirty window or open space?)'))
        lim = hw.get('lidar', {})
        if lim and (m.angle_min > lim['angle_min'] + 0.02 or m.angle_max < lim['angle_max'] - 0.02):
            out.append((WARN, f"LiDAR scan {math.degrees(m.angle_min):.0f}..{math.degrees(m.angle_max):.0f} deg "
                              f"narrower than hardware.yaml lidar angles"))
        c = _clock('LiDAR', s)
        if c:
            out.append(c)
    if sens['camera']:
        d = st['depth']
        out.append(_rate(f"depth {hw['topics']['camera_depth']}", d, MIN_HZ['depth']))
        if d.n:
            out.append((OK if d.frame == fr['camera_optical'] else FAIL,
                        f"depth frame {d.frame} (expected {fr['camera_optical']})"))
        if st['camera_info'].n == 0:
            out.append((FAIL, f"{hw['topics']['camera_info']}: no messages"))
    if sens['marvelmind']:
        out.append(_rate(f"Marvelmind {hw['topics']['marvelmind_raw']}", st['marvelmind'], MIN_HZ['marvelmind']))

    if obs.get('map'):
        out.append((OK, f"map {obs['map']}" + (' + camera layer' if obs.get('camera_map') else '')))
    else:
        out.append((WARN, 'no map yet (data/maps/current.yaml): map first (mapping mode)'))
    return out


class Checker(Node):
    def __init__(self, hw):
        super().__init__('kelo_robot_check')
        self.hw = hw
        tp = hw['topics']
        self.streams = {k: Stream() for k in ('odom', 'scan', 'depth', 'camera_info', 'marvelmind')}
        self.odom_tf_seen = False
        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, tp['odom'], lambda m: self._add('odom', m, m.child_frame_id), sd)
        self.create_subscription(LaserScan, tp['scan'], lambda m: self._add('scan', m), sd)
        self.create_subscription(Image, tp['camera_depth'], lambda m: self._add('depth', m), sd)
        self.create_subscription(CameraInfo, tp['camera_info'], lambda m: self._add('camera_info', m), sd)
        self.create_subscription(PoseStamped, tp['marvelmind_pose'], lambda m: self._add('marvelmind', m), 10)
        self.create_subscription(TFMessage, '/tf', self._tf, 50)

    def _add(self, key, msg, child=None):
        st = msg.header.stamp
        self.streams[key].add(time.time(), st.sec + st.nanosec * 1e-9, msg.header.frame_id, child, msg)

    def _tf(self, msg):
        fr = self.hw['frames']
        for t in msg.transforms:
            if t.header.frame_id == fr['odom'] and t.child_frame_id == fr['base']:
                self.odom_tf_seen = True

    def graph(self):
        tp = self.hw['topics']
        pubs = [i.node_name for i in self.get_publishers_info_by_topic(tp['cmd_vel'])]
        subs = [(i.node_name, i.topic_type) for i in self.get_subscriptions_info_by_topic(tp['cmd_vel'])]
        tf_pubs = [i.node_name for i in self.get_publishers_info_by_topic('/tf')]
        return pubs, subs, tf_pubs


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--time', type=float, default=6.0, help='seconds to listen')
    args = ap.parse_args()
    hw = load_hardware()
    rclpy.init()
    node = Checker(hw)
    end = time.time() + args.time
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    pubs, subs, tf_pubs = node.graph()
    current = os.path.join(maps_dir(), 'current.yaml')
    real = os.path.realpath(current)
    camera = (real[:-5] if real.endswith('.yaml') else real) + '_camera.yaml'
    obs = {'streams': node.streams, 'cmd_pubs': pubs, 'cmd_subs': subs, 'tf_pubs': tf_pubs,
           'odom_tf_seen': node.odom_tf_seen, 'domain': os.environ.get('ROS_DOMAIN_ID', ''),
           'map': current if os.path.isfile(current) else None,
           'camera_map': camera if os.path.isfile(camera) else None}
    results = evaluate(hw, obs)
    node.destroy_node()
    rclpy.try_shutdown()
    for level, text in results:
        print(f'[robot_check] {level:<4} {text}', flush=True)
    fails = sum(1 for level, _ in results if level == FAIL)
    warns = sum(1 for level, _ in results if level == WARN)
    print(f'[robot_check] {fails} FAIL, {warns} WARN', flush=True)
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
