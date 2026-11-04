DESCRIPTION = """Finds the robot driver on the network and proposes the hardware.yaml settings for it.

Run it with the robot connected and the KELO stack OFF (it does not move the robot). It looks for ROS 2
nodes on several ROS_DOMAIN_IDs at once, then on the domain with the robot:
  - the velocity command topic (a Twist / TwistStamped topic the driver subscribes to) and its type
  - the odometry topic (nav_msgs/Odometry), its rate, frames and clock offset
  - whether the driver publishes odom -> base_link itself (-> robot.odom_mode driver / corrected)
  - TF frames of the driver that clash with the KELO robot description
and checks that the LiDAR, the RealSense and Marvelmind are plugged into this computer.

  ros2 run kelo_bringup robot_discover            # report and proposal
  ros2 run kelo_bringup robot_discover --apply    # also write the proposal to /data/hardware.yaml
  ros2 run kelo_bringup robot_discover --domains 0-20 --time 5   # default: 0-10,42

exit code: 0 robot found, 1 not found or the KELO stack is running
"""
import argparse
import glob
import json
import os
import statistics
import sys
import time

from kelo_bringup.common import data_dir, load_hardware, sensors
from kelo_bringup.robot_check import OUR_CMD_PUBLISHERS, OUR_CMD_WATCHERS, OUR_ODOM_TF

OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'
TWIST_TYPES = {'geometry_msgs/msg/Twist': 'Twist', 'geometry_msgs/msg/TwistStamped': 'TwistStamped'}
ODOM_TYPE = 'nav_msgs/msg/Odometry'
MIN_ODOM_HZ = 10.0
MAX_CLOCK_OFFSET_S = 0.2
# 42: the robot computer's container (robot/docker/docker-compose.yml)
DEFAULT_DOMAINS = list(range(0, 11)) + [42]
# nodes of the KELO stack and its tools: never taken for the robot driver
OUR_NODES = set(OUR_CMD_PUBLISHERS) | set(OUR_CMD_WATCHERS) | set(OUR_ODOM_TF) | {
    'kelo_robot_discover', 'kelo_gui_teleop', 'collision_monitor_teleop', 'collision_monitor',
    'velocity_smoother', 'controller_server', 'bt_navigator', 'status_node'}
REALSENSE_USB_VENDOR = '8086'


def is_ours(node):
    # a KELO robot driver may well be called kelo_*: only our own exact node names count
    return node in OUR_NODES or node.startswith(('_ros2cli', 'kelo_robot_discover'))


def parse_domains(text):
    out = []
    for part in text.split(','):
        part = part.strip()
        if '-' in part:
            a, b = part.split('-', 1)
            out.extend(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return [d for d in dict.fromkeys(out) if 0 <= d <= 232]


def _name_score(topic, want, word):
    if topic == want:
        return 3
    if topic.rstrip('/').split('/')[-1] == want.split('/')[-1]:
        return 2
    return 1 if word in topic else 0


def propose(hw, obs):
    """Proposed settings and report from one domain's observation.

    obs = {'domain': int, 'current_domain': int, 'nodes': [names],
           'topics': {name: {'pubs': [[node, type]], 'subs': [[node, type]]}},
           'odom': {topic: {'hz', 'frame', 'child', 'offset'}}, 'tf_pairs': [[parent, child]],
           'devices': {'lidar': bool|None, 'camera': bool|None, 'marvelmind': bool|None}}
    returns {'found': bool, 'settings': [[[key, ...], value]], 'notes': [[level, text]]}"""
    notes, settings = [], []
    fr, tp = hw['frames'], hw['topics']
    if 'cmd_vel_gate' in obs['nodes']:
        notes.append([FAIL, 'the KELO stack is running (cmd_vel_gate): stop it first, discovery needs the '
                            'robot driver alone'])
        return {'found': False, 'settings': [], 'notes': notes}
    foreign = sorted(n for n in obs['nodes'] if not is_ours(n))
    if not foreign:
        notes.append([FAIL, 'no robot driver found (no ROS 2 nodes besides KELO tools)'])
        return {'found': False, 'settings': [], 'notes': notes}
    notes.append([OK, f"ROS_DOMAIN_ID {obs['domain']}: nodes {', '.join(foreign[:8])}"
                      + (' ...' if len(foreign) > 8 else '')])
    if obs['domain'] != obs['current_domain']:
        settings.append([['env', 'ROS_DOMAIN_ID'], obs['domain']])
        notes.append([WARN, f"the robot is on ROS_DOMAIN_ID {obs['domain']}, this computer uses "
                            f"{obs['current_domain']}: set ROS_DOMAIN_ID={obs['domain']} (.env)"])

    # velocity command: a Twist topic that a foreign node subscribes to
    cmd = []
    for name, t in obs['topics'].items():
        for node, typ in t['subs']:
            if typ in TWIST_TYPES and not is_ours(node):
                cmd.append((_name_score(name, tp['cmd_vel'], 'cmd'), name, node, TWIST_TYPES[typ]))
    cmd.sort(key=lambda c: (-c[0], c[1]))
    if not cmd:
        notes.append([FAIL, 'no velocity command topic: no driver node subscribes to a Twist / TwistStamped '
                            'topic'])
    else:
        _, name, node, typ = cmd[0]
        notes.append([OK, f'velocity commands: {name} ({typ}), subscriber {node}'])
        if name != tp['cmd_vel']:
            settings.append([['topics', 'cmd_vel'], name])
        if typ != hw['robot']['cmd_vel_type']:
            settings.append([['robot', 'cmd_vel_type'], typ])
        others = sorted({c[1] for c in cmd[1:] if c[1] != name})
        if others:
            notes.append([WARN, f"other Twist topics with driver subscribers: {', '.join(others)} "
                                f'(took {name}; check)'])
        pubs = [n for n, _ in obs['topics'][name]['pubs'] if not is_ours(n)]
        if pubs:
            notes.append([WARN, f"{name} already has publishers: {', '.join(pubs)} (only cmd_vel_gate may "
                                'command the robot: stop them)'])

    # odometry: an Odometry topic published by a foreign node
    odom = []
    for name, t in obs['topics'].items():
        if any(typ == ODOM_TYPE and not is_ours(n) for n, typ in t['pubs']):
            odom.append((_name_score(name, tp['odom'], 'odom'), name))
    odom.sort(key=lambda c: (-c[0], c[1]))
    odom_frames = None
    if not odom:
        notes.append([FAIL, 'no odometry: no driver node publishes nav_msgs/Odometry'])
    else:
        name = odom[0][1]
        if name != tp['odom']:
            settings.append([['topics', 'odom'], name])
        s = obs['odom'].get(name) or {}
        hz = s.get('hz') or 0.0
        if not s.get('frame'):
            notes.append([WARN, f'odometry: {name}, but no message received (driver idle?)'])
        else:
            odom_frames = (s['frame'], s['child'])
            notes.append([OK if hz >= MIN_ODOM_HZ else WARN,
                          f"odometry: {name} {hz:.1f} Hz, frames {s['frame']} -> {s['child']}"
                          + ('' if hz >= MIN_ODOM_HZ else f' (min {MIN_ODOM_HZ:.0f} Hz)')])
            if odom_frames != (fr['odom'], fr['base']):
                notes.append([WARN, f"odometry frames {s['frame']} -> {s['child']} differ from KELO "
                                    f"{fr['odom']} -> {fr['base']}: not changed automatically (the robot "
                                    'description uses them); rename in the driver or in hardware.yaml frames'])
            off = s.get('offset')
            if off is not None and abs(off) > MAX_CLOCK_OFFSET_S:
                notes.append([WARN, f'odometry stamps {off:+.2f} s off this clock: synchronise with chrony'])
            elif off is not None:
                notes.append([OK, f'clock offset {off * 1000:+.0f} ms'])
        if len(odom) > 1:
            notes.append([WARN, f"other odometry topics: {', '.join(n for _, n in odom[1:])} (took {name})"])

    # who owns odom -> base_link
    pairs = {tuple(p) for p in obs['tf_pairs']}
    base_pair = odom_frames or (fr['odom'], fr['base'])
    mode = 'driver' if base_pair in pairs else 'corrected'
    if odom:
        notes.append([OK, f'{base_pair[0]} -> {base_pair[1]} TF '
                          + ('published by the driver -> odom_mode driver' if mode == 'driver' else
                             'not published by the driver -> odom_mode corrected (odom_filter publishes it)')])
        if mode != hw['robot']['odom_mode']:
            settings.append([['robot', 'odom_mode'], mode])
    ours = {fr[k] for k in ('base', 'laser', 'camera', 'camera_optical', 'hedgehog')}
    clash = sorted(f'{p} -> {c}' for p, c in pairs if (p, c) != base_pair and (c in ours or c == fr['map']))
    if clash:
        notes.append([WARN, f"the driver also publishes TF {', '.join(clash)}: it clashes with the KELO robot "
                            'description (switch it off in the driver)'])

    # sensors on this computer
    want = sensors(hw)
    for key, label in (('lidar', 'LiDAR'), ('camera', 'RealSense'), ('marvelmind', 'Marvelmind')):
        seen = obs['devices'].get(key)
        if not want.get(key) or seen is None:
            continue
        notes.append([OK, f'{label} plugged in'] if seen else [WARN, f'{label} not found on this computer (USB)'])
    found = bool(cmd and odom)
    return {'found': found, 'settings': settings, 'notes': notes}


def local_devices(hw):
    ser = hw.get('serial', {})

    def port(key):
        p = ser.get(key)
        return os.path.exists(p) if p else None
    camera = None
    vendors = glob.glob('/sys/bus/usb/devices/*/idVendor')
    if vendors:
        camera = False
        for v in vendors:
            try:
                with open(v, encoding='ascii') as f:
                    camera = camera or f.read().strip() == REALSENSE_USB_VENDOR
            except OSError:
                pass
    return {'lidar': port('lidar_port'), 'camera': camera, 'marvelmind': port('marvelmind_port')}


def save_override(path, value):
    import yaml
    file = os.path.join(data_dir(), 'hardware.yaml')
    over = {}
    if os.path.isfile(file):
        with open(file, encoding='utf-8') as f:
            over = yaml.safe_load(f) or {}
    node = over
    for key in path[:-1]:
        if not isinstance(node.get(key), dict):
            node[key] = {}
        node = node[key]
    node[path[-1]] = value
    tmp = file + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write('# local settings of this robot, merged over the packaged hardware.yaml (KELO panel)\n')
        yaml.safe_dump(over, f, default_flow_style=False, sort_keys=False)
    os.replace(tmp, file)


# ---- ROS side --------------------------------------------------------------------------------------

class _Domain:
    """One participant on one ROS_DOMAIN_ID (own context, so several domains are watched at once)."""

    def __init__(self, domain):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        self.domain = domain
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=domain)
        self.node = rclpy.create_node(f'kelo_robot_discover_{domain}', context=self.ctx)
        self.exe = SingleThreadedExecutor(context=self.ctx)
        self.exe.add_node(self.node)

    def foreign_nodes(self):
        return [n for n, _ in self.node.get_node_names_and_namespaces() if not is_ours(n)]

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.exe.spin_once(timeout_sec=0.05)

    def close(self):
        import rclpy
        self.exe.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.ctx)


def _graph(d):
    n = d.node
    topics = {}
    for name, _types in n.get_topic_names_and_types():
        topics[name] = {'pubs': [[i.node_name, i.topic_type] for i in n.get_publishers_info_by_topic(name)],
                        'subs': [[i.node_name, i.topic_type] for i in n.get_subscriptions_info_by_topic(name)]}
    nodes = [name for name, _ in n.get_node_names_and_namespaces()]
    return nodes, topics


def observe(d, seconds):
    """Topic graph of the domain, then a few seconds of odometry and TF."""
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    nodes, topics = _graph(d)
    samples, pairs = {}, set()

    def odom_cb(topic):
        def cb(m):
            s = samples.setdefault(topic, {'t': [], 'off': [], 'frame': None, 'child': None})
            now = time.time()
            s['t'].append(now)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if st > 0:
                s['off'].append(now - st)
            s['frame'], s['child'] = m.header.frame_id, m.child_frame_id
        return cb

    def tf_cb(m):
        for t in m.transforms:
            pairs.add((t.header.frame_id, t.child_frame_id))

    for name, t in topics.items():
        if any(typ == ODOM_TYPE and not is_ours(nd) for nd, typ in t['pubs']):
            d.node.create_subscription(Odometry, name, odom_cb(name), qos_profile_sensor_data)
    d.node.create_subscription(TFMessage, '/tf', tf_cb, 50)
    d.node.create_subscription(TFMessage, '/tf_static', tf_cb,
                               QoSProfile(depth=50, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    d.spin(seconds)
    odom = {}
    for name, s in samples.items():
        ts = s['t']
        hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0
        odom[name] = {'hz': hz, 'frame': s['frame'], 'child': s['child'],
                      'offset': statistics.median(s['off']) if s['off'] else None}
    nodes, topics = _graph(d)   # again: late discoveries
    return {'domain': d.domain, 'nodes': nodes, 'topics': topics, 'odom': odom,
            'tf_pairs': sorted([list(p) for p in pairs])}


def discover(hw, domains, seconds, discovery_s=3.0, log=print):
    current = int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)
    domains = [current] + [d for d in domains if d != current]
    log(f"[robot_discover] looking for ROS 2 nodes on ROS_DOMAIN_ID {', '.join(map(str, domains))} "
        f'({discovery_s:.0f} s)...')
    parts = []
    try:
        for dom in domains:
            parts.append(_Domain(dom))
        end = time.time() + discovery_s
        while time.time() < end:
            for p in parts:
                p.spin(0.02)
        busy = [(p, p.foreign_nodes()) for p in parts]
        hit = [(p, n) for p, n in busy if n]
        stack = [p for p in parts if 'cmd_vel_gate' in [x for x, _ in p.node.get_node_names_and_namespaces()]]
        if stack:
            pick = stack[0]
        elif hit:
            pick = hit[0][0]   # the current domain first, then the lowest
            if len(hit) > 1:
                log(f"[robot_discover] nodes on several domains: {', '.join(str(p.domain) for p, _ in hit)}; "
                    f'using {pick.domain}')
        else:
            pick = parts[0]
        for p in parts:
            if p is not pick:
                p.close()
        parts = [pick]
        log(f'[robot_discover] listening on ROS_DOMAIN_ID {pick.domain} ({seconds:.0f} s)...')
        obs = observe(pick, seconds)
    finally:
        for p in parts:
            p.close()
    obs['current_domain'] = current
    obs['devices'] = local_devices(hw)
    return obs


def iface_state():
    iface = os.environ.get('KELO_DDS_IFACE', 'enP8p1s0')
    try:
        with open(f'/sys/class/net/{iface}/operstate', encoding='ascii') as f:
            return iface, f.read().strip()
    except OSError:
        return iface, 'missing'


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--time', type=float, default=4.0, help='seconds to listen on the robot domain')
    ap.add_argument('--domains', default='0-10,42',
                    help='ROS_DOMAIN_IDs to search, e.g. 0-10 or 0,5,42 (the current one is always searched)')
    ap.add_argument('--apply', action='store_true', help='write the proposed settings to /data/hardware.yaml')
    ap.add_argument('--json', action='store_true', help='also print the result as one JSON line (for the panel)')
    args = ap.parse_args()
    hw = load_hardware()
    iface, state = iface_state()
    if state != 'up':
        print(f'[robot_discover] WARN robot Ethernet {iface} is {state} (cable?)', flush=True)
    res = propose(hw, discover(hw, parse_domains(args.domains), args.time))
    for level, text in res['notes']:
        print(f'[robot_discover] {level:<4} {text}', flush=True)
    if res['settings']:
        print('[robot_discover] proposed settings:', flush=True)
        for path, value in res['settings']:
            print(f"[robot_discover]   {'.'.join(path)}: {value}", flush=True)
    elif res['found']:
        print('[robot_discover] hardware.yaml already matches the robot', flush=True)
    if args.apply:
        for path, value in res['settings']:
            if path[0] == 'env':
                print(f'[robot_discover] not written: {path[1]}={value} goes into .env', flush=True)
                continue
            save_override(path, value)
        print('[robot_discover] written to ' + os.path.join(data_dir(), 'hardware.yaml'), flush=True)
    if args.json:
        print('JSON ' + json.dumps(res), flush=True)
    sys.exit(0 if res['found'] else 1)


if __name__ == '__main__':
    main()
