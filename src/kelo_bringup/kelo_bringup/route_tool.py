DESCRIPTION = """Routes: stations visited one after the other (the GUI "Routes" window does the same).

usage:
  ros2 run kelo_bringup route_tool list
  ros2 run kelo_bringup route_tool show <route>
  ros2 run kelo_bringup route_tool add <route> <station> [--x X --y Y | (current position)]
                                       [--yaw DEG] [--wait S] [--pass]
  ros2 run kelo_bringup route_tool remove <route> [<station>]
  ros2 run kelo_bringup route_tool start <route> [--laps N] (0 = until stopped) [--from K]
  ros2 run kelo_bringup route_tool pause | resume | skip | stop | status
"""
import argparse
import json
import math
import sys
import time

from kelo_bringup import route as rt


def _pose():
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    from kelo_bringup.logic import yaw_from_quat
    rclpy.init()
    node = rclpy.create_node('route_tool_pose')
    buf = Buffer()
    TransformListener(buf, node)
    end = time.time() + 5.0
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = buf.lookup_transform('map', 'base_link', Time())
            except TransformException:
                continue
            q = t.transform.rotation
            return t.transform.translation.x, t.transform.translation.y, yaw_from_quat(q.x, q.y, q.z, q.w)
        sys.exit('map->base_link TF missing (is navigation running and the robot localized?)')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _send(cmd, wait_state=True):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('route_tool')
    states = []
    node.create_subscription(String, '/kelo/route/state', lambda m: states.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub = node.create_publisher(String, '/kelo/route/command', 10)
    try:
        end = time.time() + 3.0
        while time.time() < end and pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if cmd is not None:
            if pub.get_subscription_count() == 0:
                sys.exit('route_runner not running (start navigation first)')
            pub.publish(String(data=json.dumps(cmd)))
        end = time.time() + 2.0
        while time.time() < end and (wait_state or not states):
            rclpy.spin_once(node, timeout_sec=0.1)
        return json.loads(states[-1]) if states else None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def _print_route(name, r):
    print(f'{name}: {len(r["stations"])} stations')
    for k, s in enumerate(r['stations']):
        yaw = 'auto' if s['yaw'] is None else f'{math.degrees(s["yaw"]):.0f} deg'
        mode = 'stop' if s['stop'] else 'pass'
        wait = f', wait {s["wait_s"]:.0f} s' if s['wait_s'] else ''
        print(f'  {k + 1:2d}. {s["name"]:16s} x={s["x"]:7.2f} y={s["y"]:7.2f} yaw={yaw:8s} {mode}{wait}')


def main():
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list')
    sub.add_parser('show').add_argument('route')
    a = sub.add_parser('add')
    a.add_argument('route')
    a.add_argument('station')
    a.add_argument('--x', type=float)
    a.add_argument('--y', type=float)
    a.add_argument('--yaw', type=float, help='degrees; default: facing the next station')
    a.add_argument('--wait', type=float, default=0.0)
    a.add_argument('--pass', dest='pass_through', action='store_true', help='drive through without stopping')
    r = sub.add_parser('remove')
    r.add_argument('route')
    r.add_argument('station', nargs='?')
    s = sub.add_parser('start')
    s.add_argument('route')
    s.add_argument('--laps', type=int, default=1)
    s.add_argument('--from', dest='start', type=int, default=1, help='first station (1 = the first)')
    for c in ('pause', 'resume', 'skip', 'stop', 'status'):
        sub.add_parser(c)
    args = ap.parse_args()

    routes = rt.load_routes()
    if args.cmd == 'list':
        for name, route in routes.items():
            _print_route(name, route)
        if not routes:
            print(f'no routes in {rt.routes_file()}')
        return 0
    if args.cmd == 'show':
        if args.route not in routes:
            sys.exit(f'unknown route: {args.route}')
        _print_route(args.route, routes[args.route])
        return 0
    if args.cmd == 'add':
        if (args.x is None) != (args.y is None):
            sys.exit('give both --x and --y, or neither (current position)')
        x, y = (args.x, args.y) if args.x is not None else _pose()[:2]
        yaw = None if args.yaw is None else math.radians(args.yaw)
        route = routes.setdefault(args.route, {'stations': []})
        route['stations'].append(rt.station(args.station, x, y, yaw, args.wait, not args.pass_through))
        rt.save_routes(routes)
        _print_route(args.route, route)
        return 0
    if args.cmd == 'remove':
        if args.route not in routes:
            sys.exit(f'unknown route: {args.route}')
        if args.station is None:
            routes.pop(args.route)
            print(f'route {args.route} removed')
        else:
            before = len(routes[args.route]['stations'])
            routes[args.route]['stations'] = [s for s in routes[args.route]['stations'] if s['name'] != args.station]
            if len(routes[args.route]['stations']) == before:
                sys.exit(f'no station {args.station} in {args.route}')
            _print_route(args.route, routes[args.route])
        rt.save_routes(routes)
        return 0
    if args.cmd == 'start':
        if args.route not in routes:
            sys.exit(f'unknown route: {args.route}')
        st = _send({'cmd': 'start', 'route': args.route, 'laps': args.laps, 'start': args.start - 1})
    elif args.cmd == 'status':
        st = _send(None)
    else:
        st = _send({'cmd': args.cmd})
    if st is None:
        print('no route state (route_runner not running?)')
        return 1
    print(f'{st["status"]}: {st.get("message", "")}'
          + (f' | route {st["route"]}, lap {st.get("lap")}/{st.get("laps") or "inf"}, '
             f'station {st.get("index", 0) + 1}/{st["stations"]} {st.get("station", "")}' if 'index' in st else '')
          + (f' | visited: {" ".join(v["name"] for v in st.get("visits", []))}' if st.get('visits') else '')
          + (f' | skipped: {" ".join(st["skipped"])}' if st.get('skipped') else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
