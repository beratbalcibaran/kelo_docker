DESCRIPTION = """Rebuild the camera layer of a finished map from its mapping bag.

usage: ros2 run kelo_bringup camera_layer_rebuild <bag> [map.yaml] [--localizer auto|slam|amcl]
                                                     [--rate 1.0] [--domain 99]

During mapping, camera cells are placed with the slam_toolbox pose of that moment, so a later loop
closure corrects the lidar map but not the camera cells already accumulated. This tool replays the
mapping bag on the finished lidar map (default /data/maps/current.yaml) and writes
<map>_camera.yaml/.pgm again, with poses that agree with the final map. The robot is localized by
slam_toolbox on the saved pose graph (<map>.posegraph, same scan matcher as the mapping run), or by
AMCL on the PGM when there is no pose graph (--localizer amcl forces it). The previous camera layer is
kept as <map>_camera_live.yaml/.pgm.

The replay runs on its own ROS domain (localhost only), so it can run next to a live stack.
"""
import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

from kelo_bringup.common import load_hardware, maps_dir


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    return (ax + math.cos(at) * bx - math.sin(at) * by, ay + math.sin(at) * bx + math.cos(at) * by,
            math.atan2(math.sin(at + bt), math.cos(at + bt)))


def bag_start_pose(bag, map_frame, odom_frame, base_frame):
    """Robot pose in the map when slam_toolbox started, from the /tf recorded in the mapping bag:
    first map->odom composed with the odom->base just before it. (0, 0, 0) when the bag has no /tf."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    if '/tf' not in {t.name for t in reader.get_all_topics_and_types()}:
        return None
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf']))
    odom_base = None
    while reader.has_next():
        _, data, _ = reader.read_next()
        for t in deserialize_message(data, TFMessage).transforms:
            pose = (t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation))
            parent, child = t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/')
            if parent == odom_frame and child == base_frame:
                odom_base = pose
            elif parent == map_frame and child == odom_frame and odom_base is not None:
                return _compose(pose, odom_base)
    return odom_base


def _stem(map_yaml):
    real = os.path.realpath(map_yaml)
    return real[:-5] if real.endswith('.yaml') else real


def _keep_live_layer(stem):
    for ext in ('.yaml', '.pgm'):
        src = f'{stem}_camera{ext}'
        if os.path.isfile(src):
            shutil.copy2(src, f'{stem}_camera_live{ext}')
    yaml_path = f'{stem}_camera_live.yaml'
    if os.path.isfile(yaml_path):
        with open(yaml_path, encoding='utf-8') as f:
            text = f.read()
        base = os.path.basename(stem)
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(text.replace(f'image: {base}_camera.pgm', f'image: {base}_camera_live.pgm'))


def _save(stem):
    import rclpy
    from rclpy.node import Node
    from slam_toolbox.srv import SaveMap
    from std_msgs.msg import String

    rclpy.init()
    node = Node('camera_layer_rebuild')
    try:
        client = node.create_client(SaveMap, '/camera_mapper/save_map')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('camera_mapper is not running')
        future = client.call_async(SaveMap.Request(name=String(data=stem)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
        res = future.result()
        if res is None or res.result != SaveMap.Response.RESULT_SUCCESS:
            raise RuntimeError(f'camera layer save failed ({None if res is None else res.result})')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('map', nargs='?', default=None)
    ap.add_argument('--localizer', choices=('auto', 'slam', 'amcl'), default='auto')
    ap.add_argument('--rate', type=float, default=1.0, help='bag playback rate')
    ap.add_argument('--domain', type=int, default=int(os.environ.get('KELO_REBUILD_DOMAIN', '99')))
    a = ap.parse_args(argv)

    bag = os.path.abspath(a.bag)
    map_yaml = os.path.abspath(a.map or os.path.join(maps_dir(), 'current.yaml'))
    if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        sys.exit(f'ERROR: {bag} is not a bag directory (metadata.yaml missing)')
    if not os.path.isfile(map_yaml):
        sys.exit(f'ERROR: map not found: {map_yaml}')
    stem = _stem(map_yaml)
    fr = load_hardware()['frames']
    pose = bag_start_pose(bag, fr['map'], fr['odom'], fr['base'])
    if pose is None:
        print('WARNING: no /tf in the bag, assuming the mapping started at the map origin')
        pose = (0.0, 0.0, 0.0)

    # isolated replay: sim time and the replayed /map must not reach a live stack
    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('CYCLONEDDS_URI', None)
    cmd = ['ros2', 'launch', 'kelo_bringup', 'bringup.launch.py', 'source:=replay', 'mode:=camera_layer',
           f'bag:={bag}', f'map:={map_yaml}', f'bag_rate:={a.rate}', 'record:=false',
           f'rebuild_localizer:={a.localizer}',
           f'initial_pose:={pose[0]:.4f} {pose[1]:.4f} {math.degrees(pose[2]):.3f}']
    print(f'[rebuild] {os.path.basename(bag)} on {os.path.basename(stem)}.yaml, start pose '
          f'{pose[0]:.3f} {pose[1]:.3f} {math.degrees(pose[2]):.1f} deg, domain {a.domain}')
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    finished = threading.Event()
    log_path = f'{stem}_camera_rebuild.log'
    log_file = open(log_path, 'w', encoding='utf-8')

    def pump():
        # keep draining the launch output until it exits, so a full pipe never blocks the nodes
        for line in proc.stdout:
            log_file.write(line)
            if 'camera layer:' in line and 'frames' in line:
                print(f"[rebuild] {line.strip().split('camera layer:')[-1].strip()}", flush=True)
            elif 'REPLAY_FINISHED' in line:
                finished.set()
            elif '[ERROR]' in line:
                print(line.rstrip(), flush=True)
        finished.set()

    threading.Thread(target=pump, daemon=True).start()
    try:
        finished.wait()
        if proc.poll() is not None:
            sys.exit('ERROR: the replay stopped before the end of the bag')
        time.sleep(3.0)
        _keep_live_layer(stem)
        _save(stem)
        print(f'Saved: {stem}_camera.yaml (+pgm, rebuilt)')
        if os.path.isfile(f'{stem}_camera_live.yaml'):
            print(f'Previous layer kept as {stem}_camera_live.yaml')
    except RuntimeError as exc:
        sys.exit(f'ERROR: {exc}')
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        log_file.close()
        print(f'Full log: {log_path}')


if __name__ == '__main__':
    main()
