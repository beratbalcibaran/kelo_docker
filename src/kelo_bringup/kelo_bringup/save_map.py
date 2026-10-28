import os
import sys
import time

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String

from kelo_bringup.common import camera_layer_enabled, load_hardware, maps_dir


def call(node, client, request, timeout=60.0):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f'service missing: {client.srv_name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        raise RuntimeError(f'service did not respond: {client.srv_name}')
    return future.result()


def save_camera_layer(node, stem):
    if not camera_layer_enabled(load_hardware()):
        return False
    client = node.create_client(SaveMap, '/camera_mapper/save_map')
    if not client.wait_for_service(timeout_sec=3.0):
        print('WARNING: camera layer enabled but camera_mapper is not running: only the lidar map was saved')
        return False
    res = call(node, client, SaveMap.Request(name=String(data=stem)))
    if res.result != SaveMap.Response.RESULT_SUCCESS:
        raise RuntimeError(f'camera layer save failed (code {res.result})')
    return True


def main():
    if len(sys.argv) < 2:
        sys.exit('Usage: ros2 run kelo_bringup save_map <name>')
    stem = os.path.join(maps_dir(), f"{sys.argv[1]}_{time.strftime('%Y%m%d_%H%M%S')}")
    rclpy.init()
    node = Node('save_map')
    try:
        res = call(node, node.create_client(SaveMap, '/slam_toolbox/save_map'),
                   SaveMap.Request(name=String(data=stem)))
        if res.result != SaveMap.Response.RESULT_SUCCESS:
            raise RuntimeError(f'save_map failed (code {res.result})')
        res = call(node, node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map'),
                   SerializePoseGraph.Request(filename=stem))
        if res.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            raise RuntimeError(f'serialize failed (code {res.result})')
        camera = save_camera_layer(node, stem)
        link = os.path.join(maps_dir(), 'current.yaml')
        if os.path.islink(link) or os.path.exists(link):
            os.remove(link)
        os.symlink(os.path.basename(stem) + '.yaml', link)
        print(f'Saved: {stem}.yaml (+pgm, posegraph, data)\ncurrent.yaml -> {os.path.basename(stem)}.yaml')
        if camera:
            print(f'Saved: {stem}_camera.yaml (+pgm, camera layer)')
    except RuntimeError as exc:
        print(f'ERROR: {exc}')
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
