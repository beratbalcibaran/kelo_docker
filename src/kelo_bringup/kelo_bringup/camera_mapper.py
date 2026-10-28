import os
import sys
import time

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from slam_toolbox.srv import SaveMap
from tf2_ros import Buffer, TransformException, TransformListener

from kelo_bringup.camera_grid import CameraGrid, fit_to, write_map
from kelo_bringup.cloud_filter import quat_to_rot
from kelo_bringup.map_compare import read_pgm


def cloud_xyz(msg):
    if msg.width * msg.height == 0:
        return np.zeros((0, 3))
    data = np.frombuffer(msg.data, dtype='<f4').reshape(-1, msg.point_step // 4)
    return data[:, :3].astype(float)


def _tf(t):
    tr = t.transform.translation
    return quat_to_rot(t.transform.rotation), np.array([tr.x, tr.y, tr.z])


class CameraMapper(Node):
    def __init__(self):
        super().__init__('camera_mapper')
        d = self.declare_parameter
        d('map_topic', '/map')
        d('output_topic', '/map_camera')
        d('points_topic', '/camera/points_filtered')
        d('clearing_topic', '/camera/points_map_clearing')
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        d('camera_frame', 'camera_depth_optical_frame')
        d('min_height', 0.06)
        d('max_height', 1.20)
        d('height_step', 0.10)
        d('rate_hz', 5.0)
        d('publish_period', 2.0)
        d('max_tf_wait', 1.0)
        d('l_hit', 0.85)
        d('l_miss', -0.40)
        d('l_min', -2.0)
        d('l_max', 3.5)
        d('occupied_l', 1.5)
        d('free_l', -0.4)
        d('graze_margin', 0.0)
        g = lambda n: self.get_parameter(n).value
        self.map_frame, self.base, self.cam = g('map_frame'), g('base_frame'), g('camera_frame')
        self.grid = CameraGrid(z_min=g('min_height'), z_max=g('max_height'), dz=g('height_step'),
                               l_hit=g('l_hit'), l_miss=g('l_miss'), l_min=g('l_min'),
                               l_max=g('l_max'), occ_l=g('occupied_l'), free_l=g('free_l'),
                               graze_margin=g('graze_margin'))
        self.period = 1.0 / max(float(g('rate_hz')), 0.1)
        self.max_wait = float(g('max_tf_wait'))
        self.map_info = None
        self.cam_origin = None
        self.pending = {}
        self.last_used = None
        self.frames = 0
        self.buf = Buffer(cache_time=Duration(seconds=30.0))
        self.listener = TransformListener(self.buf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, g('output_topic'), latched)
        self.create_subscription(OccupancyGrid, g('map_topic'), self._map, latched)
        self.create_subscription(PointCloud2, g('points_topic'), lambda m: self._cloud(m, 0), 10)
        self.create_subscription(PointCloud2, g('clearing_topic'), lambda m: self._cloud(m, 1), 10)
        self.create_service(SaveMap, '~/save_map', self._save)
        self.create_timer(0.05, self._process)
        self.create_timer(float(g('publish_period')), self._publish)
        self.get_logger().info(
            f'camera_mapper: heights {g("min_height")}-{g("max_height")} m in {self.grid.nz} bands, '
            f'{1.0 / self.period:.1f} frames/s, following {g("map_topic")} -> {g("output_topic")}')

    def _map(self, msg):
        info = msg.info
        o = info.origin.position
        old = self.map_info
        if (old is None or old.width != info.width or old.height != info.height
                or old.resolution != info.resolution or old.origin.position != o):
            self.grid.align(info.resolution, o.x, o.y, info.width, info.height)
        self.map_info = info

    def _cloud(self, msg, slot):
        key = Time.from_msg(msg.header.stamp).nanoseconds
        self.pending.setdefault(key, [None, None])[slot] = msg

    def _camera_origin(self):
        if self.cam_origin is None:
            try:
                t = self.buf.lookup_transform(self.base, self.cam, Time())
            except TransformException as exc:
                self.get_logger().warn(f'{self.base}<-{self.cam} TF missing: {exc}',
                                       throttle_duration_sec=5.0)
                return None
            self.cam_origin = _tf(t)[1]
        return self.cam_origin

    def _process(self):
        if not self.pending:
            return
        newest = max(self.pending)
        for key in sorted(self.pending):
            pair = self.pending[key]
            if (newest - key) * 1e-9 > self.max_wait:
                del self.pending[key]
                continue
            if None in pair or not self.grid.ready:
                continue
            if self.last_used is not None and (key - self.last_used) * 1e-9 < self.period:
                del self.pending[key]
                continue
            origin = self._camera_origin()
            if origin is None:
                return
            stamp = Time(nanoseconds=key)
            if not self.buf.can_transform(self.map_frame, self.base, stamp):
                return
            rot, tr = _tf(self.buf.lookup_transform(self.map_frame, self.base, stamp))
            del self.pending[key]
            self.last_used = key
            hits = cloud_xyz(pair[0]) @ rot.T + tr
            ends = cloud_xyz(pair[1]) @ rot.T + tr
            self.grid.integrate(rot @ origin + tr, hits, ends)
            self.frames += 1

    def _publish(self):
        if self.map_info is None or not self.grid.ready:
            return
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_info
        msg.data = self.grid.occupancy().ravel().tolist()
        self.pub.publish(msg)
        self.get_logger().info(f'camera layer: {self.frames} frames, '
                               f'{int((np.asarray(msg.data) == 100).sum())} occupied cells',
                               throttle_duration_sec=30.0)

    def _save(self, req, res):
        stem = req.name.data
        if not self.grid.ready:
            self.get_logger().error('save_map: no /map received yet, camera layer is empty')
            res.result = SaveMap.Response.RESULT_NO_MAP_RECEIEVD
            return res
        grid, res_m = self.grid.occupancy(), self.grid.res
        origin, size = self.grid.origin, (self.grid.width, self.grid.height)
        lidar_yaml = stem + '.yaml'
        end = time.time() + 10.0
        while not os.path.isfile(lidar_yaml) and time.time() < end:
            time.sleep(0.2)
        if os.path.isfile(lidar_yaml):
            with open(lidar_yaml, encoding='utf-8') as f:
                meta = yaml.safe_load(f)
            img = meta['image'] if os.path.isabs(meta['image']) else \
                os.path.join(os.path.dirname(lidar_yaml), meta['image'])
            h, w = read_pgm(img).shape
            if abs(float(meta['resolution']) - res_m) > 1e-6:
                self.get_logger().error(f'save_map: resolution {meta["resolution"]} != {res_m}')
                res.result = SaveMap.Response.RESULT_UNDEFINED_FAILURE
                return res
            dst = (float(meta['origin'][0]), float(meta['origin'][1]))
            grid = fit_to(grid, origin, res_m, dst, w, h)
            origin, size, res_m = dst, (w, h), float(meta['resolution'])
        else:
            self.get_logger().warn(f'save_map: {lidar_yaml} not found, saving with the live map geometry')
        write_map(stem + '_camera', grid, res_m, origin)
        self.get_logger().info(f'saved camera layer {stem}_camera.yaml ({size[0]}x{size[1]}, '
                               f'{int((grid == 100).sum())} occupied cells, {self.frames} frames)')
        res.result = SaveMap.Response.RESULT_SUCCESS
        return res


def main():
    rclpy.init(args=sys.argv)
    node = CameraMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
