import ast
import math
import random
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Twist, TwistStamped
from marvelmind_ros2_msgs.msg import HedgePositionAngle
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String, UInt32
from tf2_ros import TransformBroadcaster

from kelo_bringup.common import config_path, load_hardware
from kelo_bringup.logic import odom_lin_speed, odom_yaw_rate, quat_from_yaw, wrap
from kelo_bringup.sim import world as W


def _approach(cur, target, max_step):
    return cur + max(-max_step, min(max_step, target - cur))


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')
        d = self.declare_parameter
        d('world', config_path('sim_world.yaml'))
        d('x0', float('nan'))
        d('y0', float('nan'))
        d('yaw0', float('nan'))
        d('seed', 0)
        d('dt', 0.02)
        d('publish_odom_tf', True)
        d('odom_model', '')
        d('mm_offset_x', float('nan'))
        d('mm_offset_y', float('nan'))
        d('lidar_enabled', True)
        d('camera_enabled', True)
        d('marvelmind_enabled', True)
        g = lambda n: self.get_parameter(n).value

        with open(g('world'), encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)
        hw = load_hardware()
        self.hw = hw
        self.rng = random.Random(int(g('seed')))
        self.np_rng = np.random.default_rng(int(g('seed')))
        self.boxes, self.cyls = W.load_world(self.cfg)
        self.dyn_boxes, self.dyn_cyls = [], []
        st = self.cfg.get('start', {})
        pick = lambda p, k, dflt: float(g(p)) if not math.isnan(float(g(p))) else float(st.get(k, dflt))
        self.x, self.y, self.yaw = pick('x0', 'x', 1.0), pick('y0', 'y', 1.0), pick('yaw0', 'yaw', 0.0)
        self.v = [0.0, 0.0, 0.0]
        self.ox = self.oy = self.oyaw = 0.0
        self.dt = float(g('dt'))
        self.publish_tf = bool(g('publish_odom_tf'))
        od = self.cfg.get('odometry', {})
        self.odom_model = g('odom_model') or od.get('model', 'faulty')
        if self.odom_model not in ('faulty', 'ideal'):
            raise ValueError("odom_model must be 'faulty' or 'ideal'")
        self.yaw_k, self.lin_err = od['yaw_k'], od['linear_error']
        rb = self.cfg.get('robot', {})
        self.height = float(rb.get('height', 0.4))
        self.max_acc, self.max_aacc = float(rb.get('max_accel', 1.0)), float(rb.get('max_ang_accel', 2.5))
        self.cmd_timeout = float(rb.get('cmd_timeout_s', 0.5))
        with open(config_path('nav2_params.yaml'), encoding='utf-8') as f:
            nav = yaml.safe_load(f)
        self.footprint = ast.literal_eval(
            nav['local_costmap']['local_costmap']['ros__parameters']['footprint'])
        mm = dict(self.cfg.get('marvelmind', {}))
        off = list(mm.get('offset', [0.0, 0.0]))
        if not math.isnan(float(g('mm_offset_x'))):
            off = [float(g('mm_offset_x')), float(g('mm_offset_y'))]
        self.mm, self.mm_off = mm, off
        self.mm_drop_until = 0.0
        self.mm_ts = 0
        self.enabled = {k: bool(g(f'{k}_enabled')) for k in ('lidar', 'camera', 'marvelmind')}
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = -1e9
        self.contacts = 0
        self.last_contact = None
        self.t0 = self._now()
        self.last_tick = None

        tp, fr = hw['topics'], hw['frames']
        self.frames = fr
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, tp['odom'], 10)
        self.scan_pub = self.create_publisher(LaserScan, tp['scan'], 10)
        self.depth_pub = self.create_publisher(Image, tp['camera_depth'], 10)
        self.info_pub = self.create_publisher(CameraInfo, tp['camera_info'], 10)
        self.color_on = bool(hw['camera'].get('color'))
        if self.color_on:
            self.color_pub = self.create_publisher(Image, tp['camera_color'], 10)
            self.color_info_pub = self.create_publisher(CameraInfo, tp['camera_color_info'], 10)
        self.mm_pub = self.create_publisher(HedgePositionAngle, tp['marvelmind_raw'], 10)
        self.gt_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)
        # people (cylinders named person*) in the base frame: the object labels of the driver views
        self.people_pub = self.create_publisher(PoseArray, '/sim/people', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.contact_pub = self.create_publisher(UInt32, '/sim/contacts', latched)
        self.contact_pub.publish(UInt32(data=0))
        if hw['robot']['cmd_vel_type'] == 'TwistStamped':
            self.create_subscription(TwistStamped, tp['cmd_vel'], lambda m: self._cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, tp['cmd_vel'], self._cmd, 10)
        self.create_subscription(String, '/sim/obstacles', self._obstacles_cb, 10)
        self.create_subscription(String, '/sim/teleport', self._teleport_cb, 10)

        lid, cam = self.cfg['lidar'], self.cfg['camera']
        self.lidar_cfg, self.cam_cfg = lid, cam
        self.intr = W.camera_intrinsics(cam['width'], cam['height'], cam['hfov_deg'], cam['vfov_deg'])
        self.create_timer(self.dt, self._tick)
        self.create_timer(1.0 / float(lid['rate_hz']), self._scan)
        self.create_timer(1.0 / float(cam['rate_hz']), self._camera)
        self.create_timer(1.0 / float(mm['rate_hz']), self._marvelmind)
        self.create_timer(0.2, self._people)
        self.get_logger().info(
            f'sim_robot: start ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f} deg), '
            f'odometry={self.odom_model}, odom TF={"yes" if self.publish_tf else "no"}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_params(self, params):
        for p in params:
            if p.name.endswith('_enabled') and p.name[:-8] in self.enabled:
                self.enabled[p.name[:-8]] = bool(p.value)
                self.get_logger().warn(f"sensor {p.name[:-8]}: {'ON' if p.value else 'CUT'}")
        return SetParametersResult(successful=True)

    def _all_obstacles(self):
        return self.boxes + self.dyn_boxes, self.cyls + self.dyn_cyls

    def _cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_t = self._now()

    def _obstacles_cb(self, msg):
        try:
            req = yaml.safe_load(msg.data) or {}
            if req.get('clear'):
                self.dyn_boxes, self.dyn_cyls = [], []
                self.get_logger().info('sim: added obstacles cleared')
            ob = req.get('add')
            if ob:
                ob = dict(ob)
                parts = [ob]
                if 'ahead' in ob:
                    ahead, lat = float(ob['ahead']), float(ob.get('lateral', 0.0))
                    c, s = math.cos(self.yaw), math.sin(self.yaw)
                    cx, cy = self.x + c * ahead - s * lat, self.y + s * ahead + c * lat
                    if ob.get('type', 'box') == 'box':
                        parts = self._oriented_box(ob, cx, cy, c, s)
                    else:
                        ob['center'] = [cx, cy]
                for part in parts:
                    b, cyl = W.obstacle_from_dict(part, t=self._now())
                    self.dyn_boxes += b
                    self.dyn_cyls += cyl
                self.get_logger().info(f'sim: obstacle added {ob} ({len(parts)} boxes)')
        except Exception as exc:
            self.get_logger().error(f'/sim/obstacles request invalid: {exc}')

    @staticmethod
    def _oriented_box(ob, cx, cy, c, s, tile=0.15):
        """A box turned with the robot heading, as axis-aligned tiles: one bounding box of a turned
        0.3 x 1.2 m box would reach back over the robot. Tiles stick out at most tile/2 * 0.41."""
        sx, sy = [float(v) for v in ob.get('size', [0.4, 0.4])]
        q = min(sx, sy, tile)
        nx, ny = max(1, math.ceil(sx / q - 1e-9)), max(1, math.ceil(sy / q - 1e-9))
        qx, qy = sx / nx, sy / ny
        hx = (abs(c) * qx + abs(s) * qy) / 2
        hy = (abs(s) * qx + abs(c) * qy) / 2
        parts = []
        for i in range(nx):
            for j in range(ny):
                u, v = -sx / 2 + (i + 0.5) * qx, -sy / 2 + (j + 0.5) * qy
                px, py = cx + c * u - s * v, cy + s * u + c * v
                parts.append(dict(ob, xy=[px - hx, py - hy, px + hx, py + hy]))
        ob['center'] = [round(cx, 3), round(cy, 3)]
        return parts

    def _teleport_cb(self, msg):
        """Evaluation only: move the robot to {x, y, yaw} (world, rad). Odometry does not jump, so the
        caller re-seeds the localization."""
        try:
            req = yaml.safe_load(msg.data) or {}
            pose = (float(req['x']), float(req['y']), wrap(float(req.get('yaw', 0.0))))
        except Exception as exc:
            self.get_logger().error(f'/sim/teleport request invalid: {exc}')
            return
        boxes, cyls = self._all_obstacles()
        hit = W.collides(self.footprint, pose, self.height, boxes, cyls)
        if hit:
            self.get_logger().error(f'sim: teleport to {pose} rejected, footprint hits {hit}')
            return
        self.x, self.y, self.yaw = pose
        self.v = [0.0, 0.0, 0.0]
        self.last_contact = None
        self.get_logger().warn(f'sim: teleported to ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f} deg)')

    def _tick(self):
        tnow = self.get_clock().now()
        now = tnow.nanoseconds * 1e-9
        for c in self.dyn_cyls:
            c.update(now)
        fresh = now - self.cmd_t < self.cmd_timeout
        target = self.cmd if fresh else (0.0, 0.0, 0.0)
        dt = self.dt if self.last_tick is None else min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now
        self.v = [_approach(self.v[0], target[0], self.max_acc * dt),
                  _approach(self.v[1], target[1], self.max_acc * dt),
                  _approach(self.v[2], target[2], self.max_aacc * dt)]
        vx, vy, wz = self.v
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        nx, ny = self.x + (c * vx - s * vy) * dt, self.y + (s * vx + c * vy) * dt
        nyaw = wrap(self.yaw + wz * dt)
        boxes, cyls = self._all_obstacles()
        hit = W.collides(self.footprint, (nx, ny, nyaw), self.height, boxes, cyls)
        if hit:
            if hit != self.last_contact:
                self.contacts += 1
                self.contact_pub.publish(UInt32(data=self.contacts))
                self.get_logger().error(f'sim: COLLISION ({hit}) - robot stopped, counter={self.contacts}')
            self.last_contact = hit
            self.v = [0.0, 0.0, 0.0]
            vx = vy = wz = 0.0
        else:
            self.last_contact = None
            self.x, self.y, self.yaw = nx, ny, nyaw

        if self.odom_model == 'faulty':
            speed = math.hypot(vx, vy)
            scale = odom_lin_speed(speed, self.lin_err) / speed if speed > 1e-9 else 1.0
            ovx, ovy, owz = vx * scale, vy * scale, odom_yaw_rate(wz, self.yaw_k)
        else:
            ovx, ovy, owz = vx, vy, wz
        oc, osn = math.cos(self.oyaw), math.sin(self.oyaw)
        self.ox += (oc * ovx - osn * ovy) * dt
        self.oy += (osn * ovx + oc * ovy) * dt
        self.oyaw = wrap(self.oyaw + owz * dt)

        stamp = tnow.to_msg()
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id, od.child_frame_id = self.frames['odom'], self.frames['base']
        od.pose.pose.position.x, od.pose.pose.position.y = self.ox, self.oy
        _, _, od.pose.pose.orientation.z, od.pose.pose.orientation.w = quat_from_yaw(self.oyaw)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = ovx, ovy, owz
        self.odom_pub.publish(od)
        if self.publish_tf:
            t = TransformStamped()
            t.header = od.header
            t.child_frame_id = self.frames['base']
            t.transform.translation.x, t.transform.translation.y = self.ox, self.oy
            t.transform.rotation = od.pose.pose.orientation
            self.tf.sendTransform(t)
        gt = Odometry()
        gt.header.stamp = stamp
        gt.header.frame_id, gt.child_frame_id = 'sim_world', self.frames['base']
        gt.pose.pose.position.x, gt.pose.pose.position.y = self.x, self.y
        _, _, gt.pose.pose.orientation.z, gt.pose.pose.orientation.w = quat_from_yaw(self.yaw)
        gt.twist.twist.linear.x, gt.twist.twist.linear.y, gt.twist.twist.angular.z = vx, vy, wz
        self.gt_pub.publish(gt)

    def _pose(self):
        return (self.x, self.y, self.yaw)

    def _people(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frames['base']
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        for cyl in self._all_obstacles()[1]:
            if cyl.name.startswith('person'):
                dx, dy = cyl.x - self.x, cyl.y - self.y
                p = Pose()
                p.position.x, p.position.y = c * dx + s * dy, -s * dx + c * dy
                p.orientation.w = 1.0
                msg.poses.append(p)
        self.people_pub.publish(msg)

    def _scan(self):
        if not self.enabled['lidar']:
            return
        hw_l = self.hw['lidar']
        n, rmax = int(self.lidar_cfg['beams']), float(self.lidar_cfg['range_max'])
        boxes, cyls = self._all_obstacles()
        r = W.lidar_ranges(self._pose(), self.hw['mounts']['laser'], boxes, cyls,
                           hw_l['angle_min'], hw_l['angle_max'], n, rmax)
        r = r + self.np_rng.normal(0.0, float(self.lidar_cfg['noise_m']), n)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frames['laser']
        msg.angle_min, msg.angle_max = float(hw_l['angle_min']), float(hw_l['angle_max'])
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (n - 1)
        msg.scan_time = 1.0 / float(self.lidar_cfg['rate_hz'])
        msg.range_min, msg.range_max = 0.06, rmax
        msg.ranges = [float(v) for v in r]
        self.scan_pub.publish(msg)

    def _camera(self):
        if not self.enabled['camera']:
            return
        cam = self.cam_cfg
        w, h = int(cam['width']), int(cam['height'])
        boxes, cyls = self._all_obstacles()
        depth = W.depth_image(self._pose(), self.hw['mounts']['camera'], boxes, cyls, w, h, self.intr,
                              float(cam['min_depth']), float(cam['max_depth']))
        noise = self.np_rng.normal(0.0, 1.0, depth.shape) * float(cam['noise_k']) * depth ** 2
        depth = np.where(depth > 0.0, depth + noise, 0.0)
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frames['camera_optical']
        img.height, img.width, img.encoding, img.is_bigendian, img.step = h, w, '16UC1', 0, w * 2
        img.data = np.clip(depth * 1000.0, 0, 65535).astype('<u2').tobytes()
        fx, fy, cx, cy = self.intr
        info = CameraInfo()
        info.header = img.header
        info.width, info.height, info.distortion_model = w, h, 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.depth_pub.publish(img)
        self.info_pub.publish(info)
        if self.color_on:
            rgb = W.color_image(self._pose(), self.hw['mounts']['camera'], boxes, cyls, w, h, self.intr,
                                float(self.hw['camera'].get('color_offset_y', 0.0)))
            cimg = Image()
            cimg.header.stamp = stamp
            cimg.header.frame_id = self.frames['camera_color_optical']
            cimg.height, cimg.width, cimg.encoding, cimg.is_bigendian, cimg.step = h, w, 'rgb8', 0, w * 3
            cimg.data = rgb.tobytes()
            cinfo = CameraInfo()
            cinfo.header = cimg.header
            cinfo.width, cinfo.height, cinfo.distortion_model = w, h, info.distortion_model
            cinfo.d, cinfo.k, cinfo.r, cinfo.p = info.d, info.k, info.r, info.p
            self.color_pub.publish(cimg)
            self.color_info_pub.publish(cinfo)

    def _marvelmind(self):
        if not self.enabled['marvelmind']:
            return
        now = self._now()
        mm = self.mm
        rate = float(mm.get('dropout_per_min', 0.0)) / 60.0 / float(mm['rate_hz'])
        if now >= self.mm_drop_until and self.rng.random() < rate:
            lo, hi = mm.get('dropout_s', [2.0, 6.0])
            self.mm_drop_until = now + self.rng.uniform(lo, hi)
        self.mm_ts += int(1000.0 / float(mm['rate_hz']))
        msg = HedgePositionAngle()
        msg.address = 1
        msg.timestamp_ms = self.mm_ts
        if now < self.mm_drop_until:
            if self.rng.random() < 0.5:
                return
            msg.flags = 1
            self.mm_pub.publish(msg)
            return
        pos, _ = W.sensor_pose(self._pose(), self.hw['mounts']['hedgehog'])
        nz = float(mm['noise_m'])
        x, y = pos[0] + self.rng.gauss(0, nz), pos[1] + self.rng.gauss(0, nz)
        if self.rng.random() < float(mm.get('outlier_prob', 0.0)):
            a = self.rng.uniform(-math.pi, math.pi)
            x += math.cos(a) * float(mm['outlier_m'])
            y += math.sin(a) * float(mm['outlier_m'])
        msg.x_m, msg.y_m, msg.z_m = x + self.mm_off[0], y + self.mm_off[1], float(pos[2])
        msg.flags = 0
        msg.angle = 0.0
        self.mm_pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
