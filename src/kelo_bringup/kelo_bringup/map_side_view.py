"""RViz helper: shows the camera layer next to the LiDAR map instead of on top of it.

Republishes /map_camera on /map_camera_side with its origin moved to the right of the map by the map
width plus a gap, and publishes the robot pose shifted the same way and a title above each map
(/map_side_view/markers). Display only: nothing in the stack subscribes to these topics.
The camera images are shown in their own window (camera_view).
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class MapSideView(Node):
    def __init__(self):
        super().__init__('map_side_view')
        d = self.declare_parameter
        d('camera_topic', '/map_camera')
        d('output_topic', '/map_camera_side')
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        d('gap', 1.0)
        g = lambda n: self.get_parameter(n).value
        self.map_frame, self.base_frame, self.gap = g('map_frame'), g('base_frame'), float(g('gap'))
        self.info = None
        self.shift = 0.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, g('output_topic'), latched)
        self.markers = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_subscription(OccupancyGrid, g('camera_topic'), self._camera, latched)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_timer(0.2, self._publish_markers)

    def _camera(self, msg):
        info = msg.info
        self.shift = info.width * info.resolution + self.gap
        msg.info.origin.position.x += self.shift
        self.info = info
        self.pub.publish(msg)

    def _marker(self, mid, mtype, frame):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'map_side_view', mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(self):
        if self.info is None:
            return
        info = self.info
        # self.info already holds the shifted origin; the LiDAR map starts at x0
        x0 = info.origin.position.x - self.shift
        y0 = info.origin.position.y
        w, h = info.width * info.resolution, info.height * info.resolution
        out = MarkerArray()
        titles = (('LiDAR map', x0), ('Camera layer', x0 + self.shift))
        for mid, (text, x) in enumerate(titles):
            m = self._marker(mid, Marker.TEXT_VIEW_FACING, self.map_frame)
            m.pose.position.x, m.pose.position.y = x + w / 2.0, y0 + h + 0.4
            m.scale.z = 0.35
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = text
            out.markers.append(m)
        try:
            tf = self.buf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            tf = None
        if tf is not None:
            t, q = tf.transform.translation, tf.transform.rotation
            m = self._marker(len(titles), Marker.ARROW, self.map_frame)
            m.pose.position.x, m.pose.position.y = t.x + self.shift, t.y
            m.pose.orientation = q
            m.scale.x, m.scale.y, m.scale.z = 0.5, 0.12, 0.12
            m.color.g, m.color.b, m.color.a = 0.8, 1.0, 1.0
            out.markers.append(m)
        self.markers.publish(out)


def main():
    rclpy.init()
    node = MapSideView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
