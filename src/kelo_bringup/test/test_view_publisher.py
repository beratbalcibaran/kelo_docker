import time

import cv2
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from kelo_bringup.view_publisher import ViewPublisher


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node('view_publisher_test')
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def spin_until(node, cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end and not cond():
        rclpy.spin_once(node, timeout_sec=0.05)
    return cond()


def test_jpeg_only_while_subscribed(node):
    pub = ViewPublisher(node, '/view_publisher_test/image')
    got = {'jpeg': [], 'raw': []}
    node.create_subscription(CompressedImage, '/view_publisher_test/image/compressed',
                             lambda m: got['jpeg'].append(m), 2)
    assert spin_until(node, lambda: pub.jpeg.get_subscription_count() == 1)
    assert pub.raw.get_subscription_count() == 0 and pub.get_subscription_count() == 1

    rgb = np.zeros((48, 64, 3), np.uint8)
    rgb[:, :32] = (255, 0, 0)
    header = Header()
    header.frame_id = 'base_link'
    pub.publish(header, rgb)
    assert spin_until(node, lambda: got['jpeg'])
    msg = got['jpeg'][0]
    assert msg.format == 'jpeg' and msg.header.frame_id == 'base_link'
    bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    assert bgr.shape == (48, 64, 3)
    assert bgr[24, 10, 2] > 200 and bgr[24, 10, 0] < 60

    node.create_subscription(Image, '/view_publisher_test/image', lambda m: got['raw'].append(m), 2)
    assert spin_until(node, lambda: pub.raw.get_subscription_count() == 1)
    pub.publish(header, rgb)
    assert spin_until(node, lambda: got['raw'])
    raw = got['raw'][0]
    assert (raw.encoding, raw.width, raw.height, raw.step) == ('rgb8', 64, 48, 192)
