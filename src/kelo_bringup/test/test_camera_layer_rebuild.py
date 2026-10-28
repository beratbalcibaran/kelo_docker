import math

import pytest

from kelo_bringup.camera_layer_rebuild import _compose, _keep_live_layer, bag_start_pose


def test_compose():
    x, y, t = _compose((1.0, 2.0, math.pi / 2), (1.0, 0.0, math.pi / 2))
    assert (x, y) == pytest.approx((1.0, 3.0))
    assert t == pytest.approx(math.pi)


def _tf(parent, child, x, y, yaw, sec):
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec = sec
    t.transform.translation.x, t.transform.translation.y = x, y
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


def _write_bag(path, msgs):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from tf2_msgs.msg import TFMessage
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    w.create_topic(rosbag2_py.TopicMetadata(id=0, name='/tf', type='tf2_msgs/msg/TFMessage',
                                            serialization_format='cdr'))
    for i, tfs in enumerate(msgs):
        w.write('/tf', serialize_message(TFMessage(transforms=tfs)), (i + 1) * 10**9)
    del w


def test_bag_start_pose(tmp_path):
    bag = tmp_path / 'bag'
    _write_bag(bag, [[_tf('odom', 'base_link', 0.5, 0.0, 0.0, 1)],
                     [_tf('odom', 'base_link', 1.0, 0.0, math.pi / 2, 2)],
                     [_tf('map', 'odom', 2.0, 1.0, math.pi / 2, 3)],
                     [_tf('map', 'odom', 9.0, 9.0, 0.0, 4)]])
    x, y, t = bag_start_pose(str(bag), 'map', 'odom', 'base_link')
    assert (x, y) == pytest.approx((2.0, 2.0))
    assert t == pytest.approx(math.pi)


def test_bag_start_pose_without_map_tf(tmp_path):
    bag = tmp_path / 'bag'
    _write_bag(bag, [[_tf('odom', 'base_link', 0.3, 0.1, 0.2, 1)]])
    assert bag_start_pose(str(bag), 'map', 'odom', 'base_link') == pytest.approx((0.3, 0.1, 0.2))


def test_keep_live_layer(tmp_path):
    stem = str(tmp_path / 'room_1')
    (tmp_path / 'room_1_camera.yaml').write_text('image: room_1_camera.pgm\nresolution: 0.05\n')
    (tmp_path / 'room_1_camera.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    _keep_live_layer(stem)
    assert (tmp_path / 'room_1_camera_live.yaml').read_text().startswith('image: room_1_camera_live.pgm')
    assert (tmp_path / 'room_1_camera_live.pgm').read_bytes() == b'P5\n1 1\n255\n\x00'
