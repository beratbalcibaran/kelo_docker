import math

import numpy as np
import pytest

from kelo_bringup import scene
from kelo_bringup.camera_view import to_image
from kelo_bringup.surround_view import Camera


def test_surround_click_hits_the_floor_point_that_was_drawn_there():
    cam = Camera((-3.4, 0.0, 2.6), (1.6, 0.0, 0.0), 960, 720, 62.0)
    for p in ((2.0, 0.5), (0.5, -1.0), (4.0, 2.0)):
        uv, _, ok = cam.project([(p[0], p[1], 0.0)])
        assert ok[0]
        hit = scene.ray_to_floor(cam.eye, cam.ray(*uv[0]), 40.0)
        assert hit == pytest.approx(p, abs=1e-6)


def test_camera_pixel_back_to_floor():
    # camera 0.68 m high looking 10 deg down (base <- optical rotation)
    pitch = math.radians(10)
    rot_y = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    rot, trans = rot_y @ optical, np.array([0.42, 0.0, 0.68])
    k = (600.0, 600.0, 424.0, 240.0)
    p = np.array([2.0, 0.3, 0.0])
    c = rot.T @ (p - trans)
    u, v = k[0] * c[0] / c[2] + k[2], k[1] * c[1] / c[2] + k[3]
    hit = scene.ray_to_floor(trans, rot @ scene.pixel_ray(u, v, k))
    assert hit == pytest.approx((2.0, 0.3), abs=1e-6)
    # above the horizon: no floor
    assert scene.ray_to_floor(trans, rot @ scene.pixel_ray(424.0, 0.0, k)) is None


def test_label_click_to_image_pixel():
    # 848x480 shown at 424x240 in a 500x300 label: centred with 38 / 30 px borders
    assert to_image(38 + 212, 30 + 120, (500, 300), (424, 240), (848, 480)) == pytest.approx((424.0, 240.0))
    assert to_image(10, 10, (500, 300), (424, 240), (848, 480)) is None
