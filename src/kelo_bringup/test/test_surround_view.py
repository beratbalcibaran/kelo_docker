import numpy as np
import pytest

from kelo_bringup.surround_view import Camera, Painter, box_faces


def test_camera_projects_the_target_to_the_centre():
    cam = Camera((-3.0, 0.0, 2.0), (2.0, 0.0, 0.0), 960, 720, 60.0)
    uv, z, ok = cam.project([(2.0, 0.0, 0.0), (-4.0, 0.0, 0.0)])
    assert uv[0] == pytest.approx((480.0, 360.0))
    assert ok.tolist() == [True, False]
    # left in the base frame is left on screen, up is up
    (ul, _), (_, vu) = cam.project([(2.0, 1.0, 0.0), (2.0, 0.0, 1.0)])[0]
    assert ul < 480.0 and vu < 360.0


def test_box_faces_point_outwards():
    for pts, n in box_faces(0.0, 0.0, 1.0, 2.0, 0.0, 1.0):
        assert np.dot(pts.mean(axis=0) - (0.0, 0.0, 0.5), n) > 0


def test_painter_hides_back_faces():
    cam = Camera((-3.0, 0.0, 2.0), (0.0, 0.0, 0.0), 320, 240, 60.0)
    p = Painter(cam)
    p.box(0.0, 0.0, 1.0, 1.0, 0.0, 1.0, np.array([200, 200, 200]), np.array([150, 150, 150]))
    # top and the face towards the camera
    assert len(p.items) == 2
    img = np.zeros((240, 320, 3), np.uint8)
    p.draw(img)
    assert img[120, 160].any()
