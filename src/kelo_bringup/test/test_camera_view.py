import numpy as np

from kelo_bringup.camera_view import fit, ppm


def test_ppm_header_and_size():
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    data = ppm(rgb)
    assert data.startswith(b'P6 3 2 255 ')
    assert len(data) == len(b'P6 3 2 255 ') + 2 * 3 * 3


def test_fit_keeps_aspect_ratio():
    rgb = np.zeros((480, 848, 3), dtype=np.uint8)
    assert fit(rgb, 424, 1000).shape[:2] == (240, 424)
    assert fit(rgb, 2000, 240).shape[:2] == (240, 424)
