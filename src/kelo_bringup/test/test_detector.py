import numpy as np
import pytest

from kelo_bringup.detector import COCO, decode, letterbox


def test_letterbox_keeps_aspect_and_pads():
    img, r = letterbox(np.zeros((480, 848, 3), np.uint8), 416)
    assert img.shape == (416, 416, 3)
    assert r == pytest.approx(416 / 848)
    assert img[-1, -1].tolist() == [114, 114, 114]


def test_decode_one_box():
    size = 416
    n = sum((size // s) ** 2 for s in (8, 16, 32))
    raw = np.zeros((n, 85), np.float32)
    raw[:, 4] = 0.0
    # cell (x 10, y 5) of the stride 8 grid, box 32 x 64 px, class person
    i = 5 * (size // 8) + 10
    raw[i, :4] = [0.5, 0.5, np.log(4.0), np.log(8.0)]
    raw[i, 4] = 0.9
    raw[i, 5 + COCO.index('person')] = 1.0
    dets = decode(raw, size, 0.5, 0.3)
    assert len(dets) == 1
    cls, score, x0, y0, x1, y1 = dets[0]
    assert COCO[cls] == 'person' and score == pytest.approx(0.9)
    # centre (84, 44) px in the model input, scaled back by 1 / 0.5
    assert (x0, y0, x1, y1) == pytest.approx((136.0, 24.0, 200.0, 152.0))
