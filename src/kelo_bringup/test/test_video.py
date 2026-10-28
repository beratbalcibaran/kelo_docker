import cv2
import numpy as np

from kelo_bringup.video_recorder import compose, open_writer, write_clip


def test_compose_scales_to_one_height_with_even_size():
    a = np.zeros((721, 961, 3), np.uint8)
    b = np.zeros((480, 848, 3), np.uint8)
    out = compose([a, b, None], 481, ['a', 'b', 'c'])
    assert out.shape[0] == 480 and out.shape[1] % 2 == 0


def test_writer_produces_a_readable_mp4(tmp_path):
    path = str(tmp_path / 'clip.mp4')
    frames = []
    for k in range(12):
        img = np.full((240, 320, 3), k * 20, np.uint8)
        frames.append(cv2.imencode('.jpg', img)[1].tobytes())
    code = write_clip(path, frames, 10)
    assert code in ('avc1', 'mp4v')
    cap = cv2.VideoCapture(path)
    n = 0
    while cap.read()[0]:
        n += 1
    assert n == 12
    w, _ = open_writer(str(tmp_path / 'x.mp4'), 10, (320, 240))
    w.release()
