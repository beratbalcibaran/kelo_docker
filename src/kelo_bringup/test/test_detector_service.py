import threading

import numpy as np
import pytest

from kelo_bringup.detector_service import RemoteDetector, Server


class Fake:
    def __init__(self):
        self.shapes = []

    def detect(self, rgb):
        self.shapes.append(rgb.shape)
        if rgb[0, 0, 0] == 255:
            raise RuntimeError('bad image')
        return [('person', 0.9, 10.0, 20.0, 110.0, 220.0), ('chair', 0.5, 0, 0, 5, 5)]


@pytest.fixture
def server():
    fake = Fake()
    srv = Server(fake, 'fake', '127.0.0.1', 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv, fake
    srv.shutdown()
    srv.server_close()


def test_remote_detection_round_trip(server):
    srv, fake = server
    cli = RemoteDetector('127.0.0.1', srv.server_address[1], classes=['person'])
    img = np.zeros((48, 64, 3), np.uint8)
    assert cli.detect(img) == [('person', 0.9, 10.0, 20.0, 110.0, 220.0)]
    assert cli.detect(img)  # the same connection again
    assert fake.shapes == [(48, 64, 3)] * 2 and cli.backend == 'fake'


def test_server_error_is_reported_and_connection_survives(server):
    srv, _ = server
    cli = RemoteDetector('127.0.0.1', srv.server_address[1])
    bad = np.full((4, 4, 3), 255, np.uint8)
    with pytest.raises(RuntimeError, match='bad image'):
        cli.detect(bad)
    assert len(cli.detect(np.zeros((4, 4, 3), np.uint8))) == 2


def test_no_service_raises_quickly():
    cli = RemoteDetector('127.0.0.1', 1, timeout=0.5)
    with pytest.raises(OSError):
        cli.detect(np.zeros((4, 4, 3), np.uint8))
