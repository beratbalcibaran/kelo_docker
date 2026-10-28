import base64
import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from kelo_bringup.web_view import Frames, make_handler


class FakeNode:
    password = 'secret'
    allow_drive = True

    def __init__(self):
        self.frames = Frames()
        self.drives, self.clicks = [], []

    def status(self):
        return {'speed': 0.1}

    def set_drive(self, vx, wz):
        self.drives.append((vx, wz))
        return True

    def click(self, *a):
        self.clicks.append(a)

    def cancel(self):
        return True

    def record(self, action):
        return True, action


@pytest.fixture
def web():
    node = FakeNode()
    srv = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(node))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield node, f'http://127.0.0.1:{srv.server_address[1]}'
    srv.shutdown()
    srv.server_close()


def req(url, pw=None, body=None):
    r = urllib.request.Request(url, data=None if body is None else json.dumps(body).encode(),
                               headers={'Content-Type': 'application/json'})
    if pw is not None:
        r.add_header('Authorization', 'Basic ' + base64.b64encode(f'kelo:{pw}'.encode()).decode())
    return urllib.request.urlopen(r, timeout=3)


def test_password_required(web):
    _, url = web
    for pw in (None, 'wrong'):
        with pytest.raises(urllib.error.HTTPError) as e:
            req(url + '/', pw)
        assert e.value.code == 401


def test_page_status_and_commands(web):
    node, url = web
    assert b'KELO' in req(url + '/', 'secret').read()
    assert json.loads(req(url + '/api/status', 'secret').read()) == {'speed': 0.1}
    req(url + '/api/drive', 'secret', {'vx': 0.2, 'wz': -0.1})
    req(url + '/api/click', 'secret', {'view': 'surround', 'u': 10, 'v': 20, 'goal': True})
    assert json.loads(req(url + '/api/record', 'secret', {'action': 'start'}).read())['message'] == 'start'
    assert node.drives == [(0.2, -0.1)] and node.clicks == [('surround', 10.0, 20.0, True)]
    with pytest.raises(urllib.error.HTTPError):
        req(url + '/api/click', 'secret', {'view': 'nope'})


def test_stream_sends_jpeg_frames(web):
    node, url = web
    r = req(url + '/stream/surround.mjpg', 'secret')

    def feed():
        for _ in range(20):
            node.frames.put('surround', b'\xff\xd8JPEG\xff\xd9')
            threading.Event().wait(0.05)
    threading.Thread(target=feed, daemon=True).start()
    data = r.read(200)
    assert b'--frame' in data and b'image/jpeg' in data
    r.close()
