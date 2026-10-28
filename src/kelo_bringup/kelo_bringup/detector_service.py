"""Object detector as a small TCP service, so that the model can run in another process or container
(on the Jetson: TensorRT in the kelo-detector container, docker/detector/) while the objects node stays
in the ROS container.

Protocol (one request after another on a connection, all lengths 4-byte big-endian):
  request:  header length, JSON {"w", "h"}, image length, raw rgb8 pixels (h x w x 3)
  response: body length, JSON {"backend": name, "ms": inference time, "dets": [[label, score, x0, y0, x1, y1], ...]}

The server takes any detector with `detect(rgb) -> [(label, score, x0, y0, x1, y1)]` (detector.Yolox on
the CPU, trt_detector.TrtYolox on the GPU). No ROS imports: the detector container has no ROS.

    python3 -m kelo_bringup.detector_service --backend opencv --model /opt/kelo/models/yolox_tiny.onnx
"""
import argparse
import json
import socket
import socketserver
import struct
import threading
import time

import numpy as np

DEFAULT_PORT = 5810


def _recv(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 1 << 20))
        if not chunk:
            raise ConnectionError('connection closed')
        buf += chunk
    return bytes(buf)


def _send_block(sock, data):
    sock.sendall(struct.pack('!I', len(data)) + data)


def _recv_block(sock, limit=64 << 20):
    (n,) = struct.unpack('!I', _recv(sock, 4))
    if n > limit:
        raise ValueError(f'block of {n} bytes is too large')
    return _recv(sock, n)


class RemoteDetector:
    """Client with the interface of detector.Yolox. A failed call raises; the caller falls back to
    its local detector and tries the service again later."""

    def __init__(self, host='127.0.0.1', port=DEFAULT_PORT, timeout=1.0, classes=None):
        self.addr, self.timeout = (host, int(port)), float(timeout)
        self.classes = set(classes) if classes else None
        self.sock = None
        self.backend = None
        self.lock = threading.Lock()

    def _connect(self):
        sock = socket.create_connection(self.addr, timeout=self.timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = sock

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def detect(self, rgb):
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        h, w = rgb.shape[:2]
        with self.lock:
            try:
                if self.sock is None:
                    self._connect()
                _send_block(self.sock, json.dumps({'w': w, 'h': h}).encode())
                _send_block(self.sock, rgb.tobytes())
                reply = json.loads(_recv_block(self.sock).decode())
            except Exception:
                self.close()
                raise
        if 'error' in reply:
            raise RuntimeError(reply['error'])
        self.backend = reply.get('backend')
        return [(d[0], float(d[1]), *map(float, d[2:6])) for d in reply.get('dets', [])
                if self.classes is None or d[0] in self.classes]


class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        srv = self.server
        while True:
            try:
                head = json.loads(_recv_block(self.request).decode())
                w, h = int(head['w']), int(head['h'])
                data = _recv_block(self.request)
            except (ConnectionError, OSError, ValueError, KeyError):
                return
            try:
                if len(data) != w * h * 3:
                    raise ValueError(f'image is {len(data)} bytes, expected {w}x{h}x3')
                rgb = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
                t0 = time.time()
                with srv.lock:  # one inference at a time on the device
                    dets = srv.detector.detect(rgb)
                body = {'backend': srv.backend, 'ms': round((time.time() - t0) * 1000.0, 1),
                        'dets': [[d[0], round(float(d[1]), 3)] + [round(float(v), 1) for v in d[2:6]] for d in dets]}
            except Exception as exc:  # report, keep the connection
                body = {'error': f'{type(exc).__name__}: {exc}'}
            try:
                _send_block(self.request, json.dumps(body).encode())
            except OSError:
                return


class Server(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, detector, backend, host='127.0.0.1', port=DEFAULT_PORT):
        self.detector, self.backend = detector, backend
        self.lock = threading.Lock()
        super().__init__((host, int(port)), _Handler)


def make_detector(backend, model, size, score_min, engine=None, engine_dir=None):
    if backend == 'opencv':
        from kelo_bringup.detector import Yolox
        return Yolox(model, size=size, score_min=score_min)
    if backend == 'tensorrt':
        from kelo_bringup.trt_detector import TrtYolox
        return TrtYolox(model, engine=engine, engine_dir=engine_dir, score_min=score_min)
    raise ValueError(f'unknown backend: {backend}')


def main(argv=None):
    ap = argparse.ArgumentParser(description='KELO object detector service')
    ap.add_argument('--backend', choices=('opencv', 'tensorrt'), default='opencv')
    ap.add_argument('--model', default='/opt/kelo/models/yolox_tiny.onnx')
    ap.add_argument('--engine', default=None, help='TensorRT engine file (built from the model when missing)')
    ap.add_argument('--engine-dir', default=None, help='folder for the built engine (default: next to the model)')
    ap.add_argument('--size', type=int, default=416, help='input size (opencv backend)')
    ap.add_argument('--score-min', type=float, default=0.3)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=DEFAULT_PORT)
    a = ap.parse_args(argv)
    det = make_detector(a.backend, a.model, a.size, a.score_min, a.engine, a.engine_dir)
    # first inference outside the timing of the clients (CUDA context, kernel selection)
    det.detect(np.zeros((480, 848, 3), np.uint8))
    srv = Server(det, a.backend, a.host, a.port)
    print(f'[detector] {a.backend} {a.model} listening on {a.host}:{a.port}', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
