"""Driving video: the surround view and the colour parking camera side by side as an mp4 in
/data/videos (demos, looking at what happened).

    ros2 service call /video_recorder/start std_srvs/srv/Trigger     # start a recording
    ros2 service call /video_recorder/stop std_srvs/srv/Trigger      # stop and close the file
    ros2 service call /video_recorder/save_clip std_srvs/srv/Trigger # dashcam: save the last seconds

The control panel (Record button) and the web view call these. With `dashcam_s` > 0 (hardware.yaml
recording.dashcam_s) the last seconds are kept in memory all the time (JPEG frames), and save_clip
writes them to a file, like a car's dashcam after an incident. Without a recording or the dashcam the
node does not subscribe to the views, so they are not rendered for it.

Frames are written at a fixed rate (`fps`): the newest image of each view is used, a view without
images is a black panel. H.264 (plays in browsers and on phones) when OpenCV's FFmpeg has an encoder
for it, else MPEG-4. A recording stops by itself after `max_minutes` or when less than `min_free_gb`
is left on the disk. State on /video_recorder/state.
"""
import collections
import os
import shutil
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from kelo_bringup.common import data_dir, load_hardware

VIEWS = ('/surround_view/image', '/parking_view/color_image')
FOURCCS = ('avc1', 'mp4v')


def compose(frames, height, labels=None):
    """Images (rgb8 arrays or None) scaled to `height` and put side by side; a missing one is a black
    4:3 panel. Width and height are even (video encoders need it)."""
    panels = []
    for k, f in enumerate(frames):
        if f is None:
            p = np.zeros((height, height * 4 // 3, 3), np.uint8)
            if labels:
                cv2.putText(p, f'{labels[k]}: no image', (16, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (120, 120, 120), 1, cv2.LINE_AA)
        else:
            w = max(2, int(round(f.shape[1] * height / float(f.shape[0]))))
            p = cv2.resize(f, (w, height), interpolation=cv2.INTER_AREA)
        panels.append(p)
    out = np.hstack(panels)
    return np.ascontiguousarray(out[:out.shape[0] // 2 * 2, :out.shape[1] // 2 * 2])


def open_writer(path, fps, size):
    """cv2.VideoWriter for the first codec that works, and its fourcc."""
    for code in FOURCCS:
        w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*code), float(fps), size)
        if w.isOpened():
            return w, code
        w.release()
    raise RuntimeError(f'no video encoder for {path}')


def write_clip(path, frames, fps):
    """JPEG frames (list of bytes) -> mp4. Returns the fourcc used."""
    first = cv2.imdecode(np.frombuffer(frames[0], np.uint8), cv2.IMREAD_COLOR)
    w, code = open_writer(path, fps, (first.shape[1], first.shape[0]))
    try:
        for buf in frames:
            w.write(cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR))
    finally:
        w.release()
    return code


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        rec = load_hardware().get('recording', {}) or {}
        d = self.declare_parameter
        d('fps', float(rec.get('fps', 10.0)))
        d('height', int(rec.get('height', 480)))
        d('dashcam_s', float(rec.get('dashcam_s', 0.0)))
        d('max_minutes', float(rec.get('max_minutes', 30.0)))
        d('min_free_gb', 1.0)
        d('out_dir', os.path.join(data_dir(), 'videos'))
        d('views', list(VIEWS))
        g = lambda n: self.get_parameter(n).value
        self.fps, self.height = float(g('fps')), int(g('height'))
        self.dashcam_s, self.max_s = float(g('dashcam_s')), 60.0 * float(g('max_minutes'))
        self.min_free = float(g('min_free_gb')) * 1e9
        self.out_dir = g('out_dir')
        self.views = list(g('views'))
        self.labels = [v.strip('/').split('/')[0] for v in self.views]
        self.latest = {}
        self.subs = []
        self.lock = threading.Lock()
        self.writer = self.pending = None
        self.path, self.size, self.started, self.frames = None, None, 0.0, 0
        self.ring = collections.deque(maxlen=max(1, int(self.dashcam_s * self.fps)))
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.create_service(Trigger, '~/start', self._start)
        self.create_service(Trigger, '~/stop', self._stop)
        self.create_service(Trigger, '~/save_clip', self._save_clip)
        self.create_timer(1.0 / self.fps, self._tick)
        self.create_timer(1.0, self._state)
        self._subscribe(self.dashcam_s > 0.0)
        self.get_logger().info(f'video_recorder: {self.out_dir}, {self.fps:.0f} fps'
                               + (f', dashcam {self.dashcam_s:.0f} s' if self.dashcam_s > 0 else ''))

    def _subscribe(self, on):
        if on and not self.subs:
            self.subs = [self.create_subscription(Image, t, lambda m, t=t: self._image(t, m), 2) for t in self.views]
        elif not on and self.subs:
            for s in self.subs:
                self.destroy_subscription(s)
            self.subs = []
            self.latest.clear()

    def _image(self, topic, msg):
        if msg.encoding != 'rgb8':
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        self.latest[topic] = (time.time(), img.reshape(msg.height, msg.width, 3))

    def _frame(self):
        now = time.time()
        frames = [self.latest[t][1] if t in self.latest and now - self.latest[t][0] < 2.0 else None
                  for t in self.views]
        if all(f is None for f in frames):
            return None
        out = compose(frames, self.height, self.labels)
        stamp = time.strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(out, stamp, (12, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, stamp, (12, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        return cv2.cvtColor(out, cv2.COLOR_RGB2BGR)

    def _tick(self):
        if self.writer is None and self.pending is None and self.dashcam_s <= 0.0:
            return
        frame = self._frame()
        if frame is None:
            return
        if self.pending is not None:
            self._open(self.pending, frame)
        if self.dashcam_s > 0.0:
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self.ring.append(buf.tobytes())
        with self.lock:
            if self.writer is None:
                return
            if (frame.shape[1], frame.shape[0]) != self.size:
                frame = cv2.resize(frame, self.size)
            self.writer.write(frame)
            self.frames += 1
            reason = None
            if time.time() - self.started > self.max_s:
                reason = 'time limit'
            elif self.frames % int(10 * self.fps) == 0 and shutil.disk_usage(self.out_dir).free < self.min_free:
                reason = 'disk almost full'
        if reason:
            self.get_logger().warn(f'recording stopped: {reason}')
            self._close()

    def _start(self, _req, res):
        with self.lock:
            if self.writer is not None:
                res.success, res.message = True, f'already recording: {self.path}'
                return res
        self._subscribe(True)
        os.makedirs(self.out_dir, exist_ok=True)
        if shutil.disk_usage(self.out_dir).free < self.min_free:
            res.success, res.message = False, 'not enough free disk space'
            return res
        # the file is opened with the first frame: its size comes from the views
        path = os.path.join(self.out_dir, time.strftime('kelo_%Y%m%d_%H%M%S.mp4'))
        self.pending, self.started = path, time.time()
        res.success, res.message = True, path
        return res

    def _open(self, path, frame):
        self.pending = None
        size = (frame.shape[1], frame.shape[0])
        try:
            writer, code = open_writer(path, self.fps, size)
        except RuntimeError as exc:
            self.get_logger().error(f'recording not started: {exc}')
            self._subscribe(self.dashcam_s > 0.0)
            return
        with self.lock:
            self.writer, self.path, self.size = writer, path, size
            self.started, self.frames = time.time(), 0
        self.get_logger().info(f'recording {path} ({code}, {size[0]}x{size[1]})')

    def _close(self):
        self.pending = None
        with self.lock:
            writer, path, frames = self.writer, self.path, self.frames
            self.writer = None
        if writer is None:
            return None
        writer.release()
        self._subscribe(self.dashcam_s > 0.0)
        self.get_logger().info(f'recording saved: {path} ({frames / self.fps:.0f} s)')
        return path

    def _stop(self, _req, res):
        path = self._close()
        res.success, res.message = path is not None, path or 'not recording'
        return res

    def _save_clip(self, _req, res):
        if self.dashcam_s <= 0.0:
            res.success, res.message = False, 'dashcam off (recording.dashcam_s in hardware.yaml)'
            return res
        frames = list(self.ring)
        if not frames:
            res.success, res.message = False, 'no frames yet'
            return res
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, time.strftime('kelo_clip_%Y%m%d_%H%M%S.mp4'))
        threading.Thread(target=write_clip, args=(path, frames, self.fps), daemon=True).start()
        res.success, res.message = True, f'{path} ({len(frames) / self.fps:.0f} s)'
        self.get_logger().info(f'dashcam clip: {res.message}')
        return res

    def _state(self):
        with self.lock:
            text = (f'recording {os.path.basename(self.path)} {time.time() - self.started:.0f}s'
                    if self.writer is not None else 'starting' if self.pending else 'idle')
        if self.dashcam_s > 0.0:
            text += f' | dashcam {len(self.ring) / self.fps:.0f}s'
        self.state_pub.publish(String(data=text))

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main():
    rclpy.init()
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()  # closes a running recording (the mp4 needs its index written)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
