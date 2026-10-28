"""Camera windows for the control panel: KELO Camera with the parking camera views (parking_view) side
by side, colour on the left, depth on the right, at full resolution and scaled to the window; and
KELO Surround below the control panel, with the 3D surround view (surround_view) or the mapping
coverage (coverage_view), switched with the buttons on top.

Double click on the floor in the colour, depth or surround image sends the robot there (Nav2 goal,
navigation mode); right click cancels the goal. The click goes to the view node that drew the image
(/kelo/view_click), which knows where that pixel is on the floor.

    ros2 run kelo_bringup camera_view --ros-args -p geometry:=1470x470+0+785 -p surround_geometry:=760x590-0-0
"""
import threading
import time
import tkinter as tk

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

PANES = (('color', 'Colour camera', '/parking_view/color_image'),
         ('depth', 'Depth camera', '/parking_view/image'))
SURROUND = ('surround', 'Surround', '/surround_view/image')
COVERAGE = ('coverage', 'Coverage', '/coverage_view/image')
STALE_S = 2.0


def ppm(rgb):
    """rgb8 array -> binary PPM bytes, which Tk's PhotoImage reads without extra libraries."""
    h, w = rgb.shape[:2]
    return b'P6 %d %d 255 ' % (w, h) + np.ascontiguousarray(rgb).tobytes()


def to_image(x, y, label_size, shown_size, image_size):
    """Pixel of the source image under a click at (x, y) in a label that shows the image scaled to
    shown_size and centred, or None outside the image."""
    (lw, lh), (sw, sh), (iw, ih) = label_size, shown_size, image_size
    ox, oy = (lw - sw) / 2.0, (lh - sh) / 2.0
    u, v = (x - ox) * iw / float(sw), (y - oy) * ih / float(sh)
    if not (0 <= u < iw and 0 <= v < ih):
        return None
    return u, v


def fit(rgb, width, height):
    """Scale to fit width x height, keeping the aspect ratio."""
    h, w = rgb.shape[:2]
    s = min(width / float(w), height / float(h))
    if s <= 0:
        return rgb
    size = (max(1, int(w * s)), max(1, int(h * s)))
    return cv2.resize(rgb, size, interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view')
        self.declare_parameter('geometry', '1470x470+0+785')
        self.declare_parameter('surround_geometry', '760x590-0-0')
        self.geometry = self.get_parameter('geometry').value
        self.surround_geometry = self.get_parameter('surround_geometry').value
        self.lock = threading.Lock()
        self.latest = {}
        for key, _, topic in PANES + (SURROUND, COVERAGE):
            self.create_subscription(Image, topic, lambda m, k=key: self._image(k, m), 10)
        self.click_pub = self.create_publisher(PointStamped, '/kelo/view_click', 10)

    def click(self, key, u, v, goal):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = key
        msg.point.x, msg.point.y, msg.point.z = float(u), float(v), 1.0 if goal else 0.0
        self.click_pub.publish(msg)

    def _image(self, key, msg):
        if msg.encoding != 'rgb8':
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        with self.lock:
            self.latest[key] = (time.time(), rgb.reshape(msg.height, msg.width, 3))

    def frame(self, key):
        with self.lock:
            item = self.latest.get(key)
        if item is None or time.time() - item[0] > STALE_S:
            return None
        return item[1]


class Window:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('KELO Camera   (double click on the floor: drive there, right click: cancel the goal)')
        self.root.geometry(node.geometry)
        self.root.configure(bg='black')
        self.labels, self.photos, self.shown = {}, {}, {}
        for col, (key, title, _) in enumerate(PANES):
            self.root.columnconfigure(col, weight=1, uniform='pane')
            tk.Label(self.root, text=title, fg='white', bg='black').grid(row=0, column=col, sticky='ew')
            label = tk.Label(self.root, bg='black', fg='grey', text='waiting for images...')
            label.grid(row=1, column=col, sticky='nsew', padx=2, pady=2)
            self.labels[key] = label
            self._bind(label, key)
        self.root.rowconfigure(1, weight=1)
        top = tk.Toplevel(self.root)
        top.title('KELO Surround   (double click: drive there, right click: cancel)')
        top.geometry(node.surround_geometry)
        top.configure(bg='black')
        bar = tk.Frame(top, bg='black')
        bar.pack(fill='x')
        self.lower = tk.StringVar(value=SURROUND[0])
        for key, title, _ in (SURROUND, COVERAGE):
            tk.Radiobutton(bar, text=title, value=key, variable=self.lower, indicatoron=False, width=12,
                           bg='#222', fg='white', selectcolor='#2a6fd6', activebackground='#333',
                           activeforeground='white').pack(side='left', padx=2, pady=2)
        label = tk.Label(top, bg='black', fg='grey', text='waiting for the surround view...')
        label.pack(fill='both', expand=True)
        self.labels['lower'] = label
        self._bind(label, 'lower')
        self.root.after(100, self._tick)

    def _bind(self, label, key):
        label.bind('<Double-Button-1>', lambda e: self._clicked(key, e, True))
        label.bind('<Button-3>', lambda e: self._clicked(key, e, False))

    def _clicked(self, key, event, goal):
        shown = self.shown.get(key)
        if shown is None:
            return
        src, (sw, sh), (iw, ih) = shown
        if src == COVERAGE[0]:
            return
        uv = to_image(event.x, event.y, (event.widget.winfo_width(), event.widget.winfo_height()),
                      (sw, sh), (iw, ih))
        if uv is not None or not goal:
            self.node.click(src, *(uv or (0.0, 0.0)), goal)

    def _tick(self):
        lower = self.lower.get()
        other = COVERAGE[0] if lower == SURROUND[0] else SURROUND[0]
        if self.node.frame(lower) is None and self.node.frame(other) is not None:
            lower = other  # the selected view has no image (e.g. coverage outside mapping)
        for key, src in [(k, k) for k, _, _ in PANES] + [('lower', lower)]:
            label = self.labels[key]
            rgb = self.node.frame(src)
            if rgb is None:
                label.configure(image='', text='no image (colour stream off?)' if key == 'color' else 'no image')
                self.photos.pop(key, None)
                self.shown.pop(key, None)
                continue
            img = fit(rgb, label.winfo_width() - 4, label.winfo_height() - 4)
            photo = tk.PhotoImage(data=ppm(img), format='PPM')
            label.configure(image=photo, text='')
            self.photos[key] = photo
            self.shown[key] = (src, (img.shape[1], img.shape[0]), (rgb.shape[1], rgb.shape[0]))
        self.root.after(100, self._tick)


def main():
    rclpy.init()
    node = CameraView()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    win = Window(node)
    try:
        win.root.mainloop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
