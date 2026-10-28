"""YOLOX object detector on the colour image (OpenCV DNN, CPU, no extra Python packages).

The model is the official YOLOX ONNX export (COCO, 80 classes), downloaded into the image at build time
(Dockerfile, /opt/kelo/models). Display only: navigation does not use the labels.
"""
import os

import cv2
import numpy as np

COCO = ('person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
        'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
        'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
        'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
        'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle',
        'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
        'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
        'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
        'teddy bear', 'hair drier', 'toothbrush')


def letterbox(rgb, size):
    """Image scaled into a size x size square (top left, grey padding) as the model expects, and the
    scale factor."""
    h, w = rgb.shape[:2]
    r = min(size / float(h), size / float(w))
    out = np.full((size, size, 3), 114, np.uint8)
    nh, nw = int(round(h * r)), int(round(w * r))
    out[:nh, :nw] = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return out, r


def decode(raw, size, ratio, score_min, nms=0.45, strides=(8, 16, 32)):
    """YOLOX head output (N x 85: grid offsets, log sizes, objectness, class scores) -> detections
    [(class index, score, x0, y0, x1, y1)] in original image pixels."""
    grids, steps = [], []
    for st in strides:
        n = size // st
        yv, xv = np.meshgrid(np.arange(n), np.arange(n), indexing='ij')
        grids.append(np.stack((xv, yv), 2).reshape(-1, 2))
        steps.append(np.full((n * n, 1), st))
    grid, step = np.concatenate(grids), np.concatenate(steps)
    out = np.asarray(raw, dtype=np.float32)
    xy = (out[:, :2] + grid) * step
    wh = np.exp(np.clip(out[:, 2:4], -10, 10)) * step
    scores = out[:, 4:5] * out[:, 5:]
    cls = scores.argmax(axis=1)
    sc = scores.max(axis=1)
    keep = sc >= score_min
    if not keep.any():
        return []
    xy, wh, cls, sc = xy[keep] / ratio, wh[keep] / ratio, cls[keep], sc[keep]
    boxes = [[float(x - w / 2), float(y - h / 2), float(w), float(h)] for (x, y), (w, h) in zip(xy, wh)]
    idx = np.array(cv2.dnn.NMSBoxes(boxes, sc.tolist(), score_min, nms)).flatten()
    return [(int(cls[i]), float(sc[i]), boxes[i][0], boxes[i][1], boxes[i][0] + boxes[i][2],
             boxes[i][1] + boxes[i][3]) for i in idx]


class Yolox:
    def __init__(self, model, size=416, score_min=0.45, classes=None):
        if not os.path.isfile(model):
            raise FileNotFoundError(model)
        self.net = cv2.dnn.readNetFromONNX(model)
        self.size, self.score_min = int(size), float(score_min)
        self.classes = None if not classes else {COCO.index(c) for c in classes if c in COCO}

    def detect(self, rgb):
        """rgb8 image -> [(label, score, x0, y0, x1, y1)]."""
        img, r = letterbox(rgb[..., ::-1], self.size)  # the model was trained on BGR
        self.net.setInput(cv2.dnn.blobFromImage(img, 1.0, (self.size, self.size), swapRB=False))
        raw = self.net.forward()[0]
        dets = decode(raw, self.size, r, self.score_min)
        h, w = rgb.shape[:2]
        return [(COCO[c], s, max(0.0, x0), max(0.0, y0), min(float(w), x1), min(float(h), y1))
                for c, s, x0, y0, x1, y1 in dets if self.classes is None or c in self.classes]
