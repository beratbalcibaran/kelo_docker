"""YOLOX on the Jetson GPU with TensorRT (FP16), for the detector service in the kelo-detector container
(docker/detector/). Same pre- and post-processing as detector.Yolox, so the two give the same boxes.

The engine is built from the ONNX model with trtexec on the first start (a few minutes) and cached
next to it: an engine only runs on the TensorRT version and GPU it was built for. GPU memory is
handled through the CUDA runtime with ctypes, so no extra Python packages (pycuda, cuda-python)
are needed. Not used in the ROS container.
"""
import ctypes
import ctypes.util
import os
import subprocess

import numpy as np

from kelo_bringup.detector import COCO, decode, letterbox

H2D, D2H = 1, 2
TRTEXEC = ('/usr/src/tensorrt/bin/trtexec', 'trtexec')


def _cudart():
    for name in ('libcudart.so', 'libcudart.so.12', 'libcudart.so.11.0',
                 '/usr/local/cuda/lib64/libcudart.so', ctypes.util.find_library('cudart')):
        if not name:
            continue
        try:
            return ctypes.CDLL(name)
        except OSError:
            continue
    raise OSError('CUDA runtime (libcudart) not found')


class _Cuda:
    def __init__(self):
        self.lib = _cudart()
        vp, sz = ctypes.c_void_p, ctypes.c_size_t
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(vp), sz]
        self.lib.cudaMemcpyAsync.argtypes = [vp, vp, sz, ctypes.c_int, vp]
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(vp)]
        self.lib.cudaStreamSynchronize.argtypes = [vp]
        self.stream = ctypes.c_void_p()
        self._check(self.lib.cudaStreamCreate(ctypes.byref(self.stream)), 'cudaStreamCreate')

    @staticmethod
    def _check(code, what):
        if code != 0:
            raise RuntimeError(f'{what} failed (CUDA error {code})')

    def malloc(self, nbytes):
        ptr = ctypes.c_void_p()
        self._check(self.lib.cudaMalloc(ctypes.byref(ptr), ctypes.c_size_t(nbytes)), 'cudaMalloc')
        return ptr

    def to_device(self, dst, array):
        self._check(self.lib.cudaMemcpyAsync(dst, array.ctypes.data_as(ctypes.c_void_p),
                                             ctypes.c_size_t(array.nbytes), H2D, self.stream), 'copy to GPU')

    def to_host(self, array, src):
        self._check(self.lib.cudaMemcpyAsync(array.ctypes.data_as(ctypes.c_void_p), src,
                                             ctypes.c_size_t(array.nbytes), D2H, self.stream), 'copy from GPU')

    def sync(self):
        self._check(self.lib.cudaStreamSynchronize(self.stream), 'cudaStreamSynchronize')


def build_engine(onnx, engine, fp16=True):
    """ONNX -> TensorRT engine file with trtexec."""
    exe = next((e for e in TRTEXEC if os.path.isfile(e)), 'trtexec')
    tmp = engine + '.part'
    cmd = [exe, f'--onnx={onnx}', f'--saveEngine={tmp}'] + (['--fp16'] if fp16 else [])
    print(f'[detector] building the TensorRT engine (first start, a few minutes): {" ".join(cmd)}', flush=True)
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    os.replace(tmp, engine)


class TrtYolox:
    def __init__(self, model, engine=None, engine_dir=None, score_min=0.3, classes=None, fp16=True):
        import tensorrt as trt
        if engine is None:
            # named after the TensorRT version: after a JetPack update a new engine is built
            stem = os.path.splitext(os.path.basename(model))[0] + f'_trt{trt.__version__}_{"fp16" if fp16 else "fp32"}.engine'
            folder = engine_dir or os.path.dirname(model)
            os.makedirs(folder, exist_ok=True)
            engine = os.path.join(folder, stem)
        if not os.path.isfile(engine):
            build_engine(model, engine, fp16)
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f'could not load the engine {engine} (built for another TensorRT? delete it)')
        self.context = self.engine.create_execution_context()
        self.cuda = _Cuda()
        self.score_min = float(score_min)
        self.classes = None if not classes else {COCO.index(c) for c in classes if c in COCO}
        self.host, self.dev = {}, {}
        self.input = self.output = None
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            self.host[name] = np.zeros(shape, dtype)
            self.dev[name] = self.cuda.malloc(self.host[name].nbytes)
            self.context.set_tensor_address(name, self.dev[name].value)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input = name
            else:
                self.output = name
        self.size = int(self.host[self.input].shape[-1])

    def detect(self, rgb):
        """rgb8 image -> [(label, score, x0, y0, x1, y1)]."""
        img, r = letterbox(rgb[..., ::-1], self.size)  # the model was trained on BGR
        blob = np.ascontiguousarray(img.transpose(2, 0, 1)[None], dtype=self.host[self.input].dtype)
        self.cuda.to_device(self.dev[self.input], blob)
        if not self.context.execute_async_v3(self.cuda.stream.value):
            raise RuntimeError('TensorRT inference failed')
        out = self.host[self.output]
        self.cuda.to_host(out, self.dev[self.output])
        self.cuda.sync()
        dets = decode(out.reshape(-1, out.shape[-1]), self.size, r, self.score_min)
        h, w = rgb.shape[:2]
        return [(COCO[c], s, max(0.0, x0), max(0.0, y0), min(float(w), x1), min(float(h), y1))
                for c, s, x0, y0, x1, y1 in dets if self.classes is None or c in self.classes]
