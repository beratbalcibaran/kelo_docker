ARG L4T_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM ${L4T_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir "numpy<2" "opencv-python-headless==4.10.0.84"

ARG YOLOX_URL=https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.onnx
ARG YOLOX_SHA256=c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063
RUN mkdir -p /opt/kelo/models \
 && curl -fsSL -o /opt/kelo/models/yolox_s.onnx "${YOLOX_URL}" \
 && echo "${YOLOX_SHA256}  /opt/kelo/models/yolox_s.onnx" | sha256sum -c -

COPY src/kelo_bringup/kelo_bringup/__init__.py src/kelo_bringup/kelo_bringup/detector.py \
     src/kelo_bringup/kelo_bringup/detector_service.py src/kelo_bringup/kelo_bringup/trt_detector.py \
     /opt/kelo/kelo_bringup/
ENV PYTHONPATH=/opt/kelo

CMD ["python3", "-m", "kelo_bringup.detector_service", "--backend", "tensorrt", \
     "--model", "/opt/kelo/models/yolox_s.onnx", "--engine-dir", "/data/models", "--host", "172.17.0.1"]
