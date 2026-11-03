#!/usr/bin/env python3
"""TensorRT backend for carton OBB inference without ROS dependencies."""

from __future__ import annotations

import ctypes
import math
import threading
import time
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
import tensorrt as trt


DEFAULT_TRT_ENGINE_PATH = (
    Path(__file__).resolve().parent / "models" / "carton_obb_fp16.engine"
)

RotatedDetection = Tuple[
    Tuple[Tuple[float, float], Tuple[float, float], float],
    float,
]


class _CudaRuntime:
    MEMCPY_HOST_TO_DEVICE = 1
    MEMCPY_DEVICE_TO_HOST = 2

    def __init__(self):
        self.lib = ctypes.CDLL("libcudart.so")
        self.lib.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaMemcpy.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
        ]
        self.lib.cudaDeviceSynchronize.argtypes = []

    @staticmethod
    def _check(code, operation):
        if code != 0:
            raise RuntimeError(f"{operation} failed with CUDA error {code}")

    def malloc(self, nbytes):
        pointer = ctypes.c_void_p()
        self._check(
            self.lib.cudaMalloc(ctypes.byref(pointer), nbytes),
            "cudaMalloc",
        )
        return pointer

    def free(self, pointer):
        self._check(self.lib.cudaFree(pointer), "cudaFree")

    def copy_to_device(self, device, host):
        self._check(
            self.lib.cudaMemcpy(
                device,
                ctypes.c_void_p(host.ctypes.data),
                host.nbytes,
                self.MEMCPY_HOST_TO_DEVICE,
            ),
            "cudaMemcpy H2D",
        )

    def copy_to_host(self, host, device):
        self._check(
            self.lib.cudaMemcpy(
                ctypes.c_void_p(host.ctypes.data),
                device,
                host.nbytes,
                self.MEMCPY_DEVICE_TO_HOST,
            ),
            "cudaMemcpy D2H",
        )

    def synchronize(self):
        self._check(
            self.lib.cudaDeviceSynchronize(),
            "cudaDeviceSynchronize",
        )


def _letterbox(image, input_width, input_height):
    height, width = image.shape[:2]
    scale = min(input_width / width, input_height / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    pad_x = (input_width - resized_width) / 2.0
    pad_y = (input_height - resized_height) / 2.0
    left = int(round(pad_x - 0.1))
    top = int(round(pad_y - 0.1))
    padded = cv2.copyMakeBorder(
        resized,
        top,
        input_height - resized_height - top,
        left,
        input_width - resized_width - left,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    blob = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    blob = blob.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(blob), scale, left, top


def _decode_obb(
    output,
    scale,
    pad_x,
    pad_y,
    confidence,
    nms_iou,
) -> List[RotatedDetection]:
    predictions = output[0].T
    predictions = predictions[predictions[:, 4] >= confidence]
    rects = []
    scores = []

    for x_center, y_center, width, height, score, angle in predictions:
        rects.append(
            (
                (
                    (float(x_center) - pad_x) / scale,
                    (float(y_center) - pad_y) / scale,
                ),
                (float(width) / scale, float(height) / scale),
                math.degrees(float(angle)),
            )
        )
        scores.append(float(score))

    if not rects:
        return []

    indices = cv2.dnn.NMSBoxesRotated(
        rects,
        scores,
        confidence,
        nms_iou,
    )
    return [
        (rects[int(index)], scores[int(index)])
        for index in np.asarray(indices).reshape(-1)
    ]


class CartonObbTensorRT:
    """Own one engine/context and serialize access to its CUDA buffers."""

    def __init__(self, engine_path=DEFAULT_TRT_ENGINE_PATH):
        self.engine_path = Path(engine_path).expanduser().resolve()
        if not self.engine_path.is_file():
            raise FileNotFoundError(
                f"TensorRT engine does not exist: {self.engine_path}"
            )

        self._lock = threading.Lock()
        self._cuda = _CudaRuntime()
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        self._engine = None
        self._context = None
        self._device_buffers = []
        self._input_buffer = None
        self._output_buffer = None
        self._input_dtype = None
        self._output = None
        self.input_shape = None
        self.closed = False

        with self.engine_path.open("rb") as engine_file:
            self._engine = self._runtime.deserialize_cuda_engine(
                engine_file.read()
            )
        if self._engine is None:
            raise RuntimeError(
                f"Failed to deserialize TensorRT engine: {self.engine_path}"
            )

        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("Failed to create TensorRT execution context")

        input_count = 0
        output_count = 0
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            shape = tuple(self._engine.get_tensor_shape(name))
            if any(dimension < 0 for dimension in shape):
                raise RuntimeError(
                    f"Dynamic tensor is not supported: name={name}, shape={shape}"
                )
            dtype = trt.nptype(self._engine.get_tensor_dtype(name))
            host_buffer = np.empty(shape, dtype=dtype)
            device_buffer = self._cuda.malloc(host_buffer.nbytes)
            self._device_buffers.append(device_buffer)
            if not self._context.set_tensor_address(
                name,
                int(device_buffer.value),
            ):
                raise RuntimeError(f"Failed to set TensorRT address for {name}")

            if self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                input_count += 1
                self._input_buffer = device_buffer
                self._input_dtype = dtype
                self.input_shape = shape
            else:
                output_count += 1
                self._output_buffer = device_buffer
                self._output = host_buffer

        if input_count != 1 or output_count != 1:
            self.close()
            raise RuntimeError(
                "Expected exactly one input and one output tensor, "
                f"got inputs={input_count}, outputs={output_count}"
            )
        if len(self.input_shape) != 4 or self.input_shape[1] != 3:
            self.close()
            raise RuntimeError(
                f"Expected NCHW image input, got {self.input_shape}"
            )

    @property
    def tensorrt_version(self):
        return trt.__version__

    def _infer_blob(self, blob):
        input_array = np.ascontiguousarray(
            blob.astype(self._input_dtype, copy=False)
        )
        self._cuda.copy_to_device(self._input_buffer, input_array)
        self._cuda.synchronize()

        started_at = time.perf_counter()
        if not self._context.execute_async_v3(0):
            raise RuntimeError("TensorRT execute_async_v3 returned false")
        self._cuda.synchronize()
        inference_ms = (time.perf_counter() - started_at) * 1000.0

        self._cuda.copy_to_host(self._output, self._output_buffer)
        return self._output.copy(), inference_ms

    def infer_bgr(
        self,
        image_bgr,
        confidence=0.25,
        nms_iou=0.45,
    ) -> Tuple[List[RotatedDetection], float]:
        if self.closed:
            raise RuntimeError("TensorRT backend is closed")
        if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("Expected a BGR image with shape (H, W, 3)")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
        if not 0.0 <= nms_iou <= 1.0:
            raise ValueError("nms_iou must be between 0 and 1")

        input_height = self.input_shape[2]
        input_width = self.input_shape[3]
        blob, scale, pad_x, pad_y = _letterbox(
            image_bgr,
            input_width,
            input_height,
        )
        with self._lock:
            output, inference_ms = self._infer_blob(blob)
        detections = _decode_obb(
            output,
            scale,
            pad_x,
            pad_y,
            confidence,
            nms_iou,
        )
        return detections, inference_ms

    def warmup(self, count=5):
        if count < 0:
            raise ValueError("warmup count must be non-negative")
        height = self.input_shape[2]
        width = self.input_shape[3]
        image = np.full((height, width, 3), 114, dtype=np.uint8)
        for _ in range(count):
            self.infer_bgr(image)

    def close(self):
        if self.closed:
            return
        for device_buffer in self._device_buffers:
            self._cuda.free(device_buffer)
        self._device_buffers.clear()
        self._context = None
        self._engine = None
        self._runtime = None
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()
