#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘 OBB / POSE 的 **TensorRT engine** 后端（不 import ROS）。

与 `carton_detect/trt_backend.py` 同一套 CUDA/TRT 调用方式，但这里只做
**原始输出推理**：前处理（letterbox）与后处理（解码/NMS/解算）仍由
`detect.py` / `detect_pose.py` 完成 —— 两条后端共用同一段算法代码，
这样"换 engine"不会偷偷改数值口径。

对外暴露的是一个 **与 `onnxruntime.InferenceSession` 鸭子同型** 的会话：
只实现检测代码用到的那两个方法

    get_inputs()[0].name        -> 输入张量名
    run(None, {name: x})[0]     -> (1, N, 8400) float32

因此 `detect._resolve_session` / `detect_pose._resolve_pose_session` 可以
原样把 session 传下去，不需要任何分支。

⚠️ engine 是**设备/TRT 版本绑定**的：在本机（Jetson Orin / TRT 8.5）用
`trtexec --onnx=... --saveEngine=... --noTF32 --buildOnly` 导出，换机/换 TRT
版本必须重导。`expected_input` 不匹配时**直接抛**（导出时尺寸写错会静默跑
错，这里把它变成启动错误）。
"""
from __future__ import annotations

import ctypes
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Optional, Sequence, Tuple

import numpy as np

try:
    import tensorrt as trt
except ImportError as _exc:                         # pragma: no cover
    trt = None
    _TRT_IMPORT_ERROR: Optional[BaseException] = _exc
else:
    _TRT_IMPORT_ERROR = None


class _CudaRuntime:
    """最小 cudart 封装（与 carton_detect/trt_backend.py 同一写法）。"""

    MEMCPY_HOST_TO_DEVICE = 1
    MEMCPY_DEVICE_TO_HOST = 2

    def __init__(self):
        self.lib = ctypes.CDLL('libcudart.so')
        self.lib.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaMemcpy.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self.lib.cudaDeviceSynchronize.argtypes = []

    @staticmethod
    def _check(code, operation):
        if code != 0:
            raise RuntimeError(f'{operation} failed with CUDA error {code}')

    def malloc(self, nbytes):
        pointer = ctypes.c_void_p()
        self._check(self.lib.cudaMalloc(ctypes.byref(pointer), nbytes),
                    'cudaMalloc')
        return pointer

    def free(self, pointer):
        self._check(self.lib.cudaFree(pointer), 'cudaFree')

    def copy_to_device(self, device, host):
        self._check(self.lib.cudaMemcpy(
            device, ctypes.c_void_p(host.ctypes.data), host.nbytes,
            self.MEMCPY_HOST_TO_DEVICE), 'cudaMemcpy H2D')

    def copy_to_host(self, host, device):
        self._check(self.lib.cudaMemcpy(
            ctypes.c_void_p(host.ctypes.data), device, host.nbytes,
            self.MEMCPY_DEVICE_TO_HOST), 'cudaMemcpy D2H')

    def synchronize(self):
        self._check(self.lib.cudaDeviceSynchronize(), 'cudaDeviceSynchronize')


class TensorRTSession:
    """一个 engine + execution context 的串行会话（onnxruntime 鸭子同型）。"""

    def __init__(self, engine_path, *,
                 expected_input: Optional[Sequence[int]] = None):
        if trt is None:                              # pragma: no cover
            raise RuntimeError(
                f'没有 tensorrt（{_TRT_IMPORT_ERROR}）—— engine 后端需要它。\n'
                f'  怎么解决：用导出了 engine 的那台机器（Orin + TRT 8.5），'
                f'或改用 backend=onnx。')
        self.engine_path = Path(engine_path).expanduser().resolve()
        if not self.engine_path.is_file():
            raise FileNotFoundError(f'TensorRT engine 不存在：{self.engine_path}')

        self._lock = threading.Lock()
        self._cuda = _CudaRuntime()
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        self._engine = None
        self._context = None
        self._device_buffers = []
        self._input_device = None
        self._output_device = None
        self._input_host = None
        self._output_host = None
        self._input_dtype = None
        self.input_name = None
        self.input_shape = None
        self.output_shape = None
        self.closed = False

        with self.engine_path.open('rb') as f:
            self._engine = self._runtime.deserialize_cuda_engine(f.read())
        if self._engine is None:
            raise RuntimeError(f'反序列化 engine 失败：{self.engine_path}')
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError('创建 TensorRT execution context 失败')

        n_in = n_out = 0
        for i in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(i)
            shape = tuple(self._engine.get_tensor_shape(name))
            if any(d < 0 for d in shape):
                raise RuntimeError(f'不支持动态张量：{name} {shape}')
            dtype = trt.nptype(self._engine.get_tensor_dtype(name))
            host = np.empty(shape, dtype=dtype)
            device = self._cuda.malloc(host.nbytes)
            self._device_buffers.append(device)
            if not self._context.set_tensor_address(name, int(device.value)):
                raise RuntimeError(f'设置张量地址失败：{name}')
            if self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                n_in += 1
                self._input_device = device
                self._input_host = host
                self._input_dtype = dtype
                self.input_name = name
                self.input_shape = shape
            else:
                n_out += 1
                self._output_device = device
                self._output_host = host
                self.output_shape = shape

        if n_in != 1 or n_out != 1:
            self.close()
            raise RuntimeError(
                f'engine 必须恰好一进一出，实际 in={n_in} out={n_out}')
        if len(self.input_shape) != 4 or self.input_shape[1] != 3:
            self.close()
            raise RuntimeError(f'engine 输入不是 NCHW 图像：{self.input_shape}')
        if expected_input is not None and tuple(expected_input) != self.input_shape:
            self.close()
            raise RuntimeError(
                f'engine 输入尺寸 {self.input_shape} 与请求的 {tuple(expected_input)} '
                f'不一致 —— 导出 engine 时的 --imgsz 与检测参数不一致。')

    # ---- onnxruntime 鸭子接口 -------------------------------------------
    def get_inputs(self):
        return [SimpleNamespace(name=self.input_name)]

    def get_providers(self):
        return ['TensorRT']

    def run(self, _output_names, feeds):
        if self.closed:
            raise RuntimeError('TensorRTSession 已关闭')
        x = np.asarray(feeds[self.input_name])
        if tuple(x.shape) != tuple(self.input_shape):
            raise ValueError(
                f'输入形状 {tuple(x.shape)} != engine 输入 {tuple(self.input_shape)}')
        x = np.ascontiguousarray(x.astype(self._input_dtype, copy=False))
        with self._lock:
            self._cuda.copy_to_device(self._input_device, x)
            self._cuda.synchronize()
            if not self._context.execute_async_v3(0):
                raise RuntimeError('TensorRT execute_async_v3 返回 false')
            self._cuda.synchronize()
            self._cuda.copy_to_host(self._output_host, self._output_device)
            return [self._output_host.copy()]

    def close(self):
        if self.closed:
            return
        for buf in self._device_buffers:
            try:
                self._cuda.free(buf)
            except Exception:                        # noqa: BLE001
                pass
        self._device_buffers.clear()
        self._context = None
        self._engine = None
        self._runtime = None
        self.closed = True

    def __del__(self):                               # pragma: no cover
        try:
            self.close()
        except Exception:                            # noqa: BLE001
            pass
