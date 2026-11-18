#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘检测的输入预处理：letterbox + BGR→CHW 张量。

⚠️ **逐行照抄 ultralytics 的 `LetterBox.__call__`**（`auto=False` 那一支），
连 `round(dh - 0.1) / round(dh + 0.1)` 这种抵消浮点误差的写法都保留 ——
补边差一个像素，框就整体平移一个像素。
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

PAD_VALUE = 114                   # ultralytics letterbox 的补边值
DEFAULT_IMGSZ = 640
DEFAULT_CONF = 0.25               # 只用于过滤与 NMS，不参与选框打分
DEFAULT_NMS_IOU = 0.45            # 导出图不带 NMS，必须自己去重


def _letterbox(img: np.ndarray, new_shape=(640, 640), *,
               pad_value: int = PAD_VALUE,
               scaleup: bool = True
               ) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """等比缩放 + **居中**补边到 `new_shape`。返回 `(图, 缩放比, (左补, 上补))`。"""
    import cv2
    h, w = img.shape[:2]
    new_h, new_w = ((new_shape, new_shape) if isinstance(new_shape, int)
                    else tuple(new_shape))
    r = min(new_h / h, new_w / w)
    if not scaleup:
        r = min(r, 1.0)
    new_unpad = (round(w * r), round(h * r))
    dw, dh = (new_w - new_unpad[0]) / 2, (new_h - new_unpad[1]) / 2
    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        if img.ndim == 2:
            img = img[..., None]
    top, bottom = round(dh - 0.1), round(dh + 0.1)
    left, right = round(dw - 0.1), round(dw + 0.1)
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT,
                             value=(pad_value,) * img.shape[2])
    return img, float(r), (float(left), float(top))


def _to_input_tensor(img_bgr: np.ndarray, imgsz: int = DEFAULT_IMGSZ
                     ) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """BGR 图 → `(1,3,H,W)` float32。顺序 **BGR→RGB→HWC→CHW→/255**（漏掉 RGB 框会整体偏）。"""
    lb, r, pad = _letterbox(img_bgr, (imgsz, imgsz))
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(x), r, pad
