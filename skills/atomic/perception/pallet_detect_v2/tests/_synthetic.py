# -*- coding: utf-8 -*-
"""`pallet_detect` 合成场景的共享夹具：`make_truth()` / `K`。

只保留**真值台面系**这一件：D 反解、法向、payload 契约的用例都只需要它，
不需要渲染图像或跑推理。
"""
from __future__ import annotations

import numpy as np

W_MM, H_MM = 1200.0, 1000.0
FX = FY = 400.0
CX, CY = 320.0, 240.0
IMG_H, IMG_W = 480, 640


class K:
    fx, fy, cx, cy = FX, FY, CX, CY


def _unit(v):
    return v / np.linalg.norm(v)


def make_truth() -> dict:
    """一个已知真值的台面系。

    ⚠️ 朝向必须**符合操作员 2026-09-22 的约定**（规格 §3）：
      origin = 画面左下角，E1 = 与画面近平行的边、指向画面右，E2 = 指向上。
    相机系里图像右 = +x、图像下 = +y，所以：
      E1 的 x 分量 > 0（向右），E1 的 y 分量 ~ 0（与画面平行）；
      E2 的 y 分量 < 0（向上）。
    """
    nrm = _unit(np.array([0.05, -0.40, -0.915]))          # 指向相机
    e1 = _unit(np.array([1.0, 0.0, 0.0]) - (np.array([1.0, 0.0, 0.0]) @ nrm) * nrm)
    if e1[0] < 0:                                          # 保证指向画面右
        e1 = -e1
    e2 = _unit(np.cross(nrm, e1))
    if e2[1] > 0:                                          # 保证指向上（图像 v 更小）
        e2 = -e2
    return dict(origin=np.array([-600.0, -300.0, 1600.0]), E1=e1, E2=e2, nrm=nrm,
                W=W_MM, H=H_MM)
