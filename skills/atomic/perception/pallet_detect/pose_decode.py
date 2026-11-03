# -*- coding: utf-8 -*-
"""YOLO11-pose 托盘模型的纯 Python 前端：解码、NMS、选目标、边拟合、求交。

只依赖 numpy（+ 标准库），不 import ROS / ultralytics / onnxruntime。

ONNX 输出格式
-------------
输入 ``(1, 3, 640, 640)`` float32（RGB，/255，letterbox 居中补边）。
输出 ``(1, 41, 8400)``：转置成 ``(N, 41)`` 后每行是

    [cx, cy, w, h, conf, x0, y0, c0, x1, y1, c1, ..., x11, y11, c11]

即第 5 列是目标置信度，之后 12 组 ``(x, y, conf)`` 是 12 个关键点。
关键点坐标是 **640 输入图坐标（像素）**，不是归一化坐标。

坐标还原
--------
letterbox 后的输入坐标 ``(u_in, v_in)`` 用同一组 ``ratio`` / ``pad`` 还原：

    u = (u_in - pad_left) / ratio
    v = (v_in - pad_top)  / ratio

``ratio``、``pad`` 与 ``detect._letterbox`` 的输出一致。**不 clamp 出画点**：
模型对出画点的外推值仍带有边方向信息，夹到画面边界会把边拉歪；低置信点
交给 ``kpt_conf_thr`` / ``fit_edges`` 的 ``kconf`` 过滤。

边分组
------
12 个点沿轮廓排列（底边→右边→上边→左边），相邻边共用角点::

    EDGES[0] = (0, 1, 2, 3)     # 底边  corner_bl -> corner_br
    EDGES[1] = (3, 4, 5, 6)     # 右边  corner_br -> corner_tr
    EDGES[2] = (6, 7, 8, 9)     # 上边  corner_tr -> corner_tl
    EDGES[3] = (9, 10, 11, 0)   # 左边  corner_tl -> corner_bl

模型角点有系统偏移，角点位置由相邻两条拟合直线的交点给出（``intersect_edges``）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

N_KP = 12
N_COLS = 4 + 1 + N_KP * 3  # 41

#: 每条边的关键点下标（相邻边共用角点）。
EDGES: Tuple[Tuple[int, int, int, int], ...] = (
    (0, 1, 2, 3),
    (3, 4, 5, 6),
    (6, 7, 8, 9),
    (9, 10, 11, 0),
)

#: 两线单位方向叉积低于此值视为近平行（sin(角)）。
_PARALLEL_SIN = 1e-3
#: 交点离画面中心超过「对角线 × 此倍数」视为离谱。
_CORNER_RANGE_DIAG = 3.0


@dataclass
class PoseDetection:
    """一帧里的一个 POSE 候选（原图像素坐标）。"""

    bbox_xyxy: np.ndarray  # (4,) float  [x1, y1, x2, y2]
    conf: float
    kp_xy: np.ndarray      # (12, 2) float  原图像素坐标（可能出画）
    kp_conf: np.ndarray    # (12,) float


@dataclass
class EdgeLine:
    """一条边的加权 TLS 拟合结果（原图像素坐标）。"""

    p0: np.ndarray         # (2,) 直线上一点
    d: np.ndarray          # (2,) 单位方向
    n_points: int          # 参与拟合的点数（2~4）
    residual_px: float     # 最大垂距
    mean_conf: float       # 参与点的平均置信度
    idx: tuple             # 用了哪些关键点下标（诊断）


def decode_pose(raw: np.ndarray, ratio: float, pad: Sequence[float],
                orig_shape: Sequence[int], conf_thr: float,
                kpt_conf_thr: float = 0.0) -> list:
    """ONNX 原始输出 → 原图坐标候选（未去重）。

    ``raw`` 形状 ``(N, 41)``（调用方传 ``raw[0].T``）。列序见模块 docstring。
    关键点置信度低于 ``kpt_conf_thr`` 的置 0（保留下标与坐标，便于诊断）；
    检测本身只按框置信度 ``conf_thr`` 过滤。坐标不做 clamp。
    """
    arr = np.asarray(raw, np.float64).reshape(-1, N_COLS)
    arr = arr[arr[:, 4] >= float(conf_thr)]
    if len(arr) == 0:
        return []

    r = float(ratio)
    left, top = float(pad[0]), float(pad[1])
    cx, cy, w, h = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
    bbox = np.stack([
        (cx - w / 2.0 - left) / r,
        (cy - h / 2.0 - top) / r,
        (cx + w / 2.0 - left) / r,
        (cy + h / 2.0 - top) / r,
    ], axis=1)

    kp = arr[:, 5:].reshape(-1, N_KP, 3)
    kp_xy = np.empty((len(arr), N_KP, 2), np.float64)
    kp_xy[:, :, 0] = (kp[:, :, 0] - left) / r
    kp_xy[:, :, 1] = (kp[:, :, 1] - top) / r
    kp_conf = kp[:, :, 2].copy()
    if kpt_conf_thr > 0.0:
        kp_conf[kp_conf < float(kpt_conf_thr)] = 0.0

    return [PoseDetection(bbox_xyxy=bbox[i].copy(),
                          conf=float(arr[i, 4]),
                          kp_xy=kp_xy[i].copy(),
                          kp_conf=kp_conf[i].copy())
            for i in range(len(arr))]


def _aabb_iou(a: np.ndarray, b: np.ndarray) -> float:
    """两个 ``[x1, y1, x2, y2]`` 的 IoU。"""
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return float(inter / union) if union > 0 else 0.0


def nms_pose(dets: list, iou_thr: float, size=None) -> list:
    """AABB IoU NMS，按 ``conf`` 降序贪心保留。

    ``size`` 仅为与 ``detect._nms`` 的调用方式对齐，AABB NMS 不需要它。
    """
    if not dets:
        return []
    order = sorted(dets, key=lambda d: -d.conf)
    keep: list = []
    for det in order:
        if all(_aabb_iou(det.bbox_xyxy, k.bbox_xyxy) <= iou_thr for k in keep):
            keep.append(det)
    return keep


def pick_pose(dets: list, size, how: str = 'bottom_center'
              ) -> Optional[PoseDetection]:
    """选主目标，规则与 ``detect._pick`` 一致。

    ``how`` ∈ ``{'bottom_center'(默认), 'conf', 'area'}``；
    ``bottom_center`` 取框中心离画面下方中心最近的那个。
    """
    if not dets:
        return None
    if how == 'conf':
        return max(dets, key=lambda d: d.conf)
    if how == 'area':
        return max(dets, key=lambda d: float(
            (d.bbox_xyxy[2] - d.bbox_xyxy[0])
            * (d.bbox_xyxy[3] - d.bbox_xyxy[1])))
    h, w = size
    ref = np.array([w / 2.0, float(h)])
    return min(dets, key=lambda d: float(np.linalg.norm(
        (d.bbox_xyxy[:2] + d.bbox_xyxy[2:]) / 2.0 - ref)))


def _tls_line(pts: np.ndarray, weights: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """加权总体最小二乘（SVD）：返回 ``(centroid, 单位主方向)``。"""
    c = np.average(pts, axis=0, weights=weights)
    q = (pts - c) * np.sqrt(weights)[:, None]
    # SVD 的右奇异向量：第一行是最大方差方向。
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    d = vt[0]
    norm = float(np.linalg.norm(d))
    if norm <= 0.0 or not np.isfinite(norm):
        return c, np.array([1.0, 0.0])
    return c, d / norm


def _perp_residual(pts: np.ndarray, p0: np.ndarray, d: np.ndarray) -> np.ndarray:
    """点到直线的有符号/无符号垂距（绝对值）。"""
    v = pts - p0
    return np.abs(v[:, 0] * d[1] - v[:, 1] * d[0])


def fit_edges(kp_xy: np.ndarray, kp_conf: np.ndarray, kconf: float = 0.5,
              line_res_px: float = 12.0) -> list:
    """按 ``EDGES`` 对每条边做加权 TLS，返回长度 4 的 ``list[EdgeLine|None]``。

    每条边取 ``conf >= kconf`` 的点；至少 2 点才拟合（权重 = conf）。
    最大垂距超过 ``line_res_px`` 时逐次丢掉最差点重拟合；不足 2 点返回 None。
    """
    kp_xy = np.asarray(kp_xy, np.float64).reshape(N_KP, 2)
    kp_conf = np.asarray(kp_conf, np.float64).reshape(N_KP)
    out: list = []

    for edge in EDGES:
        idx = np.asarray(edge)
        mask = kp_conf[idx] >= float(kconf)
        used = idx[mask]
        if len(used) < 2:
            out.append(None)
            continue

        pts = kp_xy[used]
        w = kp_conf[used]
        while True:
            p0, d = _tls_line(pts, w)
            res = _perp_residual(pts, p0, d)
            worst = int(np.argmax(res))
            if float(res[worst]) <= float(line_res_px) or len(pts) <= 2:
                break
            pts = np.delete(pts, worst, axis=0)
            w = np.delete(w, worst, axis=0)
            used = np.delete(used, worst)

        out.append(EdgeLine(
            p0=p0,
            d=d,
            n_points=int(len(pts)),
            residual_px=float(res[worst]),
            mean_conf=float(np.mean(w)),
            idx=tuple(int(i) for i in used),
        ))
    return out


def _intersect(l0: EdgeLine, l1: EdgeLine) -> Optional[np.ndarray]:
    """两条无限直线求交；近平行返回 None。"""
    den = float(l0.d[0] * l1.d[1] - l0.d[1] * l1.d[0])
    if abs(den) < _PARALLEL_SIN:
        return None
    v = l1.p0 - l0.p0
    t = (v[0] * l1.d[1] - v[1] * l1.d[0]) / den
    return l0.p0 + t * l0.d


def intersect_edges(lines: Sequence[Optional[EdgeLine]],
                    size: Optional[Sequence[int]] = None) -> Optional[np.ndarray]:
    """相邻边求交 → ``(4, 2)`` 的 ``[bl, br, tr, tl]``，失败返回 None。

    ``c0 = edge3 × edge0``、``c1 = edge0 × edge1``、``c2 = edge1 × edge2``、
    ``c3 = edge2 × edge3``。任一条边为 None、两线近平行，或交点离画面中心
    超过 3 倍对角线（仅在给出 ``size=(h, w)`` 时检查）→ 返回 None。
    """
    if len(lines) != 4 or any(ln is None for ln in lines):
        return None

    pairs = ((3, 0), (0, 1), (1, 2), (2, 3))
    corners: list = []
    for i, j in pairs:
        pt = _intersect(lines[i], lines[j])
        if pt is None:
            return None
        corners.append(pt)

    quad = np.asarray(corners, np.float64)
    if not np.all(np.isfinite(quad)):
        return None

    if size is not None:
        h, w = float(size[0]), float(size[1])
        center = np.array([w / 2.0, h / 2.0])
        limit = _CORNER_RANGE_DIAG * float(np.hypot(w, h))
        if np.any(np.linalg.norm(quad - center, axis=1) > limit):
            return None
    return quad
