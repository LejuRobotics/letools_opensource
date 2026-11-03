#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""POSE 初值层的**轻量局部梯度边修正**：只沿已解出的四条边重找梯度峰并重解。

链路（每帧只多做一次重解，不做推理）::

    frame（POSE 初解） -> 投影四角 -> 4 条初始直线
    ROI = 四角外扩 20px；只在 ROI 上灰度 + 高斯 + Sobel
    每条边沿线按 stride 采样，法向 ±half 找梯度最大峰（抛物线亚像素）
    有效采样点 >= 4 且极性一致 -> TLS 重拟合（带离群点剔除 / RMS 闸门）
    证据不成立的边保留原线，但**从 solve_frame 的 use_pos 里剔除**
    修正后的四条线 -> solve_frame（沿用调用方的 size/normal/D 锁） -> frame2

为什么门控比修正本身更关键
--------------------------
修正只改"看得见"的边；真正毁精度的是**看不见的边仍然参与位置量测**。
85 帧包的底边几乎整条不可见（``n_used == 0``），它若留在 ``use_pos`` 里会把
tr 角拉到 7~9px；剔除后同一批帧降到 ~1.8px。因此本模块把两件事分开：

* 梯度证据（采样数 / 极性 / RMS）不成立的边 -> 不修正，保留原线；
* 同一条边 -> 从第二次 ``solve_frame`` 的 ``use_pos`` 中剔除。

剔除后仍要保证至少覆盖一组对边（``{0,2}`` 与 ``{1,3}`` 各一条），否则按原
``use_pos`` 顺序补回，避免退化成无法约束的少数边解。

⚠️ 耗时：x86 上修正段 p50 ~6.9ms / p90 ~7.6ms（P2c 全量 601 实测）。
**Orin 未实测**，上机前需按目标帧率复核。

本模块是纯函数库：只依赖 numpy / cv2 和同包（numpy-only）的
``pose_decode`` / ``solve_edges`` / ``geom``；不 import ROS / ultralytics /
调试脚本。
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from . import pose_decode as PD
from . import solve_edges as SE
from .geom import _corners_mm, _project_px

# ---- 修正段参数（E16w8 已实测，不要在调用处另造一套）----------------------
ROI_MARGIN_PX = 20.0       # 托盘四角外扩
EDGE_MARGIN = 0.12         # 避开每条边两端 12%
MIN_SAMPLES = 4            # 至少 4 个有效采样点才重拟合
FIT_MAX_RESID_PX = 2.5     # 重拟合 RMS 闸门；超了视为发散，保留原边
GRAD_MIN = 20.0            # 采样点的最低梯度幅值
POLARITY_GATE = 0.6        # 极性门控：一条边"外法向往外变暗"的采样点占比 < GATE
                           # 时，判为量错了边（不修正、也不参与 solve 的位置量测）。
BLUR_SIGMA = 3.0           # ROI 灰度高斯 sigma
DEFAULT_STRIDE = 16        # 沿线采样步长（px）
DEFAULT_HALF = 8           # 法向搜索半窗（px）


def roi_gradient(color, quad_px):
    """托盘外扩 ROI 内算灰度 + 高斯 + Sobel。返回 ``(mag, gx, gy, x0, y0)``。"""
    h, w = color.shape[:2]
    x0 = int(max(0, np.floor(quad_px[:, 0].min() - ROI_MARGIN_PX)))
    x1 = int(min(w, np.ceil(quad_px[:, 0].max() + ROI_MARGIN_PX) + 1))
    y0 = int(max(0, np.floor(quad_px[:, 1].min() - ROI_MARGIN_PX)))
    y1 = int(min(h, np.ceil(quad_px[:, 1].max() + ROI_MARGIN_PX) + 1))
    crop = color[y0:y1, x0:x1]
    gray = cv2.GaussianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), (0, 0),
                            BLUR_SIGMA)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)
    return mag, gx, gy, x0, y0


def _bil_grid(A, U, V):
    """双线性插值整块版；坐标先夹到 ``[0, w-1]`` / ``[0, h-1]``。"""
    h, w = A.shape
    U = np.clip(U, 0.0, w - 1.001)
    V = np.clip(V, 0.0, h - 1.001)
    i = np.clip(np.floor(U).astype(np.int64), 0, w - 2)
    j = np.clip(np.floor(V).astype(np.int64), 0, h - 2)
    fu, fv = U - i, V - j
    return (A[j, i] * (1 - fu) * (1 - fv)
            + A[j, i + 1] * fu * (1 - fv)
            + A[j + 1, i] * (1 - fu) * fv
            + A[j + 1, i + 1] * fu * fv)


def _tls(pts):
    """TLS 直线拟合：返回 ``(centroid, unit direction)``。"""
    c = pts.mean(axis=0)
    q = pts - c
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    d = vt[0]
    n = float(np.linalg.norm(d))
    if n <= 0.0 or not np.isfinite(n):
        return c, np.array([1.0, 0.0])
    return c, d / n


def _fit_line_robust(pts):
    """TLS + 逐次丢最坏点；返回 ``(p0, d, rms, n_used)``。"""
    keep = np.ones(len(pts), bool)
    while True:
        p0, d = _tls(pts[keep])
        r = np.abs((pts[keep] - p0) @ np.array([-d[1], d[0]]))
        worst = int(np.argmax(r))
        n_keep = int(keep.sum())
        if float(r[worst]) <= FIT_MAX_RESID_PX or n_keep <= MIN_SAMPLES:
            break
        idx = np.where(keep)[0]
        keep[idx[worst]] = False
    p0, d = _tls(pts[keep])
    r = np.abs((pts[keep] - p0) @ np.array([-d[1], d[0]]))
    rms = float(np.sqrt(np.mean(r ** 2)))
    return p0, d, rms, int(keep.sum())


def refine_edge(a, b, center, mag, gx, gy, x0, y0, stride, half):
    """沿一条边做局部梯度搜索。返回 dict（新边或保留原边的诊断）。"""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    v = b - a
    L = float(np.linalg.norm(v))
    out = {'ok': False, 'n_used': 0, 'rms': None, 'polarity': None,
           'boundary_hits': 0, 'n_samples': 0, 'disp_px': 0.0,
           'reason': 'short_edge'}
    if L < 40.0:
        return out
    d = v / L
    nv = np.array([-d[1], d[0]])
    if float(np.dot((a + b) / 2.0 - center, nv)) < 0:
        nv = -nv                                   # 外法向

    n_samp = max(3, int(L * (1.0 - 2.0 * EDGE_MARGIN) / float(stride)))
    ts = np.linspace(EDGE_MARGIN, 1.0 - EDGE_MARGIN, n_samp)
    P = a[None, :] + ts[:, None] * v[None, :]

    ks = np.arange(-int(half), int(half) + 1, dtype=np.float64)
    U = P[:, 0:1] + ks[None, :] * nv[0]
    V = P[:, 1:2] + ks[None, :] * nv[1]
    h, w = mag.shape
    valid = (U >= 1) & (U < w - 1) & (V >= 1) & (V < h - 1)
    m = _bil_grid(mag, U - x0, V - y0)
    m = np.where(valid, m, -1.0)

    best = np.argmax(m, axis=1)
    rows = np.arange(len(P))
    best_m = m[rows, best]
    ok = best_m >= GRAD_MIN
    n_used = int(ok.sum())
    out['n_samples'] = int(len(P))
    if n_used == 0:
        out['reason'] = 'no_samples'
        return out

    # 抛物线亚像素（峰在窗口端点时不做，避免外推）
    off = np.zeros(len(P), float)
    nb = len(ks)
    for i in np.where(ok)[0]:
        j = int(best[i])
        if 0 < j < nb - 1:
            m0, m1, m2 = m[i, j - 1], m[i, j], m[i, j + 1]
            den = m0 - 2.0 * m1 + m2
            if abs(den) > 1e-9:
                off[i] = float(np.clip(0.5 * (m0 - m2) / den, -0.5, 0.5))
        else:
            out['boundary_hits'] += 1
    tt = ks[best] + off
    pts = P + tt[:, None] * nv[None, :]

    # 极性诊断：外法向上"由亮到暗" => 有符号梯度沿 nv 为负
    if gx is not None:
        gxv = _bil_grid(gx, pts[:, 0] - x0, pts[:, 1] - y0)
        gyv = _bil_grid(gy, pts[:, 0] - x0, pts[:, 1] - y0)
        s = gxv * nv[0] + gyv * nv[1]
        out['polarity'] = float(np.mean(s[ok] < 0.0))

    if n_used < MIN_SAMPLES:
        out['reason'] = 'few_samples'
        return out
    if (POLARITY_GATE is not None and out['polarity'] is not None
            and out['polarity'] < POLARITY_GATE):
        out['reason'] = 'polarity'
        return out
    p0, d2, rms, n_fit = _fit_line_robust(pts[ok])
    out.update({'n_used': n_used, 'rms': rms, 'n_fit': n_fit})
    if not np.isfinite(rms) or rms > FIT_MAX_RESID_PX:
        out['reason'] = 'rms'
        return out                                  # 线拟合发散 -> 保留原边
    out['ok'] = True
    out['reason'] = 'ok'
    out['p0'] = p0
    out['d'] = d2
    # 原边到新边的垂直位移（中点处），供失败分析
    mid = (a + b) / 2.0
    out['disp_px'] = float(abs(float((mid - p0) @ np.array([-d2[1], d2[0]]))))
    return out


def _frame_corners_px(frame, k):
    """frame -> 投影四角 ``[bl, br, tr, tl]``（与 solve 的边序一致）。"""
    return _project_px(_corners_mm(frame, float(frame['W']),
                                   float(frame['H'])), k)


def refine_frame(frame, color, k, normal, depth, use_pos, *,
                 size_mm, long_side_parallel, d_lock='depth',
                 stride=DEFAULT_STRIDE, half=DEFAULT_HALF,
                 d_lock_inlier=None):
    """POSE frame + 初始四边 -> 四条边梯度修正 -> 重解。

    初值固定用 ``frame`` 的**投影四角**（不用 ``diag['pose_quad']``）。
    第二次 ``solve_frame`` 的 ``size_mm`` / ``long_side_parallel`` / ``d_lock``
    **必须由调用方传真实值**；这里不写死。

    返回 ``(frame2 | None, diag)``：

    * ``frame2`` 非空：成功重解，挂 ``_obb_corners_px``（修正后四边求交）；
    * ``frame2`` 为空：重解被拒，调用方保留初解 frame，读数见 ``diag['reject']``。
    """
    t0 = time.perf_counter()
    px = np.asarray(_frame_corners_px(frame, k), float).reshape(4, 2)
    center = px.mean(axis=0)
    mag, gx, gy, x0, y0 = roi_gradient(color, px)

    # 初始边（由 frame 投影）；顺序 0,1,2,3 = 下/右/上/左
    a = [px[i] for i in range(4)]
    b = [px[(i + 1) % 4] for i in range(4)]
    edges = []
    for i in range(4):
        r = refine_edge(a[i], b[i], center, mag, gx, gy, x0, y0, stride, half)
        if r['ok']:
            edges.append({'p0': np.asarray(r['p0'], float),
                          'd': np.asarray(r['d'], float), 'weight': 1.0})
        else:
            v = b[i] - a[i]
            L = float(np.linalg.norm(v))
            edges.append({'p0': a[i].copy(), 'd': v / L, 'weight': 1.0})
        r['edge'] = i
        edges[-1]['_diag'] = r
        edges[-1]['_refit'] = bool(r['ok'])
    ms_search = (time.perf_counter() - t0) * 1000.0

    # 位置量测只保留"梯度证据成立"的边。不可见/极性不符的边若继续留在
    # use_pos 里，会把解算往错线拉；不足一组对边时按原 use_pos 顺序补回。
    orig_use = [int(i) for i in (use_pos if use_pos is not None
                                 else [0, 1, 2, 3])]

    def _rank_ok(s):
        return bool(set(s) & {0, 2}) and bool(set(s) & {1, 3})

    used = [i for i in orig_use if edges[i]['_refit']]
    if not used or not _rank_ok(used):
        for i in orig_use:
            if i not in used:
                used.append(i)
            if _rank_ok(used):
                break
    used = sorted(used) if _rank_ok(used) else orig_use

    # 四条边求交；失败则退回 frame 的投影四角
    els = [PD.EdgeLine(p0=e['p0'], d=e['d'], n_points=0, residual_px=0.0,
                       mean_conf=1.0, idx=()) for e in edges]
    h, w = color.shape[:2]
    quad = PD.intersect_edges(els, size=(h, w))
    if quad is None:
        quad = px

    t0 = time.perf_counter()
    frame2, sdiag = SE.solve_frame(
        [{'p0': e['p0'], 'd': e['d'], 'weight': 1.0} for e in edges],
        k, normal, size_mm=size_mm,
        long_side_parallel=bool(long_side_parallel), depth=depth,
        use_pos=used, d_lock=d_lock, quad_hint=quad, w_img=w, h_img=h,
        d_lock_inlier=d_lock_inlier)
    ms_solve = (time.perf_counter() - t0) * 1000.0

    # diag 要能直接 JSON 序列化：numpy 数组转 list
    edge_diag = []
    for e in edges:
        d = dict(e['_diag'])
        if 'p0' in d:
            d['p0'] = np.asarray(d['p0'], float).tolist()
        if 'd' in d:
            d['d'] = np.asarray(d['d'], float).tolist()
        edge_diag.append(d)

    diag = {
        'edge_refit': [bool(e['_refit']) for e in edges],
        'edge_diag': edge_diag,
        'n_refit': int(sum(bool(e['_refit']) for e in edges)),
        'use_pos_orig': orig_use,
        'use_pos_used': used,
        'n_dropped': len([i for i in orig_use if i not in used]),
        'ms_search': ms_search,
        'ms_solve': ms_solve,
        'ms_corr': ms_search + ms_solve,
        'reproj_px': sdiag.get('reproj_px'),
        'd_lock': sdiag.get('d_lock'),
    }
    if frame2 is None:
        diag['reject'] = sdiag.get('reject', 'solve_failed')
        return None, diag
    frame2['_obb_corners_px'] = np.asarray(quad, float)
    return frame2, diag
