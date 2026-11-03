#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**单一真相源**：用户规则里"哪个角可见、哪条边可信"的判据只在这里定义一次。

背景：这套判据曾经散在三处

* `solve_obb._vis_frac` + `EDGE_MIN_VIS` —— **初值拟合**时挑"用哪条边解位置"；
* `solve_user_rule.visible_corners`（画面内 + 不贴图像边界 + 相邻边可见 >= 0.8）
  —— **用户规则触发后**挑"用哪个角 + 哪两条相邻边"；
* `refcheck.corner_usable` / `edge_class` —— **评估**时挑"哪些真值角/边能当参照"。

本模块把**规则路径**的判据（`inspect` / `visible_corners` / `adjacent_edges`）收敛到
这一处，`solve_user_rule` 从这里 import。`refcheck` 是评估口径（拿真值 + 贴边标记
当裁判），与运行期判据用途不同、且必须独立，故**不动**。

⚠️ **`solve_obb.frame_from_obb` 的边筛选口径保持不变**，不改成这里的 `visible_corners`。
两者不是同一件事：`solve_obb` 那条是"初值拟合用哪几条 OBB 边当位置约束"，判据是
"这条边在画面内的长度占比 >= 0.5"；本模块这条是"哪个角是**真角**"，判据更严
（角的坐标要在画面内、不贴边，且**两条**相邻边的可见占比都 >= 0.8）。实测（601 帧）
把前者换成后者会让 `use_pos` 在 **482/601 帧**上改变、四角最大移动 **91.4px** ——
那是改行为，不是去重。用户已用 601 帧可视化逐帧确认过现行结果，故两条口径**并存**，
各自的作用是明确的：`_vis_frac` 是**量测原语**（画面内长度占比，留在 `solve_obb` 内部），
而"可信不可信"的判决只在 `edge_trust`。

    from pallet_detect.edge_trust import inspect, visible_corners, adjacent_edges, EDGE_VIS_MIN
"""
from __future__ import annotations

import numpy as np

from . import solve_obb as SO

WI, HI = 640.0, 480.0         # 图像尺寸（`solve_user_rule` 原样搬来）
BORDER_EPS = 0.5              # 角到图像边界小于这个距离 -> 认为贴边
EDGE_VIS_MIN = 0.80           # 两条**相邻边**的可见占比都要 >= 此值，角才算可用

# 深度**主峰占比**的闸门（`solve_obb._plane_dist_mm` 的第二个返回值）。
# 唯一真相在 `solve_obb.D_LOCK_INLIER`，这里只是为了给"规则路径"一个就近的入口；
# `refine3.D_DEPTH_INLIER` / `refine4.D_INLIER_MIN` 同样 import 那一个。
D_INLIER = SO.D_LOCK_INLIER


def _inside(p, eps=0.0):
    return bool(-eps <= p[0] <= WI - 1 + eps and -eps <= p[1] <= HI - 1 + eps)


def edge_touches_border(a, b, eps=BORDER_EPS):
    """线段 a->b 是否贴到图像边界（或出画）。"""
    ts = np.linspace(0.0, 1.0, 80)[:, None]
    P = np.asarray(a, float)[None, :] + ts * (np.asarray(b, float) - np.asarray(a, float))[None, :]
    out = ((P[:, 0] < eps) | (P[:, 0] > WI - 1 - eps) |
           (P[:, 1] < eps) | (P[:, 1] > HI - 1 - eps))
    return bool(out.any())


def inspect(quad):
    """OBB 四角 -> 可见性/贴边信息。

    `vis[i]` 用 `solve_obb._vis_frac`（画面内长度占比），与 `solve_obb` 初值拟合
    用的是**同一个量测原语**；判据（怎么用这些数）只在本模块定。
    """
    inside = [_inside(p) for p in quad]
    near_border = [(_inside(p, BORDER_EPS) and not _inside(p, -BORDER_EPS))
                   for p in quad]
    vis = [SO._vis_frac(quad[i], quad[(i + 1) % 4], WI, HI) for i in range(4)]
    border_edge = [edge_touches_border(quad[i], quad[(i + 1) % 4]) for i in range(4)]
    return dict(inside=inside, near_border=near_border, vis=vis,
                border_edge=border_edge,
                clipped=(any(not x for x in inside) or any(border_edge)))


def visible_corners(info, quad, edge_vis_min=EDGE_VIS_MIN):
    """图中**可见的角**。

    四条判据全过才算可见（校准见 `diag_usable_corner.py`，用真值贴边标记当裁判）：

    1. 落在画面内；
    2. 不贴图像边界（贴边 = 真实角在画面外被标到边界上）；
    3. **两条相邻边的可见占比都 >= `edge_vis_min`** —— 这条是关键：
       `00240` 的"下"角 (273.5,460.7) 坐标虽在画面内，但它撑着的下边只有
       72% 在画面里、另一端是外推点，所以它是**假角**，必须剔掉。
       加这条判据后 `00240` 只剩"左"一个角 —— 与用户从图像上看到的一致。

    ⚠️ 只靠 (1)(2) 判据（旧版）在全 601 帧上会**挑多 346 帧**、且真值只 1 个角
    的 141 帧里**一个都判不对**；加上 (3) 之后完全一致率 37.6% → **47.9%**、
    141 帧里判对 57 帧。
    """
    out = []
    for i in range(4):
        if not (info['inside'][i] and not info['near_border'][i]):
            continue
        e1, e2 = (i - 1) % 4, i
        if info['vis'][e1] >= edge_vis_min and info['vis'][e2] >= edge_vis_min:
            out.append(i)
    return out


def adjacent_edges(corner_idx):
    """与可见角**相邻**的两条边（角 i 属于边 i-1 与边 i）。"""
    out = []
    for i in corner_idx:
        for e in ((i - 1) % 4, i):
            if e not in out:
                out.append(e)
    return out
