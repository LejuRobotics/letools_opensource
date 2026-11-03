#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**按用户规则实现**（不自行判断对错，只实现 + 出图）。

规则（逐字）：

> 如果从图中 OBB 解算出的 D 是异常的，另外 OBB 边缘还贴图像边或者出画，
> 那就证明此时托盘位置大面积出画。这时候可以只用之前深度测算的 D，
> 和可见的角以及相邻边，确定真实托盘位置。而真实托盘位置必须是从图中
> **可见的相邻边往外推测**，而不能依靠图像边缘的边或者出图边反算。

落到代码上：

**检出**（两个条件同时成立）
  * `anomaly`：`|D_obb / D_depth - 1| > D_ANOMALY`
    —— `D_obb` = `frame_from_obb` 用 OBB 四条边线几何解出的平面距离
  * `clipped`：OBB 有角出画，**或**有边贴到图像边界
    （贴边判据：该边到图像边界的最近距离 < `BORDER_EPS`）

**触发后**（"一个角 + 两条相邻边方向"配方）
  * 平面距离 `D` **锁死**为深度测算值（`SO._plane_dist_mm`）
  * 位置约束 = **唯一那个可见角**（角点像素差，2 个约束）
  * 方向约束 = 它**两条相邻边**的方向（单位化后的 sin 夹角，2 个约束）
  * 边长由物理尺寸给定（1000 × 1200），不从图像反算
  * **不**使用贴图像边界的边、**不**使用有端点在画外的边

**未触发**的帧：沿用现行 `frame_from_obb` 的结果（条件开关，不误伤）。
"""
from __future__ import annotations

import numpy as np

from .geom import CameraIntrinsics, _corners_mm, _project_px
from scipy.optimize import least_squares
from . import solve_obb as SO
from . import dsearch as DS
from .refine_ridge import grad_mag as _grad_mag, signed_grad as _signed_grad

NAMES = ('下', '右', '上', '左')

D_ANOMALY = 0.15      # D_obb 与 D_depth 相对差超过这个值 -> 认为 D 异常

# "哪个角可见 / 哪条边可信"的判据收敛到 `edge_trust`（单一真相源）。
# `visible_corners` 是标准实现；`inspect` / `adjacent_edges` 一并从那里来，
# 并在此**重新导出**（`diag_detector.py` 等仍按 `U.inspect(...)` / `U.visible_corners(...)` 调用）。
from .edge_trust import (BORDER_EPS, EDGE_VIS_MIN, HI, WI,  # noqa: F401
                         adjacent_edges, edge_touches_border, inspect,
                         visible_corners)


def corner_from_measured_edges(f0, k, gmag, gxy, corner_i):
    """**从一个角的两条相邻边往外推测** —— 用**量到的脊线**求交，得角的像素位置。

    这是用户规则的核心量测（选 A）：真实托盘位置必须从图中**可见的相邻边**
    往外推测，不用 OBB 的角、也不用图像边缘的边。

    * 两条相邻边 `(corner_i-1)%4` 与 `corner_i` 都要能**量到脊线**；
    任意一条量不到 -> 返回 `None`（交给调用方决定要不要放弃切换）。
    * 求交结果就是"托盘真实角"的图像位置（两条真边的交点）。

    ⚠️ 实测代价（`2026-10-10-11-44-12_00240`）：量到的"上/左"两脊线交于
    (287.7, 87.5)，真值角 (295.1, 86.3) —— **偏 7.51px**，比 OBB 角（偏 5.5px）
    还远一点，因为脊线本身有量测噪声（中位 2~4px），两条线一交误差会放大。
    但它的**方向是真的**（脊线就是托盘的边），所以对"D 正确与否"更有信息。
    """
    from .refine_ridge import measure_lines
    lines = measure_lines(f0, gmag, k, gxy=gxy)
    l1, l2 = lines[(corner_i - 1) % 4], lines[corner_i]
    if l1 is None or l2 is None:
        return None, lines
    A = np.array([[l1[0], l1[1]], [l2[0], l2[1]]], float)
    b = np.array([-l1[2], -l2[2]], float)
    if abs(float(np.linalg.det(A))) < 1e-9:
        return None, lines
    p = np.linalg.solve(A, b)
    return p, lines


def solve_one_corner_two_dirs(quad, k, normal, corner_i, D, corner_px=None,
                              line_dirs=None,
                              size_mm=None, theta_seed=None):
    """**一个角 + 两条相邻边的方向**，平面距离 D 锁死，尺寸由物理尺寸给定。

    未知量只有面内 3 个 (a, b, θ)：角点给 2 个像素约束、两条边的方向给 2 个，
    4 > 3 超定。

    ⚠️ **`size_mm` 默认 `None` = 用 `solve_obb.PALLET_SIZE_MM` 兜底。** 生产路径
    必须显式传（板子上的 `pallet_size_mm`），一路从 ROS 节点接下来：
    `refine_pallet_frame(target_mm=…)` → `frame_user_rule(size_mm=…)` → 这里。
    写成 `None` 而不是直接引用那个常量，正是为了让"**没传**"看得见 ——
    曾经这里默认成常量、而调用方一个都没传，于是**换了托盘尺寸它照样按旧尺寸解**，
    残差小、`valid` 真、完全静默。

    ## ⚠️ 面内基必须用**原始法向**（与 `frame_from_obb` 逐字一致）

    `frame_from_obb` 里是 `e1, e2 = _basis(normal)`（**原始** `nrm`，z<0），
    而 `n0` 才被翻到 z>0 用于 `origin`。两者符号不同 —— 用 `_basis(n0)` 会让
    面内基反向、θ 整体偏 180°，而种子只有 `{0, π/2}`，够不着。

    ## ⚠️ 必须给 θ 种子，不能扫 {0, π/2}

    "平行"这个约束**不区分正反**：矩形绕那个可见角转 180° 之后，两条相邻边
    仍然与目标边平行、代价完全相同 —— 这是**镜像歧义**。实测 `00240`：
    用 {0, π/2} 双种子时解出来的框，可见角 (299.8, 83.4) 是对的、但整块矩形
    翻到了该角的**左侧**，画面里一个角都不剩。

    OBB 自己的**面内转角**是准的（README 既证：面内转角差中位 1.93°），
    所以拿 `frame_from_obb` 解出的 θ 当**唯一**种子，歧义就消掉了。
    """
    if size_mm is None:                    # 没传 = 兜底（见 docstring 的 ⚠️）
        size_mm = SO.PALLET_SIZE_MM
    Wb, Hr = float(min(size_mm)), float(max(size_mm))
    # ⚠️ 方向残差要用边的**法向**（转 90°），不是切向。
    # `refine4._depth_stage` 里 `dirs[i]` 来自 `measure_lines` 返回的直线
    # `(a, b, c)`，那是**法向**（`a·u + b·v + c = 0`），残差 `dirs[i] @ (C[i]-C[i+1])`
    # 量的是"投影边是否垂直于法向 = 是否与目标边平行"。拿切向去点积就变成
    # "要求两条边**反向平行**"，矩形会翻到画面另一侧。
    dirs = {}
    for i in ((corner_i - 1) % 4, corner_i):
        if line_dirs is not None and line_dirs.get(i) is not None:
            # 有**量到的脊线**就用它的法向（那是托盘真边的方向）
            dirs[i] = np.asarray(line_dirs[i], float)
            dirs[i] = dirs[i] / max(float(np.linalg.norm(dirs[i])), 1e-9)
            continue
        d = quad[(i + 1) % 4] - quad[i]
        L = float(np.linalg.norm(d))
        if L > 1e-9:
            dirs[i] = np.array([-d[1] / L, d[0] / L])          # OBB 边的法向
    e1, e2 = SO._basis(normal)                # 与 frame_from_obb 同一套基
    n0 = np.asarray(normal, float)
    n0 = n0 / np.linalg.norm(n0)
    if float(n0 @ np.array([0.0, 0.0, 1.0])) < 0:
        n0 = -n0

    def mk(p):
        a, b, th = p
        c, s = np.cos(th), np.sin(th)
        return dict(origin=D * n0 + a * e1 + b * e2, E1=c * e1 + s * e2,
                    E2=-s * e1 + c * e2, nrm=n0, W=Wb, H=Hr)

    def res(p):
        C = _project_px(_corners_mm(mk(p), Wb, Hr), k)
        # 位置约束：优先用**从量到的两条相邻边往外推测**出来的角（`corner_px`），
        # 没有就退回 OBB 的角。
        tgt = quad[corner_i] if corner_px is None else np.asarray(corner_px, float)
        r = [float(C[corner_i][0] - tgt[0]),
             float(C[corner_i][1] - tgt[1])]
        for i in dirs:
            v = C[i] - C[(i + 1) % 4]
            v = v / max(float(np.linalg.norm(v)), 1e-9)
            r.append(float(dirs[i] @ v))
        return np.asarray(r)

    seeds = [theta_seed] if theta_seed is not None else [0.0, np.pi / 2]
    best = None
    for th0 in seeds:
        lim = np.radians(SO.INPLANE_LIMIT_DEG)
        try:
            s = least_squares(res, np.array([0.0, 0.0, th0]), loss='soft_l1',
                              f_scale=5.0, max_nfev=400,
                              bounds=([-1500., -1500., th0 - lim],
                                      [1500., 1500., th0 + lim]),
                              x_scale=[300., 300., .05])
        except Exception:                                       # noqa: BLE001
            continue
        if best is None or s.cost < best.cost:
            best = s
    if best is None:
        return None
    f = mk(best.x)
    P = _project_px(_corners_mm(f, Wb, Hr), k)
    return f if np.all(np.isfinite(P)) else None


def frame_user_rule(obb, k, normal, depth, D_anomaly=D_ANOMALY, color=None,
                    gmag=None, gxy=None, size_mm=None):
    """返回 `(frame, diag)`。`diag['mode']` ∈ {keep, switch, fallback}。

    ⚠️ **`size_mm` 必须由调用方传**（板子上的 `pallet_size_mm`）。`None` = 用
    `solve_obb.PALLET_SIZE_MM` 兜底 —— 那条路只该在离线脚本 / 单测上走。
    这条规则解出来的矩形**尺寸全部来自这个量**（"一个可见角 + 两条相邻边方向"，
    边长由物理尺寸给、不从图像反算），所以给错尺寸 = 整个位姿按错的 size 摆。

    ⚠️ **没触发用户规则（`mode='keep'`）时也走这个尺寸。** 下面那个
    `frame_from_obb` 必须传同一个 `size_mm` —— 不传它就吃 `PALLET_SIZE_MM`
    兜底，于是"触发时用板子的尺寸、没触发时用写死的尺寸"，
    **同一个 `frame` 的 `W`/`H` 取决于有没有出画**。那种不一致没有任何报错。
    """
    quad = SO.order_quad(obb['corners_px'])
    if size_mm is None:                        # 没传 = 兜底（见 docstring 的 ⚠️）
        size_mm = SO.PALLET_SIZE_MM
    n0 = np.asarray(normal, float); n0 = n0 / np.linalg.norm(n0)
    if float(n0 @ np.array([0., 0., 1.])) < 0:
        n0 = -n0
    info = inspect(quad)

    f_cur, _ = SO.frame_from_obb(obb, k, normal, depth=depth, size_mm=size_mm)
    if f_cur is None:
        return None, {'mode': 'reject'}
    D_obb = DS.D_from_frame(f_cur, normal)

    D_depth = None
    if depth is not None:
        D_depth, _ = SO._plane_dist_mm(n0, depth, obb['corners_px'], k)

    anomaly = (D_depth is not None) and abs(D_obb / D_depth - 1.0) > D_anomaly
    dg = dict(mode='keep', D_obb=D_obb, D_depth=D_depth,
              clipped=info['clipped'], anomaly=bool(anomaly),
              inside=info['inside'], near_border=info['near_border'],
              border_edge=info['border_edge'])
    if not (info['clipped'] and anomaly):
        return f_cur, dg

    # 触发条件（用户指定）：**恰好一个角在画面内，其余三个角都出画或贴边**。
    # 此时只能靠这一个角 + 它两条相邻边的方向（长度由物理尺寸给）+ 深度 D 定位。
    cvis = visible_corners(info, quad)
    dg['use_corner'] = cvis
    if len(cvis) != 1:
        dg['mode'] = 'fallback_%d_corners' % len(cvis)
        return f_cur, dg
    ci = cvis[0]
    dg['use_dir'] = [(ci - 1) % 4, ci]
    # **A：从一个角的两条相邻边往外推测** —— 用**量到的脊线**求交得角的像素位置
    if gmag is None and color is not None:
        gmag = _grad_mag(color)
    if gxy is None and color is not None:
        gxy = _signed_grad(color)
    if gmag is None or gxy is None:
        dg['mode'] = 'fallback_no_image'
        return f_cur, dg
    cpx, lines = corner_from_measured_edges(f_cur, k, gmag, gxy, ci)
    dg['corner_measured'] = (None if cpx is None else
                             [round(float(cpx[0]), 2), round(float(cpx[1]), 2)])
    _dirs = {}
    for i in ((ci - 1) % 4, ci):
        l = lines[i] if i < len(lines) else None
        if l is not None:
            _dirs[i] = np.array([l[0], l[1]], float)
    dg['use_measured_dir'] = sorted(_dirs)
    # θ 种子取 OBB 自己的面内转角（消镜像歧义）
    _e1, _e2 = SO._basis(normal)
    E1_cur = np.asarray(f_cur['E1'], float)
    th_seed = float(np.arctan2(E1_cur @ _e2, E1_cur @ _e1))
    f = solve_one_corner_two_dirs(quad, k, normal, ci, float(D_depth),
                                  corner_px=cpx, line_dirs=_dirs, theta_seed=th_seed,
                                  size_mm=size_mm)
    if f is None:
        dg['mode'] = 'fallback_solve_fail'
        return f_cur, dg
    dg['mode'] = 'switch'
    return SO._to_spec(f, float(f['W']), float(f['H']), k), dg
