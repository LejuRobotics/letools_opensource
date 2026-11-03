#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""快速 4-DOF 解算：直接吃四条图像直线。

`frame_from_obb` 先用 OBB 四角反推四条直线、再用 `scipy.optimize.least_squares`
（`soft_l1`、400 次函数评估）解 `(d, a, b, θ)`。POSE 路线本来就拿到四条直线，
不需要 OBB 中转；这里用**手写 GN/Newton + IRLS**，同样的目标函数、更少的评估。

参数化与 `solve_obb.frame_from_obb` **逐字一致**（不要另造一套）：

    n̂   : 已锁定的台面法向（这里会翻到 z>0）
    e1,e2 : `solve_obb._basis(normal)` —— ⚠️ 用**未翻符号**的 normal 造基
    origin = d·n̂ + a·e1 + b·e2
    E1 = cosθ·e1 + sinθ·e2
    E2 = -sinθ·e1 + cosθ·e2
    W = min(size_mm) 沿 E1，H = max(size_mm) 沿 E2

⚠️ **`e1/e2` 必须用传入的 `normal` 原样去 `_basis`，不能先用翻正后的 `n̂`**。
`_basis` 对 `n` 和 `-n` 给出的 `e1` 是反号的，θ 会整体差 π；虽然描述的是同一个
矩形，但 θ 的数值窗口/诊断和 `frame_from_obb` 对不上。

## 残差：与 `frame_from_obb` 同一条三维直线

`frame_from_obb` 的量测不是像素垂距，而是**过图像直线与光心的三维平面**到矩形
端点（毫米）的有符号距离：`_line_plane_of(quad[i], quad[i+1]) · C[j]`。本模块
沿用同一个量（`pls[i] @ C[j]`），目标函数才与它同构。

⚠️ 不要改成"投影点再算像素垂距"：两者差一个随深度变化的因子（像素垂距 =
三维垂距 · |n₃| / Z），在 601 帧上实测角点差中位 1~7px，过不了 0.1px 的等价闸门。

## 鲁棒损失：soft_l1(f_scale=5) 的 IRLS

`frame_from_obb` 用 `loss='soft_l1', f_scale=5.0`。普通 L2 会在残差大的帧收敛到
另一个（L2 下更低、但与现行实现不同）的极小点。这里用同一个损失的 IRLS 权重

    w(r) = 1 / sqrt(1 + (r / 5)²)

在迭代里更新（每次迭代重算权重），从而收敛到与现行实现相同的鲁棒极小点。
601 帧实测：474 帧逐帧最大差中位 9e-6 px、p90 0.032 px；剩下 127 帧是
`frame_from_obb` 被自己的初值困在边界/坏盆地（见下面的"初值"一节）。

## 初值

`quad_hint` 或相邻直线求交得到 `[bl, br, tr, tl]`（世界坐标 `(0,0),(W,0),
(W,H),(0,H)`），`cv2.solvePnP(..., flags=SOLVEPNP_IPPE)` 给 R,t，再分解出
`d = t·n̂, a = t·e1, b = t·e2, θ = atan2(R[:,0]·e2, R[:,0]·e1)`。
PnP 失败时退回 `long_side_parallel` 给的 θ、`d=1000, a=b=0`。

⚠️ 这是与 `frame_from_obb` **唯一有意的不同**：它用 `a=b=0`（或 `d=1200`）起步。
在少数退化帧上，那个初值会把 `least_squares` 推到边界上的坏解（实测
`2026-09-30-11-04-26`：d 撞到 300 下界，重投影 354px）；PnP 初值会收敛到真正的
极小点。因此以现行实现的输出为基准做逐帧等价时，这些帧会出现大差 —— 是基准
自己没收敛，不是本模块算错。
"""
from __future__ import annotations

import cv2
import numpy as np

from .geom import _corners_mm, _project_px
from .solve_obb import (
    DEPTH_RANGE_MM,
    D_LOCK_INLIER,
    INPLANE_LIMIT_DEG,
    _basis,
    _line_plane_of,
    _plane_dist_mm,
    _seed_from_long_side,
    _seed_from_obb,
    _to_spec,
    order_quad,
)

# `frame_from_obb` 的鲁棒损失参数。必须与那边一致，否则极小点会漂。
SOFT_L1_F_SCALE = 5.0
AB_LIMIT_MM = 1500.0
MAX_ITER = 12
# 前两次用 Gauss-Newton（IRLS 加权 L2），之后用"含损失二阶导"的 Newton。
#
# ⚠️ 只用 Gauss-Newton 会收敛得很慢：软 L1 在残差大时近似 L1，代价面在深度/
# 面内平移方向上是**平坦的长谷**，GN 每步只走到谷底附近（实测 8 次后仍差
# 1~10px）。只用 Newton 则第一步方向正确、步长极大（损失二阶导在残差大时
# ∝ w³，Hessian 很小），回溯要折半 7 次才落地，8 次迭代同样不够。
# 先 2 次 GN 把解拉进盆底，再用 Newton 二次收敛：绝大多数帧 8 次内到 1e-3 px
# 以下；极少数平坦帧 8 次用尽后仍差 0.1~0.9px，上限放到 12 次后收敛到 <0.5px。
GN_ITERS = 2
# 单步上限（毫米；角度按 1 rad = 1000 计）。防 Newton 在平坦方向一步跨出画面。
# 实测 300mm：Newton 首步不再靠回溯折半 4~7 次才落地，迭代内收敛得更干净
# （601 帧上个别帧从 0.8px 降到 0.13px）；太小（<=100mm）反而走不完。
MAX_STEP = 300.0
# 数值雅可比的步长（毫米 / 弧度）。中心差分；残差是光滑的，这个量级足够。
_STEP_MM = 1e-3
_STEP_RAD = 1e-6


def _intersect(l1, l2):
    """两条 `(p0, d)` 直线求交；近平行返回 None。"""
    if l1 is None or l2 is None:
        return None
    p = np.asarray(l1['p0'], float)
    r = np.asarray(l1['d'], float)
    q = np.asarray(l2['p0'], float)
    s = np.asarray(l2['d'], float)
    den = float(r[0] * s[1] - r[1] * s[0])
    if abs(den) < 1e-9:
        return None
    t = float(((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / den)
    return p + t * r


def _quad_from_lines(lines):
    """相邻直线求交 -> 四角（未定序）。任意一个角求不出就返回 None。"""
    cs = [_intersect(lines[(i - 1) % 4], lines[i]) for i in range(4)]
    if any(c is None for c in cs):
        return None
    return order_quad(np.asarray(cs, float))


def _pnp_seed(quad, e1, e2, n0, k, W, H):
    """IPPE 初值 -> `(d, a, b, θ)`。失败返回 None。"""
    Kmat = np.array([[k.fx, 0.0, k.cx],
                     [0.0, k.fy, k.cy],
                     [0.0, 0.0, 1.0]], float)
    obj = np.array([[0.0, 0.0, 0.0],
                    [W, 0.0, 0.0],
                    [W, H, 0.0],
                    [0.0, H, 0.0]], float)
    try:
        ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(quad, float), Kmat, None,
                                      flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    t = np.asarray(tvec, float).ravel()
    d = float(t @ n0)
    a = float(t @ e1)
    b = float(t @ e2)
    th = float(np.arctan2(R[:, 0] @ e2, R[:, 0] @ e1))
    if not np.all(np.isfinite([d, a, b, th])):
        return None
    return d, a, b, th


def _fallback_seed(quad, e1, e2, k, long_side_parallel):
    """PnP 失败时的 θ 种子；`d=1000, a=b=0`。"""
    th = None
    if quad is not None:
        th = _seed_from_obb(quad, e1, e2, k,
                            bool(long_side_parallel) if long_side_parallel is not None else False)
    if th is None:
        th = _seed_from_long_side(e1, e2, k,
                                  bool(long_side_parallel) if long_side_parallel is not None else False)
    return 1000.0, 0.0, 0.0, float(th)


def _resolve_d_lock(d_lock, depth, quad, n0, k, use_pos, d_lock_inlier=None):
    """把 `d_lock` 参数解析成 `(D|None, 峰占比|None)`。

    * `None`：与 `frame_from_obb` 一致 —— 只有 `use_pos` 不足 4 条、且深度主峰
      占比 >= `D_LOCK_INLIER` 时锁 D，否则 D 是自由参数；
    * `'depth'`：只要深度主峰可用就锁，失败退回 4 自由度；
    * 数值：强制该 D。

    `d_lock_inlier`：覆盖 `solve_obb.D_LOCK_INLIER` 的阈值。**POSE 前端显式传
    0.30**（见 `detect_pose.POSE_D_LOCK_INLIER`）—— 那条前端在阈值降到 0 时
    可见角点距退化 26%（41.83 → 52.84px），是实测过的取舍，不能跟着 OBB 改。
    """
    if isinstance(d_lock, (int, float)) and not isinstance(d_lock, bool):
        return float(d_lock), None
    thr = D_LOCK_INLIER if d_lock_inlier is None else float(d_lock_inlier)
    want = (d_lock == 'depth') or (d_lock is None and depth is not None and len(use_pos) < 4)
    if not want or depth is None or quad is None:
        return None, None
    Dm, peak = _plane_dist_mm(n0, depth, quad, k)
    if Dm is None or peak < thr:
        return None, (None if Dm is None else float(peak))
    return float(Dm), float(peak)


def solve_frame(lines, k, normal, *, size_mm=(1200.0, 1000.0),
                long_side_parallel=False, depth=None, use_pos=None,
                d_lock=None, quad_hint=None, w_img=None, h_img=None,
                d_lock_inlier=None):
    """四条图像直线 + 法向 -> `frame`。返回 `(frame | None, diag)`。

    `lines` 长度 4，元素为 `{'p0': (2,), 'd': (2,) 单位向量, 'weight': float}`
    或 None；边序 `0,1,2,3` = 下/右/上/左（与 `order_quad` 的四角顺序一致，
    第 i 条边连接第 i 与第 (i+1)%4 个角）。

    `w_img` / `h_img` 只用于诊断（本函数不做可见性判断；可用性由调用方通过
    `use_pos` 给出，与 `frame_from_obb` 的 `usable_edges` 口径一致）。
    """
    W = float(min(size_mm))
    H = float(max(size_mm))
    normal = np.asarray(normal, float)
    n0 = normal / np.linalg.norm(normal)
    if float(n0 @ np.array([0.0, 0.0, 1.0])) < 0:
        n0 = -n0
    # ⚠️ 基用**未翻符号**的 normal 造，见模块头注释。
    e1, e2 = _basis(normal)

    if quad_hint is not None:
        quad = order_quad(quad_hint)
    else:
        quad = _quad_from_lines(lines)
    if quad is None:
        return None, {'reject': 'no_quad'}

    if use_pos is None:
        use_pos = [i for i in range(4) if lines[i] is not None]
    use_pos = [int(i) for i in use_pos]
    if not use_pos:
        return None, {'reject': 'no_usable_edge'}
    weights = [float(lines[i].get('weight', 1.0)) if lines[i] is not None else 1.0
               for i in range(4)]

    Dl, d_peak = _resolve_d_lock(d_lock, depth, quad, n0, k, use_pos,
                                 d_lock_inlier=d_lock_inlier)
    free = Dl is None

    # 量测平面：**用调用方给的四条直线**（不是从 quad 重算）。直线可能来自
    # POSE 关键点拟合，`quad_hint` 只负责初值/诊断；两者不一致时残差必须跟着
    # 直线走。某条线缺失时退回 quad 的那条边（该边通常也不在 `use_pos` 里）。
    pls = []
    for i in range(4):
        ln = lines[i] if lines[i] is not None else None
        if ln is not None:
            p0 = np.asarray(ln['p0'], float)
            d = np.asarray(ln['d'], float)
            pls.append(_line_plane_of(p0, p0 + d, k))
        else:
            pls.append(_line_plane_of(quad[i], quad[(i + 1) % 4], k))

    seed = _pnp_seed(quad, e1, e2, n0, k, W, H)
    if seed is None:
        d0, a0, b0, th0 = _fallback_seed(quad, e1, e2, k, long_side_parallel)
        seed_src = 'fallback'
    else:
        d0, a0, b0, th0 = seed
        seed_src = 'pnp'

    th_lo = th0 - np.radians(INPLANE_LIMIT_DEG)
    th_hi = th0 + np.radians(INPLANE_LIMIT_DEG)
    if free:
        x = np.array([d0, a0, b0, th0], float)
        lo = np.array([DEPTH_RANGE_MM[0], -AB_LIMIT_MM, -AB_LIMIT_MM, th_lo])
        hi = np.array([DEPTH_RANGE_MM[1], AB_LIMIT_MM, AB_LIMIT_MM, th_hi])
    else:
        x = np.array([a0, b0, th0], float)
        lo = np.array([-AB_LIMIT_MM, -AB_LIMIT_MM, th_lo])
        hi = np.array([AB_LIMIT_MM, AB_LIMIT_MM, th_hi])
    x = np.clip(x, lo, hi)

    def frame_of(v):
        if free:
            d, a, b, th = v[0], v[1], v[2], v[3]
        else:
            d, a, b, th = Dl, v[0], v[1], v[2]
        c, s = np.cos(th), np.sin(th)
        return dict(origin=d * n0 + a * e1 + b * e2,
                    E1=c * e1 + s * e2, E2=-s * e1 + c * e2,
                    nrm=n0, W=W, H=H)

    def residual(v):
        C = _corners_mm(frame_of(v), W, H)
        r = []
        for i in use_pos:
            w = weights[i]
            for p in (C[i], C[(i + 1) % 4]):
                r.append(float(pls[i] @ p) * w)
        return np.asarray(r, float)

    # 残差每一项对应哪条边的哪个角（`u,v` 是台面系坐标）：Newton 修正项要用。
    uv = [(0.0, 0.0), (W, 0.0), (W, H), (0.0, H)]
    res_uv = [(i, uv[j][0], uv[j][1], weights[i])
              for i in use_pos for j in (i, (i + 1) % 4)]

    def robust_cost(r):
        # scipy `loss='soft_l1'` 的 cost = 0.5*Σρ(f²)，ρ(z)=2(√(1+z)-1)。
        return float(np.sum(2.0 * (np.sqrt(1.0 + (r / SOFT_L1_F_SCALE) ** 2) - 1.0)))

    def irls_weights(r):
        return 1.0 / np.sqrt(1.0 + (r / SOFT_L1_F_SCALE) ** 2)

    def jacobian(v):
        """数值雅可比（中心差分），不含任何鲁棒权重。

        ⚠️ 权重不要在这里乘：Gauss-Newton 与 Newton 两种迭代用的权重次数不同
        （`w` vs `w³`），必须在组装 `A`/`g` 时按当前模式加。
        """
        n = len(v)
        r0 = residual(v)
        J = np.empty((len(r0), n))
        for j in range(n):
            h = _STEP_RAD if (j == len(v) - 1) else _STEP_MM
            vp = v.copy()
            vm = v.copy()
            vp[j] = min(v[j] + h, hi[j])
            vm[j] = max(v[j] - h, lo[j])
            dv = vp[j] - vm[j]
            if abs(dv) < 1e-15:
                J[:, j] = 0.0
            else:
                J[:, j] = (residual(vp) - residual(vm)) / dv
        return J

    iterations = 0
    cost = None
    lam = 1e-9
    for iterations in range(1, MAX_ITER + 1):
        r = residual(x)
        w = irls_weights(r)                          # soft_l1: w=(1+(r/s)²)^(-1/2)
        cost = robust_cost(r)
        J = jacobian(x)
        # Gauss-Newton 用 w，Newton 用损失的二阶导 w³；梯度始终是 Jᵀ(w·r)。
        expo = 1 if iterations <= GN_ITERS else 3
        while True:
            try:
                A = J.T @ (J * (w ** expo)[:, None])
                g = J.T @ (w * r)
                if expo == 3:
                    # Newton 的完整 Hessian 还要加 `Σ ρ'(rₖ)·∂²rₖ/∂θ²`。
                    # 矩形角 C = origin + u·E1 + v·E2，而
                    # ∂²C/∂θ² = -(u·E1 + v·E2)，且与 a/b/d 的混合导数为 0，
                    # 所以只有 (θ,θ) 这一项要修。缺了它收敛是线性的（实测
                    # 8 次后仍差 0.5~1px）；加上后同一帧 8 次内到 1e-2px。
                    th = x[-1]
                    c, s = np.cos(th), np.sin(th)
                    E1 = c * e1 + s * e2
                    E2 = -s * e1 + c * e2
                    h = np.array([wt * float(pls[i] @ (-u * E1 - v * E2))
                                  for i, u, v, wt in res_uv])
                    A[-1, -1] += float(np.sum(w * r * h))
                    # 修正项可能让 Hessian 不定（平坦方向 + 大残差）。此时
                    # 加大 Levenberg 阻尼到最小特征值之上，保持 Newton 方向
                    # 仍是下降方向。**不要直接丢掉修正项退回 GN** —— 实测那会
                    # 让个别帧（`10-10-14-22-38_01520`）8 次迭代后仍差 0.8px。
                    ev = np.linalg.eigvalsh(A)
                    if ev[0] <= 0.0:
                        lam = max(lam, -float(ev[0]) + 1e-9 * max(1.0, float(ev[-1])))
                delta = np.linalg.solve(A + lam * np.eye(len(x)), -g)
                break
            except np.linalg.LinAlgError:
                lam *= 10.0
                if lam > 1e6:
                    delta = None
                    break
        if delta is None:
            break
        # 混合单位（毫米 / 弧度）的单步上限：角度按 1 rad = 1000 计。
        scale = np.ones(len(x))
        scale[-1] = 1000.0
        nd = float(np.max(np.abs(delta) * scale))
        if nd > MAX_STEP:
            delta *= MAX_STEP / nd
        t = 1.0
        xn = x
        cn = cost
        while t > 1e-8:
            cand = np.clip(x + t * delta, lo, hi)
            cc = robust_cost(residual(cand))
            if cc < cost:
                xn, cn = cand, cc
                break
            t *= 0.5
        if cn >= cost:                               # 没有下降方向，收敛/卡住
            break
        x = xn
        lam = max(lam / 3.0, 1e-12)
        if (cost - cn) <= 1e-14 * max(1.0, cost):
            break

    f = frame_of(x)
    found = _to_spec(f, W, H, k)                    # 出口约定与现行一致
    px = _project_px(_corners_mm(found, W, H), k)
    reproj = float(np.median(np.linalg.norm(px - quad, axis=1)))
    depth_mm = float(x[0] if free else Dl)
    inplane = float(np.degrees(x[-1]))
    cost = robust_cost(residual(x))                  # 出口处的代价（不是上一步的）
    return found, {
        'method': 'edges4dof',
        'use_pos': list(use_pos),
        'd_lock': None if free else round(depth_mm, 3),
        'd_lock_src': ('forced' if isinstance(d_lock, (int, float))
                       and not isinstance(d_lock, bool)
                       else ('depth' if not free else None)),
        'd_peak': None if d_peak is None else round(d_peak, 3),
        'reproj_px': round(reproj, 4),
        'depth_mm': round(depth_mm, 3),
        'inplane_deg': round(inplane, 4),
        'iterations': int(iterations),
        'seed': seed_src,
        'cost': None if cost is None else round(cost, 3),
        'robust_loss': 'soft_l1',
    }
