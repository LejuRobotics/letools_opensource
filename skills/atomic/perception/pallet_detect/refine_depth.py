#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""细化的最终版：`refine3` 的两级 + 一级**深度给定平面距离 D** 的角点级。

    from pallet_detect.refine_depth import refine4
    f5, diag = refine4(f4, color, k, depth=fr['depth'], gmag=grad_mag(color))

## 三级

**1 脊线级**（= `refine2`）：软规则 `argmax mag·exp(-tt²/2σ²)` 量四条边线，
2 条边以上解 6 自由度（少边时窗口 ±8mm/±1°）。14.62 → 8.05px。

**2 角点级**（= `refine3`）：相邻两条量到的边线**求交**得角点，2 个角点解 4 自由度。
角点不能用 Harris/Shi-Tomasi（木托盘角上中位差 13.4px），求交只有 4.29px。
8.05 → 6.13px。

**3 深度级**（本文件新增）：把**平面距离 D 由深度定死**（框内有效深度点沿法向的
稳健中位，实测精度 ~6.8mm），未知量从 4 个降到 **3 个**（面内 a/b + 转角 θ）。
于是 **1 个角（2 个约束）+ 2 条边方向（2 个约束）= 4 个约束解 3 个未知量，超定** ——
之前"1 角 + 2 方向"的秩亏消失了（E70：那种配方的尺度无关条件数 >200，
深度方向不可观测；E76：加上 D 之后中位 135.8 → **6.22px**）。

## ⚠️ 两个单位/符号的坑（都会静默失败）

* **D 的单位**：深度反投影出来的点用**米**，解算器的 `d` 与 `origin` 用**毫米**。
  不乘 1000 等于把台面放到离相机 0.9mm 处 —— 残差照样小，角点错 1300px。
* **法向符号**：`nrm` 是台面外法向（z<0），必须翻成 z>0 才能当"从相机指向场景"
  的参数化；否则平面落到相机背后，残差照样小、角点是**镜像**的
  （与 `solve_obb` 文件头 ⚠️ 第 1 条同一个坑）。

## 已知不足

* θ 的搜索窗口 ±`solve_obb.INPLANE_LIMIT_DEG`（**跟着那个常数走**），窗口必须**以初值
  自己的 θ 为中心**；写死以 0 为中心会把已偏出窗口的帧解不回来（实测 0.00 → 141px）。
* 深度只在"框内有一层主导平面"时可用。框里主要是地面/把手时 D 会错，
  这时**脊线级的支持度闸门**会把它挡掉（不收）。
"""
from __future__ import annotations

import numpy as np

from .geom import _corners_mm, _project_px
from .solve_obb import PALLET_SIZE_MM
from . import solve_obb as SO

from scipy.optimize import least_squares

from . import refine_corner as R3
from . import refine_ridge as R2
from . import dsearch
from . import edge_trust
from . import user_rule
from .refine_ridge import grad_mag, drop_edges, measure_lines, signed_grad, support

MAX_INT_OFF = R3.MAX_INT_OFF
# D 的**闸门**：框内深度主峰的占比（见 `solve_obb._plane_dist_mm`）。
#
# ⚠️ 这个数是**峰占比**（选中峰 bin 的点数 / 全部有效点数），不是旧的"内点率"，
# 两者**不可比**（旧值中位 0.594、新值中位 ~0.45）。
#
# 2026-10-19：`solve_obb.D_LOCK_INLIER` 改成 0（真机 881 帧：该锁没锁贡献 89% 大跳），
# 本别名跟着变 0 —— 即**深度级永远用深度量到的 D**，不再有 `low_inlier` 早退。
# 真机带 refine 的全链路实测为净胜（见 `maduo/debug/jitter/WORKLOG_jitter.md` §12）。
#
# 唯一真相源在 `solve_obb.D_LOCK_INLIER`（`refine3.D_DEPTH_INLIER` 同样是它的别名）。
D_INLIER_MIN = edge_trust.D_INLIER


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def plane_dist_mm(n, k, depth, box, border=6):
    """框内有效深度点沿 `n` 的**主峰中位距离**（毫米）。返回 (D_mm, 峰占比)。

    与 `solve_obb._plane_dist_mm` 是**同一套算法**（那边内联一份是为了不让
    `solve_obb` 反过来依赖 refine*；这一边直接转发，保证两处永远一致 ——
    以前两份各写一遍，改一处忘一处就会静默不一致）。
    """
    return SO._plane_dist_mm(n, depth, box, k, border=border)


def _intersect(l1, l2):
    A = np.array([[l1[0], l1[1]], [l2[0], l2[1]]], float)
    b = np.array([-l1[2], -l2[2]], float)
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    return np.linalg.solve(A, b)


def _obb_edge_normals(obb_box):
    """OBB 框四条边的单位法向（按 `solve_obb.order_quad` 的边序 0..3）。退化边给 `None`。"""
    out = []
    if obb_box is None:
        return [None] * 4
    q = SO.order_quad(np.asarray(obb_box, float).reshape(4, 2))
    for i in range(4):
        d = q[(i + 1) % 4] - q[i]
        L = float(np.linalg.norm(d))
        out.append(None if L < 1e-9 else np.array([-d[1], d[0]]) / L)
    return out


def _trusted_edges(frame, k, img=(640.0, 480.0)):
    """`frame` 投影出来的四条边里**可信**的那些。

    判据与 `solve_obb.frame_from_obb` 的 `use_pos` **同一套**（`solve_obb.edge_usable`：
    画面内占比 + 内缩安全占比 + 邻边共同出画），只是量的是**这个 frame 自己**的边：
    初值拟合量的是 OBB 的边，到这里两者都已经是同一个矩形的估计，边的可信与否
    不因中间两级细化而改变。

    ⚠️ **必须与 `frame_from_obb` 走同一个入口**。两处各写一份的代价实测过：
    只改 `frame_from_obb` 而这里照旧，601 帧的最终输出**一帧都不变** —— 细化会把
    初值重新拉回去，判据改了等于没改。
    """
    px = _project_px(_corners_mm(frame, float(frame['W']), float(frame['H'])), k)
    return SO.usable_edges(px, img[0], img[1])


def _make_frame(f0, n0, e1, e2, W, H, D):
    """D 给定 -> 3 自由度 (a, b, θ)；D=None -> 4 自由度。返回 (frame_of, x0, lo, hi, xs)。"""
    t0 = np.asarray(f0['origin'], float)
    d0, a0, b0 = np.linalg.solve(np.stack([n0, e1, e2], axis=1), t0)
    E1_0 = np.array(f0['E1'], float)
    th0 = float(np.arctan2(E1_0 @ e2, E1_0 @ e1))
    # ⚠️ **跟着 `solve_obb.INPLANE_LIMIT_DEG` 走，不要写死** —— 2026-10-15 把那个
    # 常数从 25° 放到 40°，这里写死就意味着"detect 放开了、refine 还夹着"：
    # 细化会把 detect 已经解对的角度**重新拉回旧窗口内**，而且两处日志都正常。
    lim = np.radians(SO.INPLANE_LIMIT_DEG)
    fixed_d = D is not None

    def frame_of(p):
        if fixed_d:
            a, b, th = p
            d = D
        else:
            d, a, b, th = p
        c, s = np.cos(th), np.sin(th)
        return dict(origin=d * n0 + a * e1 + b * e2,
                    E1=c * e1 + s * e2, E2=-s * e1 + c * e2, nrm=n0, W=W, H=H)

    if fixed_d:
        return frame_of, np.array([a0, b0, th0]), \
            [-1500.0, -1500.0, th0 - lim], [1500.0, 1500.0, th0 + lim], [200., 200., .05]
    return frame_of, np.array([max(d0, 300.0), a0, b0, th0]), \
        [300.0, -1500.0, -1500.0, th0 - lim], [4000.0, 1500.0, 1500.0, th0 + lim], \
        [300., 200., 200., .05]


def _depth_stage(f0, k, gmag, depth, obb_box, gxy=None, D_override=None,
                 d_range=None, edges=None):
    """第 3 级：D 由深度定死，1 个角 + 2 条边方向（也可退化成 2 个角）。

    `edges`：**可信边**的索引（画面内长度占比 >= `solve_obb.EDGE_MIN_VIS` 的边）。
    `None` = 由本函数用 `solve_obb._vis_frac` 对 `f0` 的四条边自算。只在
    "一条脊线都量不到"的兜底分支里用到（见那里的注释）。
    """
    W, H = float(f0['W']), float(f0['H'])
    n0 = _unit(np.asarray(f0['nrm'], float))
    if float(n0 @ np.array([0., 0., 1.])) < 0:
        n0 = -n0                                       # 见文件头 ⚠️
    D, inl = plane_dist_mm(n0, k, depth, obb_box)
    if d_range is not None and D_override is None:
        D = dsearch.clamp_D(D, d_range)     # 范围只夹、不搜
        if D is None:
            return f0, {'stage': 'no_depth'}
    if D_override is not None:
        # 脊线级的 D 范围搜索已经给了自洽的 D，它比深度图的单帧测量更可信
        # （OBB 初值在这类机位稳定偏小，而搜索是拿四条边的脊线支持度投票的）。
        D, inl = float(D_override), float(inl or 0.0)
    if D is None:
        return f0, {'stage': 'no_depth'}
    if D_override is None and inl < D_INLIER_MIN:
        return f0, {'stage': 'low_inlier', 'D_mm': round(D, 1), 'inlier': round(inl, 3)}
    # ⚠️ 试过两种"自洽性"门控，**都不能用**（E91/E92，601 帧）：
    #   * 框**内**深度自己拟合的法向 vs n0：错帧 23.5° vs 好帧 21.2° —— 分不开
    #     （框内有大量非台面点，拟合本身就不可靠）；以 <=5° 门控会挡掉 95% 好帧。
    #   * 框**外**（地面）拟合的法向 vs n0：错帧 6.05° vs 好帧 7.03° —— 也分不开。
    # 结论：**内点率是这里唯一能分的量**，就用它。
    lines = drop_edges(measure_lines(f0, gmag, k, gxy=gxy))
    n_lines = sum(1 for l in lines if l is not None)
    P0 = _project_px(_corners_mm(f0, W, H), k)
    h, w = gmag.shape
    corners = {}
    for i in range(4):
        l1, l2 = lines[(i - 1) % 4], lines[i]
        if l1 is None or l2 is None:
            continue
        p = _intersect(l1, l2)
        if p is None or not (0 <= p[0] < w and 0 <= p[1] < h):
            continue
        if float(np.linalg.norm(p - P0[i])) > MAX_INT_OFF:
            continue
        corners[i] = p
    dirs = [None] * 4
    for i, l in enumerate(lines):
        if l is not None:
            dirs[i] = _unit(np.array([l[0], l[1]], float))

    R0 = np.stack([f0['E1'], f0['E2'], f0['nrm']], axis=1)
    e1 = np.array(R0[:, 0], float)
    e2 = np.array(R0[:, 1], float)
    frame_of, x0, lo, hi, xs = _make_frame(f0, n0, e1, e2, W, H, D)
    frame_of_free, x0f, lof, hif, xsf = _make_frame(f0, n0, e1, e2, W, H, None)

    def solve(corner_idx, use_dirs, free):
        fo, xx = (frame_of_free, x0f) if free else (frame_of, x0)
        ll, hh, xss = (lof, hif, xsf) if free else (lo, hi, xs)

        def res(p):
            C = _project_px(_corners_mm(fo(p), W, H), k)
            r = []
            for i in corner_idx:
                r.append(float(C[i][0] - corners[i][0]))
                r.append(float(C[i][1] - corners[i][1]))
            for i in use_dirs:
                if dirs[i] is None:
                    continue
                # ⚠️ 方向残差必须**按边长归一**。`nv @ (C[i]-C[i+1])` 是
                # "边长 × 角度误差"：长边 400px、短边 300px，同样一个小角度在
                # 长边上的残差天然大几十倍，于是 3 自由度解 4 个残差时，长边的
                # 方向把**角点位置**挤掉了 —— 表现就是"求交角（橙圈）比拟合出的
                # 绿角更贴真值"（E87：3.15px vs 4.12px，求交角更准的占 56%）。
                # 单位化后残差是无量纲的 sin 夹角，两边等权。
                v = C[i] - C[(i + 1) % 4]
                v = v / max(float(np.linalg.norm(v)), 1e-9)
                r.append(float(dirs[i] @ v))
            return np.asarray(r)
        if not res(xx).size:
            return None
        try:
            s = least_squares(res, xx, loss='soft_l1', f_scale=3.0, max_nfev=300,
                              bounds=(ll, hh), x_scale=xss)
        except Exception:                              # noqa: BLE001
            return None
        f = fo(s.x)
        P = _project_px(_corners_mm(f, W, H), k)
        return f if np.all(np.isfinite(P)) else None

    # ⚠️ **0~1 条脊线时不能直接放弃**（旧写法是 `return f0, {'stage': 'no_lines'}`）。
    #
    # 脊线采没采到**不等于**这个 frame 好不好：托盘整块出画、木纹与地面同色、角上
    # 纹理不共线……都会让 `measure_polar_aperture` 一条边都交不出来（本数据集 601
    # 帧里 326 帧就是 0~1 条）。旧写法下这些帧**深度级整段被跳过**，D 原样继承初值
    # —— 而初值的 D 恰恰是最不可信的那一个。
    #
    # 兜底：**锁 D + 只用可信边解面内 (a,b,θ)**。D 已由深度定死（上面两道闸门都过
    # 了才走到这里），未知量只剩 3 个；矩形边长是硬输入，所以**两条可信边**
    # （4 个端点约束，OBB 的矩形边必平行，4 个约束有 1 个冗余）能把 (a,b,θ)
    # 定到"贴着这两条边、尺寸不变"的姿态。只有一条可信边时不强行解（2 个约束定
    # 3 个未知量，θ 会沿边自由滑），直接退回初值。
    #
    # 方向残差用 **OBB 框自己的边方向 + 量到的脊线方向**（有脊线就用脊线，那是
    # 真边的方向；没有才退回 OBB 的边法向，与 `user_rule` 在没量到脊线时的做法
    # 一致）。位置残差用**边所在直线**（过可信边两端投影点的图像直线）。
    #
    # ⚠️ "哪条边可信"用 `solve_obb._vis_frac` 判（`edges` 参数或就地算），
    # **不用"脊线采没采到"** —— 后者正是上面说的那个错判据。
    if n_lines < 2:
        use = sorted({int(i) for i in
                      (edges if edges is not None else _trusted_edges(f0, k))
                      if 0 <= int(i) < 4})
        if len(use) < 2:
            return f0, {'stage': 'no_lines', 'D_mm': round(D, 1),
                        'inlier': round(inl, 3), 'n_lines': n_lines,
                        'edges_used': use}
        nvs = _obb_edge_normals(obb_box)               # 方向的后备：OBB 框的边
        for i in use:                                  # 有脊线方向就用脊线方向
            if dirs[i] is None and nvs[i] is not None:
                dirs[i] = nvs[i]

        s_e, _ = support(f0, gmag, k)
        f_e = solve([], use, False)                    # ★ 复用同一套最小二乘
        if f_e is not None:
            sc_e, _ = support(f_e, gmag, k)
            if sc_e > s_e + 1e-9:
                return f_e, {'stage': 'depth_refined', 'D_mm': round(D, 1),
                             'inlier': round(inl, 3), 'recipe': '可信边(D)',
                             'edges_used': use, 'n_lines': n_lines,
                             'init_score': round(s_e, 4),
                             'final_score': round(sc_e, 4)}
        return f0, {'stage': 'no_lines', 'D_mm': round(D, 1), 'inlier': round(inl, 3),
                    'edges_used': use, 'n_lines': n_lines}
    s0, _ = support(f0, gmag, k)
    cands = []
    # ⚠️ 优先"2~3 个角"(4~6 个约束解 3 个未知量)。**"1 角 + 2 方向"只在确实只有
    # 一个角时才用** —— 它虽然超定（2+2=4 > 3），但 a/b 只由那一个角定，方向上
    # 的一点点误差会直接变成平移漂移。实测 `2026-09-30-11-04-26` 那包：85 帧里
    # 46 帧走了 1角+2方向，refine3 的 3.53px 被拖到 6.96px（单帧最多 3.87 -> 11.04）。
    if len(corners) >= 2:
        ii = sorted(corners)[:3]
        f = solve(ii, [], False)
        if f is not None:
            cands.append(('多角(D)', f))
    for ci in corners:
        f = solve([ci], [(ci - 1) % 4, ci], False)
        if f is not None:
            cands.append(('1角+2方向(D)', f))
    best = None
    for name, f in cands:
        sc, _ = support(f, gmag, k)
        if best is None or sc > best[0]:
            best = (sc, f, name)
    if best is None:
        return f0, {'stage': 'solve_fail', 'D_mm': round(D, 1), 'inlier': round(inl, 3)}
    if best[0] <= s0 + 1e-9:
        return f0, {'stage': 'kept_init', 'D_mm': round(D, 1), 'inlier': round(inl, 3),
                    'init_score': round(s0, 4)}
    return best[1], {'stage': 'depth_refined', 'D_mm': round(D, 1),
                     'inlier': round(inl, 3), 'recipe': best[2],
                     'init_score': round(s0, 4), 'final_score': round(best[0], 4)}


def refine4(frame0, color, k, depth=None, gmag=None, obb_box=None,
            size_mm=PALLET_SIZE_MM, d_range=None):
    """4 自由度初值 -> 脊线级 + 角点级 + 深度级。返回 `(frame', diag)`。

    `d_range=(lo, hi)`：**只在这个范围内搜平面距离 D**（毫米）。退化帧
    （只剩两条相邻边）靠它定住"托盘有多远/多大"；给了范围就以搜索为准，
    不给则退回"锁 OBB 初值的 d0"。也可设 `dsearch.D_RANGE_MM` 全局生效。
    """
    diag = {}
    if gmag is None:
        gmag = grad_mag(color)
    s0, _ = support(frame0, gmag, k)

    old_rng = R3.D_RANGE_MM
    if d_range is not None:
        R3.D_RANGE_MM = d_range
    from . import dsearch as DS
    try:
        f1, d1 = R3.refine3(frame0, color, k, gmag=gmag,
                            depth=depth, obb_box=obb_box)
    finally:
        R3.D_RANGE_MM = old_rng
    _sd = getattr(R3._ridge_stage, 'last_D', None)
    _rng = d_range if d_range is not None else DS.D_RANGE_MM
    if _sd is not None:
        diag['D_ridge'] = _sd      # 脊线级在范围内挑出来的 D（退化帧才有）
    s1, _ = support(f1, gmag, k)
    diag['ridge_corner'] = d1
    cur = f1

    if depth is not None and obb_box is not None:
        f2, d2 = _depth_stage(cur, k, gmag, depth, obb_box,
                             gxy=signed_grad(color), D_override=_sd,
                             d_range=_rng, edges=_trusted_edges(cur, k))
        s2, _ = support(f2, gmag, k)
        if s2 > s1 + 1e-9:
            cur = f2
        diag['depth'] = d2
    diag['init_score'] = round(s0, 4)
    diag['final_score'] = round(support(cur, gmag, k)[0], 4)
    diag['stage'] = ('depth_refined' if cur is not f1
                     else d1.get('stage', 'init'))
    return cur, diag


# --------------------------------------------------------------------------- #
# 对外入口（签名与旧 `refine.py` 逐字兼容）
# --------------------------------------------------------------------------- #
#: 旧 `refine.py` 的同名常量（`payload.DEFAULT_TARGET_MM` 同一个值）。
#:
#: ⚠️ **`refine_pallet_frame(target_mm=…)` 不是死参数** —— 它一路传给
#: `user_rule.frame_user_rule(size_mm=…)`，那是"用户规则"解算矩形的**唯一尺寸来源**。
#: 这个默认值只在**离线脚本 / 单测**没传时生效（真机上节点传板子的
#: `pallet_size_mm`）。与 `solve_obb.PALLET_SIZE_MM` 是同一个量，改要一起改。
#:
#: 脊线级 / 角点级 / 深度级三级（`refine4` 往下）**不用**它 —— 那三级一律取
#: `frame['W']` / `frame['H']`（由 `frame_from_obb` 按同一个尺寸定出来）。
DEFAULT_TARGET_MM = (1200.0, 1000.0)


def refine_pallet_frame(frame, color, depth, k, *,
                        target_mm=DEFAULT_TARGET_MM, opts=None):
    """**用户规则开关 → 深度级细化**。返回 `(frame', diag)`。

    ⚠️ **输入合法时永远有 frame，不返回 `None`。** 理由：节点在 refine 返回
    `None` 时**不发布**（把 refine 的拒绝当成整帧失败），而细化没成就该退回输入
    的位姿、照常发布 —— 交回 `None` 等于凭空丢帧。所以下面每一条失败路径都
    **退回 `frame` 本身**，把落点写进 `diag`。

    链路（与 maduo 侧出图脚本 `render_user_rule.py` 的 `one()` **逐字同序**）：

        f0, dg = user_rule.frame_user_rule(obb, k, normal, depth, thr, color=color,
                                           size_mm=target_mm)
                 # 用户规则：D 异常 + 出画时，改用"1 个可见角 + 两条相邻边方向"
        f4, d4 = refine_depth.refine4(f0, color, k, depth, gmag, obb_box=...)
                 # 脊线级 + 角点级 + 深度级

    ⚠️ **`target_mm` 只有用户规则那条支路用得到**（它的矩形边长全部来自这个尺寸，
    不从图像反算），三级细化取的是 `frame['W']` / `frame['H']`。从前这里**没有把
    `target_mm` 传下去**，于是用户规则永远吃 `solve_obb.PALLET_SIZE_MM` 那个写死的
    常量 —— 板子上的 `pallet_size_mm` 只影响了 `detect` 那条路，换托盘尺寸时
    用户规则会**静默按旧尺寸**解。现在传下去了。

    `obb` 从 `frame['_obb_corners_px']` 取（`detect_pallet_frame` 放进去的 OBB
    四角）；取不到就**跳过用户规则开关**、只跑 `refine4`（`obb_box=None`，退化帧
    的 D 锁退回 OBB 初值的 d0）—— 这样手工构造的 frame 也能细化，不会断链。

    `opts` 可带：`gmag`（省一次梯度计算）/ `thr`（用户规则阈值，默认
    `user_rule.D_ANOMALY`）/ `d_range`（平面距离搜索范围，毫米）。
    旧路线的 `plane` / `edge_support_min` 收下不用（新路线法向锁死、无平面细化）。
    """
    opts = dict(opts or {})
    if frame is None:
        return None, {'reject': 'no_input_frame'}
    # ⚠️ **每帧开头清一次帧内记忆化**（`measure_lines` / `support` 的缓存，
    # 见 `refine_ridge._MEMO`）。不清的话缓存会**无界增长** —— 上一帧的 `gmag`
    # 虽然在缓存项里存着对象引用不会误命中，但条目会一直堆着。
    R2.begin_frame()
    gmag = opts.get('gmag')
    if gmag is None:
        gmag = grad_mag(color)
    obb_corners = frame.get('_obb_corners_px')
    obb = ({'corners_px': np.asarray(obb_corners, float)}
           if obb_corners is not None else None)

    diag = {'user_rule': None, 'plane': {'refined': False,
            'note': 'OBB 路线法向锁死，不做平面细化'}}
    f0 = frame
    if obb is not None:
        try:
            f_u, dg = user_rule.frame_user_rule(
                obb, k, np.asarray(frame['nrm'], float), depth,
                float(opts.get('thr', user_rule.D_ANOMALY)), color=color, gmag=gmag,
                size_mm=target_mm)
        except Exception as exc:                     # noqa: BLE001
            f_u, dg = None, {'mode': 'error', 'why': f'{type(exc).__name__}: {exc}'}
        diag['user_rule'] = dg
        if f_u is not None:
            f0 = f_u                                 # 触发则用规则解，否则 = 初值

    f4, d4 = refine4(f0, color, k, depth=depth, gmag=gmag,
                     obb_box=(None if obb is None else obb['corners_px']),
                     d_range=opts.get('d_range'))
    diag['refine'] = d4
    diag['refined'] = bool(f4 is not f0)
    out = f4 if f4 is not None else frame

    # ★ **出口必须重新归一一次手性**（2026-10-14 修）。
    #
    # `_depth_stage` 开头把法向翻到 `z>0`（那是对的 —— `d·n` 这个参数化要求
    # n 指向相机前方，见 `solve_obb` 文件头第 1 条），但它的 `_make_frame` 里
    # `E1/E2` 用的是**没翻的**基，于是输出 `E1×E2 = −nrm`、**det = −1**。
    # 而三级细化中间那几段（`refine3` / `refine2`）只动 R，**都不再跑
    # `_to_spec`** —— 只有 `detect` 那条路和 `user_rule` 那条路会跑。
    # 结果：**只要深度级改写成功，这一帧就会以 det=−1 发出去**，被
    # `NodePalletObs` 的 `require_handedness` 整帧拒掉，现场看到的是
    # "detect 单独跑有输出、一键脚本一个数都没有"。
    #
    # `_to_spec` 是唯一"只摆 E1/E2 的符号与 origin、矩形一个像素都不动"的
    # 归一函数（601 帧不变量实测 <= 7e-13 px），正是这里要的。
    # **放在出口而不是各段内部**：中间的中间态不经消息契约，多归一几次只是白算。
    if out is not None:
        out = SO._to_spec(out, float(out['W']), float(out['H']), k)
    return out, diag
