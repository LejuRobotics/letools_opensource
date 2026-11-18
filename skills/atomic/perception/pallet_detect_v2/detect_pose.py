#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**POSE 初值层**：YOLO11-pose 推理 → 四边直线 → `solve_edges.solve_frame` → frame。

入口::

    from pallet_detect.detect_pose import detect_pallet_frame_pose
    frame, diag = detect_pallet_frame_pose(color, depth, k, normal=nrm)

链路：

    detect._to_input_tensor                 BGR -> (1,3,640,640)（**直接复用**，不抄）
    pose_decode.decode_pose / nms_pose / pick_pose   ONNX -> 12 关键点
    pose_decode.fit_edges                   每条边 2~4 点加权 TLS -> 4 条直线
    pose_decode.intersect_edges             相邻边求交 -> quad（失败时逐角兜底）
    frame_geometry.usable_edges                  OBB 的保守可用边口径 -> use_pos
    solve_edges.solve_frame                 四条直线 + 锁法向 -> 4-DOF frame

⚠️ `d_lock` 默认 `'depth'`：POSE 路线默认把 D 锚到深度主峰（计划 §3.5），
因为实机 bag 已证明几何解出的 D 不稳。`opts['d_lock']` 可覆盖
（`None` = 只在可用边不足 4 条时锁、数值 = 强制该 D）。

⚠️ `quad` **只用于** `solve_frame` 的 `quad_hint`（PnP 初值/诊断）以及挂到
`frame['_quad_px']` 供诊断出图复用；最终位姿由四条边解出。

模型（**硬失败，不做任何回退**）：`opts['session']` → `opts['model_path']` →
本包 `models/pallet_pose.onnx`。缺失/加载失败直接抛异常，错误信息写清路径与
导出命令。
"""
from __future__ import annotations

from pathlib import Path

import time

import numpy as np

from . import pose_decode as PD
from . import solve_edges as SE
from . import frame_geometry as SO
from .dsearch import D_from_frame
from .preprocess import DEFAULT_CONF, DEFAULT_IMGSZ, DEFAULT_NMS_IOU, _to_input_tensor
from .frame_geometry import _basis, order_quad, usable_edges
from .geom import _corners_mm, _project_px

DEFAULT_POSE_MODEL_PATH = (Path(__file__).resolve().parent
                           / 'models' / 'pallet_pose.onnx')
# TensorRT engine（与 .onnx 同目录，**不进 git**）。默认 auto：engine 在就用它。
# ⚠️ POSE 默认用 **FP32**（`--noTF32`）：FP16 engine 的关键点亚像素差会被四边
# 求交放大（10 帧实测角点差中位 1.4px、最大 22px）；FP32 与 ONNX 差 ≤0.02px。
DEFAULT_POSE_ENGINE_PATH = (Path(__file__).resolve().parent
                            / 'models' / 'pallet_pose_fp32.engine')
_BACKENDS = ('auto', 'engine', 'onnx')

_DEFAULT_CONF = 0.25
_DEFAULT_NMS_IOU = 0.45
_DEFAULT_KCONF = 0.5
_DEFAULT_LINE_RES_PX = 12.0

#: partial 帧的可见边/平面几何一致性闸门。三条可见边（bottom/right/left）
#: 反投影到 `{n0·P=D}` 后必须像同一块 `min(size_mm)` 宽的矩形：
#:   * `|bl-br| / min(size_mm)` 在 `[0.75, 1.35]`；
#:   * bottom 与 right 夹角与 90° 差 <= 10°；
#:   * right 与 left 夹角 <= 10°（对边应平行）。
#: bag2 模型三边在 record D=1316.6mm 下 ratio≈2.05、B-R=73.0°、R-L=40.2°，
#: 会被这里拒掉；合成真三边 ratio≈1.0、B-R≈90°、R-L≈0°。
EDGE_PLANE_RATIO_RANGE = (0.75, 1.35)
EDGE_PLANE_RIGHT_ANGLE_TOL_DEG = 10.0
EDGE_PLANE_PARALLEL_TOL_DEG = 10.0

#: 两条相邻边单独做约束时，平面内反投影夹角与 90° 的最大允许偏差。
#: 两条边只剩"共享角 + 两边方向"的约束；对边（不共享角）会沿边线方向留下
#: 一个自由度（实测 510 / 2126mm），因此只接受相邻边。
EDGE_PAIR_ANGLE_TOL_DEG = 15.0

#: 边可靠性评分参数（仅 partial 帧选两条相邻边时使用）。
#:   * 贴边：参与拟合的关键点到图像边界的最小距离 < 5%·min(W,H) 时降权，
#:     越贴边分数越低；完全贴边仍保留 `EDGE_BORDER_MIN_FACTOR` 的底分；
#:   * 线残差：以 `EDGE_LINE_RES_REF_PX` 归一化，残差越大分越低；
#:   * 中心：边中点到图像中心的距离越近分越高；
#:   * 单边/边对可信分下限：低于则丢弃该边 / 判无可靠相邻对。
EDGE_BORDER_NEAR_FRAC = 0.05
EDGE_BORDER_MIN_FACTOR = 0.15
EDGE_LINE_RES_REF_PX = 2.0
EDGE_SCORE_MIN = 0.12
EDGE_PAIR_SCORE_MIN = 0.25

#: POSE 前端锁 D 的**主峰占比**阈值。
#:
#: ⚠️ **与 OBB 故意不同**（OBB 用的是 `frame_geometry.D_LOCK_INLIER = 0.0`）。
#: 真机 881 帧实测（`maduo/debug/jitter/WORKLOG_jitter.md` §7）：
#:
#:      阈值 | Δpos 中位 | max    | >100mm | 可见角点距
#:      0.30 | 41.36     | 532.6  | 149    | **41.83px**
#:      0.00 | 27.43     | 410.3  | 110    | **52.84px**  ← 抖动更好，精度差 26%
#:
#: 即 **POSE 上是取舍**：锁得更多 → 抖动小，但多一个约束去拽那 1~2 个可见角，
#: 角点距反而变大。板子上现在 `pallet_detector=pose`，所以**默认保持 0.30**
#: （精度优先）。要把这条前端也切到抖动优先，改这个常量即可，不要动
#: `frame_geometry.D_LOCK_INLIER`（那是 OBB 的）。
POSE_D_LOCK_INLIER = 0.30

#: 角索引 -> (相邻边 A, 相邻边 B, 直接预测的关键点下标)。
#: 角序 [bl, br, tr, tl]，边序 0,1,2,3 = 下/右/上/左。
_CORNER_FALLBACK = (
    (3, 0, 0),   # bl = edge3 × edge0 ; 关键点 0
    (0, 1, 3),   # br = edge0 × edge1 ; 关键点 3
    (1, 2, 6),   # tr = edge1 × edge2 ; 关键点 6
    (2, 3, 9),   # tl = edge2 × edge3 ; 关键点 9
)


def _measure_size_from_edges(lines, k, normal, D_mm):
    """从**四条拟合边线**反算台面尺寸（mm），不依赖 target_mm / 单个角点。

    每条边由 2~4 个 POSE 关键点加权 TLS 拟合（`fit_edges` 已做离群剔除）。
    这里把两条**对边**都反投影到台面平面，再沿边采样若干个点、取点到对边
    距离的**中位数** —— 单个关键点/单个角点偏了，不会把整个尺寸带偏。

    返回 `[W, H]`（`W=min`、`H=max`，与 solver 的尺寸口径一致）；任一对边
    不可用或反投影退化返回 `None`。
    """
    try:
        n = np.asarray(normal, np.float64).reshape(3)
        n_norm = float(np.linalg.norm(n))
        D = float(D_mm)
        if n_norm < 1e-9 or not np.isfinite(D) or D <= 0.0:
            return None
        n = n / n_norm
        if float(n[2]) < 0.0:
            n = -n
        K_inv = np.array([
            [1.0 / k.fx, 0.0, -k.cx / k.fx],
            [0.0, 1.0 / k.fy, -k.cy / k.fy],
            [0.0, 0.0, 1.0],
        ], np.float64)

        def backproject(uv):
            ray = K_inv @ np.array([uv[0], uv[1], 1.0], np.float64)
            denom = float(n @ ray)
            if abs(denom) < 1e-12:
                return None
            return ray * (D / denom)

        def line3d(ln):
            if ln is None:
                return None
            p0 = np.asarray(ln.p0, np.float64)
            d = np.asarray(ln.d, np.float64)
            # 沿拟合线取两个远点，反投影到平面；远点避免短基线放大噪声。
            A = backproject(p0 - d * 1000.0)
            B = backproject(p0 + d * 1000.0)
            if A is None or B is None:
                return None
            u = B - A
            L = float(np.linalg.norm(u))
            if not np.isfinite(L) or L < 1e-6:
                return None
            return A, u / L, L

        def pair_distance(ea, eb):
            A = line3d(lines[ea])
            B = line3d(lines[eb])
            if A is None or B is None:
                return None
            pa, ua, La = A
            pb, ub, _Lb = B
            perp = np.cross(n, ub)
            pn = float(np.linalg.norm(perp))
            if pn < 1e-9:
                return None
            perp = perp / pn
            ts = np.linspace(-0.5, 0.5, 5) * La
            ds = [abs(float((pa + t * ua - pb) @ perp)) for t in ts]
            ds = [d for d in ds if np.isfinite(d)]
            if not ds:
                return None
            return float(np.median(ds)), float(np.max(ds) - np.min(ds))

        # edge 0/2 = 底/上 -> H；edge 1/3 = 右/左 -> W。
        h_pair = pair_distance(0, 2)
        w_pair = pair_distance(1, 3)
        if h_pair is None or w_pair is None:
            return None
        H, h_spread = h_pair
        W, w_spread = w_pair
        if not (np.isfinite(W) and np.isfinite(H) and W > 0.0 and H > 0.0):
            return None
        return [float(min(W, H)), float(max(W, H))], max(w_spread, h_spread)
    except Exception:                                   # noqa: BLE001
        return None


def _unit2(v):
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < 1e-12:
        return None
    return np.asarray(v, float) / n


def _backproject_to_plane(uv, k, n0, D_mm):
    """像素点 -> 射线与平面 `{n0·P=D}` 的交点（相机系，毫米）。"""
    ray = np.array([(float(uv[0]) - k.cx) / k.fx,
                    (float(uv[1]) - k.cy) / k.fy,
                    1.0], float)
    den = float(n0 @ ray)
    if abs(den) < 1e-9:
        return None
    return ray * (float(D_mm) / den)


def _line_in_plane_2d(ln, k, n0, D_mm, e1, e2, span_px=1000.0):
    """图像直线 -> 托盘平面内的 2D 直线 `(q, u)`（正交基 e1/e2 坐标）。

    接受 solve 用的 `{'p0','d'}` dict 或 `pose_decode.EdgeLine`。
    """
    if isinstance(ln, dict):
        p0, d = ln['p0'], ln['d']
    else:
        p0, d = ln.p0, ln.d
    p0 = np.asarray(p0, float)
    d = _unit2(np.asarray(d, float))
    if d is None:
        return None
    A = _backproject_to_plane(p0 - span_px * d, k, n0, D_mm)
    B = _backproject_to_plane(p0 + span_px * d, k, n0, D_mm)
    if A is None or B is None:
        return None
    q = np.array([float(A @ e1), float(A @ e2)], float)
    u = _unit2(np.array([float(B @ e1), float(B @ e2)], float) - q)
    if u is None:
        return None
    return q, u


def _intersect2(l1, l2):
    """平面内两条 `(q, u)` 直线求交；近平行返回 None。"""
    if l1 is None or l2 is None:
        return None
    q1, u1 = l1
    q2, u2 = l2
    den = float(u1[0] * u2[1] - u1[1] * u2[0])
    if abs(den) < 1e-6:
        return None
    t = float(((q2[0] - q1[0]) * u2[1] - (q2[1] - q1[1]) * u2[0]) / den)
    return q1 + t * u1


def _angle_deg(u1, u2):
    """两条无向直线的夹角 `[0, 90]`。"""
    a = abs(float(np.degrees(np.arctan2(
        float(u1[0] * u2[1] - u1[1] * u2[0]),
        float(np.dot(u1, u2))))))
    return min(a, 180.0 - a)


def _is_adjacent(i, j):
    """边序 0,1,2,3 = 下/右/上/左；相邻边共享一个角。"""
    return int(i) != int(j) and (int(i) - int(j)) % 4 in (1, 3)


def _edge_reliability(ln, i, kp_xy, kp_conf, w, h):
    """单条边的可靠性评分（0~1）+ 分项，供 partial 选相邻边。

    分项：参与关键点的 mean/min conf、参与点数、线残差、**边中点**到图像边界
    的距离（贴边降权）、边中点到图像中心的距离（越靠中心越高）。

    ⚠️ 边界距离用**边中点**，不是“所有关键点到边界的最小值”。托盘被相机
    裁掉一个角时（2026-11-11 现场：`kp3.x=647.75 > 640`），最小值会被单个
    出画端点直接打到 0，把一条本来很直、置信度很高的长边判成不可靠；而这条
    边的中点仍在画面中央，几何上完全可用。贴边降权的本意是“整条边贴着边界”，
    不是“有一个端点出画”。
    """
    idx = tuple(getattr(ln, 'idx', ()) or ())
    if not idx:
        idx = tuple(PD.EDGES[i])
    pts = np.asarray(kp_xy, float).reshape(PD.N_KP, 2)[list(idx)]
    confs = np.asarray(kp_conf, float).reshape(PD.N_KP)[list(idx)]
    ok = np.isfinite(pts).all(axis=1) & np.isfinite(confs)
    if np.any(ok):
        pts, confs = pts[ok], confs[ok]
    else:
        pts = np.asarray([np.asarray(ln.p0, float)])
        confs = np.asarray([float(getattr(ln, 'mean_conf', 0.0))])

    mean_conf = float(getattr(ln, 'mean_conf', np.nan))
    if not np.isfinite(mean_conf):
        mean_conf = float(np.mean(confs))
    min_conf = float(np.min(confs))
    n_points = int(getattr(ln, 'n_points', len(idx)))
    residual = float(getattr(ln, 'residual_px', 0.0))
    if not np.isfinite(residual):
        residual = 0.0

    near_px = EDGE_BORDER_NEAR_FRAC * float(min(w, h))
    mid = pts.mean(axis=0)
    # 用参与拟合的关键点到边界距离的**中位数**：单个端点出画只影响一个样本，
    # 不会把整条边判死；整条边贴着边界时中位数仍然很小，继续降权。
    border_each = [min(float(x), float(y), float(w - 1) - float(x),
                       float(h - 1) - float(y)) for x, y in pts]
    border_dist = max(0.0, float(np.median(border_each)))
    center_dist = float(np.linalg.norm(mid - np.array([float(w) / 2.0,
                                                       float(h) / 2.0])))
    half_diag = 0.5 * float(np.hypot(w, h))
    center_norm = center_dist / max(half_diag, 1e-9)

    conf_score = (0.5 * float(np.clip(mean_conf, 0.0, 1.0))
                  + 0.5 * float(np.clip(min_conf, 0.0, 1.0)))
    n_score = float(np.clip(n_points / 4.0, 0.0, 1.0))
    res_score = 1.0 / (1.0 + max(residual, 0.0) / EDGE_LINE_RES_REF_PX)
    border_score = (EDGE_BORDER_MIN_FACTOR
                    + (1.0 - EDGE_BORDER_MIN_FACTOR)
                    * float(np.clip(border_dist / max(near_px, 1e-9), 0.0, 1.0)))
    center_score = 1.0 - float(np.clip(center_norm, 0.0, 1.0))
    score = (conf_score * (0.5 + 0.5 * n_score)
             * (0.5 + 0.5 * res_score) * border_score * center_score)
    return {
        'edge': int(i),
        'score': round(float(score), 4),
        'mean_conf': round(mean_conf, 4),
        'min_conf': round(min_conf, 4),
        'n_points': n_points,
        'residual_px': round(residual, 4),
        'border_dist_px': round(border_dist, 3),
        'border_near_px': round(near_px, 3),
        'center_dist_px': round(center_dist, 3),
        'midpoint_px': [round(float(mid[0]), 3), round(float(mid[1]), 3)],
        'conf_score': round(conf_score, 4),
        'n_score': round(n_score, 4),
        'res_score': round(res_score, 4),
        'border_score': round(border_score, 4),
        'center_score': round(center_score, 4),
    }


def _select_edge_pair(lines, kp_xy, kp_conf, w, h):
    """三边/两边可见时，按可靠性选**一对相邻边**作为位置量测。

    只接受 (0,1),(1,2),(2,3),(3,0)；两条对边会沿边线方向留下一个自由度
    （实测 right+left 偏 510mm、bottom+top 偏 2126mm），不能只靠它们。
    两条相邻边在已知 normal+D 下可精确解（实测 0.2~2.3mm）。

    返回可 JSON 序列化的 dict：每边 score/分项、选中的 pair、丢弃的边、原因。
    """
    scores = [None] * 4
    for i, ln in enumerate(lines):
        if ln is not None:
            scores[i] = _edge_reliability(ln, i, kp_xy, kp_conf, w, h)
    visible = [i for i in range(4) if lines[i] is not None]
    out = {
        'visible': visible,
        'scores': scores,
        'pair': None,
        'pair_score': None,
        'dropped': [],
        'reason': None,
        'thresholds': {
            'score_min': float(EDGE_SCORE_MIN),
            'pair_score_min': float(EDGE_PAIR_SCORE_MIN),
            'border_near_frac': float(EDGE_BORDER_NEAR_FRAC),
            'line_res_ref_px': float(EDGE_LINE_RES_REF_PX),
        },
    }
    if len(visible) < 2:
        out['reason'] = 'insufficient_visible_edges'
        return out

    eligible = [i for i in visible
                if scores[i] is not None
                and scores[i]['score'] >= EDGE_SCORE_MIN]
    cand = []
    for i, j in ((0, 1), (1, 2), (2, 3), (3, 0)):
        if i in eligible and j in eligible:
            cand.append((float(scores[i]['score'] + scores[j]['score']), i, j))
    if cand:
        pair_score, i, j = max(cand, key=lambda t: t[0])
        if pair_score >= EDGE_PAIR_SCORE_MIN:
            out.update({
                'pair': [int(i), int(j)],
                'pair_score': round(pair_score, 4),
                'dropped': [k for k in visible if k not in (i, j)],
                'dropped_reliable': [k for k in visible
                                     if k not in (i, j)
                                     and scores[k] is not None
                                     and scores[k]['score'] >= EDGE_SCORE_MIN],
                'eligible': eligible,
            })
            return out

    # 没有可接受的相邻边对。若可靠边里只剩一对外边，明确记录
    # `opposite_pair_only`（这两条边不能单独约束）；否则是可靠性不足。
    opp_eligible = any(a in eligible and b in eligible
                       for a, b in ((0, 2), (1, 3)))
    out['opposite_visible'] = any(a in visible and b in visible
                                  for a, b in ((0, 2), (1, 3)))
    out['eligible'] = eligible
    out['dropped_reliable'] = list(eligible)
    out['reason'] = ('opposite_pair_only' if opp_eligible
                     else 'no_reliable_edge_pair')
    return out


def _edge_select_reason(sel, applied=True):
    """选边结果 -> 一条字符串原因（供重叠图/黑板直接显示）。"""
    if not applied:
        return 'selection_skipped_no_D'
    if sel.get('pair') is None:
        return sel.get('reason') or 'no_reliable_edge_pair'
    dropped = sel.get('dropped') or []
    if dropped:
        parts = {s['edge']: s for s in sel.get('scores') or [] if s}
        for k in dropped:
            p = parts.get(k)
            if p is None:
                continue
            if (p['score'] < EDGE_SCORE_MIN
                    or p['border_score'] < 0.6
                    or p['min_conf'] < 0.7
                    or p['n_points'] < 3):
                return 'low_border_conf'
    return 'adjacent_pair'


def _put_edge_selection_diag(diag, kp_xy, kp_conf, scores, selected,
                             dropped, reason):
    """把选边结果写成 JSON 可序列化的顶层 diag 字段（供伺服叠加图直接用）。"""
    diag['keypoints_uv'] = np.asarray(kp_xy, float).reshape(PD.N_KP, 2).tolist()
    diag['keypoints_conf'] = (np.asarray(kp_conf, float)
                              .reshape(PD.N_KP).tolist())
    diag['edge_score'] = [None if s is None else float(s['score'])
                          for s in scores]
    diag['edge_score_parts'] = [None if s is None else s for s in scores]
    diag['selected_edges'] = (None if selected is None
                              else [int(i) for i in selected])
    diag['dropped_edges'] = [int(i) for i in (dropped or [])]
    diag['edge_select_reason'] = str(reason)


def _partial_visible_gate(lines, has_D, pair=None):
    """partial 帧的**可见边集合**闸门。

    * 调用方已按可靠性选出 `pair`（两条**相邻**边）时直接通过本闸门，
      交给 `_check_edge_plane_consistency` 检查这两条边的夹角；
    * 未显式给 pair 时：两条可见边必须相邻（对边 -> `opposite_pair_only`），
      一条 -> `insufficient_visible_edges`；三条仍按旧的
      bottom/right + (left/top) 形态判断（`unsupported_visible_edge_set`）。
    """
    visible = [i for i, ln in enumerate(lines) if ln is not None]
    if pair is not None:
        return visible, True, None
    if len(visible) == 2:
        if _is_adjacent(visible[0], visible[1]):
            return visible, True, None
        return visible, False, 'opposite_pair_only'
    if len(visible) < 2:
        return visible, False, 'insufficient_visible_edges'
    supported = (0 in visible and 1 in visible
                 and (2 in visible or 3 in visible))
    if not supported and has_D:
        return visible, supported, 'unsupported_visible_edge_set'
    return visible, supported, None


def _three_edge_plane_metrics(lines, k, n0, e1, e2, D_mm, target_mm):
    """bottom/right/left 三边在平面内的长度/角度诊断（不单独决定 pass/fail）。"""
    out = {'available': False, 'reason': None, 'fails': []}
    L = {}
    for i in (0, 1, 3):
        if lines[i] is None:
            out['reason'] = 'missing_side_edges'
            return out
        L[i] = _line_in_plane_2d(lines[i], k, n0, float(D_mm), e1, e2)
        if L[i] is None:
            out['reason'] = 'backprojection_failed'
            return out

    bl = _intersect2(L[0], L[3])
    br = _intersect2(L[0], L[1])
    a_br = _angle_deg(L[0][1], L[1][1])
    a_bl = _angle_deg(L[0][1], L[3][1])
    a_rl = _angle_deg(L[1][1], L[3][1])
    right_err = abs(a_br - 90.0)

    fails = []
    checks_run = ['bottom_right_angle']
    checks_skipped = []
    if br is not None and bl is not None:
        bl_br = float(np.linalg.norm(bl - br))
        ratio = bl_br / max(target_mm, 1e-9)
        checks_run.append('bottom_length')
        if not (EDGE_PLANE_RATIO_RANGE[0] <= ratio <= EDGE_PLANE_RATIO_RANGE[1]):
            fails.append('bottom_length')
    else:
        bl_br, ratio = None, None
        checks_skipped.append('bottom_length')
    checks_run.append('right_left_parallel')
    if a_rl > EDGE_PLANE_PARALLEL_TOL_DEG:
        fails.append('right_left_parallel')
    if right_err > EDGE_PLANE_RIGHT_ANGLE_TOL_DEG:
        fails.append('bottom_right_angle')

    out.update({
        'available': True,
        'passed': not fails,
        'ratio': None if ratio is None else round(ratio, 4),
        'bl_br_mm': None if bl_br is None else round(bl_br, 3),
        'angle_bottom_right_deg': round(a_br, 3),
        'angle_bottom_left_deg': round(a_bl, 3),
        'angle_right_left_deg': round(a_rl, 3),
        'right_angle_err_deg': round(right_err, 3),
        'fails': fails,
        'checks_run': checks_run,
        'checks_skipped': checks_skipped,
    })
    return out


def _check_edge_plane_consistency(lines, k, normal, D_mm, size_mm,
                                  long_side_parallel=False, pair=None):
    """partial 帧的**可见边 / 平面几何一致性**闸门。

    给了 `pair`（经可靠性选出的两条**相邻**边）时，硬闸门是这两条边在
    `{n0·P=D}` 平面内的夹角与 90° 的偏差 <= `EDGE_PAIR_ANGLE_TOL_DEG`。
    两条对边会沿边线方向留下一个自由度，不能单独约束，直接判不通过。

    未给 `pair` 时退回三边口径：bottom/right/left 三边必须像同一块托盘
    （`|bl-br|/target_mm`、`ang(B,R)`、`ang(R,L)`）；三边都在时，第三条边
    的结果仅作为 `three_edge_diag` / `three_edge_contradicts` 诊断，不参与
    两条相邻边解的硬判定。

    `target_mm = max(size_mm) if long_side_parallel else min(size_mm)`。
    """
    target_mm = (float(max(size_mm)) if long_side_parallel
                 else float(min(size_mm)))
    out = {
        'checked': False,
        'passed': None,
        'D_mm': float(D_mm),
        'target_edge_mm': target_mm,
        'target_edge_src': ('long_side' if long_side_parallel else 'short_side'),
        'ratio_range': [float(EDGE_PLANE_RATIO_RANGE[0]),
                        float(EDGE_PLANE_RATIO_RANGE[1])],
        'right_angle_tol_deg': float(EDGE_PLANE_RIGHT_ANGLE_TOL_DEG),
        'parallel_tol_deg': float(EDGE_PLANE_PARALLEL_TOL_DEG),
        'pair_angle_tol_deg': float(EDGE_PAIR_ANGLE_TOL_DEG),
        'pair': None,
        'angle_pair_deg': None,
        'pair_angle_err_deg': None,
        'ratio': None,
        'bl_br_mm': None,
        'angle_bottom_right_deg': None,
        'angle_bottom_left_deg': None,
        'angle_right_left_deg': None,
        'right_angle_err_deg': None,
        'fails': [],
        'checks_run': [],
        'checks_skipped': [],
        'three_edge_diag': None,
        'three_edge_contradicts': None,
    }
    out['visible_side_edges'] = [i for i in (0, 1, 3)
                                 if lines[i] is not None]

    n = np.asarray(normal, float)
    nn = float(np.linalg.norm(n))
    if not np.isfinite(nn) or nn < 1e-9:
        out['reason'] = 'bad_normal'
        return out
    n0 = n / nn
    if float(n0[2]) < 0.0:
        n0 = -n0
    e1, e2 = _basis(n0)

    # 未显式给 pair 时，两条可见边若相邻也可按两约束检查（对边则不)。
    if pair is None:
        vis = out['visible_side_edges']
        if len(vis) == 2 and _is_adjacent(vis[0], vis[1]):
            pair = (vis[0], vis[1])

    if pair is not None:
        i, j = int(pair[0]), int(pair[1])
        out['pair'] = [i, j]
        if not _is_adjacent(i, j):
            out['reason'] = 'pair_not_adjacent'
            return out
        if lines[i] is None or lines[j] is None:
            out['reason'] = 'missing_pair_edge'
            return out
        Li = _line_in_plane_2d(lines[i], k, n0, float(D_mm), e1, e2)
        Lj = _line_in_plane_2d(lines[j], k, n0, float(D_mm), e1, e2)
        if Li is None or Lj is None:
            out['reason'] = 'backprojection_failed'
            return out
        ang = _angle_deg(Li[1], Lj[1])
        err = abs(ang - 90.0)
        fails = []
        if err > EDGE_PAIR_ANGLE_TOL_DEG:
            fails.append('edge_pair_angle')
        three = None
        if all(lines[x] is not None for x in (0, 1, 3)):
            three = _three_edge_plane_metrics(lines, k, n0, e1, e2,
                                              D_mm, target_mm)
        out.update({
            'checked': True,
            'passed': not fails,
            'angle_pair_deg': round(ang, 3),
            'pair_angle_err_deg': round(err, 3),
            'fails': fails,
            'checks_run': ['edge_pair_angle'],
            'checks_skipped': [],
            'three_edge_diag': three,
            'three_edge_contradicts': (
                None if three is None or not three.get('available')
                else bool(three.get('fails'))),
        })
        if three is not None and three.get('available'):
            for key in ('ratio', 'bl_br_mm', 'angle_bottom_right_deg',
                        'angle_bottom_left_deg', 'angle_right_left_deg',
                        'right_angle_err_deg'):
                out[key] = three[key]
        return out

    # 旧的三边口径（保持兼容；detect 的 partial 分支现在总会给出 pair）。
    three = _three_edge_plane_metrics(lines, k, n0, e1, e2, D_mm, target_mm)
    if not three.get('available'):
        out['reason'] = three.get('reason') or 'insufficient_side_edges'
        return out
    out.update({
        'checked': True,
        'passed': three['passed'],
        'ratio': three['ratio'],
        'bl_br_mm': three['bl_br_mm'],
        'angle_bottom_right_deg': three['angle_bottom_right_deg'],
        'angle_bottom_left_deg': three['angle_bottom_left_deg'],
        'angle_right_left_deg': three['angle_right_left_deg'],
        'right_angle_err_deg': three['right_angle_err_deg'],
        'fails': three['fails'],
        'checks_run': three['checks_run'],
        'checks_skipped': three['checks_skipped'],
    })
    return out


def _select_pose_backend(opts: dict):
    """选 POSE 后端 → `('engine'|'onnx', 路径)`。规则与 OBB 的 `_select_backend`
    完全一致（`auto` 看 engine 文件在不在；`engine` 缺失直接报错）。"""
    backend = str(opts.get('backend', 'auto') or 'auto').strip().lower()
    if backend not in _BACKENDS:
        raise ValueError(f'backend={backend!r} 不认识，只认 {"/".join(_BACKENDS)}')
    model_path = Path(str(opts.get('model_path') or DEFAULT_POSE_MODEL_PATH))
    engine_path = Path(str(opts.get('engine_path') or DEFAULT_POSE_ENGINE_PATH))
    if backend == 'onnx':
        return 'onnx', model_path
    if engine_path.is_file():
        return 'engine', engine_path
    if backend == 'engine':
        raise FileNotFoundError(
            f'backend=engine 但 POSE 的 TensorRT engine 不存在：{engine_path}\n'
            f'  怎么解决：在目标机（Orin + TRT 8.5）上导出：\n'
            f'    trtexec --onnx={model_path} --saveEngine={engine_path} '
            f'--noTF32 --buildOnly\n'
            f'  或把 backend 改成 auto/onnx。')
    return 'onnx', model_path


def _resolve_pose_session(opts: dict):
    """`opts` -> `(session, model_src)`。拿不到就抛，不提供回退。

    session 可能是 onnxruntime 的，也可能是 `trt_backend.TensorRTSession` ——
    检测代码只用 `get_inputs()[0].name` / `run(None, {...})[0]`，不分叉。
    """
    sess = opts.get('session')
    if sess is not None:
        return sess, '<opts[session]>'

    kind, path = _select_pose_backend(opts)
    if kind == 'engine':
        imgsz = int(opts.get('imgsz', DEFAULT_IMGSZ))
        try:
            from .trt_backend import TensorRTSession
            sess = TensorRTSession(path, expected_input=(1, 3, imgsz, imgsz))
        except Exception as exc:                     # noqa: BLE001
            raise RuntimeError(
                f'托盘 POSE TensorRT engine 加载失败：{path}'
                f'（{type(exc).__name__}: {exc}）。\n'
                f'  怎么解决：确认 engine 是在本机（同一 TensorRT 版本）导出的，'
                f'且 --imgsz 与检测参数 {imgsz} 一致；或改用 backend=onnx。') from exc
        return sess, f'engine:{path}'

    if not path.exists():
        raise FileNotFoundError(
            f'没有托盘 POSE 模型：期望 {path}（或从 opts 传 session / model_path）。\n'
            f'  实际查找的路径：{path}\n'
            f'  怎么解决：\n'
            f'    1) 在有 ultralytics 的开发机上导出：\n'
            f'       cd <本包>/scripts && OMP_NUM_THREADS=1 python3 '
            f'export_onnx_pose.py --weights best.pt --verify\n'
            f'    2) 把导出的 .onnx 拷到 {path.parent}/\n'
            f'    3) 或调用时传 opts={{...}} 指定 model_path / session\n'
            f'  POSE 检测**必须**有 pose 模型，不提供任何回退。')
    try:
        import onnxruntime as ort
    except ImportError as exc:                       # pragma: no cover
        raise RuntimeError(
            f'没有 onnxruntime（{exc}）——托盘 POSE 推理要它。\n'
            f'  怎么解决：装到当前环境，如 `pip install onnxruntime`（GPU 则 '
            f'onxxruntime-gpu，并显式设 LD_LIBRARY_PATH，否则会静默退回 CPU）。') from exc

    # provider 顺序即优先级；与 OBB 路线一致：CUDA 优先，CPU 兜底。
    providers = [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider')
                 if p in ort.get_available_providers()]
    sess_opts = ort.SessionOptions()
    sess_opts.intra_op_num_threads = 1
    try:
        return ort.InferenceSession(str(path), sess_options=sess_opts,
                                    providers=providers), str(path)
    except Exception as exc:                         # noqa: BLE001
        raise RuntimeError(
            f'托盘 POSE 模型加载失败：{path}（{type(exc).__name__}: {exc}）。\n'
            f'  怎么解决：确认它是 export_onnx_pose.py 导出的、能在本机 onnxruntime '
            f'打开；换模型后重跑 `--verify` 核对输出 (1,41,8400)。') from exc


def _quad_with_corner_fallback(lines, kp_xy):
    """`intersect_edges` 整体失败时的**逐角兜底**。

    对每个角：两条相邻边都在就求交；求不出（缺边/近平行）就用该角的直接
    关键点预测 `kp_xy[idx]`。任一角仍拿不到 -> ``None``。

    返回 ``(4,2)`` 的 ``[bl, br, tr, tl]``（与 `intersect_edges` 同序）。
    """
    kp_xy = np.asarray(kp_xy, float).reshape(PD.N_KP, 2)
    quad = np.empty((4, 2), float)
    for ci, (ea, eb, kpi) in enumerate(_CORNER_FALLBACK):
        pt = None
        if lines[ea] is not None and lines[eb] is not None:
            # `pose_decode._intersect` 与 `intersect_edges` 用的是同一条求交式。
            pt = PD._intersect(lines[ea], lines[eb])   # noqa: SLF001（同包内复用）
            if pt is not None and not np.all(np.isfinite(pt)):
                pt = None
        if pt is None:
            p = kp_xy[kpi]
            if np.all(np.isfinite(p)):
                pt = p
        if pt is None:
            return None
        quad[ci] = pt
    return quad


def _rebuild_origin_from_line_pair(frame: dict, lines, pair, k, D_mm):
    """用**两条相邻观测线**的交点重建 frame origin（partial + 固定 D）。

    两邻边 partial 的 4-DOF 解有一个镜像解：矩形落在两条线的另一侧，可见边残差
    同样很小，但 origin 会跑到托盘的镜像位置（实测与正确解相隔约 1.57m，抖帧时
    `NodePalletObsV2` 每帧报 `Δpos≈1.57m`）。关键点 0 能指出正确的一侧，但
    现有的 180° 翻转只改坐标轴、不改变矩形所在的侧；这里从**观测线的交点** +
    已知 D 重建共享角点，再沿 frame 的 E1/E2 退回 origin。

    `pair` 是两条相邻边的索引（0/1/2/3 = 下/右/上/左）。交点所在角的
    `(u,v)` 是 `(0,0)/(W,0)/(W,H)/(0,H)` 之一；`origin = P_shared - u E1 - v E2`。
    任何一步退化返回 `None`（调用方保留原 frame）。
    """
    try:
        if pair is None or len(pair) != 2:
            return None
        a, b = int(pair[0]), int(pair[1])
        if (a + 1) % 4 == b:
            j = b
        elif (b + 1) % 4 == a:
            j = a
        else:
            return None
        la, lb = lines[a], lines[b]
        if la is None or lb is None:
            return None
        p = SE._intersect(la, lb)                  # noqa: SLF001（同包内复用）
        if p is None or not np.all(np.isfinite(p)):
            return None
        n0 = np.asarray(frame['nrm'], float)
        nn = float(np.linalg.norm(n0))
        if not np.isfinite(nn) or nn < 1e-9:
            return None
        n0 = n0 / nn
        if float(n0[2]) < 0.0:
            n0 = -n0
        Kmat = np.array([[k.fx, 0.0, k.cx],
                         [0.0, k.fy, k.cy],
                         [0.0, 0.0, 1.0]], float)
        ray = np.linalg.solve(Kmat, np.array([p[0], p[1], 1.0], float))
        den = float(n0 @ ray)
        if not np.isfinite(den) or abs(den) < 1e-9:
            return None
        D_now = float(D_mm)
        if not np.isfinite(D_now) or D_now <= 0.0:
            return None
        # frame 的 origin 是**毫米**；D 也是毫米。这里不要先换米，
        # 否则重建出的 origin 比原 frame 小 1000 倍（下游会把位姿当成 0.8mm）。
        P = ray * (D_now / den)                   # 交点投到 {n0·X=D} 平面（mm）
        E1 = np.asarray(frame['E1'], float)
        E2 = np.asarray(frame['E2'], float)
        W = float(frame['W']); H = float(frame['H'])
        uv = ((0.0, 0.0), (W, 0.0), (W, H), (0.0, H))[j]
        origin = P - uv[0] * E1 - uv[1] * E2
        cand = dict(frame)
        cand['origin'] = origin
        cand['nrm'] = np.cross(E1, E2)
        return SO._to_spec(cand, W, H, k)
    except (TypeError, ValueError, KeyError, np.linalg.LinAlgError):
        return None


def _align_frame_origin_to_kp0(frame: dict, kp_xy, k) -> tuple:
    """用 12 关键点的第 0 点消掉 180° 角点歧义。

    partial 帧只看到两条相邻边时，直线是**无向**的：同一组点可以解出两个
    关于两条边交点对称的解，二者平面 D 相同，只是 frame origin 落在矩形
    对角角点上。此时任何 D 闸门都看不出来，但关键点序号 0 是模型定义的
    origin 角；把两个候选的 origin 投影到图像，哪个离 kp0 近就选哪个。
    """
    try:
        kp0 = np.asarray(kp_xy, float).reshape(-1, 2)[0]
        W = float(frame['W']); H = float(frame['H'])
        E1 = np.asarray(frame['E1'], float); E2 = np.asarray(frame['E2'], float)
        o = np.asarray(frame['origin'], float)
        alt = dict(frame)
        alt['origin'] = o + W * E1 + H * E2
        alt['E1'] = -E1
        alt['E2'] = -E2
        p0 = _project_px(np.asarray([o]), k)[0]
        p_alt = _project_px(np.asarray([alt['origin']]), k)[0]
        d0 = float(np.linalg.norm(p0 - kp0))
        d_alt = float(np.linalg.norm(p_alt - kp0))
        if np.isfinite(d_alt) and d_alt < d0 * 0.75:
            alt['origin_kp_flip'] = True
            alt['origin_kp_dist_px'] = round(d_alt, 2)
            return alt, True, d0, d_alt
        frame['origin_kp_flip'] = False
        frame['origin_kp_dist_px'] = round(d0, 2)
        return frame, False, d0, d_alt
    except (TypeError, ValueError, IndexError, KeyError):
        return frame, False, None, None


def detect_pallet_frame_pose(color: np.ndarray, depth: np.ndarray, k, *,
                             normal, target_mm=(1200., 1000.),
                             long_side_parallel=False,
                             opts=None):
    """YOLO-pose 推理 + `solve_edges.solve_frame` -> `(frame | None, diag)`。

    `color` 是 BGR uint8、`depth` 是毫米（没有 call 0 记录时用于锁 D）、`k` 是
    `CameraIntrinsics`、`normal` 是台面法向外部先验（z<0 也行，内部翻正）。
    `target_mm` 是托盘物理边长；`long_side_parallel` 是面内转角先验。

    可见边不足 4 条（partial）时：有 D（record 或本帧 `d_anchor`）才按
    `_select_edge_pair` 的可靠性分数**只保留一对相邻边**；第三条只做
    `three_edge_diag` 一致性告警，不作为硬约束。真正的拒绝码只有：

      * `opposite_pair_only`：最可靠的两条是对边（单独约束会偏几百 mm）；
      * `no_reliable_edge_pair`：没有达到分数下限的相邻边对；
      * `edge_pair_without_D`：只有 <=2 条可见边且没有已知 D（两条边无法
        同时定 D 和位姿）。

    `edge_plane_check` 里的夹角/三边一致性只写诊断，**不拒绝**：call 0 已经
    提前记录了 base_link 下的法向和托盘 D，现场台面可能被箱子遮挡，不能再用
    当前帧的平面拟合/反投影夹角去否决选中的边。

    `opts` 可带：
      * `session` / `model_path` / `imgsz`：模型来源与输入尺寸；
      * `conf`（默认 0.25）/ `nms_iou`（0.45）/ `pick`（bottom_center）；
      * `kpt_conf`（默认 0.0，关键点置信度预过滤）；
      * `kconf`（默认 0.5，参与边拟合的最低关键点 conf）；
      * `line_res_px`（默认 12，边拟合逐次剔点的最大垂距）；
      * `d_lock`（默认 `'depth'`，见模块头）；
      * `free_d`（默认 False）：**完全不用深度图定高度**，D 只由四条边 /
        关键点几何 + 已知尺寸自由解。call 0 观测托盘时用 —— 记录只要 base
        下的法向与台面高度 z；深度主平面可能是板条缝下的下层/地面，锁它会
        把高度带低（现场实测低约 220mm）。此时 `depth` 参数只用于其它诊断，
        不参与解算；
      * `fit_normal`：**收下不用**（本入口锁在调用方给的法向上，与 OBB 的
        `detect_pallet_frame_pose` 默认自检不同；显式传 False 语义一致）。
    """
    opts = dict(opts or {})
    imgsz = int(opts.get('imgsz', DEFAULT_IMGSZ))
    conf_thr = float(opts.get('conf', _DEFAULT_CONF))
    nms_iou = float(opts.get('nms_iou', _DEFAULT_NMS_IOU))
    pick = str(opts.get('pick', 'bottom_center'))
    kconf = float(opts.get('kconf', _DEFAULT_KCONF))
    line_res_px = float(opts.get('line_res_px', _DEFAULT_LINE_RES_PX))

    # 模型：缺失 / 加载失败 -> 抛（不静默返回 None）
    sess, model_src = _resolve_pose_session(opts)

    img = np.asarray(color)
    if img.ndim != 3 or img.shape[2] != 3:
        return None, {'source': 'pose', 'reject': 'bad_color',
                      'shape': list(img.shape), 'model': model_src}
    h, w = int(img.shape[0]), int(img.shape[1])

    # ---- 分阶段计时（2026-11-16）------------------------------------------
    # 之前只有节点的"整帧 detect"一个总数，看不到 120~200ms 落在哪一段。
    # 这里把 **预处理 / 推理 / 后处理算法** 分开记，节点把它打进心跳。
    # 计时本身是两次 perf_counter，代价可忽略；关掉的话传 opts['timing']=False。
    _timing = bool(opts.get('timing', True))
    _t0 = time.perf_counter()
    x, ratio, pad = _to_input_tensor(img, imgsz)
    _t_pre = time.perf_counter()
    try:
        in_name = sess.get_inputs()[0].name
        raw = sess.run(None, {in_name: x})[0]
    except Exception as exc:                         # noqa: BLE001
        return None, {'source': 'pose', 'reject': 'infer_failed',
                      'why': f'{type(exc).__name__}: {exc}', 'model': model_src}
    _t_infer = time.perf_counter()

    def _stage_diag(d):
        if not _timing:
            return d
        d['t_pre_ms'] = round((_t_pre - _t0) * 1000.0, 2)
        d['t_infer_ms'] = round((_t_infer - _t_pre) * 1000.0, 2)
        d['t_algo_ms'] = round((time.perf_counter() - _t_infer) * 1000.0, 2)
        return d

    # ---- 解码 / NMS / 选主目标 ------------------------------------------------
    dets = PD.decode_pose(np.asarray(raw)[0].T, ratio, pad, (h, w),
                          conf_thr, kpt_conf_thr=float(opts.get('kpt_conf', 0.0)))
    dets = PD.nms_pose(dets, nms_iou, (h, w))
    det = PD.pick_pose(dets, (h, w), how=pick)
    if det is None:
        return None, _stage_diag({'source': 'pose', 'reject': 'no_detection',
                                  'n_candidates': len(dets),
                                  'model': model_src})

    # 选主目标的全部候选（只做诊断，不参与几何）：现场“位姿飞了”时要能分辨
    # 是主目标本身算错，还是画面里另有一个更像托盘的候选被 bottom_center 选中。
    _cand_centers, _cand_conf, _cand_bbox = [], [], []
    for d in dets:
        try:
            b = np.asarray(d.bbox_xyxy, float)
            _cand_bbox.append([round(float(v), 1) for v in b])
            _cand_centers.append([round(float((b[0] + b[2]) / 2.0), 1),
                                  round(float((b[1] + b[3]) / 2.0), 1)])
            _cand_conf.append(round(float(d.conf), 4))
        except (TypeError, ValueError, IndexError):
            continue

    # ---- 四条边 + quad -------------------------------------------------------
    lines = PD.fit_edges(det.kp_xy, det.kp_conf,
                         kconf=kconf, line_res_px=line_res_px)
    quad = PD.intersect_edges(lines, size=(h, w))
    if quad is None:
        quad = _quad_with_corner_fallback(lines, det.kp_xy)
    if quad is None:
        return None, {'source': 'pose', 'reject': 'no_quad',
                      'n_candidates': len(dets), 'model': model_src,
                      'conf': round(det.conf, 4),
                      'edge_n_points': [None if ln is None else ln.n_points
                                        for ln in lines]}

    lines_solve = [None if ln is None else
                   {'p0': np.asarray(ln.p0, float),
                    'd': np.asarray(ln.d, float),
                    'weight': 1.0}
                   for ln in lines]
    # v1 保守口径：用 OBB 的三道闸门 + 秩保底判可用边。
    use_pos = usable_edges(order_quad(quad), w, h)

    # ---- 深度 D：本帧只量一次（有 call 0 记录时**不量**）---------------------
    # `opts['fixed_d_mm']` 给了数值就直接用它，**完全不碰深度图** —— 调用方
    # （检测节点）在 call 0 之后已经有稳定的 D，每帧再量一次只会把深度噪声
    # 带进位姿，而且 `_plane_dist_mm` 是这条链的 CPU 大头之一。
    #
    # `free_d=True`：call 0 观测，D 只由 pose 四边/关键点 + 已知尺寸自由解。
    # 现场托盘是板条结构，深度主平面会从缝里落到下层/地面（实测低约 220mm）；
    # 锁它就把记录的高度 z 带低。此时 deep 图的 D 一律不参与。
    #
    # 没有记录时才走原来的路径：先按 stride=2 跳采；**主峰占比 >= 0.35 才采信**，
    # 否则用全量 stride=1 重算。601 帧实测 stride=2 只在 6 帧（`14-22-38`，
    # 峰占比 0.25~0.29）上翻到另一个峰（ΔD 约 256~262mm）；0.35 的闸门把这些帧
    # 挡回全量，其余帧的 |ΔD| 中位 0.0068mm / p90 0.022mm。
    free_d = bool(opts.get('free_d', False))
    d_anchor, d_peak, d_src = None, None, 'fallback'
    record_fixed_d = None
    depth_for_solve = depth
    if free_d:
        d_src = 'free_geom'
        depth_for_solve = None       # solve_frame 只在锁 D 时用 depth
    else:
        fixed_d = opts.get('fixed_d_mm')
        if fixed_d is not None:
            try:
                fixed_d = float(fixed_d)
            except (TypeError, ValueError):
                fixed_d = None
            if fixed_d is not None and np.isfinite(fixed_d) and fixed_d > 0.0:
                d_anchor, d_src = fixed_d, 'record'
                record_fixed_d = fixed_d
        if d_anchor is None and depth is not None:
            _n = np.asarray(normal, float)
            n0 = _n / np.linalg.norm(_n)
            if float(n0 @ np.array([0.0, 0.0, 1.0])) < 0:
                n0 = -n0
            # solve_frame 内部同样先 `order_quad(quad_hint)`，此处保持同一块面。
            _quad_o = order_quad(quad)
            Dm2, peak2 = SO._plane_dist_mm(n0, depth, _quad_o, k, stride=2)
            if Dm2 is not None and peak2 >= 0.35:
                Dm, peak, d_src = Dm2, peak2, 'depth_s2'
            else:
                Dm, peak = SO._plane_dist_mm(n0, depth, _quad_o, k, stride=1)
                d_src = 'depth_s1'
            d_peak = float(peak)
            if Dm is not None and peak >= POSE_D_LOCK_INLIER:
                d_anchor = float(Dm)
            else:
                d_src = 'fallback'
    d_lock_use = d_anchor if d_anchor is not None else opts.get('d_lock', 'depth')

    diag = {
        'source': 'pose',
        'model': model_src,
        'conf': round(det.conf, 4),
        'n_candidates': len(dets),
        'pick': pick,
        'candidate_boxes': _cand_bbox,
        'candidate_centers_uv': _cand_centers,
        'candidate_conf': _cand_conf,
        'target_mm': [float(target_mm[0]), float(target_mm[1])],
        'long_side_parallel': bool(long_side_parallel),
        'edge_n_points': [None if ln is None else int(ln.n_points)
                          for ln in lines],
        'edge_residual_px': [None if ln is None else round(float(ln.residual_px), 4)
                             for ln in lines],
        'pose_quad': np.asarray(quad, float).tolist(),
        'use_pos': list(use_pos),
        'd_anchor': d_anchor,
        'd_peak': d_peak,
        'd_anchor_src': d_src,
    }

    # ---- partial 帧：按可靠性选两条相邻边 ----------------------------------
    # 模型给三条可见边时，不要求三条都一致：按 mean/min conf、参与点数、
    # 线残差、贴边程度、靠中心程度给每条边打分，只保留得分最高的一对**相邻**
    # 边作为位置量测。对边（不共享角）会沿边线方向留一个自由度（实测
    # right+left 偏 510mm、bottom+top 偏 2126mm），不能只靠它们。
    #
    # 四条边完整时保持原行为（全部 use_pos），不做这套选择。
    visible_lines = [i for i in range(4) if lines_solve[i] is not None]
    d_gate = record_fixed_d if record_fixed_d is not None else d_anchor
    selected_pair = None
    if len(visible_lines) <= 3:
        sel = _select_edge_pair(lines, det.kp_xy, det.kp_conf, w, h)
        diag['edge_reliability'] = sel
        diag['visible_edges'] = sel['visible']
        scores = sel['scores']
        dropped = list(sel['dropped'])
        if d_gate is not None:
            if sel['pair'] is None:
                reason = sel['reason'] or 'no_reliable_edge_pair'
                _put_edge_selection_diag(diag, det.kp_xy, det.kp_conf,
                                         scores, None, dropped, reason)
                diag['reject'] = reason
                return None, _stage_diag(diag)
            selected_pair = tuple(int(i) for i in sel['pair'])
            use_pos = list(selected_pair)
            diag['use_pos'] = list(use_pos)
            diag['use_pos_selected'] = list(use_pos)
            _put_edge_selection_diag(
                diag, det.kp_xy, det.kp_conf, scores, selected_pair, dropped,
                _edge_select_reason(sel, applied=True))
        else:
            # 没有 D（free_d / call 0）：两条边无法同时定出 D 和位姿，保留
            # >=3 条可见边走自由 D（旧行为）；只有 <=2 条才拒。
            diag['use_pos_selected'] = None
            diag['edge_pair_note'] = 'selection_skipped_no_D'
            if len(visible_lines) >= 3:
                use_pos = list(visible_lines)
                diag['use_pos'] = list(use_pos)
                _put_edge_selection_diag(
                    diag, det.kp_xy, det.kp_conf, scores, None, dropped,
                    'selection_skipped_no_D')
            else:
                reason = ('opposite_pair_only'
                          if sel['reason'] == 'opposite_pair_only'
                          else 'edge_pair_without_D')
                _put_edge_selection_diag(diag, det.kp_xy, det.kp_conf,
                                         scores, None, [], reason)
                diag['reject'] = reason
                return None, _stage_diag(diag)
    else:
        diag['visible_edges'] = visible_lines
        scores = [None if lines_solve[i] is None else
                  _edge_reliability(lines[i], i, det.kp_xy, det.kp_conf, w, h)
                  for i in range(4)]
        _put_edge_selection_diag(diag, det.kp_xy, det.kp_conf, scores,
                                 None, [], 'full_four_edges')

    # ---- partial 帧：可见边与平面几何一致性闸门 ------------------------------
    # 给了 selected_pair 时硬闸门是两条相邻边在 `{n0·P=D}` 平面内的夹角；
    # 三条都在时第三条只作为 `three_edge_diag` 诊断，不参与硬判定。
    # 为什么必须在 solve 前拒：bag2 模型选出的 (bottom,right) 在 record D 下
    # 夹角只有 73°（不是同一块托盘），任何 4-DOF 解都只能拟合错线。
    partial = (len(use_pos) < 4) or any(ln is None for ln in lines_solve)
    if partial:
        visible, supported, vreject = _partial_visible_gate(
            lines_solve, d_gate is not None, pair=selected_pair)
        diag['visible_edges'] = visible
        if vreject:
            diag['reject'] = vreject
            if vreject == 'unsupported_visible_edge_set':
                diag['edge_plane_check'] = {
                    'checked': False, 'passed': None,
                    'unsupported_set': visible,
                    'reason': 'unsupported_visible_edge_set',
                }
            return None, diag
        if selected_pair is not None and d_gate is None:
            # 两条边必须配合已知 D；自由 D 下只剩一个自由度，不能解。
            diag['reject'] = 'edge_pair_without_D'
            diag['edge_plane_check'] = {
                'checked': False, 'passed': None, 'reason': 'pair_requires_D',
                'pair': list(selected_pair), 'D_mm': None, 'D_src': None,
            }
            return None, diag
        if not supported:
            # 可见边 >=3 条但不是本闸门支持的形态，且没有 D 可判：显式跳过。
            diag['edge_plane_check'] = {
                'checked': False, 'passed': None,
                'unsupported_set': visible,
                'reason': 'unsupported_set_no_D',
                'D_mm': None, 'D_src': None,
            }
        elif d_gate is None:
            diag['edge_plane_check'] = {
                'checked': False, 'passed': None, 'reason': 'no_D',
                'D_mm': None, 'D_src': None,
                'target_edge_mm': (float(max(target_mm)) if long_side_parallel
                                   else float(min(target_mm))),
                'target_edge_src': ('long_side' if long_side_parallel
                                    else 'short_side'),
                'ratio_range': [float(EDGE_PLANE_RATIO_RANGE[0]),
                                float(EDGE_PLANE_RATIO_RANGE[1])],
                'right_angle_tol_deg': float(EDGE_PLANE_RIGHT_ANGLE_TOL_DEG),
                'parallel_tol_deg': float(EDGE_PLANE_PARALLEL_TOL_DEG),
            }
        else:
            epc = _check_edge_plane_consistency(
                lines_solve, k, normal, d_gate, target_mm,
                long_side_parallel=bool(long_side_parallel),
                pair=selected_pair)
            epc['D_src'] = ('record' if record_fixed_d is not None else d_src)
            if (epc.get('three_edge_contradicts')
                    and diag.get('edge_reliability', {}).get('dropped_reliable')):
                # 被丢弃的边本身可靠，却与选中的两条矛盾：只记录，不参与解算。
                epc['reliable_dropped_edge_contradiction'] = True
            diag['edge_plane_check'] = epc
            # 用户裁定（2026-11-09）：call 0 已经提前测好 base_link 下的法向和
            # 托盘 D；现场台面被箱子遮挡时，不能用当前帧的“反投影夹角”去否决
            # 选中的边。夹角/三边一致性只作为诊断，不拒绝输出。
            if epc.get('checked') and not epc.get('passed'):
                epc['gate'] = 'diagnostic_only'

    frame, sdiag = SE.solve_frame(
        lines_solve, k, normal, size_mm=target_mm,
        long_side_parallel=bool(long_side_parallel), depth=depth_for_solve,
        use_pos=use_pos, d_lock=d_lock_use, quad_hint=quad,
        d_lock_inlier=POSE_D_LOCK_INLIER)

    if frame is None:
        diag.update(sdiag)
        diag.setdefault('reject', 'solve_failed')
        return None, _stage_diag(diag)

    frame['_quad_px'] = np.asarray(quad, float)

    # ---- partial + 固定 D：从两条相邻观测线的交点重建 origin -----------------
    # partial 解可能落到镜像盆地：可见边残差同样小，但 origin 跑到托盘
    # 对称位置。观测线交点 + frame 轴是唯一确定的那一组角点，先重建；只有重建
    # 后的 origin 明显更靠近关键点 0 才采用（kp0 位置稳、置信度低也能用）。
    if (partial and selected_pair is not None
            and isinstance(d_lock_use, (int, float))
            and not isinstance(d_lock_use, bool)):
        cand = _rebuild_origin_from_line_pair(
            frame, lines_solve, selected_pair, k, float(d_lock_use))
        if cand is not None:
            try:
                kp0 = np.asarray(det.kp_xy, float).reshape(-1, 2)[0]
                o = np.asarray(frame['origin'], float)
                oc = np.asarray(cand['origin'], float)
                d_old = float(np.linalg.norm(
                    _project_px(o[None, :], k)[0] - kp0))
                d_new = float(np.linalg.norm(
                    _project_px(oc[None, :], k)[0] - kp0))
                if np.isfinite(d_new) and d_new < 0.5 * d_old:
                    frame = cand
                    diag['origin_rebuilt_from_pair'] = True
                    diag['origin_rebuild_kp0_px'] = [round(d_old, 2),
                                                     round(d_new, 2)]
            except (TypeError, ValueError, IndexError, KeyError):
                pass

    # ---- 关键点序号消歧：frame origin 必须对到 12 点里的第 0 点 --------------
    # 两邻边 partial 解有 180° 对称解；D 相同、D 闸门看不出来，但原点会落到
    # 对角角点（位移≈托盘对角线，z 差≈对角线×sin 倾角）。用关键点 0 的
    # 像点选正确的那一支，属于选解，不是拒绝。
    if partial:
        frame, _flip, _d0, _d_alt = _align_frame_origin_to_kp0(
            frame, det.kp_xy, k)
        diag['origin_kp_flip'] = bool(_flip)
        diag['origin_kp_dist_px'] = None if _d0 is None else round(_d0, 2)
        diag['origin_kp_alt_dist_px'] = None if _d_alt is None else round(_d_alt, 2)

    # 尺寸用本帧实际锚定的 D：固定 D（call 0 记录）就直接用，避免再走一次
    # `D_from_frame` 的几何反解；没有锚定值才从 frame 反解。
    D_meas = (float(d_lock_use)
              if isinstance(d_lock_use, (int, float)) and not isinstance(d_lock_use, bool)
              else D_from_frame(frame))
    measured = _measure_size_from_edges(
        lines, k, frame.get('nrm', normal), D_meas)
    if measured is not None:
        diag['measured_size_mm'] = measured[0]
        diag['measured_size_spread_mm'] = measured[1]
        diag['measured_size_src'] = 'pose_edges'

    diag.update({
        'reproj_px': sdiag.get('reproj_px'),
        'd_lock': sdiag.get('d_lock'),
        'depth_mm': sdiag.get('depth_mm'),
        'inplane_deg': sdiag.get('inplane_deg'),
        'iterations': sdiag.get('iterations'),
        # 数值 d_lock 时 solve_frame 不再回传峰占比，这里保留本帧那一次测量。
        'd_peak': (d_peak if d_anchor is not None else sdiag.get('d_peak')),
    })
    return frame, _stage_diag(diag)
