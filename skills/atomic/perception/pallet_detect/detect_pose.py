#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**POSE 初值层**：YOLO11-pose 推理 → 四边直线 → `solve_edges.solve_frame` → frame。

与 `detect.detect_pallet_frame`（OBB 路线）同一契约::

    from pallet_detect.detect_pose import detect_pallet_frame_pose
    frame, diag = detect_pallet_frame_pose(color, depth, k, normal=nrm)

链路：

    detect._to_input_tensor                 BGR -> (1,3,640,640)（**直接复用**，不抄）
    pose_decode.decode_pose / nms_pose / pick_pose   ONNX -> 12 关键点
    pose_decode.fit_edges                   每条边 2~4 点加权 TLS -> 4 条直线
    pose_decode.intersect_edges             相邻边求交 -> quad（失败时逐角兜底）
    solve_obb.usable_edges                  OBB 的保守可用边口径 -> use_pos
    solve_edges.solve_frame                 四条直线 + 锁法向 -> 4-DOF frame

⚠️ `d_lock` 默认 `'depth'`：POSE 路线默认把 D 锚到深度主峰（计划 §3.5），
因为实机 bag 已证明几何解出的 D 不稳。`opts['d_lock']` 可覆盖
（`None` = 只在可用边不足 4 条时锁、数值 = 强制该 D）。

⚠️ `quad` **只用于** `solve_frame` 的 `quad_hint`（PnP 初值/诊断）以及挂到
`frame['_obb_corners_px']` 供 `refine_pallet_frame` 复用；最终位姿由四条边解出。

模型（**硬失败，不做任何回退**）：`opts['session']` → `opts['model_path']` →
本包 `models/pallet_pose.onnx`。缺失/加载失败直接抛异常，错误信息写清路径与
导出命令。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import pose_decode as PD
from . import pose_edge_refine as PER
from . import solve_edges as SE
from . import solve_obb as SO
from .detect import DEFAULT_CONF, DEFAULT_IMGSZ, DEFAULT_NMS_IOU, _to_input_tensor
from .solve_obb import order_quad, usable_edges

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

#: POSE 前端锁 D 的**主峰占比**阈值。
#:
#: ⚠️ **与 OBB 故意不同**（OBB 用的是 `solve_obb.D_LOCK_INLIER = 0.0`）。
#: 真机 881 帧实测（`maduo/debug/jitter/WORKLOG_jitter.md` §7）：
#:
#:      阈值 | Δpos 中位 | max    | >100mm | 可见角点距
#:      0.30 | 41.36     | 532.6  | 149    | **41.83px**
#:      0.00 | 27.43     | 410.3  | 110    | **52.84px**  ← 抖动更好，精度差 26%
#:
#: 即 **POSE 上是取舍**：锁得更多 → 抖动小，但多一个约束去拽那 1~2 个可见角，
#: 角点距反而变大。板子上现在 `pallet_detector=pose`，所以**默认保持 0.30**
#: （精度优先）。要把这条前端也切到抖动优先，改这个常量即可，不要动
#: `solve_obb.D_LOCK_INLIER`（那是 OBB 的）。
POSE_D_LOCK_INLIER = 0.30

#: 角索引 -> (相邻边 A, 相邻边 B, 直接预测的关键点下标)。
#: 角序 [bl, br, tr, tl]，边序 0,1,2,3 = 下/右/上/左。
_CORNER_FALLBACK = (
    (3, 0, 0),   # bl = edge3 × edge0 ; 关键点 0
    (0, 1, 3),   # br = edge0 × edge1 ; 关键点 3
    (1, 2, 6),   # tr = edge1 × edge2 ; 关键点 6
    (2, 3, 9),   # tl = edge2 × edge3 ; 关键点 9
)


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


def detect_pallet_frame_pose(color: np.ndarray, depth: np.ndarray, k, *,
                             normal, target_mm=(1200., 1000.),
                             long_side_parallel=False,
                             opts=None):
    """YOLO-pose 推理 + `solve_edges.solve_frame` -> `(frame | None, diag)`。

    参数与 `detect.detect_pallet_frame` 同契约：
    `color` 是 BGR uint8、`depth` 是毫米（只用于锁 D）、`k` 是
    `CameraIntrinsics`、`normal` 是台面法向外部先验（z<0 也行，内部翻正）。
    `target_mm` 是托盘物理边长；`long_side_parallel` 是面内转角先验。

    `opts` 可带：
      * `session` / `model_path` / `imgsz`：模型来源与输入尺寸；
      * `conf`（默认 0.25）/ `nms_iou`（0.45）/ `pick`（bottom_center）；
      * `kpt_conf`（默认 0.0，关键点置信度预过滤）；
      * `kconf`（默认 0.5，参与边拟合的最低关键点 conf）；
      * `line_res_px`（默认 12，边拟合逐次剔点的最大垂距）；
      * `d_lock`（默认 `'depth'`，见模块头）；
      * `edge_refine`（默认 True）：初解成功后做一次局部梯度边修正并重解，
        失败时静默保留初解（diag 记 `edge_refine_reject`）；
      * `edge_refine_stride`（默认 16）/ `edge_refine_half`（默认 8）：
        修正段的沿线采样步长 / 法向搜索半窗，见 `pose_edge_refine`；
      * `fit_normal`：**收下不用**（本入口锁在调用方给的法向上，与 OBB 的
        `detect_pallet_frame` 默认自检不同；显式传 False 语义一致）。
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

    x, ratio, pad = _to_input_tensor(img, imgsz)
    try:
        in_name = sess.get_inputs()[0].name
        raw = sess.run(None, {in_name: x})[0]
    except Exception as exc:                         # noqa: BLE001
        return None, {'source': 'pose', 'reject': 'infer_failed',
                      'why': f'{type(exc).__name__}: {exc}', 'model': model_src}

    # ---- 解码 / NMS / 选主目标 ------------------------------------------------
    dets = PD.decode_pose(np.asarray(raw)[0].T, ratio, pad, (h, w),
                          conf_thr, kpt_conf_thr=float(opts.get('kpt_conf', 0.0)))
    dets = PD.nms_pose(dets, nms_iou, (h, w))
    det = PD.pick_pose(dets, (h, w), how=pick)
    if det is None:
        return None, {'source': 'pose', 'reject': 'no_detection',
                      'n_candidates': len(dets), 'model': model_src}

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

    # ---- 深度 D：本帧只量一次 -------------------------------------------------
    # 初解与边修正重解原本各调一次 `_plane_dist_mm`（Orin 上约 10~13ms/次），
    # 但两处用的是同一帧、同一 quad/法向/深度 —— D 完全相同。这里先量一次，
    # 解出数值后两次 `solve_frame` 都以数值 `d_lock` 传下去，跳过内部重复测量。
    #
    # 提速：先按 stride=2 跳采；**主峰占比 >= 0.35 才采信**，否则用全量
    # stride=1 重算。601 帧实测 stride=2 只在 6 帧（`14-22-38`，峰占比
    # 0.25~0.29）上翻到另一个峰（ΔD 约 256~262mm）；0.35 的闸门把这些帧
    # 挡回全量，其余帧的 |ΔD| 中位 0.0068mm / p90 0.022mm。
    d_anchor, d_peak, d_src = None, None, 'fallback'
    if depth is not None:
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

    frame, sdiag = SE.solve_frame(
        lines_solve, k, normal, size_mm=target_mm,
        long_side_parallel=bool(long_side_parallel), depth=depth,
        use_pos=use_pos, d_lock=d_lock_use, quad_hint=quad,
        d_lock_inlier=POSE_D_LOCK_INLIER)

    diag = {
        'source': 'pose',
        'model': model_src,
        'conf': round(det.conf, 4),
        'n_candidates': len(dets),
        'pick': pick,
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
    if frame is None:
        diag.update(sdiag)
        diag.setdefault('reject', 'solve_failed')
        return None, diag

    frame['_obb_corners_px'] = np.asarray(quad, float)

    # ---- 轻量边修正（只多用已解出的 frame，不重跑推理）------------------------
    if bool(opts.get('edge_refine', True)):
        frame2, rdiag = PER.refine_frame(
            frame, color, k, normal, depth, use_pos,
            size_mm=target_mm,
            long_side_parallel=bool(long_side_parallel),
            d_lock=d_lock_use,
            stride=int(opts.get('edge_refine_stride',
                                PER.DEFAULT_STRIDE)),
            half=int(opts.get('edge_refine_half', PER.DEFAULT_HALF)),
            d_lock_inlier=POSE_D_LOCK_INLIER)
        if frame2 is not None:
            frame = frame2                 # 自带修正后四边求交的 _obb_corners_px
            rdiag = {**rdiag, 'applied': True}
        else:
            # 静默回退：保留初解 frame，只记诊断
            diag['edge_refine_reject'] = rdiag.get('reject', 'solve_failed')
            rdiag = {**rdiag, 'applied': False}
        diag['edge_refine'] = rdiag

    diag.update({
        'reproj_px': sdiag.get('reproj_px'),
        'd_lock': sdiag.get('d_lock'),
        'depth_mm': sdiag.get('depth_mm'),
        'inplane_deg': sdiag.get('inplane_deg'),
        'iterations': sdiag.get('iterations'),
        # 数值 d_lock 时 solve_frame 不再回传峰占比，这里保留本帧那一次测量。
        'd_peak': (d_peak if d_anchor is not None else sdiag.get('d_peak')),
    })
    return frame, diag
