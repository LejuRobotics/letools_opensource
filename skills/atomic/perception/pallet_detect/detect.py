#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**初值层**：YOLO-OBB 推理 → `solve_obb.frame_from_obb` → 台面坐标系 `frame`。

旧的无初值路线（`algorithm.py`：颜色/深度掩码 + 旋转矩形搜索）已整体删除。
现在检测**必须有 OBB 模型** —— 模型缺失 / 加载失败**立即抛异常**，不静默返回
`None`（见下 §模型）。OBB 只给"大概在哪、面内转多少度"，真正的位姿由
`solve_obb.frame_from_obb` 用**锁法向 + 四条边所在直线**的 4 自由度解算给出
（为什么用边不用角、为什么必须锁法向，见 `solve_obb.py` 文件头）。

    from pallet_detect import detect_pallet_frame
    frame, diag = detect_pallet_frame(color, depth, k, normal=nrm)

`color` / `depth` / 返回值 `frame` 的约定与 `geom.py` 一致；**本文件不 import ROS**。

## 输入输出

| | 形状 / 类型 | 说明 |
|---|---|---|
| `color` | `(H, W, 3)` uint8 **BGR** | 模型看到的那张（数据集坐标系） |
| `depth` | `(H, W)` float32 **毫米** | 无效像素为 0；**只用于退化帧锁 D** |
| `k` | `CameraIntrinsics` | `fx, fy, cx, cy`（数据集坐标系等效内参） |
| `normal` | `(3,)` | 台面法向**先验**（相机系，**z<0** 也行，内部翻到 z>0） |

返回 `(frame | None, diag)`：`frame` 是 `geom.py` 约定的台面坐标系（相机系毫米），
`None` = 本帧没检出（`diag['reject']` 有值）；`diag` 还带 `reproj_px` / `inplane_deg`
/ `depth_mm` / `conf` 等落点信息。

## 模型（**硬失败**：缺失 / 加载失败直接抛）

模型依赖**必须**有，不给兜底：

* 入口接受的模型来源（优先级从高到低，都从 `opts` 里取）：
  `opts['session']`（一个已建好的 `onnxruntime.InferenceSession`）→
  `opts['model_path']`（`.onnx` 路径）→ 本模块同目录
  `models/pallet_obb.onnx`。
* 拿不到可用的 session **立刻 `raise`**，错误信息写清**期望路径**、**实际查找的
  路径**、**怎么解决**（导出 `export_onnx.py` / 拷到 `models/`）。
* ⚠️ 不静默返回 `None`、不返回空结果。理由：`None` 在下游是**合法的"本帧没检出"**
  （节点据此不发布、心跳照常），模型缺失混进去后，症状是"一直检不到"——
  与"现场真没有托盘"完全无法区分，等于把配置错误静默成了业务失败。

## 选框

一帧可能出多个 OBB（没有 NMS 的导出图更会出重复框）。默认判据是**离画面下方
中心最近**的那个 —— 与箱子那条链（`carton_obb/obb_decoder.py`）同一约定，因为
托盘同样在俯视相机的正下方。`opts['pick']` 可换成 `'conf'`（置信度最高）或
`'area'`（最大）。

## `long_side_parallel`：面内转角的先验（**必传**）

OBB 的宽高比 ≈ 1，"哪条边是长边"在框上**不可分** —— 靠残差试两个面内转角候选
（`{0, π/2}`）等于把这件事交给数据投票，同一场景不同帧可能投出不同结果。
**这个信息用户给得出、而且给得对**：`long_side_parallel` 说明与画面近平行的
那条托盘边是不是长边，直接定死转角种子（`solve_obb._seed_from_long_side`）。
⚠️ 给错就**明确偏 90°**（而不是被"两个都试"兜住）—— 错误应当可见。

## 法向是**两级**的（`normals.py`，**不要跳过第二级**）

`normal` 只是 **TF 先验**：TF 链通了、转进相机系拿到 `base_link` 的 z，但它与
托盘实际台面法向会有 **0~10°** 的偏差。第二级是**当前帧深度自检**
（`normals.fit_floor_normal`，先验当搜索初值、夹角窗 ±25° 内拟合"画面最大的面"），
在现场实测把 12.4° 的静态偏差纠回来（`too_few_edges` 37/37 全拒 -> 37/37 全过）。

⚠️ 调用方**必须**做第二级。`detect_pallet_frame` 里 `frame_from_obb` 把法向**锁死**
成传进来的那个值（自由度 6→4），所以偏了**没有任何下游步骤会纠正它** ——
0~10° 会直接进位姿。节点（`pallet_detection_node.py`）就是这么做的：
`fit_floor_normal` 失败时才退回 `prior`（那时只是"没被纠正"，不是"错的"）。

    normal = normal_from_tf(...)              # 第一级：TF 先验
    fitted, n_inl = fit_floor_normal(depth, k, normal)   # 第二级：深度自检
    if fitted is None:  fitted = normal       # 拟合失败 -> 退回先验（不拒帧）
    frame, diag = detect_pallet_frame(color, depth, k, normal=fitted)
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

from .geom import CameraIntrinsics, _corners_mm, _project_px
from . import solve_obb as SO
from .normals import fit_floor_normal

# ---- 与旧 `algorithm.py` 同名的两个常量（保持调用点不变） --------------------
TARGET_MM = (1200.0, 1000.0)      # 操作员给：托盘 120cm × 100cm（长, 短）
# 与画面近平行的托盘边是不是长边。**必须由 board.json 给**（现场短边近平行 -> False）。
# 它定死面内转角，见 `solve_obb._seed_from_long_side`。
LONG_SIDE_PARALLEL = False

# 默认权重位置（**不进 git**，见同目录 `models/.gitignore`）。
DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / 'models' / 'pallet_obb.onnx'
# TensorRT engine（与 .onnx 同目录，**不进 git**）。默认 auto：engine 在就用它。
# 默认用 **FP32**（`--noTF32`）：实测 OBB 与 ONNX 的角点差 ≤0.001px、POSE ≤0.03px；
# 同目录的 `*_fp16.engine` 是可选加速版（POSE 上实测中位差 1.4px、最大 22px，
# 因为关键点亚像素差会被四边求交放大），要更快可把 `engine_path` 指过去。
DEFAULT_ENGINE_PATH = (Path(__file__).resolve().parent / 'models'
                       / 'pallet_obb_fp32.engine')
_BACKENDS = ('auto', 'engine', 'onnx')

# ---- OBB 解码参数（与训练 / 导出口径一致） ----------------------------------
PAD_VALUE = 114                   # ultralytics letterbox 的补边值
DEFAULT_IMGSZ = 640
DEFAULT_CONF = 0.25               # 只用于过滤与 NMS，不参与选框打分
DEFAULT_NMS_IOU = 0.45            # 导出图不带 NMS，必须自己去重


# --------------------------------------------------------------------------- #
# 模型加载（**硬失败**）
# --------------------------------------------------------------------------- #
def _select_backend(opts: dict):
    """选推理后端 → `('engine'|'onnx', 路径)`。**纯选择，不加载任何东西**。

    `opts['backend']`：`auto`（默认，engine 文件在就用）/ `engine` / `onnx`。
    `opts['engine_path']` / `opts['model_path']` 可覆盖默认路径。

    ⚠️ `auto` 只在 **engine 文件存在** 时选 engine；engine 存在但加载失败会抛
    （不静默退回 ONNX —— 那会把"engine 导出错了"藏成"性能怎么没提升"）。
    """
    backend = str(opts.get('backend', 'auto') or 'auto').strip().lower()
    if backend not in _BACKENDS:
        raise ValueError(f'backend={backend!r} 不认识，只认 {"/".join(_BACKENDS)}')
    model_path = Path(str(opts.get('model_path') or DEFAULT_MODEL_PATH))
    engine_path = Path(str(opts.get('engine_path') or DEFAULT_ENGINE_PATH))
    if backend == 'onnx':
        return 'onnx', model_path
    if engine_path.is_file():
        return 'engine', engine_path
    if backend == 'engine':
        raise FileNotFoundError(
            f'backend=engine 但 TensorRT engine 不存在：{engine_path}\n'
            f'  怎么解决：在目标机（Orin + TRT 8.5）上导出：\n'
            f'    trtexec --onnx={model_path} --saveEngine={engine_path} '
            f'--noTF32 --buildOnly\n'
            f'  或把 backend 改成 auto/onnx。')
    return 'onnx', model_path


def _resolve_session(opts: dict):
    """`opts` -> `(session, model_src)`。拿不到就抛，见模块 docstring。

    session 可能是 `onnxruntime.InferenceSession`，也可能是
    `trt_backend.TensorRTSession` —— 两者只要求 `get_inputs()[0].name` 与
    `run(None, {name: x})[0]`，检测代码不分叉。
    """
    sess = opts.get('session')
    if sess is not None:
        return sess, '<opts[session]>'

    kind, path = _select_backend(opts)
    if kind == 'engine':
        imgsz = int(opts.get('imgsz', DEFAULT_IMGSZ))
        try:
            from .trt_backend import TensorRTSession
            sess = TensorRTSession(path, expected_input=(1, 3, imgsz, imgsz))
        except Exception as exc:                     # noqa: BLE001
            raise RuntimeError(
                f'托盘 OBB TensorRT engine 加载失败：{path}'
                f'（{type(exc).__name__}: {exc}）。\n'
                f'  怎么解决：确认 engine 是在本机（同一 TensorRT 版本）导出的，'
                f'且 --imgsz 与检测参数 {imgsz} 一致；或改用 backend=onnx。') from exc
        return sess, f'engine:{path}'

    if not path.exists():
        raise FileNotFoundError(
            f'没有托盘 OBB 模型：期望 {path}（或从 opts 传 session / model_path）。\n'
            f'  实际查找的路径：{path}\n'
            f'  怎么解决：\n'
            f'    1) 在有 ultralytics 的开发机上导出：\n'
            f'       cd <本包>/scripts && OMP_NUM_THREADS=1 python3 export_onnx.py '
            f'--weights best.pt --verify\n'
            f'    2) 把导出的 .onnx 拷到 {path.parent}/\n'
            f'    3) 或调用时传 opts={{...}} 指定 model_path / session\n'
            f'  托盘检测**必须**有 OBB 模型，不提供任何回退。')
    try:
        import onnxruntime as ort
    except ImportError as exc:                       # pragma: no cover
        raise RuntimeError(
            f'没有 onnxruntime（{exc}）——托盘 OBB 推理要它。\n'
            f'  怎么解决：装到当前环境，如 `pip install onnxruntime`（GPU 则 '
            f'onxxruntime-gpu，并显式设 LD_LIBRARY_PATH，否则会静默退回 CPU）。') from exc
    # ⚠️ **provider 顺序就是优先级**，第一个能用的胜出。
    #
    # **这里刻意不列 `TensorrtExecutionProvider`。** 它确实更快（Orin AGX 实测
    # 稳态 8.76ms vs CUDA 14.56ms vs CPU 904ms），但**建 session 要 ~3.5 分钟没跑完**
    # （TRT 引擎现场编译；2026-10-15 实测，掐表掐掉的），而 ONNX Runtime 的 TRT EP
    # **默认不把引擎落盘**，所以**每起一次节点都要重编**。为 5.8ms/帧（约占现场整帧
    # 220ms 的 2.6%）让机器人启动多等三分钟，是亏的。
    #
    # 真要换 TRT，用本模块的 engine 后端（`pallet_detect/trt_backend.py`）——
    # 那是预先导出好的静态 engine，启动不编译。
    #
    # ⚠️ CUDA 自己的**首次推理**也要 ~19s（cuDNN 选核），那不是 bug ——
    # 现场首帧日志那条 `本帧 19041ms` 就是它。稳态 14.56ms。
    providers = [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider')
                 if p in ort.get_available_providers()]
    sess_opts = ort.SessionOptions()
    sess_opts.intra_op_num_threads = 1               # 与 OMP_NUM_THREADS=1 同口径
    try:
        return ort.InferenceSession(str(path), sess_options=sess_opts,
                                    providers=providers), str(path)
    except Exception as exc:                         # noqa: BLE001
        raise RuntimeError(
            f'托盘 OBB 模型加载失败：{path}（{type(exc).__name__}: {exc}）。\n'
            f'  怎么解决：确认它是 export_onnx.py 导出的、能在本机 onnxruntime 打开；'
            f'换模型后要重跑 `--verify` 核对输出格式。') from exc


# --------------------------------------------------------------------------- #
# OBB 前/后处理（**照抄** carton_obb/obb_decoder.py，那边有逐式注释与实测依据）
# --------------------------------------------------------------------------- #
def _letterbox(img: np.ndarray, new_shape=(640, 640), *, pad_value: int = PAD_VALUE,
               scaleup: bool = True) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """等比缩放 + **居中**补边到 `new_shape`。返回 `(图, 缩放比, (左补, 上补))`。

    ⚠️ **逐行照抄 ultralytics 的 `LetterBox.__call__`**（`auto=False` 那一支），
    连 `round(dh - 0.1) / round(dh + 0.1)` 这种抵消浮点误差的写法都保留 ——
    补边差一个像素，框就整体平移一个像素。
    """
    import cv2
    h, w = img.shape[:2]
    new_h, new_w = (new_shape, new_shape) if isinstance(new_shape, int) else tuple(new_shape)
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


def _quad_from_xywhr(cx, cy, w, h, angle) -> np.ndarray:
    """`[cx, cy, w, h, 弧度]` → 四角（与 ultralytics `xywhr2xyxyxyxy` 逐式相同）。"""
    c, s = float(np.cos(angle)), float(np.sin(angle))
    v1 = np.array([w / 2.0 * c, w / 2.0 * s])
    v2 = np.array([-h / 2.0 * s, h / 2.0 * c])
    p = np.array([cx, cy])
    return np.vstack([p + v1 + v2, p + v1 - v2, p - v1 - v2, p - v1 + v2])


def _decode(raw: np.ndarray, ratio: float, pad: Sequence[float], *, conf_thr: float,
            orig_shape: Sequence[int]) -> list:
    """ONNX 原始输出 `(N, 6) = [cx, cy, w, h, conf, angle]` → 原图坐标候选（**未去重**）。

    ⚠️ 第 6 列是**原始弧度**（实测：直接当弧度 IoU 0.8728，按 sigmoid 解 0.6524）。
    **换模型时必须用 export_onnx.py `--verify` 复核**：yolo26 的 `OBB26` 头会对角度
    做 sigmoid。

    坐标反变换是**先转四角、再在像素空间减补边** `(u - pad_u) / r`；反过来会得到
    错的框（实测 IoU 0.99 → 0.66）。完全跑出图外的框丢掉。
    """
    out: list = []
    arr = np.asarray(raw, np.float64).reshape(-1, 6)
    arr = arr[arr[:, 4] >= float(conf_thr)]
    if len(arr) == 0:
        return out
    px, py = float(pad[0]), float(pad[1])
    h, w = int(orig_shape[0]), int(orig_shape[1])
    for row in arr:
        q = _quad_from_xywhr(row[0], row[1], row[2], row[3], row[5])
        q[:, 0] = (q[:, 0] - px) / ratio
        q[:, 1] = (q[:, 1] - py) / ratio
        if not (q[:, 0].max() > 0 and q[:, 0].min() < w
                and q[:, 1].max() > 0 and q[:, 1].min() < h):
            continue
        out.append({'corners_px': q, 'conf': float(row[4]), 'angle_rad': float(row[5])})
    return out


def _rot_iou(a: np.ndarray, b: np.ndarray, size) -> float:
    """两个凸四边形的旋转 IoU（栅格化）。"""
    import cv2
    h, w = size
    m1 = np.zeros((h, w), np.uint8)
    m2 = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m1, [np.round(a).astype(np.int32)], 1)
    cv2.fillPoly(m2, [np.round(b).astype(np.int32)], 1)
    u = int((m1 | m2).sum())
    return float((m1 & m2).sum()) / u if u else 0.0


def _nms(dets: list, iou_thr: float, size) -> list:
    """旋转框 NMS，按 `conf` 降序保留。

    ⚠️ **必须自己解耦**：导出图里没有 NMS、也没有 query 上限，同一目标会出 2~3 行
    IoU>0.9 的重复框。用**栅格化旋转 IoU**，别用轴对齐近似（同中心转 45° 会给出
    0.5 的假 IoU）。
    """
    if not dets:
        return []
    order = sorted(range(len(dets)), key=lambda i: -dets[i]['conf'])
    keep: list = []
    for i in order:
        if all(_rot_iou(dets[i]['corners_px'], dets[j]['corners_px'], size) <= iou_thr
               for j in keep):
            keep.append(i)
    return [dets[i] for i in sorted(keep)]


def _pick(dets: list, size, how: str = 'bottom_center') -> Optional[dict]:
    """从若干候选里挑一个。`how` ∈ {`bottom_center`(默认), `conf`, `area`}。"""
    if not dets:
        return None
    if how == 'conf':
        return max(dets, key=lambda d: d['conf'])
    if how == 'area':
        def _area(d):
            q = d['corners_px']
            return float(np.prod(q.max(axis=0) - q.min(axis=0)))
        return max(dets, key=_area)
    h, w = size
    ref = np.array([w / 2.0, float(h)])
    return min(dets, key=lambda d: float(
        np.linalg.norm(d['corners_px'].mean(axis=0) - ref)))


# --------------------------------------------------------------------------- #
# 入口
# --------------------------------------------------------------------------- #
def detect_pallet_frame(color: np.ndarray, depth: np.ndarray, k, *,
                        normal, target_mm=TARGET_MM,
                        long_side_parallel=LONG_SIDE_PARALLEL,
                        prior=None, camera_z_mm=None, deck_z_mm=None,
                        opts=None) -> Tuple[Optional[dict], dict]:
    """YOLO-OBB 推理 + `frame_from_obb` → `(frame | None, diag)`。

    `normal` 是**台面法向外部先验**（必需 —— 法向锁死在它上面，见 `solve_obb.py`）。
    `long_side_parallel` 是**面内转角的先验**：与画面近平行的托盘边是不是长边。
    它是**必须传**的（OBB 的宽高比 ≈ 1，"哪条边是长边"在框上不可分，
    靠残差试两个候选等于数据投票）—— 见 `solve_obb._seed_from_long_side`。
    `prior` / `camera_z_mm` / `deck_z_mm` 收下不用（旧掩码路线遗留），保留只为签名兼容。

    `opts` 可带：`session` / `model_path` / `imgsz` / `conf` / `nms_iou` /
    `pick` / `tilt_deg` / `d_range`。模型缺失或加载失败**抛异常**（见模块 docstring）。

    ⚠️ **单位**：`target_mm` 是毫米、`frame['origin']` 也是毫米（`frame_from_obb`
    内部从头到尾毫米；曾经在这里乘过 1000，位姿飞到 1000 米外而残差照样小）。
    """
    opts = dict(opts or {})
    imgsz = int(opts.get('imgsz', DEFAULT_IMGSZ))
    conf_thr = float(opts.get('conf', DEFAULT_CONF))
    nms_iou = float(opts.get('nms_iou', DEFAULT_NMS_IOU))
    pick = str(opts.get('pick', 'bottom_center'))

    # 模型：缺失 / 加载失败 -> 抛（不静默返回 None）
    sess, model_src = _resolve_session(opts)

    img = np.asarray(color)
    if img.ndim != 3 or img.shape[2] != 3:
        return None, {'reject': 'bad_color', 'shape': list(img.shape)}
    h, w = int(img.shape[0]), int(img.shape[1])

    x, ratio, pad = _to_input_tensor(img, imgsz)
    try:
        in_name = sess.get_inputs()[0].name
        raw = sess.run(None, {in_name: x})[0]
    except Exception as exc:                         # noqa: BLE001
        return None, {'reject': 'infer_failed', 'why': f'{type(exc).__name__}: {exc}'}

    dets = _decode(np.asarray(raw)[0].T, ratio, pad, conf_thr=conf_thr,
                   orig_shape=(h, w))
    dets = _nms(dets, nms_iou, (h, w))
    det = _pick(dets, (h, w), how=pick)
    if det is None:
        return None, {'reject': 'no_detection', 'n_candidates': len(dets),
                      'model': model_src, 'source': 'obb'}

    # ---- 法向第二级：TF 先验 + **当前帧深度自检** ----------------------------
    # `normal` 只是 TF 先验，可能有 0~10° 的静态偏差；`frame_from_obb` 会把它**锁死**
    # （自由度 6→4），所以必须在这里先纠正它（`opts['fit_normal']` 默认 True）。
    # 失败了才退回先验（不是拒帧 —— 那时只是"没被纠正"）。见 `normals.py`。
    n_prior = np.asarray(normal, float)
    n_fit_log = None
    if bool(opts.get('fit_normal', True)) and depth is not None:
        fitted, n_inl = fit_floor_normal(np.asarray(depth), k, n_prior)
        if fitted is not None:
            ang = float(np.degrees(np.arccos(np.clip(abs(float(fitted @ n_prior)), -1, 1))))
            normal = fitted
            n_fit_log = {'n_inliers': int(n_inl), 'delta_deg': round(ang, 3)}
        else:
            n_fit_log = {'n_inliers': int(n_inl), 'delta_deg': None, 'fallback': 'prior'}

    # ---- 初值解算：锁法向 + 四条边所在直线 ------------------------------------
    # 传 `depth` 进去：**退化帧**（只剩两条相邻边、深度不可辨）`frame_from_obb`
    # 会用框内深度主峰把平面距离 D 锁死（见 `solve_obb.py` 文件头）。传的是自检
    # 之后的 `normal`，所以锁 D 用的也是纠正过的法向。
    #
    # `long_side_parallel` **必须传**：它定死面内转角（OBB 的宽高比 ≈ 1，
    # "哪条边是长边"在框上不可分，靠残差试两个候选 = 数据投票、不可靠）。
    # 这是用户给的先验、优先级高于任何数据推断。见 `solve_obb._seed_from_long_side`。
    size_mm = [float(target_mm[0]), float(target_mm[1])]
    frame, fdiag = SO.frame_from_obb(
        {'corners_px': det['corners_px']}, k, normal, size_mm=size_mm,
        tilt_deg=float(opts.get('tilt_deg', SO.TILT_DEG)),
        depth=None if depth is None else np.asarray(depth),
        d_range=opts.get('d_range'),
        long_side_parallel=long_side_parallel)

    diag = {'source': 'obb', 'model': model_src, 'conf': round(det['conf'], 4),
            'n_candidates': len(dets), 'pick': pick, 'target_mm': size_mm,
            'long_side_parallel': bool(long_side_parallel)}
    if n_fit_log is not None:
        diag['normal_fit'] = n_fit_log
    diag.update(fdiag or {})
    if frame is None:
        diag.setdefault('reject', 'lsq_failed')
        return None, diag
    # 把 OBB 四角挂回 frame（**下划线键，不进任何契约**）：`refine_pallet_frame`
    # 的用户规则开关要用它重跑一次 `frame_user_rule`（见 `refine_depth` 的入口）。
    frame['_obb_corners_px'] = np.asarray(det['corners_px'], float)
    return frame, diag
