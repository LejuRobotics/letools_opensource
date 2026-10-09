#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纸箱检测的**纯函数核心** —— 前处理 / 后处理，不 import ROS、不 import ultralytics。

`carton_box_detect.py`（ROS 节点）只做搬运：收图像 → 调这里 → 发话题。推理本身
用 onnxruntime 跑一个 `.onnx` 文件，**不依赖 ultralytics 的版本**。

为什么是 ONNX 而不是 `.pt`
--------------------------
`.pt` 权重把「网络结构」存成了一份 `yaml` 描述，加载时由**当前装的 ultralytics**
去实例化。于是同一个权重在不同 ultralytics 版本下会给出**不同的数**：

    ultralytics 8.4.41（训练用的）  conf=0.986  框=[212.98, 243.79, 470.48, 422.62]
    ultralytics 8.3.163（仓库里现成）conf=0.911  框=[213.22, 244.19, 468.08, 422.98]

**不报错，只是数字悄悄变了** —— 最阴的一类坑。而 LeTools 里现成的 pin 是
`ultralytics==8.3.163`（`third_party/basket_vision` 的 jetpack5 运行时清单）。
导成 ONNX 之后结构被冻结进图里，推理端只认 onnxruntime，这条漂移就消失了。

实测（同一份权重、同一张图、同一个输入张量）：

    PyTorch 原始输出  vs  ONNX 原始输出   最大差 6.1e-05     ← 等价

两个必须知道的点
----------------
1. **letterbox 必须用「方形 640×640 + 补边」**，不能用「rect / 不补边」。
   训练时 ultralytics 用的是方形补边（`LetterBox(new_shape=(640,640), auto=False)`），
   推理要跟训练一致。实测 8_test 上两种模式差 6.7px（v 方向），原因是那批数据里
   有 8 帧的标注是**由无补边的模型产出的**（labels 的 v 正好在 0/480 边界），
   方形补边才是训练分布。6_test 上有四角真值，两种模式的最终精度一样
   （11.6 vs 11.7px），所以按「与训练一致」这条走，不吃亏。

2. **输出是 NMS-free 的**（yolo26 的端到端头），形状 `(1, 300, 6)`：
   `[x1, y1, x2, y2, conf, cls]`，**每行已经是一个目标**，不需要再做 NMS。
   300 是 query 数上限，没有目标的那些行 conf≈0（实测第 2 名 query 只有 0.0014）。

运行与测试
----------
    # 纯函数自检（不需要 onnxruntime、不需要 ROS、不需要模型文件）
    python3 tests/test_carton_detector.py

    # 导出 ONNX（在装了 ultralytics 的开发机上跑一次）
    python3 scripts/export_onnx.py --weights /path/to/best_cartonbb.pt
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 与训练一致的 letterbox 参数（ultralytics 的默认值）
PAD_VALUE = 114                 # 灰度填充值
STRIDE = 32


def letterbox(img: np.ndarray, new_shape=(640, 640), *,
              pad_value: int = PAD_VALUE, scaleup: bool = True,
              center: bool = True, stride: int = STRIDE
              ) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """等比缩放 + 补边到 `new_shape`。返回 `(图, 缩放比, (左侧补边, 上方补边))`。

    **逐行照抄 ultralytics 的 `LetterBox.__call__`**（`auto=False` 那一支），
    包括 `round(dh - 0.1) / round(dh + 0.1)` 这种为了抵消浮点误差的写法 ——
    补边差一个像素，框就会整体平移一个像素。

    `scaleup=True` 表示小图也会放大（训练时的行为）。`scaleup=False` 时只缩不放，
    那是 ultralytics **验证**时的口径（为了 val mAP 更好看），推理不要用。
    """
    h, w = img.shape[:2]
    new_h, new_w = (new_shape, new_shape) if isinstance(new_shape, int) else tuple(new_shape)

    r = min(new_h / h, new_w / w)
    if not scaleup:
        r = min(r, 1.0)

    new_unpad = (round(w * r), round(h * r))
    dw = new_w - new_unpad[0]
    dh = new_h - new_unpad[1]
    if center:
        dw /= 2
        dh /= 2

    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        if img.ndim == 2:
            img = img[..., None]

    top, bottom = (round(dh - 0.1) if center else 0), round(dh + 0.1)
    left, right = (round(dw - 0.1) if center else 0), round(dw + 0.1)
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=(pad_value,) * img.shape[2])
    return img, float(r), (float(left), float(top))


def to_input_tensor(img_bgr: np.ndarray, imgsz: int = 640, *,
                    pad_value: int = PAD_VALUE, scaleup: bool = True
                    ) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """BGR 图（cv2 / cv_bridge 给的那种）→ `(1,3,H,W)` float32 输入张量。

    顺序是 **BGR → RGB → HWC → CHW → /255**。RGB 这一步是必须的：实测漏掉它
    框会整体偏掉；而做对了之后与 ultralytics 的原始输出**逐位一致**（6.1e-05）。
    """
    lb, r, pad = letterbox(img_bgr, (imgsz, imgsz),
                           pad_value=pad_value, scaleup=scaleup)
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(x), r, pad


def decode(raw: np.ndarray, ratio: float, pad: Sequence[float],
           conf_thr: float = 0.25, orig_shape: Optional[Sequence[int]] = None,
           max_det: int = 300) -> List[dict]:
    """ONNX 原始输出 → 原图坐标下的检测框列表，按置信度从高到低。

    `raw` 形状 `(N, 6)`：`[x1, y1, x2, y2, conf, cls]`，**已经是 NMS 之后的结果**
    （yolo26 端到端头），所以这里**不做 NMS**。没有目标的 query 行 conf 约等于 0，
    被 `conf_thr` 一并滤掉。

    `orig_shape` 给了就顺带把**完全跑出图外**的框丢掉 —— 那种框在 letterbox 的
    补边区域里，是真模型偶尔会吐的噪声。
    """
    out: List[dict] = []
    if raw is None or len(raw) == 0:
        return out
    arr = np.asarray(raw, np.float64).reshape(-1, 6)
    keep = arr[:, 4] >= float(conf_thr)
    arr = arr[keep]
    if len(arr) == 0:
        return out
    # 按置信度降序，最多留 max_det 个
    arr = arr[np.argsort(-arr[:, 4])][:int(max_det)]

    px, py = float(pad[0]), float(pad[1])
    boxes = arr[:, :4].copy()
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - px) / ratio
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - py) / ratio

    if orig_shape is not None:
        h, w = int(orig_shape[0]), int(orig_shape[1])
        inside = ((boxes[:, 2] > 0) & (boxes[:, 0] < w)
                  & (boxes[:, 3] > 0) & (boxes[:, 1] < h))
        boxes, arr = boxes[inside], arr[inside]

    for b, row in zip(boxes, arr):
        out.append({
            'box_uv': [float(b[0]), float(b[1]), float(b[2]), float(b[3])],
            'confidence': float(row[4]),
            'class_id': int(round(float(row[5]))),
        })
    return out


# --------------------------------------------------------------------------- #
# 选箱打分：置信度 + 面积 + 中心深度
# --------------------------------------------------------------------------- #
# ⚠️ **这三个权重与两个距离边界是拍的，没有数据支撑。** 参数化就是为了让现场
# 能调 —— 默认值只是起点。落地后应当拿一帧多箱子的真实数据核一遍。
DEFAULT_SCORE_W_CONF = 0.5
DEFAULT_SCORE_W_AREA = 0.2
DEFAULT_SCORE_W_DEPTH = 0.3
DEFAULT_Z_NEAR_MM = 500.0        # 近于此 = 深度项满分
DEFAULT_Z_FAR_MM = 1500.0        # 远于此 = 深度项 0
DEFAULT_DEPTH_PATCH_PX = 15      # 中心采样方块半边长


def box_area_px(box_uv: Sequence[float]) -> float:
    """框面积（像素²）。宽或高为负时按 0 算。"""
    u0, v0, u1, v1 = (float(t) for t in box_uv[:4])
    return max(0.0, u1 - u0) * max(0.0, v1 - v0)


def depth_score(z_mm: float, *, z_near: float = DEFAULT_Z_NEAR_MM,
                z_far: float = DEFAULT_Z_FAR_MM) -> float:
    """中心深度 → `[0,1]`，**越近分越高**。

    `z <= z_near` 得 1.0、`z >= z_far` 得 0.0，中间线性。`z_far <= z_near`
    （参数写反）时退化成阶跃：近于 `z_near` 得 1，否则 0。
    """
    if z_far <= z_near:
        return 1.0 if float(z_mm) <= float(z_near) else 0.0
    t = (float(z_far) - float(z_mm)) / (float(z_far) - float(z_near))
    return float(min(1.0, max(0.0, t)))


def sample_depth_mm(depth_mm, box_uv: Sequence[float], *,
                    half: int = DEFAULT_DEPTH_PATCH_PX,
                    z_near: float = DEFAULT_Z_NEAR_MM,
                    z_far: float = DEFAULT_Z_FAR_MM) -> Optional[float]:
    """框**中心**一个 `(2*half+1)²` 方块内的**有效深度中位数**；取不到返回 `None`。

    `depth_mm` 是 `(H, W)` 的**毫米**深度图（调用方负责把 `32FC1` 的米换算好）。

    **为什么取中位数而不是中心那一个像素**：深度图到处是空洞与飞点（黑箱子、
    反光、遮挡边缘），单点很容易落空，一帧有值一帧没值会让排序跳。中位数对
    少数坏值不敏感，且不需要额外的参数。

    **有效像素** = `z_near <= z <= z_far`（顺便滤掉 0 与 `65535` 这类哨兵）。
    落在窗口外的深度**不参与中位数** —— 那多半是背景或别的物体，不是这个箱子。

    方块越界时**裁剪**到图像范围内；裁剪后一个有效像素都没有 → `None`。
    """
    if depth_mm is None:
        return None
    arr = np.asarray(depth_mm)
    if arr.ndim != 2 or arr.size == 0:
        return None

    h, w = int(arr.shape[0]), int(arr.shape[1])
    u0, v0, u1, v1 = (float(t) for t in box_uv[:4])
    cu, cv = (u0 + u1) / 2.0, (v0 + v1) / 2.0

    r = max(0, int(half))
    c0 = max(0, int(round(cu)) - r)
    c1 = min(w, int(round(cu)) + r + 1)
    r0 = max(0, int(round(cv)) - r)
    r1 = min(h, int(round(cv)) + r + 1)
    if c1 <= c0 or r1 <= r0:                       # 中心完全在图外
        return None

    patch = arr[r0:r1, c0:c1].astype(np.float64, copy=False).ravel()
    ok = np.isfinite(patch) & (patch >= float(z_near)) & (patch <= float(z_far))
    if not ok.any():
        return None
    return float(np.median(patch[ok]))


def candidate_score(conf: float, area_px: float, z_mm: Optional[float], *,
                    max_area_px: float,
                    w_conf: float = DEFAULT_SCORE_W_CONF,
                    w_area: float = DEFAULT_SCORE_W_AREA,
                    w_depth: float = DEFAULT_SCORE_W_DEPTH,
                    z_near: float = DEFAULT_Z_NEAR_MM,
                    z_far: float = DEFAULT_Z_FAR_MM) -> Tuple[float, dict]:
    """一个候选的 `(总分, 分项明细)`。分项明细给日志用，**不进契约**。

    三个量归一化到 `[0,1]` 后加权求和：

        score = w_conf * conf
              + w_area * (area_px / max_area_px)          # 同帧相对归一
              + w_depth * depth_score(z_mm)

    **面积为什么用同帧最大值归一**：这样自动适应相机分辨率与箱子尺寸，不必
    引入一个"面积满分对应多少像素"的常量（那种常量换相机就得跟着改）。
    代价是**单候选时该项恒为 1**，退化成常数 —— 此时本来也不需要它来区分。

    ⚠️ **`z_mm is None` 时把深度项的权重按比例让给另两项**：

        score = (w_conf * conf + w_area * area_norm) / (w_conf + w_area)

    取不到深度**不罚** —— 深度空洞往往是近物遮挡造成的，罚它会适得其反。
    代价是**归一化到 `[0,1]`**：一个"在 conf 与面积上满分"的候选，无论有没有
    深度都拿 1.0，所以深度项**只能压低远处的候选，不能额外抬高近处的**。
    全部权重为 0 时返回 0.0（不除零）。
    """
    c = float(min(1.0, max(0.0, float(conf))))
    a = 0.0 if max_area_px <= 0.0 else min(1.0, max(0.0, float(area_px) / float(max_area_px)))
    d = None if z_mm is None else depth_score(z_mm, z_near=z_near, z_far=z_far)

    if d is None:
        wsum = float(w_conf) + float(w_area)
        if wsum <= 0.0:
            score = 0.0
        else:
            score = (float(w_conf) * c + float(w_area) * a) / wsum
    else:
        score = float(w_conf) * c + float(w_area) * a + float(w_depth) * d

    return score, {'conf': c, 'area_norm': a, 'depth': d, 'z_mm': z_mm}


def score_candidates(dets: Sequence[dict], z_list: Optional[Sequence] = None,
                     **score_kw) -> List[Tuple[dict, float, dict]]:
    """给每个候选打分，返回 `[(det, score, parts), ...]`（**顺序与 `dets` 一致**）。

    `z_list` 与 `dets` 等长，元素是各自中心深度或 `None`；不给就全部按"无深度"
    处理。面积归一的基准 `max_area_px` 从**本帧全部候选**里取，不用外部常量。
    """
    areas = [box_area_px(d['box_uv']) for d in dets]
    max_area = max(areas) if areas else 0.0
    zs = list(z_list) if z_list is not None else [None] * len(dets)

    out: List[Tuple[dict, float, dict]] = []
    for i, d in enumerate(dets):
        z = zs[i] if i < len(zs) else None
        score, parts = candidate_score(d.get('confidence', 0.0), areas[i], z,
                                       max_area_px=max_area, **score_kw)
        out.append((d, score, parts))
    return out


def pick_best(dets: Sequence[dict], label: Optional[int] = None,
              z_list: Optional[Sequence] = None,
              **score_kw) -> Optional[dict]:
    """从若干检测里挑一个，没有就返回 `None`。

    判据是**置信度 + 面积 + 中心深度**三项归一化加权求和，取最高分 ——
    见 `candidate_score`。**不再是单纯的"面积最大"或"置信度最高"**：那两个
    各自都有反例（邻箱可能更大，邻箱的置信度也常常很高）。

    `label` 为 `None` 时不筛类别（**先筛类别，再在类别内打分**）。
    `z_list` 见 `score_candidates`。分数并列时取**靠前**的那个（`max` 语义）。

    只想要分数明细（比如打日志）时用 `score_candidates`。
    """
    cand = [(i, d) for i, d in enumerate(dets)
            if label is None or d['class_id'] == int(label)]
    if not cand:
        return None
    zs = list(z_list) if z_list is not None else [None] * len(dets)
    sub = [d for _, d in cand]
    sub_z = [zs[i] if i < len(zs) else None for i, _ in cand]
    scored = score_candidates(sub, sub_z, **score_kw)
    return max(scored, key=lambda t: t[1])[0]


def box_to_polygon_points(box_uv: Sequence[float]) -> List[Tuple[float, float]]:
    """`(u0,v0,u1,v1)` → **两个点** `[(左上), (右下)]`。

    这是 `/box/yolo_box`（`geometry_msgs/PolygonStamped`）的契约顺序，由下游
    `box_detection` 包的 `_box_uv_from_msg` 定死：`points[0]`=左上、
    `points[1]`=右下。**给反了不报错** —— 那边取的是两点的外接框，顺序反了照样
    能算出同一个框（它就是为容忍这个才写的 `min/max`）。但契约还是要照写，
    免得将来换成别的消费者。
    """
    u0, v0, u1, v1 = (float(t) for t in box_uv[:4])
    return [(min(u0, u1), min(v0, v1)), (max(u0, u1), max(v0, v1))]
