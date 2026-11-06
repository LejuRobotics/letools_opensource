#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纸箱 OBB 的**纯函数核心** —— 前处理 / 后处理 / 选箱，不 import ROS、不 import ultralytics。

`carton_obb_detect.py`（ROS 节点）只做搬运：收图像 → 调这里 → 发话题。
推理本身用 onnxruntime 跑一个 `.onnx` 文件。

为什么是 OBB 而不是「框 + 深度边缘拟合」
----------------------------------------
不这么做的理由：

* 旋转角由网络直接给出，不再依赖深度边缘的可观测性（手一挡、深度空洞就退化成
  `yolo_fallback`，而那种帧的四角**照样是个规规矩矩的矩形**，不比对真值看不出来）。
* 少了「平面分割 + 脊线拟合 + 时间窗重算」三段，整条链从 ~60ms/帧 降到 ~4ms/帧。

ONNX 的输入输出（**这两条是实测定的，不是读文档推的**）
-------------------------------------------------------
输入 `images`：`(1, 3, 640, 640)` float32，**方形 letterbox 补边**、BGR→RGB、/255。

输出 `output0`：`(1, 6, 8400)`，转置后每行 `[cx, cy, w, h, conf, angle]`：

* 前四列是**中心 + 宽高，不是角点**，单位是 letterbox 之后的 640 空间。
* **第 6 列是原始弧度，不需要任何变换。** 这三种假设在同一批帧上实测过
  （对真值的旋转 IoU 中位）：直接把第 6 列当弧度 **0.8728**；按
  `(a-0.25)*pi/2` 解 **0.6812**；按 `(sigmoid(a)-0.25)*pi/2` 解 **0.6524**。
  第一种是对的 —— yolo11-obb 的导出图里角度就是弧度，只有 yolo26 的
  `OBB26` 头才做 sigmoid。**换模型时这条要重验。**
* 角度语义：`w` 沿角度方向、`h` 垂直，四角顺序
  `右下 → 左下 → 左上 → 右上`（与 BoxDetection 契约同向，但仍要过 `order_corners_uv`
  兜一次——推导依赖头部的实现细节，不该让契约悬在它上面）。
* **没有 NMS 也没有 query 上限**：8400 个锚点行全在，`conf` 高的可能有几十行。
  实测同一目标会出 2~3 行、IoU>0.9 的重复框，所以**必须自己解耦**，见 `nms_rotated`。
  同一个模型还会出**只框了箱子一角的小框**（整个套在大框里）—— 那类 IoU 天生
  够不着，由 `nms_rotated` 的 `iomin_thr` 兜住，见 `_rot_overlaps`。

选箱：为什么不是「置信度最高」也不是「面积最大」
------------------------------------------------
相机固定俯视，**手里的箱子永远在画面下方中心**（操作员 2026-10-13 确认）。所以判据是
**到「画面下方中心」的距离，越近分越高**，再叠加面积与中心深度。

置信度**不再参与打分**。实测反例：现场 val 的 36 个多框帧里，有一帧手里那个箱子
conf 0.9773、旁边的邻箱 conf **0.9642**，而**邻箱面积还更大**（46147 > 45118）——
「conf 最高」与「面积最大」在这一帧上都会挑错，而「离下方中心最近」挑对了。
距离的分离度实测：手标双箱 10 帧**中位 192.9px（最小 191.5）**、模型预测双箱帧
中位 127.7px。

距离项用**绝对刻度**（`d_near_px` / `d_far_px`）而不是帧内相对归一：相对归一在
单候选时恒为 1（退化成常数），而单候选是绝大多数帧（val 330 帧里 294 帧单箱）。
绝对刻度还顺带能当闸门用 —— 全部候选都远于 `d_far_px` 时这一项全是 0。
"""
from __future__ import annotations

import warnings
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 与训练一致的 letterbox 参数（ultralytics 的默认值）
PAD_VALUE = 114

# ---- 选箱打分：到画面下方中心的距离 + 面积 + 中心深度 ----------------------
# ⚠️ `d_near_px` / `d_far_px` 是 640x480 口径的**绝对刻度**，换分辨率要跟着改
#    （或按 H 的比例给，见 `d_center_score`）。默认值依据：实测手里那个箱子的
#    框中心 d 中位 140px（new_yolo 手标 10 帧）与 val 330 帧的 d 中位 150px，
#    邻箱 244~463px，所以 100/450 让手箱拿 0.89~0.94、邻箱拿 0.03~0.59。
DEFAULT_SCORE_W_DCENTER = 0.5
DEFAULT_SCORE_W_AREA = 0.2
DEFAULT_SCORE_W_DEPTH = 0.3
DEFAULT_D_NEAR_PX = 100.0
DEFAULT_D_FAR_PX = 450.0
DEFAULT_Z_NEAR_MM = 500.0
DEFAULT_Z_FAR_MM = 1500.0
DEFAULT_DEPTH_PATCH_PX = 15

# ---- 深度下沿修正（2026-11-06） -------------------------------------------
# 现场 bag `2026-10-30-19-08-19`：手持箱相对相机静止（宽 349~358、角度 ±0.7°），
# 上边稳定在 ~142，下边却在 365~480 之间跳。箱体深度 580mm，下沿以下背景 830mm，
# 深度跳变稳定在 362~364px；把下边收回去后 h 从 227~333 收到 218~229（std 1.9）。
# ⚠️ 只做**收缩**：找不到明确的远背景跳变就保持模型框；箱子放在料垛上（下方
#    同深度）或下沿出画时不会触发。这是保守的兜底，不是第二个检测器。
DEFAULT_REFINE_BOTTOM_STEP = 2
DEFAULT_REFINE_BOTTOM_BAND_PX = 45
DEFAULT_REFINE_BOTTOM_JUMP_MM = 150.0
DEFAULT_REFINE_BOTTOM_START_PX = 40
DEFAULT_REFINE_BOTTOM_MIN_SHRINK_PX = 3.0

# ---- 旋转 NMS 的重叠判据（见 `nms_rotated`） -------------------------------
# `iou_thr` 管「两个框互相错着盖」（同目标的两条预测）。
# `iomin_thr` 管「小框整个套在大框里」——**这一类比 IoU 管不着**：小框被包住时
# `IoU = 交/并 ≈ 小面积/大面积`。实测这个模型会把一个箱子拟成 47 个原始框，
# 里面就有「只框了一小块」的碎框（IoU 0.25、IoMin 1.0），IoU-NMS 全放过去了。
DEFAULT_NMS_IOU = 0.45
DEFAULT_NMS_IOMIN = 0.85

# ---- 权重路径（**不进 git**，见同目录 `models/.gitignore`） ------------------
# ⚠️ **与托盘那条链同构**：权重放在**算法包自己**的 `models/` 下，路径常量也住在
#    算法包里（托盘是 `pallet_detect/detect.py` 的 `DEFAULT_MODEL_PATH`）。
#    2026-10-19 之前它在 ROS 壳里（`carton_obb/models/`，由节点用
#    `Path(__file__).parent.parent` 算出来），结果是**纯算法包反而依赖 ROS 包的
#    目录结构**才能拿到权重 —— 想脱离 ROS 复用算法的人撞上才知道。
#
# ⚠️ 这里只给**路径**，不读文件：本模块是纯函数库（零状态、不 import ROS /
#    ultralytics），推理在 ROS 壳里做。
DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / 'models' / 'carton_obb.onnx'


def letterbox(img: np.ndarray, new_shape=(640, 640), *,
              pad_value: int = PAD_VALUE, scaleup: bool = True,
              center: bool = True
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
    框会整体偏掉。
    """
    lb, r, pad = letterbox(img_bgr, (imgsz, imgsz),
                           pad_value=pad_value, scaleup=scaleup)
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(x), r, pad


def quad_from_xywhr(cx: float, cy: float, w: float, h: float, angle: float) -> np.ndarray:
    """一个 `[cx, cy, w, h, 弧度]` → 四角，**不依赖 torch**。

    与 `ultralytics.utils.ops.xywhr2xyxyxyxy` **逐式相同**：

        v1 = ( w/2*cosθ,  w/2*sinθ)      v2 = (-h/2*sinθ,  h/2*cosθ)
        p1 = c + v1 + v2   p2 = c + v1 - v2   p3 = c - v1 - v2   p4 = c - v1 + v2

    当 `θ ∈ [0, π/2)` 时，`p1` 是**最靠右下**的那个角、`p1→p2` 是短边（长度 `h`）。
    这个顺序我没有直接信：落地时用同一批帧与 ultralytics 自己的
    `OBBPredictor` 逐帧比对过四点，全 330 帧 IoU 中位 **0.9926**、最小 0.9744。
    """
    c, s = float(np.cos(angle)), float(np.sin(angle))
    v1 = np.array([w / 2.0 * c, w / 2.0 * s])
    v2 = np.array([-h / 2.0 * s, h / 2.0 * c])
    p = np.array([cx, cy])
    return np.vstack([p + v1 + v2, p + v1 - v2, p - v1 - v2, p - v1 + v2])


def order_corners_uv(corners_uv) -> np.ndarray:
    """四角排序为 **右下 → 左下 → 左上 → 右上**（`BoxDetection.corners_uv` 契约）。

    起点取 u+v 最大的角（图像上最靠右下），随后按屏幕顺时针。与输入顺序无关。
    """
    p = np.asarray(corners_uv, np.float64).reshape(-1, 2)
    if len(p) != 4:
        raise ValueError(f'要 4 个角，收到 {len(p)} 个')
    start = int(np.argmax(p[:, 0] + p[:, 1]))
    c = p.mean(axis=0)
    a0 = np.arctan2(p[start][1] - c[1], p[start][0] - c[0])
    rest = [i for i in range(4) if i != start]
    rest.sort(key=lambda i: (np.arctan2(p[i][1] - c[1], p[i][0] - c[0]) - a0) % (2 * np.pi))
    return p[[start] + rest]


def decode_obb(raw: np.ndarray, ratio: float, pad: Sequence[float], *,
               conf_thr: float = 0.25, orig_shape: Optional[Sequence[int]] = None,
               max_det: int = 300) -> List[dict]:
    """ONNX 原始输出 → 原图坐标下的候选列表（**未去重**，见 `nms_rotated`）。

    `raw` 形状 `(N, 6)` = `[cx, cy, w, h, conf, angle]`（**调用方负责转置**，
    导出图给的是 `(1, 6, 8400)`）。第 6 列是**原始弧度**，见模块 docstring。

    坐标反变换是**先转四角、再在像素空间减补边**：`(u - pad_u) / r`。反过来
    （先在 640 空间减补边再转四角）会得到错的框 —— 同一批帧上实测 IoU 从
    0.99 掉到 0.66。

    `orig_shape` 给了就把**完全跑出图外**的框丢掉。这比旧 AABB 那条链更需要：
    OBB 会把被画面边界裁掉的目标拟合成完整矩形，四角常常伸到画面外
    （val 330 帧里实测 145 个框越界）。
    """
    out: List[dict] = []
    if raw is None or len(raw) == 0:
        return out
    arr = np.asarray(raw, np.float64).reshape(-1, 6)
    keep = arr[:, 4] >= float(conf_thr)
    arr = arr[keep]
    if len(arr) == 0:
        return out
    arr = arr[np.argsort(-arr[:, 4])][:int(max_det)]

    px, py = float(pad[0]), float(pad[1])
    for row in arr:
        q = quad_from_xywhr(row[0], row[1], row[2], row[3], row[5])
        q[:, 0] = (q[:, 0] - px) / ratio
        q[:, 1] = (q[:, 1] - py) / ratio
        q = order_corners_uv(q)
        if orig_shape is not None:
            h, w = int(orig_shape[0]), int(orig_shape[1])
            if not (q[:, 0].max() > 0 and q[:, 0].min() < w
                    and q[:, 1].max() > 0 and q[:, 1].min() < h):
                continue
        out.append({'corners_uv': q,
                    'box_uv': [float(q[:, 0].min()), float(q[:, 1].min()),
                               float(q[:, 0].max()), float(q[:, 1].max())],
                    'confidence': float(row[4]),
                    'angle_deg': float(row[5])})
    return out


def dets_from_rotated_rects(center_x, center_y, width, height, angle_deg, scores, *,
                            orig_shape: Optional[Sequence[int]] = None,
                            conf_thr: float = 0.25,
                            max_det: int = 300) -> List[dict]:
    """共享推理服务（`box_detection_msgs/InferCartonObb`）的输出 → 候选列表。

    参数就是响应里的六个**等长数组**：原图像素坐标系下的 `cv2.RotatedRect`
    （中心、宽高、角度**度**）与置信度。返回的字典与 `decode_obb()` **逐字段
    同构**，下游（`nms_rotated` / 选箱 / 时间窗）不需要知道数据是本地 ONNX 还是
    服务来的。

    ⚠️ **这条路径必须与 `decode_obb` 同口径**，否则"用公共输出"与"自己推理"会
    出现肉眼看不出的差异：

    * 置信度过滤 `>= conf_thr`；
    * 按置信度降序、截断到 `max_det`（服务端没有这一步）；
    * 四角排序走 `order_corners_uv`（右下 → 左下 → 左上 → 右上）；
    * `orig_shape` 给了就丢掉完全跑出图外的框。

    坐标换算用 `cv2.boxPoints`：与 `quad_from_xywhr`（ultralytics 逐式同源）在
    2 万组随机矩形的实测最大差 **9.3e-5 px**，排序后取整前完全等价。

    ⚠️ **调用方应当请求服务端不做 NMS**（`nms_iou=1.0`），把去重留给
    `nms_rotated`：服务端只有 IoU 一条判据，本地还有 IoMin（专管小框套大框），
    且栅格化口径与本地不同 —— 让服务端先按 0.45 砍一刀，两边结果就可能差一两个
    候选，而这里要的是**逐位一样**。

    `det['angle_deg']` 存**弧度**（与 `decode_obb` 的字段名/口径保持一致，尽管
    这个名字起得不好）；当前没有消费方读它。
    """
    n = len(scores)
    arrays = {'center_x': center_x, 'center_y': center_y, 'width': width,
              'height': height, 'angle_deg': angle_deg, 'scores': scores}
    for name, seq in arrays.items():
        if len(seq) != n:
            raise ValueError(f'{name} 长度 {len(seq)} 与 scores 长度 {n} 不一致')
    if n == 0:
        return []

    arr = np.column_stack([
        np.asarray(center_x, np.float64), np.asarray(center_y, np.float64),
        np.asarray(width, np.float64), np.asarray(height, np.float64),
        np.asarray(scores, np.float64), np.asarray(angle_deg, np.float64),
    ])
    arr = arr[arr[:, 4] >= float(conf_thr)]
    if len(arr) == 0:
        return []
    arr = arr[np.argsort(-arr[:, 4])][:int(max_det)]

    out: List[dict] = []
    for cx, cy, w, h, conf, deg in arr:
        pts = cv2.boxPoints(((float(cx), float(cy)), (float(w), float(h)),
                             float(deg)))
        q = order_corners_uv(np.asarray(pts, np.float64))
        if orig_shape is not None:
            oh, ow = int(orig_shape[0]), int(orig_shape[1])
            if not (q[:, 0].max() > 0 and q[:, 0].min() < ow
                    and q[:, 1].max() > 0 and q[:, 1].min() < oh):
                continue
        out.append({'corners_uv': q,
                    'box_uv': [float(q[:, 0].min()), float(q[:, 1].min()),
                               float(q[:, 0].max()), float(q[:, 1].max())],
                    'confidence': float(conf),
                    'angle_deg': float(np.radians(deg))})
    return out


def _rot_iou(a: np.ndarray, b: np.ndarray, size=(480, 640)) -> float:
    """两个凸四边形的旋转 IoU（栅格化，640x480 够快也够准）。

    生产路径**不走这里**（NMS 用 `_mask` + `_rot_overlaps`，掩码只算一次）。
    留着是因为它的**并集是独立写的** —— 这里是 `|`，`_rot_overlaps` 那边是
    `a1 + a2 - 交`；两条式子互为对照，`test_obb_decoder` 拿它钉住
    「同中心转 45° = 1/√2」这个有闭式解的值。
    """
    h, w = size
    m1 = np.zeros((h, w), np.uint8)
    m2 = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m1, [np.round(a).astype(np.int32)], 1)
    cv2.fillPoly(m2, [np.round(b).astype(np.int32)], 1)
    u = int((m1 | m2).sum())
    return float((m1 & m2).sum()) / u if u else 0.0


def _mask(corners_uv, size=(480, 640)) -> np.ndarray:
    """一个四边形栅格化成 `(h, w)` 的 0/1 掩码。NMS 里每个候选**只算一次**。"""
    h, w = size
    m = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m, [np.round(np.asarray(corners_uv)).astype(np.int32)], 1)
    return m


def _rot_overlaps(m1: np.ndarray, a1: int, m2: np.ndarray, a2: int
                  ) -> Tuple[float, float]:
    """两个已栅格化的掩码 → `(IoU, IoMin)`，**共用同一次求交**。

    `IoMin = 交 / min(面积)`，即「重叠部分占较小那个框的比例」。它专门管
    IoU 管不着的那一类 —— **小框整个套在大框里**时 `IoU ≈ 小面积/大面积`
    （实测碎框 IoU 0.25、IoMin 1.0），所以 IoU 阈值再低也压不住碎框，
    而 IoMin 一压一个准。

    ⚠️ **不能拿 IoMin 当 IoU 用**：两个**不重叠**的框算出来是 `(0, 0)`；
    两个只是**擦着边**的框，IoU 很低但 IoMin 可能是 0.3~0.4 —— 它们是两个
    真箱子，不是重复框。两条判据是「或」的关系，各自守各自的形态。
    """
    inter = int((m1 & m2).sum())
    uni = int(a1) + int(a2) - inter
    mn = int(a1) if int(a1) < int(a2) else int(a2)
    return (float(inter) / uni if uni else 0.0,
            float(inter) / mn if mn else 0.0)


def nms_rotated(dets: Sequence[dict], iou_thr: float = DEFAULT_NMS_IOU,
                size=(480, 640), *, iomin_thr: float = DEFAULT_NMS_IOMIN
                ) -> List[dict]:
    """旋转框 NMS，按 `confidence` 降序保留。

    **这一步是必须的，不是保险。** 导出图里没有 NMS、也没有 query 上限
    （8400 行全在），同一目标会出 2~3 行 IoU>0.9 的重复框；实测 val 330 帧里
    36 帧出多框，其中 33 帧的多余框与首框 IoU>0.9。不去重的话下游会拿着
    两个几乎重合的框当成两个箱子。

    **两种重叠各有一条判据**（`or`，见 `_rot_overlaps`）：

    * `iou_thr`：两个框**互相错着盖**（同目标的两条预测）。
    * `iomin_thr`：**小框整个套在大框里**。这一类 IoU 天生够不着 —— 碎框只框
      了箱子一角，`IoU ≈ 小面积/大面积 ≈ 0.25`，阈值给多低都放它过去；
      而 `IoMin = 交/较小面积` 在这类上恒 ≈1.0。**取置信度最高的那个**这条规则
      由它兜住。`iomin_thr >= 1.0` = 关掉这一条（与只有 IoU 的旧行为等价）。

    IoU 用栅格化算（不是 AABB 近似）—— 旋转框用轴对齐 IoU 去比会在
    「同中心、转 45°」这种情形上给出 0.5 左右的假 IoU。

    ⚠️ **掩码每个候选只算一次**再在循环里复用。旧的写法是每比一对就
    `fillPoly` 两个多边形，候选一多就白烧一半时间（实测 640x480、745 个候选：
    15.4ms/张 → 11.1ms/张）。关掉 `iomin_thr` 时结果与旧写法**逐位一致**。
    """
    if not dets:
        return []
    th_i, th_m = float(iou_thr), float(iomin_thr)
    ms = [_mask(d['corners_uv'], size) for d in dets]
    areas = [int(m.sum()) for m in ms]

    order = sorted(range(len(dets)), key=lambda i: -dets[i]['confidence'])
    keep: List[int] = []
    for i in order:
        ok = True
        for j in keep:
            iou, iomin = _rot_overlaps(ms[i], areas[i], ms[j], areas[j])
            if iou > th_i or iomin > th_m:
                ok = False
                break
        if ok:
            keep.append(i)
    return [dets[i] for i in sorted(keep)]


# --------------------------------------------------------------------------- #
# 选箱打分：画面下方中心距离 + 面积 + 中心深度
# --------------------------------------------------------------------------- #
def d_center_px(corners_uv, ref_uv) -> float:
    """四角质心到 `ref_uv` 的欧氏距离（像素）。"""
    q = np.asarray(corners_uv, np.float64).reshape(-1, 2)
    c = q.mean(axis=0)
    return float(np.hypot(c[0] - float(ref_uv[0]), c[1] - float(ref_uv[1])))


def d_center_score(d_px: float, *, d_near: float = DEFAULT_D_NEAR_PX,
                   d_far: float = DEFAULT_D_FAR_PX) -> float:
    """到下方中心的距离 → `[0,1]`，**越近分越高**。

    `d <= d_near` 得 1.0、`d >= d_far` 得 0.0，中间线性。`d_far <= d_near`
    （参数写反）时退化成阶跃：近于 `d_near` 得 1，否则 0。
    """
    if d_far <= d_near:
        return 1.0 if float(d_px) <= float(d_near) else 0.0
    t = (float(d_far) - float(d_px)) / (float(d_far) - float(d_near))
    return float(min(1.0, max(0.0, t)))


def depth_score(z_mm: float, *, z_near: float = DEFAULT_Z_NEAR_MM,
                z_far: float = DEFAULT_Z_FAR_MM) -> float:
    """中心深度 → `[0,1]`，**越近分越高**。口径同旧链路的 `depth_score`。"""
    if z_far <= z_near:
        return 1.0 if float(z_mm) <= float(z_near) else 0.0
    t = (float(z_far) - float(z_mm)) / (float(z_far) - float(z_near))
    return float(min(1.0, max(0.0, t)))


def sample_depth_mm(depth_mm, box_uv: Sequence[float], *,
                    half: int = DEFAULT_DEPTH_PATCH_PX,
                    z_near: float = DEFAULT_Z_NEAR_MM,
                    z_far: float = DEFAULT_Z_FAR_MM) -> Optional[float]:
    """框**中心**一个 `(2*half+1)²` 方块内的**有效深度中位数**；取不到返回 `None`。

    **为什么取中位数而不是中心那一个像素**：深度图到处是空洞与飞点（黑箱子、
    反光、遮挡边缘），单点很容易落空，一帧有值一帧没值会让排序跳。

    **有效像素** = `z_near <= z <= z_far`（顺便滤掉 0 与 65535 这类哨兵）。
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
    if c1 <= c0 or r1 <= r0:
        return None

    patch = arr[r0:r1, c0:c1].astype(np.float64, copy=False).ravel()
    ok = np.isfinite(patch) & (patch >= float(z_near)) & (patch <= float(z_far))
    if not ok.any():
        return None
    return float(np.median(patch[ok]))


def refine_bottom_by_depth(depth_mm, corners_uv, *,
                           step: int = DEFAULT_REFINE_BOTTOM_STEP,
                           band_px: int = DEFAULT_REFINE_BOTTOM_BAND_PX,
                           jump_mm: float = DEFAULT_REFINE_BOTTOM_JUMP_MM,
                           start_off_px: int = DEFAULT_REFINE_BOTTOM_START_PX,
                           min_shrink_px: float = DEFAULT_REFINE_BOTTOM_MIN_SHRINK_PX,
                           z_near: float = DEFAULT_Z_NEAR_MM,
                           z_far: float = DEFAULT_Z_FAR_MM
                           ) -> Tuple[Optional[np.ndarray], dict]:
    """用**深度下沿**把手持箱被拉长的下边收回去；不满足条件返回 `(None, info)`。

    ## 为什么需要它

    现场 bag（2026-10-30-19-08-19）里箱子相对相机是静止的：宽度 349~358px、
    角度 ±0.7°、上边 141~147px 都不动，只有下边在 365~480px 之间跳 —— 模型
    偶尔把下方**背景**一起框进来。深度图里箱体在 580mm、下方背景在 830mm，
    下沿的深度跳变始终在 362~364px。因此判据不是「框应该多高」，而是
    **框内箱体与框外背景的深度不连续处**，与箱子大小/距离无关。

    ## 判据（全部满足才改）

    1. 框内（上边下方 60~100px）能取到稳定的箱体深度 `zi`；
    2. 从 `top + start_off_px` 往下，出现**持续**（约 20px）`> zi + jump_mm` 的
       远背景；
    3. 在跳变附近找到深度穿越点 `y_new`（箱体/背景中值的中点）；
    4. 框的「高度方向」（下边中点到上边中点）要基本竖直（`up[1] < -0.2`）；
    5. 只收缩：`y_new` 至少比原下边高 `min_shrink_px`。

    ## 输入/输出

    * `depth_mm`：16UC1 或 float，**单位毫米**；`None`/形状不对 → 不改。
    * `corners_uv`：四点，任意顺序（内部会过 `order_corners_uv`）；
      返回的四点是**契约序**（右下 左下 左上 右上），只沿高度方向移动下边两点。
    * 返回 `(new_corners, info)`。`info['reason']` 说明改/没改的原因，
      供节点日志排查；`info` 不是下游契约。

    ⚠️ 只移动下边，不动上边/宽度/角度。上边在这个 bag 本来就和深度上沿
    （~147px）重合；如果以后遇到上边也漂的场景，应该单独加一条上沿判据，
    不要在这里顺手改。
    """
    info = {'changed': False, 'reason': 'ok', 'zi_mm': None, 'zb_mm': None,
            'y_new': None, 'h_before': None, 'h_after': None}
    if depth_mm is None:
        info['reason'] = 'no_depth'
        return None, info
    arr = np.asarray(depth_mm)
    if arr.ndim != 2 or arr.size == 0:
        info['reason'] = 'bad_depth'
        return None, info
    q = order_corners_uv(np.asarray(corners_uv, np.float64).reshape(4, 2))
    h_img, w_img = int(arr.shape[0]), int(arr.shape[1])

    top = float(q[:, 1].min())
    bot = float(q[:, 1].max())
    cx = float(q[:, 0].mean())
    info['h_before'] = bot - top

    step = max(1, int(step))
    band_px = max(5, int(band_px))
    x0 = max(0, int(round(cx - band_px)))
    x1 = min(w_img, int(round(cx + band_px)) + 1)
    if x1 - x0 < 5:
        info['reason'] = 'bad_band'
        return None, info

    # 箱体内部深度：上边下方 60~100px（避开上沿的混合像素）
    iy0 = max(0, int(round(top)) + 60)
    iy1 = min(h_img, int(round(top)) + 100, int(round(bot)) + 1)
    if iy1 - iy0 < 5:
        info['reason'] = 'too_short'
        return None, info
    inner = arr[iy0:iy1, x0:x1].astype(np.float64, copy=False).ravel()
    ok = np.isfinite(inner) & (inner >= float(z_near)) & (inner <= float(z_far))
    if int(ok.sum()) < 30:
        info['reason'] = 'too_few_inner'
        return None, info
    zi = float(np.median(inner[ok]))
    info['zi_mm'] = zi

    sy = max(0, int(round(top)) + max(0, int(start_off_px)))
    ey = min(h_img - 1, int(round(bot)) + 10)
    if ey - sy < 60:
        info['reason'] = 'no_search_range'
        return None, info

    band = arr[sy:ey:step, x0:x1:step].astype(np.float64, copy=False)
    band = np.where((band >= float(z_near)) & (band <= float(z_far)), band, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # 整行空是常态
        prof = np.nanmedian(band, axis=1)
    if int(np.isfinite(prof).sum()) < 10:
        info['reason'] = 'too_few_profile'
        return None, info

    # 持续跳变：滑动窗口均值 + 有效样本数。逐窗 nanmedian 在节点里要 1ms+，
    # 这里用 cumsum 一次算完（实测整函数 0.67ms，含深度切片）。
    win = max(4, 20 // step)
    filled = np.where(np.isfinite(prof), prof, 0.0)
    valid = np.isfinite(prof).astype(np.float64)
    cs = np.concatenate(([0.0], np.cumsum(filled)))
    cv = np.concatenate(([0.0], np.cumsum(valid)))
    sums = cs[win:] - cs[:-win]
    cnts = cv[win:] - cv[:-win]
    means = sums / np.maximum(cnts, 1.0)
    okj = (cnts >= win * 0.6) & (means > zi + float(jump_mm))
    if not okj.any():
        info['reason'] = 'no_jump'
        return None, info
    ys = int(np.flatnonzero(okj)[0])

    tail = prof[ys + win:ys + 2 * win]
    if int(np.isfinite(tail).sum()) < 3:
        info['reason'] = 'no_background'
        return None, info
    zb = float(np.nanmedian(tail))
    info['zb_mm'] = zb
    thr = (zi + zb) / 2.0
    cr = None
    for i in range(max(0, ys - 3), min(len(prof), ys + 2 * win)):
        w = prof[max(0, i - 1):i + 2]
        if int(np.isfinite(w).sum()) >= 2 and np.nanmedian(w) > thr:
            cr = i
            break
    if cr is None:
        info['reason'] = 'no_crossing'
        return None, info
    y_new = float(sy + cr * step)
    info['y_new'] = y_new

    bm = (q[0] + q[1]) / 2.0
    tm = (q[2] + q[3]) / 2.0
    up = tm - bm
    n = float(np.linalg.norm(up))
    if n < 1e-6:
        info['reason'] = 'degenerate'
        return None, info
    up /= n
    if up[1] > -0.2:               # 高度方向不竖直：深度扫描沿图像列，不可靠
        info['reason'] = 'not_vertical'
        return None, info
    if bot - y_new < float(min_shrink_px):
        info['reason'] = 'already_ok'
        return None, info

    delta = (y_new - bm[1]) / up[1]
    new = q.copy()
    new[0] = q[0] + up * delta
    new[1] = q[1] + up * delta
    info['changed'] = True
    info['h_after'] = float(new[:, 1].max() - new[:, 1].min())
    return new, info


def angle_deg_from_corners(corners_uv) -> float:
    """**下边**（左下 → 右下）的方向角，单位度，折进 (−90, 90]。

    与伺服要求的 `theta` **同一个口径**（操作员 2026-10-13 确认：角度沿用原算法，
    **不要用 OBB 自己的角度**）。图像右手系
    （+x 向右、+y 向下），正 = 右边比左边低。

    这一条不是小事：伺服要的 `theta` 就是这个量，而 OBB 的输出角度是
    「长边相对水平」的角度 —— 两者在箱子转 90° 时会差一个直角。

    输入必须是 **`order_corners_uv` 排过的**（右下 左下 左上 右上）。
    """
    q = np.asarray(corners_uv, np.float64).reshape(4, 2)
    d = q[0] - q[1]                                        # 右下 − 左下
    ang = float(np.degrees(np.arctan2(d[1], d[0])))
    while ang > 90.0:
        ang -= 180.0
    while ang <= -90.0:
        ang += 180.0
    return ang


def candidate_score(d_px: float, area_px: float, z_mm: Optional[float], *,
                    max_area_px: float,
                    w_dcenter: float = DEFAULT_SCORE_W_DCENTER,
                    w_area: float = DEFAULT_SCORE_W_AREA,
                    w_depth: float = DEFAULT_SCORE_W_DEPTH,
                    d_near: float = DEFAULT_D_NEAR_PX,
                    d_far: float = DEFAULT_D_FAR_PX,
                    z_near: float = DEFAULT_Z_NEAR_MM,
                    z_far: float = DEFAULT_Z_FAR_MM) -> Tuple[float, dict]:
    """一个候选的 `(总分, 分项明细)`。分项明细给日志用，**不进契约**。

    三个量归一化到 `[0,1]` 后加权求和：

        score = w_dcenter * d_center_score(d_px)          # 越靠画面下方中心越高
              + w_area    * (area_px / max_area_px)       # 同帧相对归一
              + w_depth   * depth_score(z_mm)

    ⚠️ **置信度不在这里**（2026-10-13 改）。它曾经是三项之一（权重 0.5），但
    现场 val 的 36 个多框帧里有一帧 conf 完全分不开（手箱 0.9773 / 邻箱 0.9642）。
    conf 仍然用来**过滤**（`conf_thr`）和**去重**（NMS 保留高分那个），只是不参与打分。

    ⚠️ **`z_mm is None` 时把深度项的权重按比例让给另两项**：

        score = (w_dcenter*d + w_area*area_norm) / (w_dcenter + w_area)

    取不到深度**不罚** —— 深度空洞往往是近物遮挡造成的，罚它会适得其反。
    代价是**归一化到 `[0,1]`**，所以深度项**只能压低远处的候选，不能额外抬高近处的**。
    全部权重为 0 时返回 0.0（不除零）。
    """
    d = d_center_score(d_px, d_near=d_near, d_far=d_far)
    a = 0.0 if max_area_px <= 0.0 else min(1.0, max(0.0, float(area_px) / float(max_area_px)))
    z = None if z_mm is None else depth_score(z_mm, z_near=z_near, z_far=z_far)

    if z is None:
        wsum = float(w_dcenter) + float(w_area)
        score = 0.0 if wsum <= 0.0 else (float(w_dcenter) * d + float(w_area) * a) / wsum
    else:
        score = float(w_dcenter) * d + float(w_area) * a + float(w_depth) * z

    return score, {'d_center': d, 'd_px': float(d_px), 'area_norm': a,
                   'depth': z, 'z_mm': z_mm}


def score_candidates(dets: Sequence[dict], ref_uv, z_list: Optional[Sequence] = None,
                     **score_kw) -> List[Tuple[dict, float, dict]]:
    """给每个候选打分，返回 `[(det, score, parts), ...]`（**顺序与 `dets` 一致**）。

    `z_list` 与 `dets` 等长，元素是各自中心深度或 `None`；不给就全部按"无深度"
    处理。面积归一的基准 `max_area_px` 从**本帧全部候选**里取。
    """
    ds = [d_center_px(d['corners_uv'], ref_uv) for d in dets]
    areas = [max(0.0, d['box_uv'][2] - d['box_uv'][0])
             * max(0.0, d['box_uv'][3] - d['box_uv'][1]) for d in dets]
    max_area = max(areas) if areas else 0.0
    zs = list(z_list) if z_list is not None else [None] * len(dets)

    out: List[Tuple[dict, float, dict]] = []
    for i, d in enumerate(dets):
        z = zs[i] if i < len(zs) else None
        score, parts = candidate_score(ds[i], areas[i], z,
                                       max_area_px=max_area, **score_kw)
        out.append((d, score, parts))
    return out


def pick_best(dets: Sequence[dict], ref_uv,
              z_list: Optional[Sequence] = None,
              **score_kw) -> Optional[dict]:
    """从若干检测里挑「手里那个」，没有就返回 `None`。

    判据是**画面下方中心距离 + 面积 + 中心深度**三项归一化加权求和，取最高分 ——
    见 `candidate_score`。分数并列时取**靠前**的那个（`max` 语义）。
    """
    if not dets:
        return None
    scored = score_candidates(dets, ref_uv, z_list, **score_kw)
    return max(scored, key=lambda t: t[1])[0]
