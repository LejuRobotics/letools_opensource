"""滑动时间窗：把 `algorithm.fit_box_frame` 的单帧结果平滑成伺服要的输出。

**为什么是独立一层**：`fit_box_frame` 是**零状态纯函数**（架构硬约束）——
它必须能被 YOLO 直接调用、能单帧复现、能脱离一切上下文测试。时间窗天然
有状态（要缓存前几帧），所以只能包在它外面，不能塞进去。

本文件**也不 import ROS**：ROS 那一层在
`infrastructure/ros_packages/src/ros_vision/detection_industrial_yolo/box_detection/`。

规则（操作员 2026-09-21 定）：

* 相机 30Hz，**每 3 帧处理一次**（≈10Hz）—— 由本类的 `process_every` 控制。
* 窗口 = 最近 **5 个处理槽**。每个处理槽跑一次 `fit_box_frame`，结果进窗。
* 出结果时把窗口里**所有有结果的帧**求平均（点对点平均，四角顺序固定，
  见 `fit_box_frame.order_corners_uv`：右下 → 左下 → 左上 → 右上）。
    - 5 帧里只有 1 帧有结果 → 就用这一帧（求平均与单帧等价）。
    - 5 帧全都失败 → **退回原始 YOLO 框**，并在结果里标 `source='yolo_fallback'`
      （`render_window_check` 会在图上打醒目红字）。
* 窗口前几次调用还不满 5 个槽时，有几帧算几帧 —— 不额外等。

**耗时**：单帧 `fit_box_frame` 实测中位 **58ms**（i9-14900KF，`OMP_NUM_THREADS=1`）。
本类的目标调用间隔是 100ms（每 3 帧），**够用**。最初是 864ms，见 WORKLOG §15。

**朝向先验**（操作员 2026-09-21 追加）：上游 `/infer_carton_pose` 会告诉我们
箱子长边在图像里是**横着还是竖着**。把它经 `orientation=` 或
`set_orientation()` 传进来，就能掐掉「整框转 90°」那种错（那种错在数据里看不
出来 —— 输出四角照样是个规规矩矩的矩形）。传 `None` / `-1` 就恢复自动选。

用法：
    win = BoxFrameWindow(k, target_mm=(530.0, 350.0))
    win.set_orientation(tape_orientation_deg)   # 每帧或收到服务回包时更新
    out = win.push(color, depth, box_uv)     # 非处理槽返回 None
    if out is not None:
        servo.send(out['corners_uv'], out['source'])
"""
from __future__ import annotations

from collections import deque

import numpy as np

from .algorithm import (CameraIntrinsics, fit_box_frame,
                        normalize_orientation, tape_orientation_to_hint)

# 四角顺序（与 fit_box_frame.order_corners_uv 一致）：右下 → 左下 → 左上 → 右上。
# 渲染时用**序号**标角（`putText(str(i+1))`），所以这里不需要一个名字表。


def yolo_fallback_corners(box_uv) -> np.ndarray:
    """把 YOLO 的轴对齐框 (u0,v0,u1,v1) 拆成四角，顺序同 `fit_box_frame`。

    放弃恢复角度时的兜底：输出就是外接框本身（= 等于没做角度恢复）。
    """
    u0, u1 = sorted((float(box_uv[0]), float(box_uv[2])))
    v0, v1 = sorted((float(box_uv[1]), float(box_uv[3])))
    return np.array([[u1, v1],     # 右下
                     [u0, v1],     # 左下
                     [u0, v0],     # 左上
                     [u1, v0]],    # 右上
                    np.float64)


class BoxFrameWindow:
    """滑动时间窗。有状态，但不做 I/O、不画图、不起线程。"""

    def __init__(self, k: CameraIntrinsics, *, target_mm=(530.0, 350.0),
                 window: int = 5, process_every: int = 3,
                 agg: str = 'mean', opts: dict | None = None,
                 orientation=None):
        if window < 1:
            raise ValueError('window 至少 1')
        if process_every < 1:
            raise ValueError('process_every 至少 1')
        if agg not in ('mean', 'median'):
            raise ValueError("agg 只能是 'mean' 或 'median'")
        self.k = k
        self.target_mm = tuple(target_mm)
        self.window = int(window)
        self.process_every = int(process_every)
        self.agg = agg
        self.opts = dict(opts) if opts else {}
        # 朝向先验（长边在图像里是横是竖）。**单独一个构造参数**而不是塞进 opts：
        # 它跟 `opts` 里那些「算法开关」不是一回事 —— 它是**随场景变的运行时输入**，
        # 由上游 `/infer_carton_pose` 每帧给。所以有 `set_orientation()`，
        # 服务回一次就能改，不用重建时间窗。
        if orientation is not None:
            self.opts['orientation'] = orientation
        self.opts = self.opts or None
        self._slots: deque = deque(maxlen=int(window))   # 每个处理槽一条记录
        self._n_seen = 0                                 # 喂进来的帧数（含跳过的）

    # ---- 朝向先验（运行时可改） ------------------------------------------
    def set_orientation(self, orientation) -> str | None:
        """更新朝向提示。`None` / `-1` / `'auto'` = 不提示（恢复自动选）。

        返回归一化后的提示（`'horizontal'` / `'vertical'` / `None`），
        方便调用方把「服务到底给了什么」记进日志。
        """
        hint = normalize_orientation(orientation)
        if hint is None:
            if self.opts:
                self.opts.pop('orientation', None)
                if not self.opts:
                    self.opts = None
        else:
            self.opts = dict(self.opts or {})
            self.opts['orientation'] = hint
        return hint

    @property
    def orientation(self) -> str | None:
        return (self.opts or {}).get('orientation')

    # ---- 状态查询 -------------------------------------------------------
    @property
    def n_slots(self) -> int:
        return len(self._slots)

    @property
    def ready(self) -> bool:
        """窗口满了没有。没满也能出结果，只是参与平均的帧少。"""
        return len(self._slots) >= self.window

    def reset(self) -> None:
        self._slots.clear()
        self._n_seen = 0

    # ---- 主入口 ---------------------------------------------------------
    def push(self, color, depth, k_or_none=None, box_uv=None) -> dict | None:
        """喂一帧原始数据。**非处理槽返回 None**（这一帧不用算）。

        两种调用形式：
            win.push(color, depth, box_uv)          # 用构造时的内参
            win.push(color, depth, k, box_uv)       # 显式传内参
        """
        if box_uv is None:
            box_uv, k = k_or_none, self.k
        else:
            k = k_or_none if k_or_none is not None else self.k
        self._n_seen += 1
        if (self._n_seen - 1) % self.process_every != 0:
            return None
        return self._process(color, depth, k, box_uv)

    def _process(self, color, depth, k, box_uv) -> dict:
        try:
            res, diag = fit_box_frame(color, depth, k, box_uv,
                                      target_mm=self.target_mm, opts=self.opts)
        except Exception as exc:                 # 单帧异常不能拖垮整条伺服链路
            res, diag = None, {'ok': False, 'reject': f'exception:{type(exc).__name__}'}
        hint, conflict = _hint_of(diag)
        if res is None:
            self._slots.append({'ok': False, 'reject': diag.get('reject'),
                                'confidence': None, 'corners_uv': None,
                                'orientation': hint, 'orient_conflict': conflict})
        else:
            self._slots.append({'ok': True,
                                'reject': None,
                                'confidence': diag.get('confidence'),
                                'corners_uv': np.asarray(res['corners_uv'], np.float64),
                                'orientation': hint, 'orient_conflict': conflict})
        return self._aggregate(box_uv)

    def _aggregate(self, box_uv) -> dict:
        good = [s for s in self._slots if s['ok']]
        n_bad = len(self._slots) - len(good)
        if not good:
            # 整个窗口一帧都没做出来 —— 退回原始 YOLO 框，并**标注**
            return {'corners_uv': yolo_fallback_corners(box_uv),
                    'source': 'yolo_fallback',
                    'n_used': 0, 'n_slots': len(self._slots), 'n_failed': n_bad,
                    'spread_px': None,
                    'orient_conflict': any(s.get('orient_conflict')
                                           for s in self._slots),
                    'rejects': [s['reject'] for s in self._slots if not s['ok']],
                    'per_frame': _brief(self._slots)}
        stack = np.stack([s['corners_uv'] for s in good])       # (n,4,2)
        corners = (np.median(stack, axis=0) if self.agg == 'median'
                   else stack.mean(axis=0))
        # 帧间离散度：各帧四角到平均值的最大距离。用来判断这一窗稳不稳，
        # 不参与输出 —— 只是给上层和诊断图看的。
        spread = float(np.abs(stack - corners[None]).max())
        return {'corners_uv': corners,
                'source': 'single' if len(good) == 1 else 'window',
                'n_used': len(good), 'n_slots': len(self._slots), 'n_failed': n_bad,
                'spread_px': spread,
                'orient_conflict': any(s.get('orient_conflict') for s in self._slots),
                'rejects': [s['reject'] for s in self._slots if not s['ok']],
                'per_frame': _brief(self._slots)}


def _brief(slots) -> list[dict]:
    return [{'ok': s['ok'], 'reject': s['reject'], 'confidence': s['confidence'],
             'orientation': s.get('orientation'),
             'orient_conflict': s.get('orient_conflict', False)} for s in slots]


def _hint_of(diag: dict) -> tuple[str | None, bool]:
    """从单帧 diag 里取出朝向先验的**处置结果**：`(实际用上的提示, 是否冲突)`。

    `(None, False)` = 没给提示 / 服务没接；`(hint, False)` = 用上了；
    `(None, True)` = 给了但和数据打架，退回了自动选 —— 这三种在诊断上**不一样**，
    所以分开返回（项目规矩：人工/上游输入必须留痕）。
    """
    od = (diag or {}).get('orientation_hint') or {}
    return (od.get('hint') if od.get('applied') else None), bool(od.get('conflict'))


# --------------------------------------------------------------------------- #
# `/infer_carton_pose` 的回包 -> 朝向提示（**纯函数，不 import ROS**）
# --------------------------------------------------------------------------- #
def _iou_xyxy(a, b) -> float:
    """两个 `[x0,y0,x1,y1]` 框的交并比。不重叠 = 0。"""
    ax0, ay0, ax1, ay1 = (min(a[0], a[2]), min(a[1], a[3]),
                          max(a[0], a[2]), max(a[1], a[3]))
    bx0, by0, bx1, by1 = (min(b[0], b[2]), min(b[1], b[3]),
                          max(b[0], b[2]), max(b[1], b[3]))
    iw = min(ax1, bx1) - max(ax0, bx0)
    ih = min(ay1, by1) - max(ay0, by0)
    if iw <= 0.0 or ih <= 0.0:
        return 0.0
    inter = iw * ih
    union = ((ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter)
    return float(inter / union) if union > 0.0 else 0.0


def hint_from_instances(bbox_xyxy, tape_orientation_deg, box_uv, *,
                        index: int | None = None):
    """把 `/infer_carton_pose` 的回包折成 `'horizontal'` / `'vertical'` / `None`。

    回包里是**每个实例一条**的并列数组（`bbox_xyxy` 每 4 个数一个框，
    `tape_orientation_deg` 每个实例一个）。必须挑出**我们这只箱子**对应的那条 ——
    场上有两个箱子时，拿错实例的朝向比不给提示更坏（会把正确的朝向筛掉）。

    挑法：与 YOLO 框 **IoU 最大**的那条，且 IoU > 0。**没有阈值**：0 就是「一点
    都不重叠」这个事实本身，不是可以按场景调的参数（本项目对固定参数的态度见
    WORKLOG §14）。全都不重叠 → `None`（宁可不提示）。

    `index` 给定时直接取第 index 条（上游如果已经和 YOLO 对齐过顺序，用它更省事）。
    """
    degs = list(tape_orientation_deg or [])
    if not degs:
        return None
    if index is not None:
        if not (0 <= int(index) < len(degs)):
            return None
        return tape_orientation_to_hint(int(degs[int(index)]))
    boxes = list(bbox_xyxy or [])
    if len(boxes) < 4 * len(degs):
        return None                     # 数组对不齐，判不了
    best, best_iou = None, 0.0
    for i in range(len(degs)):
        iou = _iou_xyxy(boxes[4 * i:4 * i + 4], box_uv)
        if iou > best_iou:
            best, best_iou = i, iou
    if best is None:
        return None
    return tape_orientation_to_hint(int(degs[best]))


# --------------------------------------------------------------------------- #
# 对外 payload：**与 ROS 无关**的一层
# --------------------------------------------------------------------------- #
def build_payload(out: dict, box_uv, latency_ms: float | None = None) -> dict:
    """把窗口输出整理成给下游的 payload（纯函数，不 import ROS）。

    单独拎出来是为了：**业务逻辑能被单元测试覆盖，ROS 那一层只剩搬运**。
    `box_detection_node.py` 拿这个 dict 往消息里填，一个字段一个字段对应。

    `angle_deg` 的约定（全项目统一，WORKLOG 里那些角度数字都是这个口径）：
    **下边 = 左下 → 右下**，`atan2(v右 − v左, u右 − u左)`，折进 (−90, 90]。
    图像右手系（+x 向右、+y 向下），正 = 右边比左边低。
    """
    uv = np.asarray(out['corners_uv'], np.float64)      # 右下 左下 左上 右上
    d = uv[0] - uv[1]                                   # 右下 − 左下
    ang = float(np.degrees(np.arctan2(d[1], d[0])))
    while ang > 90.0:
        ang -= 180.0
    while ang <= -90.0:
        ang += 180.0
    bu0, bu1 = sorted((float(box_uv[0]), float(box_uv[2])))
    bv0, bv1 = sorted((float(box_uv[1]), float(box_uv[3])))
    # 这一窗**实际用上的**朝向先验（长边在图像里横还是竖）。窗口里所有帧都用
    # 同一个设置，所以取最后一帧的记录即可。没用上（含 `conflict`）就是空串 ——
    # 消费端据此知道「四角里有没有人为先验的功劳」（项目规矩：人工输入必须留痕）。
    hint = ''
    for s in reversed(out.get('per_frame') or []):
        if s.get('orientation'):
            hint = s['orientation']
            break
    # 上游给了朝向、但没有任何候选对得上 → 退回了自动选。**必须和「上游没给」
    # 区分开**：前者是「先验可能是错的」，后者只是「没接服务」。
    conflict = bool(out.get('orient_conflict'))
    return {
        'corners_uv': [(float(u), float(v)) for u, v in uv],
        'valid': out['source'] != 'yolo_fallback',
        'source': out['source'],
        'n_used': int(out['n_used']),
        'n_slots': int(out['n_slots']),
        'n_failed': int(out['n_failed']),
        'spread_px': (-1.0 if out['spread_px'] is None else float(out['spread_px'])),
        'angle_deg': ang,
        'box_uv': [(bu0, bv0), (bu1, bv1)],             # [0]=左上 [1]=右下
        'latency_ms': (0.0 if latency_ms is None else float(latency_ms)),
        'orientation': ('conflict' if conflict else hint),
        'rejects': ','.join(str(r) for r in out.get('rejects') or []),
    }


# --------------------------------------------------------------------------- #
# 可视化：把窗口结果画出来（**`yolo_fallback` 必须一眼看得出来**）
# --------------------------------------------------------------------------- #
def render_window_check(color: np.ndarray, out: dict, box_uv) -> np.ndarray:
    """单帧渲染：窗口输出四角 + YOLO 框 + 来源标注。

    沿用项目规矩（设计文档 §7）：**低置信度/兜底结果必须在图上可见**，
    否则调试期的漂亮结果会被误当成算法真实能力。
    """
    import cv2
    vis = color.copy()
    u0, u1 = sorted((float(box_uv[0]), float(box_uv[2])))
    v0, v1 = sorted((float(box_uv[1]), float(box_uv[3])))
    cv2.rectangle(vis, (int(round(u0)), int(round(v0))),
                  (int(round(u1)), int(round(v1))), (0, 200, 255), 1)   # 黄 = 输入框

    uv = np.asarray(out['corners_uv'], np.float64)
    fallback = (out['source'] == 'yolo_fallback')
    col = (0, 0, 255) if fallback else (0, 255, 0)
    cv2.polylines(vis, [np.round(uv).astype(np.int32)], True, col, 2)
    for i, (u, v) in enumerate(uv):
        cv2.circle(vis, (int(round(u)), int(round(v))), 5, col, -1)
        cv2.putText(vis, str(i + 1), (int(round(u)) + 8, int(round(v)) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if fallback:
        tag = (f"*** YOLO FALLBACK ({out['n_failed']}/{out['n_slots']} 帧全失败) ***"
               f"   reject={out['rejects']}")
    else:
        tag = (f"source={out['source']}  used={out['n_used']}/{out['n_slots']}"
               f"  spread={out['spread_px']:.1f}px")
    cv2.putText(vis, tag, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 255) if fallback else (255, 255, 255), 2)
    # 朝向先验属人工输入，用了必须看得见（项目规矩：人工输入必须留痕）
    hint = next((s.get('orientation') for s in reversed(out.get('per_frame') or [])
                 if s.get('orientation')), None)
    if hint:
        cv2.putText(vis, f"*** ORIENT={hint} ***", (6, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    elif out.get('orient_conflict'):
        # 上游说了朝向、数据里没一个候选对得上 —— 这比「没给」严重得多，标红
        cv2.putText(vis, "*** ORIENT CONFLICT (自动选) ***", (6, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return vis
