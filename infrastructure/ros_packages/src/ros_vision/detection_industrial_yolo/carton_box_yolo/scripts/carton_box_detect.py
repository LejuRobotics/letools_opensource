#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纸箱检测 ROS 节点：图像话题 → YOLO 推理 → `/box/yolo_box`。

**这是 `/box/yolo_box`（`geometry_msgs/PolygonStamped`）的生产者。** 在它之前
这个话题全仓没有真生产者 —— `box_detection` 的输入一直靠假相机
（`maduo/tools/pub_test_frames.py`，在 LeTools 之外）喂。

链路位置
--------
    本节点 ──/box/yolo_box──→ box_detection ──/box/detection──→ LeTools NodeBoxObs
    （YOLO 框）                （恢复箱子旋转角）                   （写 latest_box_obs）

**上游契约**（由 `box_detection_node.py` 的 `_box_uv_from_msg` 定死）：
`points[0]` = 左上、`points[1]` = 右下。那边取两点的外接框，所以顺序给反了也
不会出错，但契约照写。

为什么推理在**独立进程**里
--------------------------
YOLO 一帧几十毫秒，放进行为树的 tick 里会把整棵树拖住。这里是标准 ROS 节点，
与行为树只通过话题相连。也正因如此，**它不 import LeTools 的任何东西**，
单独 `rosrun` 就能跑。

参数（`rosparam`，都是**私有**参数 `~name`，都有默认值）
-------------------------------------------------------
| 名字 | 默认 | 说明 |
|---|---|---|
| `image` | `/camera/color/image_raw` | 输入彩色图（BGR8 或 RGB8 都认，见下） |
| `image_depth` | `/camera/depth/image_raw` | 深度图，**只用于选箱打分**；空串 = 不订阅 |
| `depth_scale_mm` | `1.0` | 深度单位→毫米系数（`16UC1` 已是毫米填 1.0） |
| `box_out` | `/box/yolo_box` | 输出框话题 |
| `model_path` | `<包>/models/best_cartonbb.onnx` | ONNX 权重（**留空 = 用默认值**） |
| `imgsz` | `640` | 输入边长，必须与导出时一致 |
| `conf` | `0.25` | 置信度阈值 |
| `target_class` | `-1` | 要挑的类别 id；`-1` = 不筛 |
| `publish_empty` | `false` | 没检到框时要不要也发一条空消息 |
| `max_rate_hz` | `0.0` | >0 时降频到该频率（相机 30Hz、YOLO 只要 10Hz 就够） |
| `queue_size` | `1` | 订阅队列，只要最新帧 |

选箱打分（见 `carton_detector.py` 的 `candidate_score`）：
`score_w_conf` `0.5` / `score_w_area` `0.2` / `score_w_depth` `0.3` /
`z_near_mm` `500` / `z_far_mm` `1500` / `depth_patch_px` `15`。

**`~pick` 已废弃**（2026-09-24）：它从来没被传给选框函数，现在选框恒为综合打分。

⚠️ 这些参数是**私有**的，launch 里 `<rosparam>` / `<param>` 必须写在 `<node>`
**内部**；写在 `<launch>` 顶层会落到全局 `/name`，节点读不到，**静默用默认值**。

**`publish_empty=false`（默认）的含义**：没检到箱子时**一个字都不发**，下游
`box_detection_node` 继续等。这与 `NodeBoxObs` 对 `valid=false` 的处置同构 ——
宁可让下游用上一帧的好值，也不要喂一个空的进去。

跑法
----
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1     # 推理前必做，见 README
    rosrun carton_box_yolo carton_box_detect.py _model_path:=/path/to/model.onnx

`OMP_NUM_THREADS=1` 是**必须的**：onnxruntime 与 OpenBLAS 在这个规模的模型上
开多线程是纯开销，而且会和 ROS 的回调线程抢核。
"""
from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

# 让 `carton_detector`（同目录的纯函数模块）能被 import —— rosrun 不保证
# 脚本目录在 sys.path 里。
sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import rospy
    from geometry_msgs.msg import Point32, PolygonStamped
    from sensor_msgs.msg import Image
except ImportError as exc:                                   # pragma: no cover
    raise SystemExit(
        f"缺少 ROS1 依赖（{exc}）。本节点要在 ROS Noetic 环境里跑：\n"
        f"    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH"
    )

try:
    import cv2
    import numpy as np
except ImportError as exc:                                   # pragma: no cover
    raise SystemExit(f"缺少 cv2/numpy（{exc}）")

from carton_detector import (  # noqa: E402
    DEFAULT_DEPTH_PATCH_PX,
    DEFAULT_SCORE_W_AREA,
    DEFAULT_SCORE_W_CONF,
    DEFAULT_SCORE_W_DEPTH,
    DEFAULT_Z_FAR_MM,
    DEFAULT_Z_NEAR_MM,
    box_to_polygon_points,
    decode,
    sample_depth_mm,
    score_candidates,
    to_input_tensor,
)

PKG_DIR = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = PKG_DIR / "models" / "best_cartonbb.onnx"


def _depth_to_mm(msg: Image, scale: float) -> np.ndarray:
    """`sensor_msgs/Image` 深度 → `(H, W)` **毫米** `np.ndarray`（float64）。

    与 `box_detection` 的 `_arr_to_depth_mm` 同一套口径：Orbbec 头部相机发
    **16UC1 且已经是毫米**；有些驱动发 `32FC1` 的**米**。两种都认，`scale`
    是额外的兜底系数。认不出来的编码**报错而不是猜** —— 猜错了是静默错
    （深度差 1000 倍会直接把打分带偏）。

    ⚠️ 行要按 `msg.step` 切，**不是 `width`** —— 一行字节数是 `step`，按 `width`
    切会剩不下（`box_detection` 那边踩过这个坑）。
    """
    enc = (msg.encoding or "").strip()
    itemsize = 2 if enc in ("16UC1", "mono16") else 4
    if msg.step % itemsize != 0:
        raise ValueError(f"step={msg.step} 与 {enc!r} 的元素宽度对不齐")
    row = msg.step // itemsize
    if msg.width > row:
        raise ValueError(f"一行只有 {row} 个元素，装不下 {msg.width} 像素"
                         f"（encoding={msg.encoding!r}）")
    if enc in ("16UC1", "mono16"):
        a = np.frombuffer(msg.data, np.uint16).reshape(msg.height, row)[:, :msg.width]
        a = a.astype(np.float64)
        return a if scale == 1.0 else a * float(scale)
    if enc == "32FC1":
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, row)[:, :msg.width]
        return a.astype(np.float64) * 1000.0 * float(scale)
    raise ValueError(f"不支持的深度编码 {msg.encoding!r}（要 16UC1 或 32FC1）")


class CartonBoxDetectNode:
    def __init__(self) -> None:
        self.image_topic = rospy.get_param("~image", "/camera/color/image_raw")
        self.out_topic = rospy.get_param("~box_out", "/box/yolo_box")
        # ⚠️ **空串要当成"没给"**：`get_param` 只在参数**不存在**时才回落默认值，
        # 而本包 yaml（`config/carton_box_yolo.yaml`）与 launch 都**显式**把
        # `model_path` 设成 `""` —— 参数存在，于是这里拿到 `""`，`Path("")` 的
        # `is_file()` 是 False，节点直接 `SystemExit("找不到模型")`。
        # 实测踩过：照 README 的默认流程 `roslaunch carton_box_yolo ...` 起不来。
        # 判空放这里而不是改 yaml，是因为"留空=用默认"这句话**写在 yaml 的注释里**，
        # 是承诺过的语义；把承诺改掉不如让承诺成立。
        raw_model = str(rospy.get_param("~model_path", "")).strip()
        self.model_path = raw_model or str(DEFAULT_MODEL)
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.conf = float(rospy.get_param("~conf", 0.25))
        self.target_class = int(rospy.get_param("~target_class", -1))
        # ⚠️ `~pick` **已废弃**（2026-09-24）。它以前是 `largest`/`best` 二选一，
        # 但那个参数**从来没有被传给选框函数** —— 读了、打了日志、没生效，是死
        # 参数；而且那两个语义各自都有反例（邻箱可能更大，邻箱的置信度也常常很
        # 高）。现在选框恒为「置信度 + 面积 + 中心深度」综合打分，没有模式可切。
        # 配置里还写着它的**点名一次**，别让"我明明写了 pick 却没反应"变成哑谜。
        if rospy.has_param("~pick"):
            rospy.logwarn("[carton_box_yolo] ~pick=%r 已废弃并被忽略 —— 选框现在是"
                          "「置信度 + 面积 + 中心深度」综合打分（见 "
                          "~score_w_* / ~z_near_mm / ~z_far_mm），没有模式可切。"
                          "请从 yaml 里删掉这一项。",
                          rospy.get_param("~pick"))
        self.publish_empty = self._as_bool(rospy.get_param("~publish_empty", False))
        self.max_rate = float(rospy.get_param("~max_rate_hz", 0.0))
        queue_size = int(rospy.get_param("~queue_size", 1))

        # ---- 打分参数（选箱判据）------------------------------------------
        self.depth_topic = str(rospy.get_param("~image_depth",
                                               "/camera/depth/image_raw")).strip()
        self.depth_scale = float(rospy.get_param("~depth_scale_mm", 1.0))
        self.depth_patch_px = int(rospy.get_param("~depth_patch_px",
                                                  DEFAULT_DEPTH_PATCH_PX))
        self.z_near_mm = float(rospy.get_param("~z_near_mm", DEFAULT_Z_NEAR_MM))
        self.z_far_mm = float(rospy.get_param("~z_far_mm", DEFAULT_Z_FAR_MM))
        self.w_conf = float(rospy.get_param("~score_w_conf",
                                            DEFAULT_SCORE_W_CONF))
        self.w_area = float(rospy.get_param("~score_w_area",
                                            DEFAULT_SCORE_W_AREA))
        self.w_depth = float(rospy.get_param("~score_w_depth",
                                             DEFAULT_SCORE_W_DEPTH))
        # 参数写反只会在打分时静默给出恒定 0/1，现场查不出来 —— 这里点名一次。
        if self.z_far_mm <= self.z_near_mm:
            rospy.logwarn("[carton_box_yolo] ~z_far_mm(%.1f) <= ~z_near_mm(%.1f)，"
                          "深度项退化成阶跃（近于 z_near 得 1、否则 0）",
                          self.z_far_mm, self.z_near_mm)
        if self.w_conf + self.w_area + self.w_depth <= 0.0:
            rospy.logwarn("[carton_box_yolo] 三个 ~score_w_* 全 <= 0，所有候选"
                          "得分恒为 0，选框退化成「取第一个」")

        self._sess = None
        self._lock = threading.Lock()          # onnxruntime 的 session 不是线程安全的
        self._depth = None                     # **最新一帧深度**，只存不排队
        self._depth_lock = threading.Lock()
        self._last_pub = 0.0
        self._n_in = 0
        self._n_pub = 0
        self._n_empty = 0
        self._n_skip = 0

        self.pub = rospy.Publisher(self.out_topic, PolygonStamped, queue_size=1)
        self._load_model()
        self.sub = rospy.Subscriber(self.image_topic, Image, self._on_image,
                                    queue_size=queue_size, buff_size=2 ** 24)
        self.depth_sub = None
        if self.depth_topic:
            # **只存最新一帧，不排队**（`box_detection` 那边是双路同步；这里不同：
            # 深度只是打分的一个输入，配不上就用"无深度"降级，不值得为它丢帧）。
            self.depth_sub = rospy.Subscriber(self.depth_topic, Image,
                                              self._on_depth, queue_size=1)
        rospy.loginfo("[carton_box_yolo] %s → %s（模型 %s，imgsz=%d，conf=%.2f）",
                      self.image_topic, self.out_topic, self.model_path,
                      self.imgsz, self.conf)
        rospy.loginfo("[carton_box_yolo] 选箱打分 w_conf=%.2f w_area=%.2f "
                      "w_depth=%.2f；深度 %s（patch=±%dpx，z∈[%.0f, %.0f]mm，scale=%.3f）",
                      self.w_conf, self.w_area, self.w_depth,
                      self.depth_topic or "**关闭**（全候选降级）",
                      self.depth_patch_px, self.z_near_mm, self.z_far_mm,
                      self.depth_scale)

    # ------------------------------------------------------------------ 工具
    @staticmethod
    def _as_bool(raw) -> bool:
        """`bool("false") is True` —— 场景 JSON 真会传字符串，不能直接 `bool()`。"""
        if isinstance(raw, bool):
            return raw
        return str(raw).strip().lower() in ("1", "true", "yes", "on")

    def _on_depth(self, msg: Image) -> None:
        """深度回调：**只解包存最新一帧**，不做推理、不排队。

        解包失败（编码不认识）就丢掉这一帧并记一次 —— 下一次回调还会来，不需要
        在这里做任何重试。解包放在回调里而不是 `_on_image` 里，是为了让耗时
        发生在深度线程上，不拖慢 YOLO 那条路径。
        """
        try:
            arr = _depth_to_mm(msg, self.depth_scale)
        except Exception as exc:                              # noqa: BLE001
            rospy.logwarn_throttle(5.0, "[carton_box_yolo] 解不了深度图：%s", exc)
            return
        with self._depth_lock:
            self._depth = arr

    def _load_model(self) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise SystemExit(
                f"缺少 onnxruntime（{exc}）。本节点跑 ONNX 权重，不依赖 ultralytics：\n"
                f"    pip install onnxruntime        # 有 GPU 用 onnxruntime-gpu"
            )
        if not Path(self.model_path).is_file():
            raise SystemExit(
                f"找不到模型 {self.model_path}。权重不进 git，要手工拷进去：\n"
                f"    {PKG_DIR}/models/README.md"
            )
        providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider")
                     if p in ort.get_available_providers()] or ["CPUExecutionProvider"]
        self._sess = ort.InferenceSession(self.model_path, providers=providers)
        rospy.loginfo("[carton_box_yolo] onnxruntime %s，provider=%s",
                      ort.__version__, self._sess.get_providers()[0])

    # ------------------------------------------------------------------ 回调
    def _on_image(self, msg: Image) -> None:
        self._n_in += 1
        now = time.monotonic()
        if self.max_rate > 0.0 and (now - self._last_pub) < 1.0 / self.max_rate:
            self._n_skip += 1
            return

        try:
            img = self._to_bgr(msg)
        except Exception as exc:                              # noqa: BLE001
            rospy.logwarn_throttle(5.0, "[carton_box_yolo] 解不了图像：%s", exc)
            return

        t0 = time.monotonic()
        try:
            x, r, pad = to_input_tensor(img, self.imgsz)
            with self._lock:
                raw = self._sess.run(None, {"images": x})[0]
        except Exception as exc:                              # noqa: BLE001
            rospy.logerr_throttle(5.0, "[carton_box_yolo] 推理失败：%s", exc)
            return
        dt_ms = (time.monotonic() - t0) * 1000.0

        dets = decode(raw[0], r, pad, conf_thr=self.conf, orig_shape=img.shape[:2])

        # ---- 选箱：置信度 + 面积 + 中心深度 综合打分 ----------------------
        label = None if self.target_class < 0 else self.target_class
        with self._depth_lock:
            depth = self._depth                    # 只取引用，别在锁里做重活
        zs = [sample_depth_mm(depth, d['box_uv'], half=self.depth_patch_px,
                              z_near=self.z_near_mm, z_far=self.z_far_mm)
              for d in dets]
        scored = score_candidates(dets, zs, w_conf=self.w_conf, w_area=self.w_area,
                                  w_depth=self.w_depth,
                                  z_near=self.z_near_mm, z_far=self.z_far_mm)
        cand = [(d, s, p) for d, s, p in scored
                if label is None or d['class_id'] == int(label)]
        best, best_score, best_parts = (
            max(cand, key=lambda t: t[1]) if cand else (None, None, None))

        if best is None:
            self._n_empty += 1
            rospy.loginfo_throttle(5.0, "[carton_box_yolo] 本帧没检到箱子（累计 %d 帧）",
                                   self._n_empty)
            if not self.publish_empty:
                return
            pts = []
        else:
            pts = box_to_polygon_points(best["box_uv"])

        out = PolygonStamped()
        out.header = msg.header                     # **沿用图像的时间戳与 frame_id**
        for u, v in pts:
            p = Point32()
            p.x, p.y, p.z = float(u), float(v), 0.0
            out.polygon.points.append(p)
        self.pub.publish(out)
        self._last_pub = now
        self._n_pub += 1

        if best is not None:
            # 日志带上**分项明细** —— 现场"为什么挑了这个"只能靠它回答。
            # 格式不进任何契约，给人看。
            rospy.loginfo_throttle(
                2.0, "[carton_box_yolo] %.1fms score=%.3f（conf=%.2f 面积归一=%.2f "
                     "深度=%s）box=[%.1f, %.1f, %.1f, %.1f]（本帧 %d 个检测，"
                     "发布 %d 次）",
                dt_ms, best_score, best_parts['conf'], best_parts['area_norm'],
                ("none" if best_parts['depth'] is None
                 else "%.2f@%.0fmm" % (best_parts['depth'], best_parts['z_mm'])),
                *best["box_uv"], len(dets), self._n_pub)

    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray:
        """`sensor_msgs/Image` → BGR `np.ndarray`。

        **不依赖 cv_bridge**（那个包在某些精简镜像里没有），自己按 `encoding` 解。
        认 `bgr8` / `rgb8` / `mono8`，其余编码直接报错而不是猜 —— 猜错了是静默错
        （通道顺序反了框会偏，而且看不出来）。
        """
        enc = (msg.encoding or "").lower()
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if enc == "bgr8":
            return buf.reshape(msg.height, msg.width, 3)
        if enc == "rgb8":
            return buf.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
        if enc in ("mono8", "8uc1"):
            return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        raise ValueError(f"不支持的图像编码 {msg.encoding!r}"
                         f"（认 bgr8 / rgb8 / mono8）")


def main() -> None:
    rospy.init_node("carton_box_detect", anonymous=False)
    CartonBoxDetectNode()
    rospy.spin()


if __name__ == "__main__":
    main()
