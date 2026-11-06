#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 节点：`/box/detection` —— 箱子顶面四角的实时检测结果（OBB 版）。

**分层**（架构硬约束，与托盘那条链同构）：

    skills/atomic/perception/carton_detect/obb_decoder.py
                              纯函数、零状态、**不 import ROS / ultralytics** ← 算法核心
    carton_obb_detect.py      本文件：只做搬运（订阅 / 推理 / 出队 / 填消息 / 发布）

⚠️ **时间窗在搬运层，不在算法层**：OBB 已经是端到端输出，没有「有状态的算法」
    可分层。但窗本身有用 —— 它把输出从 30Hz 降到 10Hz，并压掉单帧抖动。
    语义：`window` 个**处理槽**、每 `process_every` 帧处理一次、各帧四角**逐点平均**。

订阅（两路都要时间戳接近，用 `ApproximateTimeSynchronizer` 对齐）：
  * `~image_color`  sensor_msgs/Image          彩色图（默认 `/camera/color/image_raw`）
  * `~image_depth`  sensor_msgs/Image          对齐后的深度图，16UC1 单位 mm；用于选箱打分与深度下沿修正（默认开）
  * `~camera_info`  sensor_msgs/CameraInfo     内参。OBB 推理本身**不需要**内参，
    但下游 / 叠加图要用于反投影，且换相机时要能对上，所以照订。

发布：
  * `/box/detection`   box_detection_msgs/BoxDetection   ← 伺服那条链吃这个

⚠️ **从前还发一路 `/box/yolo_box`（`PolygonStamped`），2026-10-15 删了。**
它只被 `node_pallet_servo` 的 `box_uv_in` 拿去除叠加图上画个灰框，不参与任何计算；
而它发的内容就是 `/box/detection` 那四个角的**轴对齐包围盒** —— 同一个
`out['corners_uv']` 取 min/max。伺服节点手里的 `BoxObservation.u1/v1/u2/v2`
（`node_box_obs.py` 用 min/max 填的）**本来就是同一个数**，所以删掉零信息损失，
而且那条订阅也不用加。三个话题简化成两个。

参数（`rosparam`，都有默认值）：
  | 名字 | 默认 | 说明 |
  |---|---|---|
  | `image_color` | `/camera/color/image_raw` | 彩色话题 |
  | `image_depth` | `/camera/depth/image_raw` | 深度话题，**用于选箱打分与可选的深度下沿修正** |
  | `camera_info` | `/camera/color/camera_info` | 内参话题 |
  | `detection_out` | `/box/detection` | 四角输出 |
  | `model_path` | `<算法包>/models/carton_obb.onnx` | ONNX 权重（留空 = 用默认值） |
  | `imgsz` | `640` | 输入边长，**必须与导出时一致** |
  | `conf` | `0.25` | 置信度阈值（只用于过滤与 NMS，**不参与选箱打分**） |
  | `nms_iou` | `0.45` | 旋转框 NMS 阈值：两个框**互相错着盖**。导出图没有 NMS，**必须自己去重** |
  | `nms_iomin` | `0.85` | 旋转框 NMS 的另一条：**小框整个套在大框里**（`交/较小面积`）。见 §3 ② |
  | `window` | `3` | 滑动窗长（处理槽数） |
  | `process_every` | `1` | 每几帧进一个处理槽（相机 30Hz → 30Hz）。**它直接决定 `/box/detection` 的上限，而 `/pallet/dis` 被 min(托盘率, 纸箱率) 封顶** —— 2026-10-19 实测默认 3 时纸箱只有 7.8Hz，把 `/pallet/dis` 拖到 7~8Hz；单帧 27~40ms，30Hz 撑得住，故默认改 1。 |
  | `agg` | `mean` | 多帧聚合：`mean` / `median` |
  | `d_near_px` / `d_far_px` | `100.0` / `450.0` | **选箱主判据**：到画面下方中心的距离刻度 |
  | `score_w_dcenter` | `0.5` | 距离项权重 |
  | `score_w_area` | `0.2` | 面积项权重（同帧相对归一） |
  | `score_w_depth` | `0.3` | 中心深度项权重 |
  | `z_near_mm` / `z_far_mm` | `500.0` / `1500.0` | 深度项刻度 |
  | `depth_patch_px` | `15` | 中心深度采样方块的半边长 |
  | `depth_refine_bottom` | `true` | 用**深度下沿**把手持箱被拉长的下边收回去；只在框内箱体与下方背景有明确深度跳变时生效，没有跳变保持模型框 |
  | `depth_refine_step` | `2` | 深度剖面的行/列采样步长（越大越快、y 的量化越粗） |
  | `depth_refine_band_px` | `45` | 沿框中心左右各取多少像素做深度剖面 |
  | `depth_refine_jump_mm` | `150.0` | 箱体到下方背景的最小持续深度跳变 |
  | `depth_refine_min_shrink_px` | `3.0` | 至少收缩这么多像素才改框（只收缩，不放大） |
  | `depth_scale_mm` | `1.0` | 深度图单位换算到 mm 的系数 |
  | `sync_slop_s` | `0.05` | 两路时间戳允许的最大差 |
  | `queue_warn_ms` | `200.0` | 处理慢于这个就告警一次（节流） |
  | `max_rate_hz` | `0.0` | >0 时降频（相机 30Hz、OBB 10Hz 就够） |
  | `publish_empty` | `false` | 没检到时要不要发一帧 `valid=false` |
  | `heartbeat_s` | `5.0` | 心跳周期（秒）。**成功、没检到、没数据都打**，见 §心跳。`0` = 关掉 |
  | `infer_mode` | `auto` | 推理后端：`auto`（启动时探测公共服务，在就用、不在就本地 ONNX）/ `shared` / `local` |
  | `infer_service` | `/carton_obb/infer` | 公共服务名（`box_detection_msgs/InferCartonObb`） |
  | `infer_probe_s` | `2.0` | 启动时等服务的秒数 |
  | `shared_fail_limit` | `3` | 共享服务连续失败几次后切回本地 ONNX（不自动切回） |

**选箱判据**（2026-10-13 改，见 `obb_decoder.candidate_score`）：
**到画面下方中心的距离**（权重 0.5）+ 面积（0.2）+ 中心深度（0.3）。
**置信度不再参与打分** —— 现场实测反例：一帧里手箱 conf 0.9773、邻箱 0.9642
（分不开），而邻箱面积还更大；「离下方中心最近」挑对了。

**`orientation` 字段恒为空串**：旧链路的朝向先验（`/infer_carton_pose` 的
`tape_orientation_deg`）是为「矩形搜索里整框转 90°」那个歧义服务的，OBB 直接给出
角度，没有这个歧义。参数与那个 ServiceProxy 一并去掉。

跑法（`OMP_NUM_THREADS=1` 是必须的，实测差 1.6x，见 WORKLOG §15.1）：
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
    export LD_LIBRARY_PATH=<conda-env>/lib/python3.*/site-packages/nvidia/cudnn/lib:...:$LD_LIBRARY_PATH
    python carton_obb_detect.py
"""
from __future__ import annotations

import os
import queue
import sys
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def _repo_root() -> Path:
    """仓库根 = **第一个同时含 `skills/` 与 `infrastructure/` 的祖先目录**。

    ⚠️ 故意**不数 `parents[N]`**：本节点的目录深度是会变的（2026-10-15 从
    `detection_industrial_yolo/carton_obb/` 挪到 `ros_vision/carton_obb/`，与
    `pallet_detection` 并排），数层数的写法**挪一次断一次**，而且断在
    `import` 上、只有在真机起节点时才炸。按特征找则与位置无关。

    找不到时返回最外层祖先（不抛）—— 调用方本来就允许"拿不到仓库根就退回
    `rospy.log*`"，不该因为路径布局不认识就直接起不来。
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'skills').is_dir() and (parent / 'infrastructure').is_dir():
            return parent
    return here.parents[-1]


_REPO_ROOT = _repo_root()
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

try:
    import rospy
    from geometry_msgs.msg import Point32
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Header
    from message_filters import ApproximateTimeSynchronizer, Subscriber
except ImportError as exc:                       # pragma: no cover
    raise SystemExit(
        f'缺少 ROS1 依赖（{exc}）。本节点要在 ROS Noetic 环境里跑：\n'
        '    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH')

try:
    from box_detection_msgs.msg import BoxDetection
except ImportError as exc:                       # pragma: no cover
    raise SystemExit(
        f'没找到消息包 box_detection_msgs（{exc}）。把本仓库的 '
        'infrastructure/ros_packages/src/ros_vision/box_detection_msgs 放进 catkin '
        '工作区的 src/ 下 catkin_make，再 source devel/setup.bash：\n'
        '    cp -r <LeTools>/infrastructure/ros_packages/src/ros_vision/'
        'box_detection_msgs <工作区>/src/ && cd <工作区> && catkin_make')

# 公共服务不是硬依赖：旧部署机上 srv 可能还没编译，那种情况只告警、退回本地 ONNX。
try:
    from box_detection_msgs.srv import InferCartonObb
except ImportError:                              # pragma: no cover
    InferCartonObb = None

from skills.atomic.perception.carton_detect import (        # noqa: E402
    DEFAULT_D_FAR_PX, DEFAULT_D_NEAR_PX, DEFAULT_DEPTH_PATCH_PX,
    DEFAULT_MODEL_PATH, DEFAULT_NMS_IOMIN, DEFAULT_NMS_IOU,
    DEFAULT_REFINE_BOTTOM_BAND_PX, DEFAULT_REFINE_BOTTOM_JUMP_MM,
    DEFAULT_REFINE_BOTTOM_MIN_SHRINK_PX, DEFAULT_REFINE_BOTTOM_STEP,
    DEFAULT_SCORE_W_AREA, DEFAULT_SCORE_W_DCENTER, DEFAULT_SCORE_W_DEPTH,
    DEFAULT_Z_FAR_MM, DEFAULT_Z_NEAR_MM,
    angle_deg_from_corners, decode_obb, dets_from_rotated_rects,
    nms_rotated, pick_best, score_candidates,
    refine_bottom_by_depth, sample_depth_mm, to_input_tensor,
)


# ⚠️ 默认权重路径**由算法包给出**（`carton_detect.DEFAULT_MODEL_PATH` →
# `<算法包>/models/carton_obb.onnx`），与托盘那条链同构（那边是
# `pallet_detect.detect.DEFAULT_MODEL_PATH`）。本节点只**取用**，不自己算路径 ——
# 从前这里用 `Path(__file__).parent.parent/'models'` 指向 ROS 包，于是算法搬去
# `skills/` 之后权重还留在 `infrastructure/`，纯算法包反而依赖 ROS 包的目录结构。


# --------------------------------------------------------------------------- #
# 统一日志（`core/common/logger.py`）—— **带回退**
# --------------------------------------------------------------------------- #
# 与托盘节点同一套：每行带 `[TRACE:xxxxxxxx] [file.py:行号]`，日志写进
# `log/` 下带时间戳的文件，支持"写满另开一个 + 按天清理"。
#
# ⚠️ **必须带回退**：单独 `roslaunch carton_obb …` 时仓库根不一定在 `sys.path` 上
# （一键脚本会 export PYTHONPATH，别的跑法不会）。import 不到就退回 `rospy.log*`。
try:
    from core.common.logger import init_logging as _init_logging
except Exception:                                    # noqa: BLE001
    _init_logging = None


def _console_output() -> bool:
    """统一 logger 还要不要往终端打一份。

    一键启动脚本会把它关掉（`LETOOLS_LOG_CONSOLE=0`）：那个脚本已经把本进程的
    stdout 重定向进**共用的日志文件**，而 logger 的控制台 handler 也写 stdout ——
    两份叠在一起，就是**每条日志在同一个文件里出现两次**。关掉之后每次只写一遍。

    手工 `roslaunch carton_obb …` 时不设这个变量，终端照常有输出。
    """
    return os.environ.get('LETOOLS_LOG_CONSOLE', '1').strip().lower() not in (
        '0', 'false', 'no', 'off')


def _init_unified_logging(tag: str) -> None:
    """**必须在 `rospy.init_node()` 之后调** —— init_node 会往 root logger 上
    重装它自己的 handler，顺序反了就白配（既有写法见
    `orchestration/nodes/boxcarry.py`）。"""
    if _init_logging is None:
        rospy.logwarn('[carton_obb] 没找到 core.common.logger —— 日志只走 rospy，'
                      '不落盘、不轮转。注意仓库根**本节点自己会插进 sys.path**，'
                      '所以这多半不是路径问题，而是缺依赖（requirements.txt 里的 '
                      'colorlog）或部署的是旧代码')
        return
    try:
        _init_logging(force=True, log_tag=tag, console_output=_console_output())
    except Exception as exc:                         # noqa: BLE001
        rospy.logwarn('[carton_obb] 统一日志初始化失败，退回 rospy：%s', exc)


def _as_bool(value) -> bool:
    """`bool("false") is True` —— rosparam / 场景 JSON 真会传字符串。"""
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on', 'y', 't')
    return bool(value)


def _image_to_array(msg, dtype, channels: int = 1):
    """sensor_msgs/Image -> np.ndarray (H, W*channels)，**按 msg.step 处理行填充**。

    ⚠️ 列要按 `width * channels` 切，**不是 `width`** —— 一个像素三字节的 bgr8 图，
    按 `width` 切会只剩三分之一的缓冲。**不依赖 cv_bridge**（它拖 boost_python）。
    """
    itemsize = np.dtype(dtype).itemsize
    if msg.step % itemsize != 0:
        raise ValueError(f'step={msg.step} 与 dtype {dtype} 对不齐')
    row = msg.step // itemsize
    a = np.frombuffer(msg.data, dtype).reshape(msg.height, row)
    n = msg.width * channels
    if n > row:
        raise ValueError(f'一行只有 {row} 个元素，装不下 {msg.width} 像素 × '
                         f'{channels} 通道；检查 encoding={msg.encoding!r}')
    return a[:, :n]


def _arr_to_bgr(msg):
    """-> (H,W,3) BGR。**认不出的编码直接报错，不猜** —— 猜错了通道顺序反了
    框会偏，而且看不出来。"""
    import cv2
    if msg.encoding in ('bgr8', '8UC3'):
        return _image_to_array(msg, np.uint8, 3).reshape(msg.height, msg.width, 3).copy()
    if msg.encoding == 'rgb8':
        a = _image_to_array(msg, np.uint8, 3).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    if msg.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(_image_to_array(msg, np.uint8), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'不支持的彩色编码 {msg.encoding!r}（要 bgr8 / rgb8 / mono8）')


def _arr_to_depth_mm(msg, scale: float):
    """-> (H,W) uint16，**单位 mm**（选箱打分用）。

    Orbbec 头部相机发 16UC1 且**已经是毫米**；有些驱动发 32FC1 的**米**。
    """
    if msg.encoding in ('16UC1', 'mono16'):
        a = _image_to_array(msg, np.uint16)
        return a if scale == 1.0 else np.rint(a * scale).astype(np.uint16)
    if msg.encoding == '32FC1':
        a = _image_to_array(msg, np.float32)
        return np.rint(a * 1000.0 * scale).astype(np.uint16)
    raise ValueError(f'不支持的深度编码 {msg.encoding!r}（要 16UC1 或 32FC1）')


class CornerWindow:
    """最近 `window` 个**处理槽**的四角，逐点平均（或取中位）。

    ⚠️ `push()` 在非处理槽返回 `None`；窗没满也出结果（有几帧算几帧）。
    空窗返回 `None`（**不退回任何东西** —— 把原始外接框当四角发出来是
    「看着规矩、其实角度是错的」的静默错，这里没有那个兜底）。
    """

    def __init__(self, window: int = 3, process_every: int = 1, agg: str = 'mean'):
        if window < 1:
            raise ValueError('window 至少 1')
        if process_every < 1:
            raise ValueError('process_every 至少 1')
        if agg not in ('mean', 'median'):
            raise ValueError("agg 只能是 'mean' 或 'median'")
        self.window = int(window)
        self.process_every = int(process_every)
        self.agg = agg
        self._slots: deque = deque(maxlen=int(window))
        self._n_seen = 0

    @property
    def n_slots(self) -> int:
        return len(self._slots)

    def push(self, corners_uv, confidence=None):
        """喂一帧的四角。**非处理槽返回 None**。返回 payload 或 None。"""
        self._n_seen += 1
        if (self._n_seen - 1) % self.process_every != 0:
            return None
        self._slots.append({'corners': np.asarray(corners_uv, np.float64),
                            'confidence': confidence})
        stack = np.stack([s['corners'] for s in self._slots])
        corners = (np.median(stack, axis=0) if self.agg == 'median'
                   else stack.mean(axis=0))
        spread = float(np.abs(stack - corners[None]).max())
        return {'corners_uv': corners,
                'source': 'single' if len(self._slots) == 1 else 'window',
                'n_used': len(self._slots),
                'n_slots': self.n_slots,
                'spread_px': spread}


class CartonObbNode:
    def __init__(self):
        self.image_color = rospy.get_param('~image_color', '/camera/color/image_raw')
        self.image_depth = rospy.get_param('~image_depth', '/camera/depth/image_raw')
        self.info_topic = rospy.get_param('~camera_info', '/camera/color/camera_info')
        self.out_topic = rospy.get_param('~detection_out', '/box/detection')
        self.model_path = str(rospy.get_param('~model_path', '') or DEFAULT_MODEL_PATH)
        self.imgsz = int(rospy.get_param('~imgsz', 640))
        self.conf = float(rospy.get_param('~conf', 0.25))
        self.nms_iou = float(rospy.get_param('~nms_iou', DEFAULT_NMS_IOU))
        self.nms_iomin = float(rospy.get_param('~nms_iomin', DEFAULT_NMS_IOMIN))
        self.depth_scale = float(rospy.get_param('~depth_scale_mm', 1.0))
        self.sync_slop = float(rospy.get_param('~sync_slop_s', 0.05))
        self.queue_warn_ms = float(rospy.get_param('~queue_warn_ms', 200.0))
        self.publish_empty = _as_bool(rospy.get_param('~publish_empty', False))
        self.max_rate = float(rospy.get_param('~max_rate_hz', 0.0))

        # 选箱参数（见 obb_decoder.candidate_score）
        self.d_near = float(rospy.get_param('~d_near_px', DEFAULT_D_NEAR_PX))
        self.d_far = float(rospy.get_param('~d_far_px', DEFAULT_D_FAR_PX))
        self.w_dcenter = float(rospy.get_param('~score_w_dcenter', DEFAULT_SCORE_W_DCENTER))
        self.w_area = float(rospy.get_param('~score_w_area', DEFAULT_SCORE_W_AREA))
        self.w_depth = float(rospy.get_param('~score_w_depth', DEFAULT_SCORE_W_DEPTH))
        self.z_near = float(rospy.get_param('~z_near_mm', DEFAULT_Z_NEAR_MM))
        self.z_far = float(rospy.get_param('~z_far_mm', DEFAULT_Z_FAR_MM))
        self.depth_patch = int(rospy.get_param('~depth_patch_px', DEFAULT_DEPTH_PATCH_PX))
        # 深度下沿修正（默认开；见 obb_decoder.refine_bottom_by_depth）。
        # 只在「框内箱体 → 下方远背景」有持续跳变时收缩下边，没有跳变保持模型框。
        self.refine_bottom = _as_bool(rospy.get_param('~depth_refine_bottom', True))
        self.refine_step = max(1, int(rospy.get_param('~depth_refine_step',
                                                     DEFAULT_REFINE_BOTTOM_STEP)))
        self.refine_band = max(5, int(rospy.get_param('~depth_refine_band_px',
                                                     DEFAULT_REFINE_BOTTOM_BAND_PX)))
        self.refine_jump = float(rospy.get_param('~depth_refine_jump_mm',
                                                 DEFAULT_REFINE_BOTTOM_JUMP_MM))
        self.refine_min_shrink = float(rospy.get_param(
            '~depth_refine_min_shrink_px', DEFAULT_REFINE_BOTTOM_MIN_SHRINK_PX))
        if self.z_far <= self.z_near:
            rospy.logwarn('[carton_obb] ~z_far_mm(%.1f) <= ~z_near_mm(%.1f)，'
                          '深度项退化成阶跃', self.z_far, self.z_near)
        if self.w_dcenter <= 0.0 and self.w_area <= 0.0 and self.w_depth <= 0.0:
            rospy.logwarn('[carton_obb] 三个 ~score_w_* 全 <= 0，所有候选同分，'
                          '选箱退化成「取第一个」')

        self.win = CornerWindow(window=int(rospy.get_param('~window', 3)),
                                process_every=int(rospy.get_param('~process_every', 1)),
                                agg=str(rospy.get_param('~agg', 'mean')))

        # ---- 推理后端（2026-10-20）------------------------------------------
        # `auto`   ：启动时探测公共服务 `/carton_obb/infer`；在就用服务的输出，
        #            不在就自己加载本地 ONNX。**探测只在启动时做一次。**
        # `shared` ：强制用公共服务（不在就报错并退回本地）。
        # `local`  ：强制本地 ONNX，连探测都不做。
        #
        # ⚠️ **两条路必须逐位同口径**，否则"用公共输出"与"自己推理"会出现肉眼
        #    看不出的差异。服务端只做前处理 + 推理 + NMS，这里请求 `nms_iou=1.0`
        #    让它**不要 NMS**，把去重留给本节点的 `nms_rotated`（IoU + IoMin）——
        #    服务端只有 IoU 一条判据，且栅格化口径不同。转换与过滤由
        #    `dets_from_rotated_rects` 按 `decode_obb` 的口径做。
        self.infer_service = str(rospy.get_param(
            '~infer_service', '/carton_obb/infer')).strip()
        self.infer_mode = str(rospy.get_param('~infer_mode', 'auto')).strip().lower()
        if self.infer_mode not in ('auto', 'shared', 'local'):
            rospy.logwarn('[carton_obb] ~infer_mode=%r 不认识（只认 auto/shared/local），'
                          '按 auto 处理', self.infer_mode)
            self.infer_mode = 'auto'
        self.infer_probe_s = float(rospy.get_param('~infer_probe_s', 2.0))
        self.shared_fail_limit = max(1, int(rospy.get_param('~shared_fail_limit', 3)))

        self._mode = 'local'
        self._sess = None
        self._in_name = None
        self._service = None
        self._shared_fail = 0
        self._last_shared_warn = 0.0
        self._last_src_stamp = None
        self._n_dup_frames = 0
        self._select_backend()

        # 深度单独订阅（不同步进时间窗 —— 它只参与打分）。按 stamp 留一小段历史：
        # 用公共服务时推理帧是**服务端**那一帧，可能比本节点队列里的晚几十毫秒，
        # 精确配对靠这张表（见 `_depth_for`）。本地后端两路同组同步，stamp 恒等。
        self._depth = None
        self._depth_lock = threading.Lock()
        self._depth_by_stamp = OrderedDict()
        self._depth_hist = 8
        self._depth_exact_hits = 0
        self._depth_fallbacks = 0
        rospy.Subscriber(self.image_depth, Image, self._on_depth, queue_size=1)
        # ⚠️ **必须接住返回值**：`_on_camera_info` 里要靠它 `unregister()` 把订阅摘掉。
        # 从前这里是裸调用、且 `_on_camera_info` 里那句 `self.__dict__['_info_sub']`
        # 取的是**从来没赋过的键**，必抛 KeyError 又被 `except: pass` 吞掉 ——
        # 于是内参那行 loginfo **以相机频率反复打印**（30Hz 刷屏），
        # 把日志里真正有用的东西全埋了。托盘节点同一处是对的（有守卫 + 真的注销）。
        self._info_sub = rospy.Subscriber(self.info_topic, CameraInfo,
                                          self._on_camera_info, queue_size=1)

        self.pub = rospy.Publisher(self.out_topic, BoxDetection, queue_size=5)

        # **队列长度 1**：宁可丢旧帧也不要排队 —— 排队的后果是输出的是几百毫秒前
        # 那一帧的结果，伺服拿去会晚。
        self.q: queue.Queue = queue.Queue(maxsize=1)
        self._n_dropped = 0
        self._n_empty = 0
        self._n_pub = 0
        self._last_warn = 0.0
        self._last_pub = 0.0
        # 心跳用（见 `_heartbeat`）。`self.k` 在收到第一帧 camera_info 之后才有，
        # 它是"内参只打一次"那个守卫的判据。
        self.k = None
        self._n_processed = 0
        self._last_good_ms = None
        self._last_reject = None
        self._last_heartbeat = 0.0
        try:
            self.heartbeat_s = float(rospy.get_param('~heartbeat_s', 5.0))
        except (TypeError, ValueError):
            self.heartbeat_s = 5.0

        # ---- 检测门控（2026-10-18）------------------------------------------
        # 现场流程是"先抓箱、到位后才 call slot"，码放之前不一定看得见箱子。
        # `~gated=true` 时：**空闲不出数**，只在服务器的门控话题说"有档位激活"
        # 之后才算；但启动时**先跑一帧预热**（首次推理要几十秒，必须挪到启动时，
        # 不能等 call 了服务才付这笔钱）。
        # 默认 **false** = 一直算，旧的独立 roslaunch 跑法行为不变。
        self.gated = bool(rospy.get_param('~gated', False))
        self.detect_mode_topic = str(rospy.get_param(
            '~detect_mode_topic', '/pallet_servo/detect_mode')).strip()
        self._gated_active = not self.gated
        self._warmed = False
        if self.gated:
            from std_msgs.msg import Int32
            self._mode_sub = rospy.Subscriber(
                self.detect_mode_topic, Int32, self._on_detect_mode,
                queue_size=1)

        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(self.image_color, Image), Subscriber(self.image_depth, Image)],
            queue_size=5, slop=self.sync_slop)
        self.sync.registerCallback(self._on_synced)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        # 日志落在哪个文件必须打出来：现场"日志里没东西"第一件要排除的，
        # 就是**根本没写进你翻的那个文件**（回退路径 / 别的目录 / 别的 tag）。
        try:
            from core.common.logger import get_active_log_file
            rospy.loginfo('[carton_obb] 日志文件：%s', get_active_log_file())
        except Exception:                            # noqa: BLE001
            pass                                     # 回退路径下没有它，不是错误
        rospy.loginfo('[carton_obb] 订阅 %s + %s -> 发布 %s',
                      self.image_color, self.image_depth, self.out_topic)
        rospy.loginfo('[carton_obb] 选箱：离画面下方中心越近分越高（w=%g，刻度 %g~%gpx）'
                      '+ 面积(w=%g) + 深度(w=%g)', self.w_dcenter, self.d_near,
                      self.d_far, self.w_area, self.w_depth)
        # 一行打全**会改变输出节奏**的几个量：时间窗与降频决定"10Hz 是怎么来的"，
        # imgsz/conf/nms_iou/nms_iomin 决定"为什么这个框没被选上"。现场对
        # "输出怎么是这样"的第一手依据就在这一行。
        rospy.loginfo('[carton_obb] 参数：imgsz=%d conf=%g nms_iou=%g nms_iomin=%g '
                      'window=%d process_every=%d max_rate=%gHz 心跳 %gs 空帧发布=%s '
                      '后端=%s',
                      self.imgsz, self.conf, self.nms_iou, self.nms_iomin,
                      self.win.window,
                      self.win.process_every, self.max_rate, self.heartbeat_s,
                      self.publish_empty, self._mode)
        if self.refine_bottom:
            rospy.loginfo('[carton_obb] 深度下沿修正：step=%d band=±%dpx '
                          'jump=%gmm min_shrink=%gpx',
                          self.refine_step, self.refine_band,
                          self.refine_jump, self.refine_min_shrink)

    # ------------------------------------------------------------ 推理后端
    def _select_backend(self) -> None:
        """启动时选一次后端：**公共服务在就用它，不在就自己加载 ONNX**。

        ⚠️ 探测只做一次（`infer_mode=auto`）。公共服务的生命周期长于本节点时，
        中途起/停不会切换 —— 中途杀服务由 `_shared_fail_limit` 兜到本地后端。
        """
        if self.infer_mode == 'local':
            self._load_local()
            return
        if InferCartonObb is None:
            rospy.logwarn('[carton_obb] box_detection_msgs/InferCartonObb 没有编译进'
                          '当前工作空间 —— 无法使用公共服务，改用本地 ONNX。'
                          '（重编：catkin_make box_detection_msgs）')
            self._load_local()
            return

        try:
            rospy.wait_for_service(self.infer_service, timeout=self.infer_probe_s)
            self._service = rospy.ServiceProxy(self.infer_service, InferCartonObb)
            self._mode = 'shared'
            rospy.loginfo('[carton_obb] 使用公共 OBB 推理服务 %s（不再本地加载 ONNX）',
                          self.infer_service)
            return
        except rospy.ROSException:
            if self.infer_mode == 'shared':
                rospy.logerr('[carton_obb] ~infer_mode=shared 但 %s 在 %.1fs 内没有'
                             '出现 —— 退回本地 ONNX', self.infer_service,
                             self.infer_probe_s)
            else:
                rospy.loginfo('[carton_obb] 没发现公共推理服务 %s（等了 %.1fs）'
                              '—— 本地加载 ONNX', self.infer_service,
                              self.infer_probe_s)
        self._load_local()

    def _load_local(self) -> None:
        """加载本地 ONNX 会话。失败直接抛 —— 两条路都不可用时节点不该起来。"""
        if self._sess is not None:
            return
        try:
            import onnxruntime as ort
            providers = [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider')
                         if p in ort.get_available_providers()]
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 1          # 与 OMP_NUM_THREADS=1 同口径
            self._sess = ort.InferenceSession(self.model_path, sess_options=opts,
                                              providers=providers)
        except Exception as exc:                   # noqa: BLE001
            rospy.logerr('[carton_obb] 加载 %s 失败：%s', self.model_path, exc)
            raise
        self._in_name = self._sess.get_inputs()[0].name
        self._lock = threading.Lock()
        self._mode = 'local'
        rospy.loginfo('[carton_obb] onnxruntime %s，provider=%s，模型 %s',
                      ort.__version__, self._sess.get_providers()[0], self.model_path)

    def _infer_shared(self):
        """调公共服务 → `(dets, ms, stamp, (h, w))`。失败抛异常，由调用方处置。

        ⚠️ `nms_iou=1.0` 是**故意的**：让服务端不要 NMS。服务端只有 IoU 一条
        判据，而本地 `nms_rotated` 还有 IoMin（管小框套大框），且栅格化口径不同。
        这里要的是**与本地推理逐位一样**，所以去重只在本地做一次。
        """
        t0 = time.perf_counter()
        resp = self._service.call(confidence=self.conf, nms_iou=1.0)
        ms = (time.perf_counter() - t0) * 1000.0
        if not resp.success:
            raise RuntimeError(resp.message or 'service returned success=false')
        h, w = int(resp.image_height), int(resp.image_width)
        if h <= 0 or w <= 0:
            raise RuntimeError(f'service returned bad image size {w}x{h}')
        dets = dets_from_rotated_rects(
            resp.center_x, resp.center_y, resp.width, resp.height,
            resp.angle_deg, resp.scores,
            orig_shape=(h, w), conf_thr=self.conf)
        return dets, ms, resp.source_header.stamp, (h, w)

    # ------------------------------------------------------------ 深度配对
    def _depth_for(self, stamp):
        """取与**实际推理帧**同 stamp 的深度图；没有同一帧就退回最新一帧。

        ⚠️ 用公共服务时，推理帧是**服务端缓存的最新帧**，不是本节点队列里那一帧
        —— 两路都可能差几十毫秒。深度只用于**选箱打分**（不参与四角），所以这里
        只在同一 stamp 存在时精确配对，否则退回最新一帧并留一个诊断计数。
        本地推理时两路本来就来自同一个同步组，stamp 恒等，行为与从前一致。
        """
        with self._depth_lock:
            exact = self._depth_by_stamp.get(float(stamp.to_sec())
                                             if hasattr(stamp, 'to_sec') else stamp)
            latest = self._depth
        if exact is not None:
            self._depth_exact_hits += 1
            return exact
        if self._depth is not None:
            self._depth_fallbacks += 1
        return latest

    def _shared_failed(self) -> None:
        """共享服务失败：连续 `~shared_fail_limit` 次就切回本地 ONNX。

        ⚠️ 切换**单向**（服务挂了不会自动切回去）：中途起服务的场景由重启节点解决，
        自动切回会在两路之间来回跳，而两条路的数值口径虽然对齐，切换瞬间的时间窗
        里混着两帧来源不同的四角 —— 那正是要避免的。
        """
        if self._mode != 'shared':
            return
        self._shared_fail += 1
        if self._shared_fail < self.shared_fail_limit:
            return
        now = time.time()
        if now - self._last_shared_warn > 30.0:
            self._last_shared_warn = now
            rospy.logerr('[carton_obb] 公共推理服务 %s 连续失败 %d 次 —— 切回本地 ONNX'
                         '（本节点不再恢复共享模式，要恢复请重启）',
                         self.infer_service, self._shared_fail)
        self._service = None
        self._mode = 'local'                       # 不再尝试服务（返回 early 的判据）
        try:
            self._load_local()
        except Exception as exc:                   # noqa: BLE001
            # 工作线程里抛出去会**静默杀死线程**（心跳随之停摆），所以这里必须接住。
            # `_load_local` 每帧会再试一次（模型文件可能刚补齐），不会卡死。
            rospy.logerr('[carton_obb] 共享服务失败后本地 ONNX 也加载不了（%s）——'
                         '本节点暂时没有可用后端，请检查 %s',
                         exc, self.model_path)
        self._shared_fail = 0

    # ------------------------------------------------------------------ 输入
    def _on_camera_info(self, msg):
        """内参：OBB 推理**不需要**，但换相机时希望启动日志里能对上。**只记一次。**

        ⚠️ 守卫不能省。没有 `if self.k is not None: return` 的话，这行会**以相机
        频率反复打印**（内参不变，等于纯噪声），把日志里真正要看的拒绝原因、
        耗时、心跳全淹掉。托盘节点同一处有这个守卫。
        """
        if self.k is not None:                     # 已经记过，不再打印
            return
        K = list(msg.K)
        self.k = K
        rospy.loginfo('[carton_obb] 内参 fx=%.2f fy=%.2f cx=%.2f cy=%.2f '
                      '（OBB 推理不用它，仅留痕）', K[0], K[4], K[2], K[5])
        try:
            self._info_sub.unregister()            # 内参不变，收一帧就够
        except Exception:                          # noqa: BLE001
            pass

    def _on_depth(self, msg):
        try:
            d = _arr_to_depth_mm(msg, self.depth_scale)
        except Exception as exc:                   # noqa: BLE001
            rospy.logwarn_throttle(5.0, '[carton_obb] 深度解不了：%s', exc)
            return
        stamp = msg.header.stamp.to_sec()
        with self._depth_lock:
            self._depth = d
            if stamp > 0.0:
                self._depth_by_stamp[stamp] = d
                while len(self._depth_by_stamp) > self._depth_hist:
                    self._depth_by_stamp.popitem(last=False)

    def _on_detect_mode(self, msg):
        """伺服节点广播的检测门控（2026-10-18）。

        `0` = 停；`>=1` = 有档位激活、要出数。⚠️ **纸箱在两种档位下都要**，
        所以判据是 `>=1` 而不是 `==2`（托盘检测器才认 `==2`）。
        """
        try:
            mode = int(msg.data)
        except (TypeError, ValueError):
            return
        active = mode >= 1
        if active != self._gated_active:
            rospy.loginfo('[carton_obb] 检测门控：%s（mode=%d）',
                          '开 —— 开始出数' if active else '关 —— 待命，不出数',
                          mode)
        self._gated_active = active

    def _on_synced(self, color_msg, depth_msg):
        try:
            item = (color_msg, depth_msg)
            self.q.put_nowait(item)
        except queue.Full:
            self._n_dropped += 1
            try:
                self.q.get_nowait()
                self.q.put_nowait(item)
            except queue.Empty:
                pass

    # ------------------------------------------------------------------ 工作线程
    def _run(self):
        while not rospy.is_shutdown():
            try:
                color_msg, _depth_msg = self.q.get(timeout=0.2)
            except queue.Empty:
                # ⚠️ 「相机停了 / 上游一个包都没来」走的就是这条路。**这里也要打
                # 心跳** —— 否则"什么都没收到"在日志上是一片空白，与"工作线程死了"
                # 长得一模一样，而这两件事的处理方式完全不同。
                self._heartbeat()
                continue
            now = time.monotonic()
            # ---- 门控 / 预热（2026-10-18）----------------------------------
            # 空闲时**不算**（省 GPU、不刷拒绝日志）；但启动后的第一帧要跑一次
            # 推理做预热 —— 首次推理实测 27.8s（CUDA 内核/显存初始化），
            # 这笔钱必须在启动时付掉，不能等到操作员 call 了服务才付。
            warm = False
            if not self._gated_active:
                if self._warmed:
                    self._heartbeat()
                    continue
                warm = True
            if self.max_rate > 0.0 and (now - self._last_pub) < 1.0 / self.max_rate:
                self._heartbeat()
                continue
            t0 = time.perf_counter()
            self._n_processed += 1
            src_header = color_msg.header
            try:
                if self._mode == 'shared':
                    # 服务端用的是**它自己**缓存的最新帧，不是我们这一帧 ——
                    # 所以发布时必须用响应里的 `source_header`（与真实推理帧一致），
                    # 深度也按它的 stamp 去历史里找。
                    dets, infer_ms, src_stamp, (h, w) = self._infer_shared()
                    # 服务端帧率可能低于本节点的处理节奏：同一帧不重复进时间窗
                    # （重复帧会把平均值往同一帧上加权，等于缩小窗口）。
                    stamp_s = float(src_stamp.to_sec())
                    if stamp_s > 0.0 and stamp_s == self._last_src_stamp:
                        self._n_dup_frames += 1
                        self._heartbeat()
                        continue
                    self._last_src_stamp = stamp_s
                    src_header = Header(stamp=src_stamp, frame_id=color_msg.header.frame_id)
                    self._shared_fail = 0
                else:
                    self._load_local()      # 上一步加载失败过的话在这里再试一次
                    img = _arr_to_bgr(color_msg)
                    x, r, pad = to_input_tensor(img, self.imgsz)
                    with self._lock:
                        raw = self._sess.run(None, {self._in_name: x})[0]
                    h, w = img.shape[:2]
                    dets = decode_obb(raw[0].T, r, pad, conf_thr=self.conf,
                                      orig_shape=(h, w))
            except Exception as exc:               # noqa: BLE001
                rospy.logwarn_throttle(5.0, '[carton_obb] 推理失败（%s）：%s',
                                       self._mode, exc)
                self._last_reject = f'infer_failed:{exc}'
                self._shared_failed()
                self._heartbeat()
                continue
            if warm:
                self._warmed = True
                self._last_good_ms = (time.perf_counter() - t0) * 1000.0
                rospy.loginfo('[carton_obb] 预热完成（首次推理 %.0fms，后端=%s）—— '
                              '待命，call /pallet_servo/slot 之后才开始出数',
                              self._last_good_ms, self._mode)
                self._heartbeat()
                continue

            dets = nms_rotated(dets, self.nms_iou, (h, w), iomin_thr=self.nms_iomin)

            # ---- 选箱：离画面下方中心最近的那个 --------------------------
            ref_uv = (w / 2.0, float(h))
            depth = self._depth_for(src_header.stamp)
            zs = [sample_depth_mm(depth, d['box_uv'], half=self.depth_patch,
                                  z_near=self.z_near, z_far=self.z_far) for d in dets]
            cand = score_candidates(dets, ref_uv, zs, w_dcenter=self.w_dcenter,
                                    w_area=self.w_area, w_depth=self.w_depth,
                                    d_near=self.d_near, d_far=self.d_far,
                                    z_near=self.z_near, z_far=self.z_far)
            best, best_score, best_parts = (max(cand, key=lambda t: t[1]) if cand
                                            else (None, None, None))

            if best is None:
                self._n_empty += 1
                # ⚠️ **候选数必须进日志**：只写"没检到箱子"排查不了 ——
                # `n_after_nms=0` 是网络一个框都没出（模型/输入的问题），
                # `>0` 则是出了框但被选箱/深度过滤掉了（参数或深度的问题），
                # 两条完全不同的排查路线，而这个数本来就在手上。
                self._last_reject = f'no_detection,n_after_nms={len(dets)}'
                rospy.loginfo_throttle(
                    5.0, '[carton_obb] 本帧没检到箱子（累计 %d 帧，NMS 后候选 %d 个）',
                    self._n_empty, len(dets))
                if self.publish_empty:
                    self._publish_empty(src_header)
                self._heartbeat()
                continue

            # ---- 深度下沿修正（默认开，可关）---------------------------
            # 现场问题：手持箱相对相机静止，但模型下边会跳到下方背景上。
            # 用框内箱体与下方背景的深度跳变把下边收回去；找不到跳变就不动。
            # 必须在时间窗 push 之前改，窗口才会用修正后的四角。
            if self.refine_bottom:
                nc, rinfo = refine_bottom_by_depth(
                    depth, best['corners_uv'], step=self.refine_step,
                    band_px=self.refine_band, jump_mm=self.refine_jump,
                    min_shrink_px=self.refine_min_shrink,
                    z_near=self.z_near, z_far=self.z_far)
                if nc is not None:
                    best['corners_uv'] = nc
                    best['box_uv'] = [float(nc[:, 0].min()), float(nc[:, 1].min()),
                                      float(nc[:, 0].max()), float(nc[:, 1].max())]
                    rospy.logdebug_throttle(
                        5.0, '[carton_obb] 深度下沿修正 %.1f -> %.1fpx'
                             '（y_new=%.1f，%s）',
                        rinfo['h_before'], rinfo['h_after'],
                        rinfo['y_new'], rinfo['reason'])

            out = self.win.push(best['corners_uv'], best['confidence'])
            if out is None:
                self._heartbeat()
                continue                           # 不是处理槽
            ms = (time.perf_counter() - t0) * 1000.0
            self._last_good_ms = ms
            self._last_reject = None
            # ⚠️ `dets` 是 **NMS 之后、选箱之前**的完整候选列表，`best` 就是其中
            #    一条（`max(cand, ...)` 返回的对象**就是** `dets` 里那个，`is` 同一）。
            #    一起发出去，下游的邻居伺服要靠它找上/侧邻居的框（见
            #    `box_detection_msgs/BoxDetection.msg` 的 `candidates_uv` 段）。
            self._publish(out, best, dets, ms, src_header)
            if ms > self.queue_warn_ms and time.time() - self._last_warn > 5.0:
                self._last_warn = time.time()
                rospy.logwarn('[carton_obb] 本帧 %.0fms，超过 %.0fms（丢帧 %d）',
                              ms, self.queue_warn_ms, self._n_dropped)
            rospy.loginfo_throttle(
                2.0, '[carton_obb] %.1fms score=%.3f（距下中 %.2f（%.0fpx）面积归一 %.2f '
                     '深度 %.2f）本帧 %d 个候选，发布 %d 次',
                ms, best_score, best_parts['d_center'], best_parts['d_px'],
                best_parts['area_norm'],
                -1.0 if best_parts['depth'] is None else best_parts['depth'],
                len(dets), self._n_pub)
            self._heartbeat()

    def _heartbeat(self):
        """**周期心跳** —— 每 `~heartbeat_s` 秒一行，**无论成功、没检到、还是没数据**。

        ⚠️ **为什么必须有**：本节点原来的日志点全是一次性的（启动那几行）或只在
        异常/节流时才打 —— 于是「**一切正常**」与「**工作线程死了 / 相机停了**」
        在日志上**长得一模一样**：都是什么都不打。托盘那条链 2026-09-29 就是这么
        被卡过一次（首帧之后一片安静，分不清在跑还是挂了），它补了心跳，**这条没有**。

            proc=123 pub=120 empty=3 drop=456  最近一帧 21ms  最近一次发布 0.9s 前
              最近一次拒绝：no_detection,n_after_nms=0

        `pub` 在涨 = 在发布；`proc` 涨而 `pub` 不涨 = 在拒（第二行给原因）；
        **`proc` 都不涨 = 线程死了或相机停了**；`drop` 一直涨是正常的
        （30Hz 相机配 10Hz 输出），只要 `pub` 也在涨就不用管。

        `~heartbeat_s <= 0` = 关掉。
        """
        if self.heartbeat_s <= 0.0:
            return
        now = time.time()
        if now - self._last_heartbeat < self.heartbeat_s:
            return
        self._last_heartbeat = now
        # `_last_pub` 是 monotonic 的，别和 time.time() 混用（两个时基不可比）
        stale = (f'{time.monotonic() - self._last_pub:.1f}s' if self._last_pub
                 else '从未')
        last_ms = self._last_good_ms if self._last_good_ms is not None else 0.0
        # 门控状态必须进心跳：空闲时 `proc` 不涨、`pub` 不涨，与"线程死了"
        # 在日志上长得一模一样 —— 这一格把它们分开（2026-10-18）。
        gate = ''
        if self.gated:
            gate = (' 门控=' + ('开' if self._gated_active
                                else ('关（已预热，待命）' if self._warmed
                                      else '关（预热中）')))
        # 后端（local / shared）也进心跳：两路的耗时口径不同（shared 含 ROS 往返），
        # 现场对比"为什么这帧慢了"第一件事就是确认走的是哪条路。
        rospy.loginfo(
            '[carton_obb] proc=%d pub=%d empty=%d drop=%d  最近一帧 %.0fms  '
            '最近一次发布 %s 前%s 后端=%s',
            self._n_processed, self._n_pub, self._n_empty, self._n_dropped,
            last_ms, stale, gate, self._mode)
        if self._last_reject:
            rospy.loginfo('[carton_obb]   最近一次拒绝：%s', self._last_reject)

    # ------------------------------------------------------------------ 发布
    def _publish(self, out: dict, best: dict, dets, ms: float, header) -> None:
        corners = np.asarray(out['corners_uv'], np.float64)
        us = corners[:, 0]
        vs = corners[:, 1]
        u0, v0, u1, v1 = float(us.min()), float(vs.min()), float(us.max()), float(vs.max())

        m = BoxDetection()
        # **沿用图像的时间戳与 frame_id** —— 下游靠它跟托盘位姿配对，不做时间换算
        m.header = Header(stamp=header.stamp, frame_id=header.frame_id)
        m.corners_uv = [Point32(float(u), float(v), 0.0) for u, v in corners]
        m.valid = True
        m.source = out['source']                    # 'single' / 'window'（OBB 没有兜底档）
        m.n_used = int(out['n_used'])
        m.n_slots = int(out['n_slots'])
        m.n_failed = int(out['n_slots'] - out['n_used'])
        m.spread_px = float(out['spread_px'])
        m.angle_deg = angle_deg_from_corners(corners)
        m.box_uv = [Point32(u0, v0, 0.0), Point32(u1, v1, 0.0)]   # [0]=左上 [1]=右下
        m.latency_ms = float(ms)
        # OBB 没有「整框转 90°」那个歧义，旧的朝向先验一并去掉 —— 恒为空串。
        m.orientation = ''
        m.rejects = ''
        # 候选框（老字段之后**追加**，老消费方不读）。**扁平存放**：每个候选
        # 4 个点一组，组内顺序与 `corners_uv` 相同（右下 → 左下 → 左上 → 右上）。
        # `dets` 里**含** `best` 那一个（`best` 就是 `dets` 中的同一对象），
        # 所以这里遍历 `dets` 即可，既不重复也不遗漏。
        m.candidates_uv = [
            Point32(float(u), float(v), 0.0)
            for d in dets for u, v in np.asarray(d['corners_uv'], np.float64)
        ]
        m.candidates_conf = [float(d['confidence']) for d in dets]
        self.pub.publish(m)
        self._last_pub = time.monotonic()
        self._n_pub += 1

    def _publish_empty(self, header) -> None:
        """发一帧 `valid=false`。`header` 是**实际参与推理那一帧**的 header。"""
        m = BoxDetection()
        m.header = Header(stamp=header.stamp, frame_id=header.frame_id)
        m.corners_uv = [Point32(0.0, 0.0, 0.0)] * 4
        m.valid = False
        m.source = ''
        m.spread_px = -1.0
        m.box_uv = [Point32(0.0, 0.0, 0.0)] * 2
        m.rejects = 'no_detection'
        # 空帧没有候选：**空数组**（不是 `None`）。下游按 `len(candidates_uv) / 4`
        # 判候选数，空数组与"生产者没填这个字段"在老消费方那边是同一件事。
        m.candidates_uv = []
        m.candidates_conf = []
        self.pub.publish(m)


def main():
    rospy.init_node('carton_obb_detect')
    _init_unified_logging('_carton')
    CartonObbNode()
    rospy.spin()


if __name__ == '__main__':
    main()
