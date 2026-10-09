#!/usr/bin/env python3
"""ROS1 节点：`/box/detection` —— 箱子顶面四角的实时检测结果。

**分层**（架构硬约束，见设计文档 §3）：

    skills/.../box_frame/algorithm.py   纯函数、零状态、**不 import ROS**  ← 算法核心
    skills/.../box_frame/window.py      有状态的滑动时间窗，**不 import ROS** ← 时序策略
    box_detection_node.py   本文件：只做搬运（订阅 / 出队 / 填消息 / 发布）

前两层都能脱离 ROS 单独跑和测；本文件里**没有业务逻辑**，改算法不用动它。

订阅（三个都要时间戳接近，用 `ApproximateTimeSynchronizer` 对齐）：
  * `~image_color`  sensor_msgs/Image          彩色图（默认 `/camera/color/image_raw`）
  * `~image_depth`  sensor_msgs/Image          对齐后的深度图，**16UC1，单位 mm**
  * `~box_in`       geometry_msgs/PolygonStamped  YOLO 框：`points[0]`=左上、`points[1]`=右下
  * `~camera_info`  sensor_msgs/CameraInfo      **内参来源**（标准的 K 矩阵）
发布：
  * `/box/detection`  box_detection_msgs/BoxDetection

参数（`rosparam`，都有默认值）：
  | 名字 | 默认 | 说明 |
  |---|---|---|
  | `image_color` | `/camera/color/image_raw` | 彩色话题 |
  | `image_depth` | `/camera/depth/image_raw` | 深度话题 |
  | `box_in` | `/box/yolo_box` | YOLO 框话题 |
  | `camera_info` | `/camera/color/camera_info` | 内参话题（用它的 K 矩阵） |
  | `detection_out` | `/box/detection` | 输出话题 |
  | `target_mm` | `[530.0, 350.0]` | 箱子实际尺寸（长,短） |
  | `window` | `5` | 滑动窗长（处理槽数） |
  | `process_every` | `3` | 每几帧处理一次（相机 30Hz → 10Hz） |
  | `agg` | `mean` | 多帧聚合：`mean` / `median` |
  | `depth_scale_mm` | `1.0` | 深度图单位换算到 mm 的系数 |
  | `sync_slop_s` | `0.05` | 三路时间戳允许的最大差 |
  | `queue_warn_ms` | `200.0` | 处理慢于这个就告警一次（节流） |
  | `pose_service` | `/infer_carton_pose` | **朝向先验服务**（见下）；给空串 = 不接 |
  | `pose_period_s` | `0.5` | 多久问一次朝向（朝向是慢变量，不必每帧问） |

**朝向先验**（可选，2026-09-21 追加）：上游 `dynamic_biped/InferCartonPose`
的 `tape_orientation_deg`（0=horizontal / 90=vertical / -1=unknown）告诉我们
箱子长边在图像里是横还是竖，用来掐掉「整框转 90°」那种错。**服务不存在、
调用失败、给 -1，都只是退回自动选，不影响出结果**（编译不进来这个包也一样能跑，
只是启动时告警一次）。用了会在输出消息的 `orientation` 字段里留痕。

跑法（`OMP_NUM_THREADS=1` 是必须的，实测差 1.6x，见 WORKLOG §15.1）：
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
    python box_detection_node.py

⚠️ 需要先编译消息包（catkin 工作区里 `catkin_make`），并把生成的 Python 模块
放进 `PYTHONPATH`。消息定义在本仓库的
`infrastructure/ros_packages/src/ros_vision/box_detection_msgs/`。
没编译的话本文件会给出明确报错。
"""
from __future__ import annotations

import queue
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import rospy
    from geometry_msgs.msg import Point32, PolygonStamped
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Header
    from message_filters import ApproximateTimeSynchronizer, Subscriber
except ImportError as exc:                       # pragma: no cover
    raise SystemExit(
        f'缺少 ROS1 依赖（{exc}）。本节点要在 ROS Noetic 环境里跑：\n'
        '    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH\n'
        '（算法本身不依赖 ROS，`test_fit_box_frame.py` 不需要 ROS 就能跑）')

try:
    from box_detection_msgs.msg import BoxDetection
except ImportError as exc:                       # pragma: no cover
    raise SystemExit(
        f'没找到消息包 box_detection_msgs（{exc}）。把本仓库的 '
        'infrastructure/ros_packages/src/ros_vision/box_detection_msgs 放进 catkin '
        '工作区的 src/ 下 catkin_make，再 source devel/setup.bash：\n'
        '    cp -r <LeTools>/infrastructure/ros_packages/src/ros_vision/'
        'box_detection_msgs <工作区>/src/ && cd <工作区> && catkin_make')

# 算法层在 LeTools 的 skills 里（纯函数 + 时间窗，都不 import ROS）。
# 本脚本从基础设施层反查仓库根 —— 与 `apps/` 下那些脚本同一手法。
_REPO_ROOT = Path(__file__).resolve().parents[7]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from skills.atomic.perception.box_frame.window import (   # noqa: E402
    BoxFrameWindow, build_payload, hint_from_instances)


def _image_to_array(msg, dtype, channels: int = 1):
    """sensor_msgs/Image -> np.ndarray (H, W*channels)，**按 msg.step 处理行填充**。

    ⚠️ 列要按 `width * channels` 切，**不是 `width`** —— `width` 是像素数，
    而一行字节数是 `width * channels`。一个像素三字节的 bgr8 图，按 `width`
    切会只剩三分之一的缓冲（实测踩过：reshape 报
    `cannot reshape array of size 307200 into shape (480,640,3)`）。

    不依赖 `cv_bridge`（它会拖进 boost_python 等一堆东西），这里只需要把
    已对齐好的缓冲 reshape 一下。
    """
    import numpy as np
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
    """-> np.ndarray (H,W,3) BGR，喂给 `fit_box_frame`。"""
    import cv2
    import numpy as np
    if msg.encoding in ('bgr8', '8UC3'):
        return _image_to_array(msg, np.uint8, 3).reshape(msg.height, msg.width, 3).copy()
    if msg.encoding == 'rgb8':
        a = _image_to_array(msg, np.uint8, 3).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    if msg.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(_image_to_array(msg, np.uint8), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'不支持的彩色编码 {msg.encoding!r}（要 bgr8 / rgb8 / mono8）')


def _arr_to_depth_mm(msg, scale: float):
    """-> np.ndarray (H,W) uint16，**单位 mm**（`fit_box_frame` 要求 mm）。

    ⚠️ 常见坑：**Orbbec 头部相机的深度话题是 `/camera/depth/image_raw`**（默认值），
    RealSense 那套命名是 `aligned_depth_to_color/image_raw` —— 换相机时改 `~image_depth`。
    格式上，Orbbec 发 **16UC1 且已经是毫米**；有些驱动却发 `32FC1` 的**米**。
    两种都支持，用 `depth_scale_mm` 兜底。
    """
    import numpy as np
    if msg.encoding in ('16UC1', 'mono16'):
        a = _image_to_array(msg, np.uint16)
        return a if scale == 1.0 else np.rint(a * scale).astype(np.uint16)
    if msg.encoding == '32FC1':
        a = _image_to_array(msg, np.float32)
        return np.rint(a * 1000.0 * scale).astype(np.uint16)
    raise ValueError(f'不支持的深度编码 {msg.encoding!r}（要 16UC1 或 32FC1）')


def _box_uv_from_msg(msg) -> tuple:
    """geometry_msgs/PolygonStamped -> (u0,v0,u1,v1)，取两点的外接框。

    约定 `points[0]` = 左上、`points[1]` = 右下。**换成别的框消息格式时，
    只改这一个函数。** 这里用外接框而不是直接取两点，是为了容忍两个点给反。
    """
    pts = list(msg.polygon.points)
    if len(pts) < 2:
        raise ValueError(f'框消息至少要有 2 个点，收到 {len(pts)}')
    us = [float(p.x) for p in pts[:2]]
    vs = [float(p.y) for p in pts[:2]]
    return (min(us), min(vs), max(us), max(vs))


class BoxDetectionNode:
    def __init__(self):
        self.image_color = rospy.get_param('~image_color', '/camera/color/image_raw')
        self.image_depth = rospy.get_param('~image_depth',
                                           '/camera/depth/image_raw')
        self.box_in = rospy.get_param('~box_in', '/box/yolo_box')
        self.info_topic = rospy.get_param('~camera_info',
                                          '/camera/color/camera_info')
        self.out_topic = rospy.get_param('~detection_out', '/box/detection')
        self.depth_scale = float(rospy.get_param('~depth_scale_mm', 1.0))
        self.sync_slop = float(rospy.get_param('~sync_slop_s', 0.05))
        self.queue_warn_ms = float(rospy.get_param('~queue_warn_ms', 200.0))

        # 内参从标准的 `camera_info` 话题拿（K = [fx 0 cx; 0 fy cy; 0 0 1]）。
        # **不要从文件读** —— 那是离线跑测试数据集的做法，真机上应该以相机驱动
        # 发出来的为准，换了相机/分辨率也不用改代码。
        self.win = None                     # 收到第一帧 camera_info 之后才建
        self.k = None
        self._win_cfg = dict(
            target_mm=tuple(rospy.get_param('~target_mm', [530.0, 350.0])),
            window=int(rospy.get_param('~window', 5)),
            process_every=int(rospy.get_param('~process_every', 3)),
            agg=str(rospy.get_param('~agg', 'mean')))

        # 朝向先验服务（可选）。`pose_service` 给空串 = 完全不接这条线。
        self.pose_srv = None
        self.pose_period_s = float(rospy.get_param('~pose_period_s', 0.5))
        self._last_pose_call = 0.0
        self._last_pose_warn = 0.0
        pose_name = str(rospy.get_param('~pose_service', '/infer_carton_pose'))
        if pose_name:
            try:
                from dynamic_biped.srv import InferCartonPose, InferCartonPoseRequest
            except ImportError as exc:
                rospy.logwarn(f'[box_detection] 没有 dynamic_biped/InferCartonPose'
                              f'（{exc}）—— 朝向先验关闭，按边支持率自动选。'
                              f'要启用就把该包放进工作区编译。')
            else:
                self._pose_req = InferCartonPoseRequest
                # ⚠️ 服务**现在**可能还没起来，`ServiceProxy` 只是建个句柄，
                # 调用时才连。所以这里不会阻塞启动。
                self.pose_srv = rospy.ServiceProxy(pose_name, InferCartonPose)
                rospy.loginfo(f'[box_detection] 朝向先验接 {pose_name}'
                              f'（每 {self.pose_period_s:g}s 问一次，失败不影响出结果）')

        self.pub = rospy.Publisher(self.out_topic, BoxDetection, queue_size=5)
        # **队列长度 1**：算法慢（~60ms/帧）时宁可丢旧帧也不要排队 ——
        # 排队的后果是输出的是几百毫秒前那一帧的结果，伺服拿去会晚。
        self.q: queue.Queue = queue.Queue(maxsize=1)
        self._n_dropped = 0
        self._last_warn = 0.0

        subs = [Subscriber(self.image_color, Image),
                Subscriber(self.image_depth, Image),
                Subscriber(self.box_in, PolygonStamped)]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=5,
                                                slop=self.sync_slop)
        self.sync.registerCallback(self._on_synced)

        self._info_sub = rospy.Subscriber(self.info_topic, CameraInfo,
                                          self._on_camera_info, queue_size=1)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        rospy.loginfo(f'[box_detection] 订阅 {self.info_topic} + {self.image_color} '
                      f'+ {self.image_depth} + {self.box_in}  ->  发布 {self.out_topic}')
        rospy.loginfo('[box_detection] 等第一帧 camera_info 建时间窗…')

    # ---- 内参：第一帧到了才建时间窗 -------------------------------------
    def _on_camera_info(self, msg):
        if self.k is not None:
            return
        from skills.atomic.perception.box_frame import CameraIntrinsics
        K = list(msg.K)
        self.k = CameraIntrinsics(float(K[0]), float(K[4]), float(K[2]), float(K[5]))
        self.win = BoxFrameWindow(self.k, **self._win_cfg)
        self._info_sub.unregister()          # 内参不变，收一帧就够
        rospy.loginfo(f'[box_detection] 内参 fx={self.k.fx:.2f} fy={self.k.fy:.2f} '
                      f'cx={self.k.cx:.2f} cy={self.k.cy:.2f}，时间窗已就绪')

    # ---- 朝向先验：问 /infer_carton_pose（可选，缺服务也能跑） --------------
    #
    # 为什么要有这一步：`fit_box_frame` 会枚举「530mm 配哪条轴」两种假设，
    # 箱子被挡得只剩一小条时两种都能装下，就靠边支持率选 —— 而手臂的深度边缘
    # 会给错误候选凭空送分。选错就是**整框转 90°**，而输出四角照样是个规矩的
    # 矩形，不比对真值看不出来。上游服务的 `tape_orientation_deg`
    # （0=horizontal / 90=vertical / -1=unknown）正好能把这个歧义说死。
    #
    # **缺服务 / 服务报错 / 给 -1 都不影响出结果** —— 只是退回自动选。这条链路
    # 不能因为一个可选先验就断掉。三个保命措施：起不来直接关掉不再试、
    # 调用失败只告警不抛、按 `pose_period_s` 限频（朝向是慢变量，没必要每帧问）。
    def _on_carton_pose(self, box_uv):
        """问一次服务，把朝向写进时间窗。**任何异常都吞掉**（可选增强）。"""
        if self.pose_srv is None or box_uv is None:
            return
        now = time.time()
        if now - self._last_pose_call < self.pose_period_s:
            return
        self._last_pose_call = now
        try:
            res = self.pose_srv(self._pose_req())
        except Exception as exc:
            if now - self._last_pose_warn > 30.0:
                self._last_pose_warn = now
                rospy.logwarn(f'[box_detection] /infer_carton_pose 调用失败（{exc}），'
                              f'朝向先验跳过，按边支持率自动选')
            return
        if not getattr(res, 'success', False):
            return
        hint = hint_from_instances(getattr(res, 'bbox_xyxy', None),
                                   getattr(res, 'tape_orientation_deg', None),
                                   box_uv)
        if hint != self.win.orientation:
            rospy.loginfo(f'[box_detection] 朝向先验 -> {hint}'
                          f'（tape={list(getattr(res, "tape_orientation_deg", []))}）')
        self.win.set_orientation(hint)

    # ---- 回调只入队，不做计算 -------------------------------------------
    def _on_synced(self, color_msg, depth_msg, box_msg):
        if self.win is None:                 # 还没拿到内参
            return
        try:
            item = (color_msg, depth_msg, box_msg)
            self.q.put_nowait(item)
        except queue.Full:
            self._n_dropped += 1
            try:                       # 丢最旧的、放最新的：保证输出尽量新鲜
                self.q.get_nowait()
                self.q.put_nowait(item)
            except queue.Empty:
                pass

    # ---- 工作线程：算 + 发布 --------------------------------------------
    def _run(self):
        while not rospy.is_shutdown():
            try:
                color_msg, depth_msg, box_msg = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            t0 = time.perf_counter()
            try:
                color = _arr_to_bgr(color_msg)
                depth = _arr_to_depth_mm(depth_msg, self.depth_scale)
                box_uv = _box_uv_from_msg(box_msg)
            except Exception as exc:
                rospy.logwarn_throttle(5.0, f'[box_detection] 输入转换失败：{exc}')
                continue
            self._on_carton_pose(box_uv)      # 可选：拿朝向先验（限频、失败无害）
            out = self.win.push(color, depth, box_uv)
            if out is None:
                continue                      # 不是处理槽（process_every 跳过的帧）
            ms = (time.perf_counter() - t0) * 1000.0
            self.pub.publish(self._to_msg(out, box_uv, ms, color_msg.header))
            if ms > self.queue_warn_ms and time.time() - self._last_warn > 5.0:
                self._last_warn = time.time()
                rospy.logwarn(f'[box_detection] 本帧 {ms:.0f}ms，超过 '
                              f'{self.queue_warn_ms:.0f}ms（丢帧 {self._n_dropped}）')

    @staticmethod
    def _to_msg(out: dict, box_uv, ms: float, header) -> BoxDetection:
        """payload -> 消息。**这里只搬运，不含任何判断。**"""
        p = build_payload(out, box_uv, ms)
        m = BoxDetection()
        m.header = Header(stamp=header.stamp, frame_id=header.frame_id)
        m.corners_uv = [Point32(u, v, 0.0) for u, v in p['corners_uv']]
        m.valid = bool(p['valid'])
        m.source = p['source']
        m.n_used = p['n_used']
        m.n_slots = p['n_slots']
        m.n_failed = p['n_failed']
        m.spread_px = p['spread_px']
        m.angle_deg = p['angle_deg']
        m.box_uv = [Point32(u, v, 0.0) for u, v in p['box_uv']]
        m.latency_ms = p['latency_ms']
        m.orientation = p['orientation']
        m.rejects = p['rejects']
        return m


def main():
    rospy.init_node('box_detection')
    BoxDetectionNode()
    rospy.spin()


if __name__ == '__main__':
    main()
