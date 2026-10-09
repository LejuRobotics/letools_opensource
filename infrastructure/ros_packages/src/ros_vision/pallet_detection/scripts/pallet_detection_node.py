#!/usr/bin/env python3
"""ROS1 节点：`/pallet/detection` —— 木托盘台面坐标系（无初值）的实时检测结果。

**分层**（与 `box_detection_node.py` 同一结构，架构硬约束）：

    skills/atomic/perception/pallet_detect/   纯函数、零状态、**不 import ROS**  ← 算法核心
        algorithm.py   detect_pallet_frame()  无初值检测
        refine.py      refine_pallet_frame()  细化
        payload.py     build_payload() / format_*()        纯函数搬运层（不 import ROS）
        tf_normal.py   normal_from_tf()                    要查 TF，在 infrastructure 层
    pallet_detection_node.py                  本文件：节点类只做搬运
                                              （订阅 / 出队 / 填消息 / 发布）

算法层在 LeTools 的 `skills/` 里、**不 import ROS**，所以它能脱离 ROS 单独跑和测；
本文件里**没有业务逻辑**，改算法不用动它。

订阅（两路时间戳要接近，用 `ApproximateTimeSynchronizer` 对齐）：
  * `~image_color`  sensor_msgs/Image          彩色图（默认 `/camera/color/image_raw`）
  * `~image_depth`  sensor_msgs/Image          对齐后的深度图，**16UC1，单位 mm**
  * `~camera_info`  sensor_msgs/CameraInfo      **内参来源**（标准的 K 矩阵）
发布：
  * `/pallet/detection`  pallet_detection_msgs/PalletDetection

参数（`rosparam`，都有默认值）：
  | 名字 | 默认 | 说明 |
  |---|---|---|
  | `image_color` | `/camera/color/image_raw` | 彩色话题 |
  | `image_depth` | `/camera/depth/image_raw` | 深度话题 |
  | `camera_info` | `/camera/color/camera_info` | 内参话题（用它的 K 矩阵） |
  | `detection_out` | `/pallet/detection` | 输出话题 |
  | `target_mm` | `[1200.0, 1000.0]` | 托盘实际尺寸（长, 短） |
  | `long_side_parallel` | `false` | 与画面近平行的边是不是长边 |
  | `normal` | `''` | `"x,y,z"`；**给空串 = 从 TF 查**（部署默认） |
  | `camera_frame` | `camera_color_optical_frame` | TF 里的相机系 |
  | `base_frame` | `base_link` | TF 里的基座系 |
  | `sync_slop_s` | `0.05` | 两路时间戳允许的最大差 |
  | `depth_scale_mm` | `1.0` | 深度单位换算到 mm 的系数 |
  | `process_every` | `1` | 每几帧处理一次 |
  | `queue_warn_ms` | `1000.0` | 处理慢于这个就告警一次（节流） |
  | `refine` | `true` | 检出后是否跑 `refine_pallet_frame`（见下）|
  | `plane` | `true` | refine 的平面细化开关；**部署必须开**（见下）|
  | `use_prior` | `true` | 用上一帧的结果当先验跟踪（**`search` 段 599ms -> 66~80ms**）|
  | `deck_z_mm` | `''` | `"lo,hi"`：台面高度限定在 base_link 的**绝对 z 带**内；空 = 不约束 |
  | `fit_normal` | `true` | **TF 只当搜索初值，法向从当前帧深度拟合**（见下）|
  | `normal_refit` | `false` | `false` = 只在首帧拟合一次、之后复用；动腰/换工位要设 true |
  | `heartbeat_s` | `5.0` | 心跳周期（秒）。**成功时也打**，见 §心跳 |

## ⚠️ 心跳：分不清"在跑"和"挂了"是这个节点的老毛病

改之前，本节点的日志点全是**一次性的**（`法向来源` / `台面 z 带` 只打首帧）、
**只在失败时**（`如实拒绝`）或**只在超时时**（`本帧 Xms 超过`）。于是
「一切正常」与「工作线程死了 / 一直在拒但被节流吞了」在日志上**长得一模一样**：
都是什么都不打。2026-09-29 现场就这么卡过一次 —— 首帧那条 3458ms 的 WARN 之后
一片安静，分不清是跑还是挂。

现在每 `~heartbeat_s` 秒**无条件**打一行（成功也打）：

```
[pallet_detection] proc=123 pub=120 rej=3 drop=456  本帧 880ms  最近一次发布 0.9s 前
[pallet_detection]   最近一次拒绝：reject=too_few_edges,n_observed_edges=0<2,score=0.4135,source=color
```

怎么读：

| 现象 | 含义 |
|---|---|
| `proc` 在涨、`pub` 也在涨 | 正常 |
| `proc` 涨、`pub` 不涨、`rej` 在涨 | 在拒 —— 第二行就是原因，不用翻别处 |
| `proc` 都不涨 | **工作线程死了，或者相机停了**（两者再靠 `rostopic hz /camera/color/image_raw` 分开）|
| `drop` 一直涨 | 正常（30Hz 相机配 ~1s/帧，不快才怪），只要 `pub` 也在涨就不用管 |

⚠️ **`header.stamp` 填的是彩色图那帧的 `header.stamp`**（`color_msg.header.stamp`），
**不是 `rospy.Time.now()`**。托盘检测实测 ~2s/帧（合成 480x640，见下方"耗时"），
比箱子（~60ms）慢一个量级；伺服要把两者的位姿**相减**，配对依据是"输入图像的
采集时刻"。填成处理完成时刻的话，耗时的差会被当成场景的时间差 —— 要么永远配不上，
要么配上一对其实是不同帧的图像，相减出来的误差里混着相机运动，**而三个数照样
算得出来**。契约原文见 `infrastructure/ros_packages/src/ros_vision/pallet_detection_msgs/msg/PalletDetection.msg` 末尾。

## ⚠️ 法向来源必须可见（本节点最重要的一条行为）

`detect_pallet_frame` 需要一个**台面法向**的外部先验，而 **detect 单独**对法向
敏感：**法向就是投影平面本身**，detect 只搜平面内 3 个自由度，法向偏 ε 时真值
台面在"高度"坐标里变成一条斜坡，跨度 ≈ `tan(ε) × 781mm`；一旦超过
`DECK_BAND_MM (40mm)`，台面就装不进那个高度带、掩码塌成月牙。**实测临界角 ≈ 3~4°**。

> ⚠️ **2026-09-23 更正**：本文件早先的版本把 detect-only 的数字（5.6° → 131mm）
> 当成了**部署路径**的结论，**那是错的**。detect 确实不细化法向，但 **refine 会**
> —— 它的 `_refine_plane` 是 SVD 拟合台面平面，法向 2 + 高度 1 自己解。

整条链 `detect -> refine` 的终点误差（合成场景，已有实测）：

| ε | detect 单独 | `plane=False` 终点 | **`plane=True` 终点** |
|---|---|---|---|
| 0° | 3.8 mm | 5.0 mm / 0.1° | **4.2 mm / 0.1°** |
| 2° | 3.5 mm | 14.3 mm / 2.1° | **4.8 mm / 0.1°** |
| **5.6°** | **180.4 mm** | 34.1 mm / **5.7°** | **13.2 mm / 0.1°** |
| 8° | 304.5 mm | 拒绝 `no_edges` | 拒绝 `no_edges` |

三条结论：

1. **TF 的 5.6° 偏差 < `PLANE_MAX_TILT_DEG = 8.0` 守卫，能被 refine 修回来**
   （终点 13.2mm、dθ 0.1°）。
2. **余量只剩 2.4°**，而且这是**合成场景、完美深度**下测的；真实深度有噪声，
   平面拟合的 tilt 估计本身会抖。**这是要报告的发现，不是"没问题"。**
3. ⚠️ **部署必须开 `plane=True`**（见下）。

本节点能做、也必须做的是：把**用的是哪个法向、从哪来的**如实写进 `diag`
（`normal_source=tf|param`）与日志，让这件事**可见**。

另：**没有法向先验时检测会静默地算下去** —— 5_test 上给一个完全错误的法向
（竖直向下）它不会崩、不会报错，只是换个 score 继续"如实拒绝"（`source`
从 color 变 depth）。所以本节点**绝不**在 TF 缺失时回退到单位阵/竖直向下，
而是**不发布**并给出明确报错。

## ⚠️ refine 必须显式开 `plane=True`

`refine_pallet_frame` 的 **CLI 默认是 `plane=False`**（`--plane` 是 opt-in，理由
写在那个函数的注释里：april_test7 静止场景上一帧的法向动了 3.28°，把跨帧一致性
顶坏了）。但 `plane=False` 时 **5.6° 的法向偏差会让朝向停在 5.7°** —— 伺服拿去
会歪。所以本节点调用 refine 时**显式传 `plane=True`**（`opts=dict(plane=True)`），
并由 `~refine`（默认 `true`）/ `~plane`（默认 `true`）两个参数控制。
**这两个默认值与 refine 的 CLI 默认值不同，是刻意的** —— 部署路径要的是修得回来。

跑法（`OMP_NUM_THREADS=1` 是必须的，实测差 1.6x，见 WORKLOG_3test §15.1）：
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
    # 正式跑：roslaunch（target_mm 由 launch 里的 <rosparam> 给成 list）
    roslaunch pallet_detection pallet_detection.launch
    # 单节点直跑（`_x:=` 命令行赋值 roslaunch 的 auto 转换认不出 list，
    # 给的是**字符串** —— 本节点两种都吃，见 `_parse_target_mm`）
    python3 pallet_detection_node.py _normal:="" _target_mm:="[1200,1000]"

⚠️ 需要先编译消息包 `infrastructure/ros_packages/src/ros_vision/pallet_detection_msgs`（catkin 工作区里 `catkin_make`），
并把生成的 Python 模块放进 `PYTHONPATH`。没编译的话本文件会给出明确报错。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

# 算法层在 LeTools 的 skills 里（纯函数，不 import ROS）。本脚本从基础设施层
# 反查仓库根 —— 与 `box_detection_node.py` 同一手法。
_REPO_ROOT = Path(__file__).resolve().parents[6]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from skills.atomic.perception.pallet_detect import (   # noqa: E402
    CameraIntrinsics,
    build_payload,
    detect_pallet_frame,
    fit_floor_normal,
    format_rejects,
    refine_pallet_frame,
)
# ⚠️ `DEFAULT_TARGET_MM` 与 `CHAIN` **必须显式 import**：它们定义在纯函数层
# `payload.py` 里，本文件只是使用者。漏了不会在 import 时报错 ——
# `PalletDetectionNode.__init__` 里才 NameError，而**单元测试从来不构造这个类**
# （只调静态方法），所以整条 ROS 路径零覆盖。测试里有一条 AST 静态检查钉住这点。
from skills.atomic.perception.pallet_detect.payload import (  # noqa: E402
    CHAIN,
    DEFAULT_TARGET_MM,
)
# ⚠️ `normal_from_tf` **不**从 skills 取：它要 import rospy，按分层约束只能待在
# 基础设施层，就在本脚本旁边（同目录 `tf_normal.py`）。本脚本开头已经把
# `Path(__file__).parent` 插进了 sys.path，所以直接 import 得到。
from tf_normal import camera_origin_from_tf, normal_from_tf  # noqa: E402

# --------------------------------------------------------------------------- #
# ROS 依赖（只在节点类里用；测试不 import 这一层）
# --------------------------------------------------------------------------- #
def _ros():
    """延迟 import ROS。缺依赖时给出可操作的报错。"""
    try:
        import rospy
        from geometry_msgs.msg import Point32
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Header
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        import tf2_ros
    except ImportError as exc:                       # pragma: no cover
        raise SystemExit(
            f'缺少 ROS1 依赖（{exc}）。本节点要在 ROS Noetic 环境里跑：\n'
            '    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH\n'
            '（算法本身不依赖 ROS，`test_pallet_detection_node.py` 不需要 ROS 就能跑）')
    try:
        from pallet_detection_msgs.msg import PalletDetection
    except ImportError as exc:                       # pragma: no cover
        raise SystemExit(
            f'没找到消息包 pallet_detection_msgs（{exc}）。先把 '
            'infrastructure/ros_packages/src/ros_vision/pallet_detection_msgs 放进 catkin 工作区的 src/ 下 catkin_make，'
            '再 source devel/setup.bash：\n'
            '    cp -r infrastructure/ros_packages/src/ros_vision/pallet_detection_msgs <工作区>/src/ && cd <工作区> && catkin_make')
    return dict(rospy=rospy, Point32=Point32, CameraInfo=CameraInfo, Image=Image,
                Header=Header, ApproximateTimeSynchronizer=ApproximateTimeSynchronizer,
                Subscriber=Subscriber, tf2_ros=tf2_ros, PalletDetection=PalletDetection)


def _parse_target_mm(raw) -> tuple:
    """`~target_mm` → `(长, 短)`。**list / tuple / 字符串都认。**

    ⚠️ 为什么不能只写 `tuple(float(v) for v in raw)`：**roslaunch 的
    `<param value="[1200.0, 1000.0]"/>` 给的是字符串，不是 list。**
    实测（`roslaunch.loader.convert_value`，`type` 缺省 = `'auto'`）：

        value="[1200.0, 1000.0]"              ->  '[1200.0, 1000.0]'  (str)
        value="[1200.0, 1000.0]" type="yaml"  ->  [1200.0, 1000.0]    (list)
        <rosparam param="...">[...]</rosparam>->  [1200.0, 1000.0]    (list)

    字符串那一路进到 `tuple(float(v) for v in raw)` 里逐**字符**迭代，第一个字符
    就是 `[` —— `ValueError: could not convert string to float: '['`，**节点起不来**。

    ⚠️ **命令行 `_target_mm:="[1200,1000]"` 走的是另一条路**：`rospy.init_node`
    用 `yaml.safe_load` 解析（`rospy.client.load_command_line_node_params`），
    所以它**给的是 list**（实测 `[1200, 1000]`）。但 `_target_mm:="1200,1000"`
    没有方括号时 yaml 认成字符串，`_target_mm:=""` 更是给 `None`。
    三种形态都得吃 —— 这也是为什么这个函数对 `list` / `tuple` / `str` / `None`
    一律显式处理，而不是只写一行 `tuple(float(v) for v in raw)`。

    ⚠️ 解析不了时**抛 ValueError 并说清该怎么写** —— 不要静默退回默认值：
    尺寸是搜索用的**已知先验**，静默用错尺寸会一路算出个像模像样的错位姿
    （检测器是按已知尺寸搜索的，`size_mm` 必然接近配置值，看不出来）。
    """
    if isinstance(raw, str):
        text = raw.strip()
        if not text:
            raise ValueError('~target_mm 是空串；写成 "[1200.0, 1000.0]" 或 '
                             '[1200.0, 1000.0]')
        # 容错：去掉方括号，逗号 / 空格 / 分号都能当分隔符
        text = text.strip('[]()')
        parts = [t for t in text.replace(';', ',').replace(' ', ',').split(',') if t]
    else:
        try:
            parts = list(raw)
        except TypeError as exc:
            raise ValueError(f'~target_mm 既不是字符串也不是序列：{raw!r}') from exc
    if len(parts) != 2:
        raise ValueError(f'~target_mm 要有两个数（长, 短），收到 {len(parts)} 个：{raw!r}')
    try:
        vals = tuple(float(v) for v in parts)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'~target_mm 解析不出两个浮点数：{raw!r}') from exc
    if not all(v > 0.0 for v in vals):
        raise ValueError(f'~target_mm 必须都是正数：{vals!r}')
    return vals


def _image_to_array(msg, dtype, channels: int = 1):
    """sensor_msgs/Image -> np.ndarray (H, W*channels)，**按 msg.step 处理行填充**。

    ⚠️ 列要按 `width * channels` 切，**不是 `width`** —— `width` 是像素数，
    而一行字节数是 `width * channels`。一个像素三字节的 bgr8 图，按 `width`
    切会只剩三分之一的缓冲（实测踩过：reshape 报
    `cannot reshape array of size 307200 into shape (480,640,3)`）。

    （与 `box_detection_node.py` 的同名函数逐字相同 —— 它跑过真机，直接抄。）
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
    """-> np.ndarray (H,W,3) BGR，喂给 `detect_pallet_frame`。"""
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
    """-> np.ndarray (H,W) float32，**单位 mm**（`detect_pallet_frame` 要求 mm）。

    ⚠️ 常见坑：**Orbbec 头部相机的深度话题是 `/camera/depth/image_raw`**（默认值），
    RealSense 那套命名是 `aligned_depth_to_color/image_raw` —— 换相机时改
    `~image_depth`。Orbbec 发 **16UC1 且已经是毫米**；有些驱动却发 `32FC1` 的**米**。
    两种都支持，用 `depth_scale_mm` 兜底。

    ⚠️ 与 `box_detection_node.py` 的同名函数只有一处不同：那里返回 `uint16`
    （`fit_box_frame` 要整数），这里返回 `float32` —— `detect_pallet_frame`
    的深度链（`ground_detector._ground_points`）走的是浮点反投影。
    """
    import numpy as np
    if msg.encoding in ('16UC1', 'mono16'):
        a = _image_to_array(msg, np.uint16).astype(np.float32)
        return a if scale == 1.0 else a * scale
    if msg.encoding == '32FC1':
        return _image_to_array(msg, np.float32) * 1000.0 * scale
    raise ValueError(f'不支持的深度编码 {msg.encoding!r}（要 16UC1 或 32FC1）')


class PalletDetectionNode:
    """订阅 彩色+深度+camera_info → 检测 → 发布 `PalletDetection`。"""

    def __init__(self, ros):
        self._ros = ros
        rospy = ros['rospy']
        self.image_color = rospy.get_param('~image_color', '/camera/color/image_raw')
        self.image_depth = rospy.get_param('~image_depth', '/camera/depth/image_raw')
        self.info_topic = rospy.get_param('~camera_info', '/camera/color/camera_info')
        self.out_topic = rospy.get_param('~detection_out', '/pallet/detection')
        self.depth_scale = float(rospy.get_param('~depth_scale_mm', 1.0))
        self.sync_slop = float(rospy.get_param('~sync_slop_s', 0.05))
        self.queue_warn_ms = float(rospy.get_param('~queue_warn_ms', 1000.0))
        self.process_every = max(1, int(rospy.get_param('~process_every', 1)))
        self.target_mm = _parse_target_mm(
            rospy.get_param('~target_mm', list(DEFAULT_TARGET_MM)))
        self.long_side_parallel = self._as_bool(
            rospy.get_param('~long_side_parallel', False))
        # 法向：`~normal` 给 "x,y,z" 时**直接用，不查 TF**（真机 TF 没起来时能跑，
        # 也是测试路径）；给空串 = **从 TF 查**（部署默认）。
        self.normal_param = self._parse_normal(rospy.get_param('~normal', ''))
        # 法向自检：**TF 只当搜索初值，答案从当前帧深度拟合**（默认开）。
        # 关掉 = 退回历史行为（TF 给什么就用什么）。现场 TF 法向实测偏 12.4°，
        # 那时 detect 是 37/37 全拒；换成拟合值 37/37 全过。
        self.fit_normal = self._as_bool(rospy.get_param('~fit_normal', True))
        # 重新拟合：false（默认）= **只在首帧拟合一次，之后一直复用**。
        # ⚠️ 依据是"伺服时机器人只做平面内平移"（不动腰），所以法向是不变量。
        # 真动了腰 / 换了工位要把它设成 true，否则会拿过期法向静默继续算。
        self.normal_refit = self._as_bool(rospy.get_param('~normal_refit', False))
        self._normal_fit = None            # 缓存的法向（只在工作线程读写）
        self._normal_fit_log = None        # 首帧拟合结果的一行说明，写进日志
        self.camera_frame = str(rospy.get_param('~camera_frame',
                                                'camera_color_optical_frame'))
        self.base_frame = str(rospy.get_param('~base_frame', 'base_link'))
        # ---- 台面高度的绝对约束（可选）----------------------------------------
        # `deck_z_mm = "lo,hi"`：把台面高度的选择约束在 base_link 的**绝对 z 带**
        # 里。给空串 = 不约束（保持历史行为）。
        #
        # ⚠️ **现场为什么需要它**：相机装在头上、光轴朝下，**手里抱着的纸箱**会
        # 同时出现在画面里，而且它橙黄、离得近、面积大 —— 实测它的像素把
        # `_adaptive_deck_height` 的中位拉到台面与箱子之间（`h_deck` 落到 615mm），
        # 掩码整个建在箱子上，1200x1000 的框再也套不上托盘。
        # 现场实测的绝对高度（用 TF 链换算）：托盘 z ≈ +43mm、地面 ≈ −107mm、
        # **手里的纸箱 ≈ +600mm** —— 一条 `[-40, 130]` 的带就能把箱子整个排除。
        #
        # ⚠️ **换工位 / 换托盘 / 机器人站姿变了都要重标**：这个带是**绝对高度**，
        # 而绝对高度依赖 TF 的平移（相机装多高、站姿如何）。带错了不是崩，而是
        # 「带内选不出台面」→ 拒帧，日志里能看到 `deck_h_rel_mm=None`。
        self.deck_z_mm = self._parse_z_band(rospy.get_param('~deck_z_mm', ''))
        # 心跳周期（秒）。**必须有一个"成功时也打"的日志** —— 本节点其余的日志点
        # 全是一次性/只在失败时，正常工作时一片安静，与挂掉长得一模一样。
        # 0（或负）= 关掉。
        try:
            self.heartbeat_s = float(rospy.get_param('~heartbeat_s', 5.0))
        except (TypeError, ValueError):
            self.heartbeat_s = 5.0
        # refine：检出之后**要不要跑** `refine_pallet_frame`，以及**要不要开
        # `plane=True`**。
        #
        # ⚠️ `plane` 的默认值是 `True`，而 `refine_pallet_frame` 的 **CLI 默认是
        # `False`**（`--plane` 是 opt-in）—— **这个不一致是刻意的**。
        # 实测（整条链 detect -> refine，合成场景）：法向偏 5.6° 时
        #   plane=False -> 终点 34.1mm / **dθ 5.7°**（朝向停在偏差上，伺服拿去会歪）
        #   plane=True  -> 终点 13.2mm / dθ 0.1°
        # TF 的法向与真值实测差 5.60°，所以部署路径**必须**开。
        # 关掉的理由（april_test7 上单帧法向动了 3.28°）写在 refine 那个函数的注释里，
        # 那是**静止场景的跨帧一致性**问题，与本节点"给伺服一帧初值"的用法不同。
        self.use_refine = self._as_bool(rospy.get_param('~refine', True))
        self.use_plane = self._as_bool(rospy.get_param('~plane', True))
        # 先验跟踪：把上一帧的检测结果喂给下一帧，θ 与位移都只在上一帧附近搜。
        # **默认开** —— 伺服跟踪时托盘不动、动的是机器人，相邻帧相对位移实测
        # 最大 9.9mm / 0.47°（见算法层 `PRIOR_*` 的注释），先验几乎是白给的：
        # `search` 段实测 **599ms -> 66~80ms**（本机，同一批现场帧）。
        # 关掉只在"托盘会跳变 / 一个画面里多个托盘"这种需要全画布搜索的场合有意义。
        self.use_prior = self._as_bool(rospy.get_param('~use_prior', True))
        # 上一帧成功的 frame。**只由工作线程读写**（`_run` 是唯一使用者，单线程，
        # 不用加锁）。**失败时不清空** —— 偶尔丢一帧不该让下一帧退回慢路径；
        # 先验真过期了，算法层的 `window_saturated` / `prior_score_low` 会自己
        # 退回全画布搜索，`diag['prior_reason']` 里写明原因。
        self._prior_frame = None

        self.k = None                       # 收到第一帧 camera_info 之后才有
        self.pub = ros['rospy'].Publisher(self.out_topic, ros['PalletDetection'],
                                          queue_size=5)
        # **队列长度 1**：算法慢（实测 ~2s/帧，合成 480x640）时宁可丢旧帧也不要
        # 排队 —— 排队的后果是输出的是几秒前那一帧的结果，伺服拿去会晚。
        import queue as _queue
        self.q = _queue.Queue(maxsize=1)
        self._n_dropped = 0
        self._n_processed = 0
        self._n_pub = 0
        self._n_reject = 0
        self._last_reject = None        # 最近一次拒绝的 `format_rejects` 原文
        self._last_good_ms = None       # 最近一次成功那帧的耗时
        self._last_pub_wall = None      # 最近一次发布的墙钟时刻
        self._last_heartbeat = 0.0
        self._last_warn = 0.0
        self._last_tf_warn = 0.0
        self._normal_checked = False
        self._zband_checked = False

        self.tf_buffer = None
        self.tf_listener = None
        if self.normal_param is None:
            self.tf_buffer = ros['tf2_ros'].Buffer()
            self.tf_listener = ros['tf2_ros'].TransformListener(self.tf_buffer)

        self._info_sub = rospy.Subscriber(self.info_topic, ros['CameraInfo'],
                                          self._on_camera_info, queue_size=1)
        subs = [ros['Subscriber'](self.image_color, ros['Image']),
                ros['Subscriber'](self.image_depth, ros['Image'])]
        self.sync = ros['ApproximateTimeSynchronizer'](
            subs, queue_size=5, slop=self.sync_slop)
        self.sync.registerCallback(self._on_synced)

        import threading
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        rospy.loginfo(f'[pallet_detection] 订阅 {self.info_topic} + {self.image_color} '
                      f'+ {self.image_depth}  ->  发布 {self.out_topic}')
        rospy.loginfo('[pallet_detection] 等第一帧 camera_info 建内参…')
        if self.normal_param is None:
            rospy.loginfo('[pallet_detection] 法向来源 = TF（%s -> %s，%d 段链）'
                          '；缺任何一段都不发布',
                          self.base_frame, self.camera_frame, len(CHAIN))
        else:
            rospy.logwarn('[pallet_detection] 法向来源 = 参数 ~normal=%s（**没查 TF**）。'
                          '部署时应给空串走 TF —— 参数法向是调试/无 TF 时的后路，'
                          '它的精度直接决定 detect 的初值质量'
                          '（差 5° 时 detect 单独偏 ~180mm；refine 的平面拟合'
                          '能修到 13mm，但前提是 plane=True）',
                          np.round(self.normal_param, 4).tolist())

    # ---- 参数工具 -------------------------------------------------------
    @staticmethod
    def _as_bool(v) -> bool:
        """字符串布尔：`bool("false") is True`，rosparam 常给字符串。"""
        if isinstance(v, str):
            return v.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(v)

    @staticmethod
    def _parse_normal(raw):
        """`"x,y,z"` → 单位向量；空串 / 认不出来 → None（= 从 TF 查）。"""
        if raw is None:
            return None
        s = str(raw).strip()
        if not s:
            return None
        try:
            v = np.array([float(x) for x in s.split(',')], float)
        except ValueError:
            return None
        if v.size != 3 or not np.all(np.isfinite(v)):
            return None
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            return None
        return v / n

    @staticmethod
    def _parse_z_band(raw):
        """`"lo,hi"` → `(lo, hi)`（mm）；空串 / 认不出来 / `lo >= hi` → None。

        `None` = **不约束**（历史行为）。认不出来时**返回 None 而不是抛** ——
        与 `_parse_normal` 同一条规矩：参数是辅助手段，不能因为它写错就让
        整条链路起不来；真要看它有没有生效，读日志里那行 `台面 z 带=...`。
        """
        if raw is None:
            return None
        s = str(raw).strip()
        if not s:
            return None
        try:
            v = [float(x) for x in s.split(',')]
        except ValueError:
            return None
        if len(v) != 2 or not all(np.isfinite(v)) or v[0] >= v[1]:
            return None
        return (v[0], v[1])

    # ---- 内参 -----------------------------------------------------------
    def _on_camera_info(self, msg):
        if self.k is not None:
            return
        K = list(msg.K)
        self.k = CameraIntrinsics(float(K[0]), float(K[4]), float(K[2]), float(K[5]))
        self._info_sub.unregister()          # 内参不变，收一帧就够
        self._ros['rospy'].loginfo(
            f'[pallet_detection] 内参 fx={self.k.fx:.2f} fy={self.k.fy:.2f} '
            f'cx={self.k.cx:.2f} cy={self.k.cy:.2f}')

    # ---- 法向 -----------------------------------------------------------
    def _resolve_normal(self, color_msg):
        """本帧用的台面法向 + 来源标签。查不到返回 `(None, 原因)` —— **不回退**。"""
        if self.normal_param is not None:
            return self.normal_param, 'param'
        missing: list = []
        stamp = color_msg.header.stamp
        n = normal_from_tf(self.tf_buffer, self.camera_frame,
                           source_frame=self.base_frame, stamp=stamp,
                           missing=missing)
        if n is None:
            return None, f'TF 链缺边 {missing}'
        return n, 'tf'

    def _normal_for_frame(self, color_msg, depth) -> tuple:
        """本帧实际用的法向 + 来源标签。**TF 只当搜索初值，答案从深度拟合。**

        返回 `(normal | None, 来源)`；`None` = 连初值都没有，调用方拒帧。

        ## 为什么不让 TF 当答案（2026-09-29 现场实测）

        TF 链的**朝向会在某个环节静默偏掉**，而 detect 对法向极其敏感（见模块
        docstring）。test_jia4 那 37 帧实测：

            地面平面（纯深度 RANSAC）      [0.035, -0.344, -0.938]
            托盘台面平面（木色∧近台面）    [0.037, -0.339, -0.940]
            TF 链给出的                    [0.003, -0.135, -0.991]
            地面 vs 托盘 1.14°；**TF vs 地面 12.52°**

        **37 帧里 TF 法向一个数都没变**（逐帧最大差 0.000°），所以不是抖动，
        是静态偏差。偏 12.4° 的后果是 `too_few_edges` **37/37 全拒**；换成拟合
        值、其它一个参数都不动，**37/37 全过**。

        ## 缓存（`~normal_refit`）

        ⚠️ **伺服时机器人只做平面内平移**（前后左右，不动腰），所以
        **相机相对 base 的朝向不变、base 的 z 与地面法向的关系也不变** ——
        法向是这个过程中唯一的**常量**。首帧花 ~10ms 拟合一次就够，
        之后每帧复用，不必重复付这个成本。

        ⚠️ 缓存的**前提是"朝向不变"**。真动了腰（或换了工位）要把
        `~normal_refit` 设成 true 重新拟合，否则会拿一个过期的法向继续算 ——
        这是**静默**的（法向偏了只会换一个 score 继续跑），所以日志里每次都
        写明这一帧用的是 `cache` 还是 `fit`，别去猜。
        """
        if self.normal_param is not None:
            return self.normal_param, 'param'
        prior, why = self._resolve_normal(color_msg)
        if prior is None:
            return None, why
        if not self.fit_normal:
            return prior, 'tf'
        if self._normal_fit is not None and not self.normal_refit:
            return self._normal_fit, 'cache'
        t0 = time.perf_counter()
        fitted, n_inl = fit_floor_normal(depth, self.k, prior)
        ms = (time.perf_counter() - t0) * 1000.0
        if fitted is None:
            # **回退策略 (A)**：拟合失败就用 TF 的初值，而不是拒帧 ——
            # 初值本身在夹角窗内（窗是 ±25°），它只是"没被纠正"，不是"错的"。
            self._normal_fit_log = (f'拟合失败（内点 {n_inl}）-> 回退 TF 初值 {ms:.0f}ms')
            return prior, 'tf'
        ang = float(np.degrees(np.arccos(np.clip(abs(float(fitted @ prior)), -1, 1))))
        self._normal_fit = fitted
        self._normal_fit_log = (f'与 TF 初值差 {ang:.2f}°（内点 {n_inl}，{ms:.0f}ms）')
        return fitted, 'fit'

    def _resolve_camera_z(self, color_msg):
        """相机原点在 base_link 里的 **z（mm）** —— `deck_z_mm` 那套绝对高度要用。

        没配 `~deck_z_mm` 时**根本不查 TF**（省一次 13 段链的查询）。查不到返回
        `None`，调用方据此**放弃 z 带约束**（不是拒帧 —— z 带是辅助手段，
        缺了退回历史行为，`_adaptive_deck_height` 的旧路径仍然有效）。

        ⚠️ 与 `_resolve_normal` **走同一条 TF 链但分开查**：两者都拼 `T_base_cam`，
        分开查会多一次链遍历。这么做是刻意的 —— 法向是**必需**的、缺了就拒帧，
        而相机 z 是**可选**的、缺了只降级；把它们捆在一起会让"可选参数的失败"
        污染"必需参数的成功路径"。
        """
        if self.deck_z_mm is None:
            return None
        # ⚠️ `~normal` 参数路径下**没有 base_link 的概念**（法向是手给的，相机系
        # 就是参考系），此时 z 带无从谈起 —— 直接返回 None，退回无约束。
        if self.normal_param is not None or self.tf_buffer is None:
            return None
        return camera_origin_from_tf(self.tf_buffer, self.camera_frame,
                                     source_frame=self.base_frame,
                                     stamp=color_msg.header.stamp)

    # ---- 回调只入队，不做计算 -------------------------------------------
    def _on_synced(self, color_msg, depth_msg):
        if self.k is None:                   # 还没拿到内参
            return
        try:
            self.q.put_nowait((color_msg, depth_msg))
        except Exception:                    # queue.Full
            self._n_dropped += 1
            try:                             # 丢最旧的、放最新的：保证输出尽量新鲜
                self.q.get_nowait()
                self.q.put_nowait((color_msg, depth_msg))
            except Exception:                # noqa: BLE001
                pass

    # ---- 工作线程：算 + 发布 --------------------------------------------
    def _run(self):
        import queue as _queue
        rospy = self._ros['rospy']
        while not rospy.is_shutdown():
            try:
                color_msg, depth_msg = self.q.get(timeout=0.2)
            except _queue.Empty:
                continue
            self._n_processed += 1
            if (self._n_processed - 1) % self.process_every != 0:
                continue                     # 不是处理槽
            t0 = time.perf_counter()
            try:
                color = _arr_to_bgr(color_msg)
                depth = _arr_to_depth_mm(depth_msg, self.depth_scale)
            except Exception as exc:         # noqa: BLE001
                rospy.logwarn_throttle(5.0, f'[pallet_detection] 输入转换失败：{exc}')
                continue

            normal, src_or_why = self._normal_for_frame(color_msg, depth)
            if normal is None:
                # **法向缺失 = 不发布**。绝不回退到单位阵/竖直向下 ——
                # 那样它不会崩、不会报错，只是换个 score 继续"如实拒绝"，
                # 而"没有法向先验时检测会静默地算下去"是已知的坑（见 docstring）。
                now = time.time()
                if now - self._last_tf_warn > 5.0:
                    self._last_tf_warn = now
                    rospy.logwarn(f'[pallet_detection] {src_or_why} —— 本帧**不发布**'
                                  f'（法向是必需先验，缺了就是静默乱算；检查 TF 树'
                                  f'或临时用 ~normal 参数）')
                continue

            cam_z = self._resolve_camera_z(color_msg)
            try:
                found, diag = detect_pallet_frame(
                    color, depth, self.k, normal=normal, target_mm=self.target_mm,
                    long_side_parallel=self.long_side_parallel,
                    prior=self._prior_frame if self.use_prior else None,
                    camera_z_mm=cam_z, deck_z_mm=self.deck_z_mm)
            except Exception as exc:         # noqa: BLE001
                rospy.logwarn_throttle(5.0, f'[pallet_detection] 检测异常：{exc}')
                continue
            if not self._zband_checked:
                self._zband_checked = True
                if self.deck_z_mm is None:
                    rospy.loginfo('[pallet_detection] 台面 z 带=**未启用**'
                                  '（~deck_z_mm 空）—— 手里抱着的纸箱等'
                                  '"高度上不可能"的东西不会被排除')
                elif cam_z is None:
                    rospy.logwarn('[pallet_detection] 配了 ~deck_z_mm=%s 但**查不到'
                                  '相机在 base_link 里的位置**（TF 缺边或走了 ~normal'
                                  ' 参数路径）—— 本帧起退回**无约束**',
                                  list(self.deck_z_mm))
                else:
                    rospy.loginfo('[pallet_detection] 台面 z 带=%s mm'
                                  '（相机在 base_link z=%.0fmm）',
                                  list(self.deck_z_mm), float(cam_z))
            if found is not None:
                self._prior_frame = found    # 给下一帧当先验（见 __init__ 的说明）
            ms = (time.perf_counter() - t0) * 1000.0
            if not self._normal_checked:
                self._normal_checked = True
                extra = f'  {self._normal_fit_log}' if self._normal_fit_log else ''
                # ⚠️ **来源标签必须看**：`cache` = 复用首帧的拟合结果（默认），
                # `fit` = 这一帧真的重新拟合了，`tf` = 拟合失败回退了初值。
                # 缓存的**前提是相机相对 base 的朝向不变**（伺服只做平面内平移）；
                # 前提被破坏时症状是"标签看着没问题、数就是不对"，所以这里点名。
                rospy.loginfo(f'[pallet_detection] 法向来源={src_or_why} '
                              f'normal={np.round(normal, 4).tolist()}'
                              f'（fit_normal={self.fit_normal} '
                              f'normal_refit={self.normal_refit}）{extra}')
                if src_or_why == 'cache':
                    rospy.loginfo('[pallet_detection] 法向走**缓存**（首帧拟合的结果）'
                                  '—— 伺服只做平面内平移时这是对的；'
                                  '**动了腰或换了工位**要把 ~normal_refit 设成 true')
            if found is None:
                self._n_reject += 1
                self._last_reject = format_rejects(diag)
                rospy.loginfo_throttle(
                    5.0, f'[pallet_detection] 如实拒绝：{self._last_reject} '
                         f'（{ms:.0f}ms）')
                self._heartbeat(rospy, ms)
                continue

            # ---- refine（**默认开，且显式 plane=True**）---------------------
            # detect 只搜平面内 3 个自由度，法向偏 5.6° 时它单独会偏 180mm；
            # refine 的 `_refine_plane` 用 SVD 把台面平面（法向 2 + 高度 1）
            # 自己解一遍，终点落回 13.2mm / dθ 0.1°。**但前提是 plane=True** ——
            # plane=False 时朝向会停在 5.7°（见模块 docstring）。
            refine_diag = None
            refined = None
            final = found                      # 发布用的位姿：refine 过就用 refine 的
            if self.use_refine:
                try:
                    final, refine_diag = refine_pallet_frame(
                        found, color, depth, self.k, target_mm=self.target_mm,
                        opts=dict(plane=self.use_plane))
                except Exception as exc:     # noqa: BLE001
                    rospy.logwarn_throttle(
                        5.0, f'[pallet_detection] refine 异常：{exc} —— 退回 detect '
                             f'的结果（法向未细化）')
                    final, refine_diag, refined = found, None, False
                else:
                    if final is None:
                        # refine 拒了：**不发布**。它拒的理由（`no_edges` 等）
                        # 与 detect 的不是一回事，混在一起会看不出是谁拒的。
                        rospy.loginfo_throttle(
                            5.0, f'[pallet_detection] refine 如实拒绝：'
                                 f"reject={refine_diag.get('reject')} "
                                 f"plane={refine_diag.get('plane')} （{ms:.0f}ms）")
                        self._n_reject += 1
                        self._last_reject = f"refine:{refine_diag.get('reject')}"
                        self._heartbeat(rospy, ms)
                        continue
                    refined = True

            payload = build_payload(final, diag, self.k, latency_ms=ms,
                                    normal_source=src_or_why, normal=normal,
                                    refine_diag=refine_diag, refined=refined)
            if self.use_refine and refined:
                plane = (refine_diag or {}).get('plane') or {}
                if not plane.get('refined'):
                    # ⚠️ 平面细化**没生效**（守卫挡了 / 点数不够）。这不是错，
                    # 但意味着朝向停在 detect 的法向上 —— 必须留痕，否则
                    # "法向偏 5.6° 时朝向停在 5.7°"这件事会静默发生。
                    rospy.logwarn_throttle(
                        10.0, f'[pallet_detection] refine 的平面细化没生效'
                              f"（note={plane.get('note')}）—— 位姿的法向仍是 "
                              f'detect 的先验，偏差不会被子细化')
            self.pub.publish(self._to_msg(payload, color_msg.header))
            self._n_pub += 1
            self._last_pub_wall = time.time()
            self._last_good_ms = ms
            if ms > self.queue_warn_ms and time.time() - self._last_warn > 5.0:
                self._last_warn = time.time()
                rospy.logwarn(f'[pallet_detection] 本帧 {ms:.0f}ms，超过 '
                              f'{self.queue_warn_ms:.0f}ms（丢帧 {self._n_dropped}）')
            self._heartbeat(rospy, ms)

    def _heartbeat(self, rospy, ms):
        """**周期心跳** —— 每 `~heartbeat_s` 秒一行，无论成功还是拒绝。

        ⚠️ **为什么必须有一条"成功时也打"的日志**：本节点原来的日志点全是
        *一次性的*（`法向来源`/`台面 z 带` 只打首帧）或*只在失败时*（`如实拒绝`）
        或*只在超时时*（`本帧 Xms 超过`）—— 于是「**一切正常**」与「**工作线程死了 /
        一直在拒但节流吞了消息**」在日志上**长得一模一样**：都是什么都不打。
        2026-09-29 现场就是这么被卡的：首帧那条 3458ms 的 WARN 之后一片安静，
        分不清是在跑还是挂了。心跳把这两种情况分开。

            proc=123 pub=120 rej=3 drop=456  本帧 880ms  最近一次发布 0.9s 前
              最近一次拒绝：reject=too_few_edges,n_observed_edges=0<2,score=0.4135

        `pub` 在涨 = 在发布；`proc` 涨而 `pub` 不涨 = 在拒（第二行给原因）；
        `proc` 都不涨 = **线程死了或相机停了**（两者再靠 `rostopic hz
        /camera/color/image_raw` 分开）。`drop` 一直涨是正常的（30Hz 相机配
        ~1s/帧），只要 `pub` 也在涨就不用管。

        ⚠️ **第二行（最近一次拒绝）只在真有拒绝时打**，且**不随心跳重复刷屏** ——
        拒绝内容不变时内容也一样，看的人会当噪声。但它**每次心跳都会重打**，
        因为"上一次拒绝是什么"正是排查时要对的东西；真嫌吵把 `~heartbeat_s` 调大。
        """
        if self.heartbeat_s <= 0.0:
            return
        now = time.time()
        if now - self._last_heartbeat < self.heartbeat_s:
            return
        self._last_heartbeat = now
        stale = (f'{now - self._last_pub_wall:.1f}s' if self._last_pub_wall
                 else '从未')
        last_ms = self._last_good_ms if self._last_good_ms is not None else ms
        rospy.loginfo(
            f'[pallet_detection] proc={self._n_processed} pub={self._n_pub} '
            f'rej={self._n_reject} drop={self._n_dropped}  最近一帧 {last_ms:.0f}ms  '
            f'最近一次发布 {stale} 前')
        if self._last_reject:
            rospy.loginfo(
                f'[pallet_detection]   最近一次拒绝：{self._last_reject}')

    def _to_msg(self, p: dict, header):
        """payload -> 消息。**这里只搬运，不含任何判断。**

        ⚠️ `header.stamp` **就是传进来的那个**（= 彩色图那帧的采集时刻），
        这里**不碰**它 —— 换 `rospy.Time.now()` 就是契约里那条硬要求说的错。
        """
        ros = self._ros
        m = ros['PalletDetection']()
        m.header = ros['Header'](stamp=header.stamp, frame_id=header.frame_id)
        m.T_cam_pallet = [float(v) for v in p['T_cam_pallet']]
        m.det = float(p['det'])
        m.valid = bool(p['valid'])
        m.source = p['source']
        m.size_mm = [float(v) for v in p['size_mm']]
        m.corners_uv = [ros['Point32'](float(u), float(v), 0.0)
                        for u, v, _ in p['corners_uv']]
        m.n_used = int(p['n_used'])
        m.n_slots = int(p['n_slots'])
        m.n_failed = int(p['n_failed'])
        m.spread_mm = float(p['spread_mm'])
        m.spread_deg = float(p['spread_deg'])
        m.latency_ms = float(p['latency_ms'])
        m.diag = p['diag']
        m.rejects = p['rejects']
        return m



def main():
    ros = _ros()
    ros['rospy'].init_node('pallet_detection')
    PalletDetectionNode(ros)
    ros['rospy'].spin()


if __name__ == '__main__':
    main()
