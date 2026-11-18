      
#!/usr/bin/env python3
"""ROS1 节点：`/pallet_v2/detection` —— 木托盘台面坐标系的实时检测结果。

**前端**：YOLO-pose 12 关键点 → 四边解算（15Hz 路径）。

**分层**（与仓库里其他检测节点同一结构，架构硬约束）：

    skills/atomic/perception/pallet_detect/   纯函数、零状态、**不 import ROS**  ← 算法核心
        detect_pose.py     detect_pallet_frame_pose()  YOLO-pose -> 四边 -> solve_edges
        frame_geometry.py       共享的矩形/平面几何（基、边筛选、规格归一）
        payload.py         build_payload() / format_*()     纯函数搬运层（不 import ROS）
    pallet_detection_node.py                  本文件：节点类只做搬运
                                              （订阅 / 推理会话 / 出队 / 填消息 / 发布）
    tf_normal.py       normal_from_tf()                   要查 TF，在 infrastructure 层

算法层在 LeTools 的 `skills/` 里、**不 import ROS**，所以它能脱离 ROS 单独跑和测；
本文件里**没有业务逻辑**（推理会话在启动时建一次，解码/选框/细化全在算法层）。

订阅（两路时间戳要接近，用 `ApproximateTimeSynchronizer` 对齐）：
  * `~image_color`  sensor_msgs/Image          彩色图（默认 `/camera/color/image_raw`）
  * `~image_depth`  sensor_msgs/Image          对齐后的深度图，**16UC1，单位 mm**
  * `~camera_info`  sensor_msgs/CameraInfo      **内参来源**（标准的 K 矩阵）
发布：
  * `/pallet_v2/detection`  pallet_detection_v2_msgs/PalletDetection

参数（`rosparam`，都有默认值）：
  | 名字 | 默认 | 说明 |
  |---|---|---|
  | `image_color` | `/camera/color/image_raw` | 彩色话题 |
  | `image_depth` | `/camera/depth/image_raw` | 深度话题（只用于法向拟合；不参与 D） |
  | `camera_info` | `/camera/color/camera_info` | 内参话题（用它的 K 矩阵） |
  | `detection_out` | `/pallet_v2/detection` | 输出话题 |
  | `target_mm` | `[1200.0, 1000.0]` | 托盘实际尺寸（长, 短）。**call 0 记录存在时会被记录里的 `pallet_size_mm` 覆盖** |
  | `pallet_record_path` | `''` | call 0 记录路径；空 = `<仓库根>/log/pallet_servo_pallet_record.json`。记录里的尺寸 / 法向 / D 优先；**有记录时直接用记录 D，不重算 D、不做 D 闸门** |
  | `pallet_record_epoch_topic` | `/pallet_servo_v2/pallet_record_epoch` | call 0 写记录成功的版本通知（latched Int32），收到就重读整条记录 |
  | `pallet_normal_rot_topic` | `/pallet_servo_v2/pallet_normal_rot_deg` | 伺服切档时发来的法向视角旋转 `[roll,pitch,yaw]`（latched `Float64MultiArray`）。机器人换到托盘对面后，用它把记录的 base_link 法向换到当前 base 姿态；空 = 不订阅 |
  | `long_side_parallel` | `false` | **与画面近平行的托盘边是不是长边**。定死面内转角（短边近平行 -> `false`） |
  | `normal` | `''` | `"x,y,z"`；**给空串 = 从 TF 查**（部署默认） |
  | `fit_normal` | `true` | **没有 call 0 记录时**：TF 只当搜索初值，实际法向从当前帧深度拟合 |
  | `normal_refit` | `false` | `false` = 只在首帧拟合一次、之后复用；动腰/换工位要设 true |
  | `normal_max_tilt_deg` | `60.0` | 首帧拟合的**合理性自检**：法向与相机光轴的最大夹角，`0` = 关。拦住"TF 初值本身侧向"导致的歪拟合 |
  | `camera_frame` | `camera_color_optical_frame` | TF 里的相机系 |
  | `base_frame` | `base_link` | TF 里的基座系 |
  | `infer_backend` | `auto` | `auto`（engine 文件在就用它）/ `engine` / `onnx`。**只认这三个值** |
  | `imgsz` | `640` | 输入边长，**必须与导出时一致** |
  | `nms_iou` | `0.45` | POSE 候选框的 AABB NMS 阈值（导出图没有 NMS，**必须自己去重**） |
  | `pose_model_path` | `''` | POSE 的 `.onnx`；空 = 包内 `models/pallet_pose.onnx` |
  | `pose_engine_path` | `''` | POSE 的 `.engine`；空 = 包内 `models/pallet_pose_fp32.engine` |
  | `pose_conf` | `0.25` | POSE 置信度阈值 |
  | `pose_kconf` | `0.5` | POSE 参与边拟合的最低关键点置信度 |
  | `pose_pick` | `bottom_center` | POSE 多候选怎么挑：`bottom_center` / `conf` / `area` |
  | `sync_slop_s` | `0.05` | 两路时间戳允许的最大差 |
  | `depth_scale_mm` | `1.0` | 深度单位换算到 mm 的系数 |
  | `process_every` | `1` | 每几帧处理一次 |
  | `queue_warn_ms` | `1000.0` | 处理慢于这个就告警一次（节流） |
  | `heartbeat_s` | `5.0` | 心跳周期（秒）。**成功时也打**，见 §心跳 |

  `long_side_parallel` 是**面内转角的先验、必须给对**：关键点四边形的宽高比 ≈ 1，
  "哪条边是长边"在框上不可分，所以**不能靠数据去挑**（试两个转角候选、用残差选，
  同一场景不同帧会投出不同结果）。它直接定死转角种子 —— ⚠️ **给错就明确偏 90°**，
  这是刻意的（错误应当可见，而不是被兜底掩盖）。现场：短边近平行 -> `false`。

## ⚠️ 托盘检测**必须**有模型（缺失即起不来，没有回退）

模型走 `pose_model_path` / `pose_engine_path`（或包内
`models/pallet_pose.{onnx,fp32.engine}`）。拿不到可用 session 时 `__init__`
**直接抛**、节点起不来 —— 与 `carton_obb_detect` 同一做法。理由：模型缺失若只是
每帧静默返回 `None`，症状是"一直检不到"，与"现场真没有托盘"在日志上**无法区分**。
导出见 `pallet_detect/scripts/export_onnx_pose.py`；
engine 用 `trtexec --onnx=... --saveEngine=... --noTF32 --buildOnly` 在**目标机**上导。
`~infer_backend=auto` 时 engine 文件在就用 engine（预先导出、启动不编译），
否则用 ONNX；engine 存在但加载失败**直接报错**，不静默退回 ONNX。

## ⚠️ 心跳：分不清"在跑"和"挂了"是这个节点的老毛病

本节点的日志点大多是**一次性的**（`法向来源` 只打首帧）、**只在失败时**
（`如实拒绝`）或**只在超时时**（`本帧 Xms 超过`）。于是「一切正常」与「工作线程
死了 / 一直在拒但被节流吞了」在日志上**长得一模一样**：都是什么都不打。
2026-09-29 现场就这么卡过一次。现在每 `~heartbeat_s` 秒**无条件**打一行：

```
[pallet_detection_v2] proc=123 pub=120 rej=3 drop=456  本帧 40ms  最近一次发布 0.9s 前
```

| 现象 | 含义 |
|---|---|
| `proc` 在涨、`pub` 也在涨 | 正常 |
| `proc` 涨、`pub` 不涨、`rej` 在涨 | 在拒 —— 第二行就是原因 |
| `proc` 都不涨 | **工作线程死了，或者相机停了**（再用 `rostopic hz` 分开）|
| `drop` 一直涨 | 正常，只要 `pub` 也在涨就不用管 |

⚠️ **`header.stamp` 填的是彩色图那帧的 `header.stamp`**（`color_msg.header.stamp`），
**不是 `rospy.Time.now()`**。伺服要把托盘与箱子的位姿**相减**，配对依据是"输入图像
的采集时刻"。填成处理完成时刻的话，耗时的差会被当成场景的时间差 —— 要么永远配不上，
要么配上一对其实是不同帧的图像，相减出来的误差里混着相机运动，**而三个数照样
算得出来**。契约原文见 `pallet_detection_v2_msgs/msg/PalletDetection.msg` 末尾。

## ⚠️ 法向是**两级**的，第二级（深度自检）不能跳

1. **TF 先验**：TF 链通了、`base_link` 的 z 转进相机系 —— 但它与托盘实际台面法向
   会有 **0~10° 的静态偏差**（现场事实）。
2. **深度自检**：`fit_floor_normal` 在当前帧深度上、以先验为初值（夹角窗 ±25°）
   拟合"画面最大的面"，把这 0~10° 纠正回来（`~fit_normal`，默认开）。

⚠️ **为什么必须做第二级**：托盘解算把法向**锁死**成先验（自由度 6→4），
偏了**没有任何下游步骤会纠正它** —— 那 0~10° 会直接进位姿。实测（test_jia4 37 帧）
TF 与地面差 12.52°，旧路线 `too_few_edges` 37/37 全拒，拟合后 37/37 全过。

来源写进 `diag`（`normal_source=fit|cache|tf|param`）与首帧日志。**TF 缺任何一段
都不发布** —— 绝不回退到单位阵/竖直向下（那不会崩、不报错，只是换个 score 继续算）。
拟合失败才退回 TF 先验（那时只是"没被纠正"，不是"错的"，不拒帧）。换工位 / 动腰后
要重新确认（必要时 `~normal_refit=true` 或 `~normal` 手动给定）。

跑法（`OMP_NUM_THREADS=1` 是必须的，实测差 1.6x，见 WORKLOG_3test §15.1）：
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
    export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
    # 正式跑：roslaunch（target_mm 由 launch 里的 <rosparam> 给成 list）
    roslaunch pallet_detection pallet_detection.launch
    # 单节点直跑（`_x:=` 命令行赋值 roslaunch 的 auto 转换认不出 list，给的是字符串
    # —— 本节点两种都吃，见 `_parse_target_mm`）
    python3 pallet_detection_node.py _normal:="" _target_mm:="[1200,1000]"

⚠️ 需要先编译消息包 `infrastructure/ros_packages/src/ros_vision/pallet_detection_v2_msgs`
（catkin 工作区里 `catkin_make`），并把生成的 Python 模块放进 `PYTHONPATH`。
没编译的话本文件会给出明确报错。
"""
from __future__ import annotations

import os
import sys
import time
import json
import math
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

# 算法层在 LeTools 的 skills 里（纯函数，不 import ROS）。本脚本从基础设施层
# 反查仓库根 —— 与仓库里其他检测节点同一手法。
def _repo_root() -> Path:
    """仓库根 = **第一个同时含 `skills/` 与 `infrastructure/` 的祖先目录**。

    ⚠️ 从前这里写的是 `Path(__file__).resolve().parents[6]` —— **数层数**。
    那种写法把"本包在仓库里的深度"钉进了代码：2026-10-15 挪 `carton_obb` 时
    就发现它是全仓最硬的一处依赖（挪一层就断，而且断在 `import` 上、
    **只有真机起节点时才现形**，单元测试构造不到这个类）。
    按特征找则与位置无关。

    ⚠️ 这个函数在**每个入口点各有一份**（本脚本、纸箱节点），不是疏忽：
    它存在的意义就是"在仓库根还不知道在哪的时候把它找出来"，所以不能从
    仓库根的模块（`core.common.*`）import —— 那是循环依赖。

    找不到时返回最外层祖先（不抛）：调用方本来就允许"拿不到仓库根就退回
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
from skills.atomic.perception.pallet_detect_v2 import (   # noqa: E402
    CameraIntrinsics,
    build_payload,
    fit_floor_normal_checked,
    format_rejects,
)
# ⚠️ `DEFAULT_TARGET_MM` 与 `CHAIN` **必须显式 import**：它们定义在纯函数层
# `payload.py` 里，本文件只是使用者。漏了不会在 import 时报错 ——
# `PalletDetectionNode.__init__` 里才 NameError，而**单元测试从来不构造这个类**
# （只调静态方法），所以整条 ROS 路径零覆盖。测试里有一条 AST 静态检查钉住这点。
# POSE 前端：`(frame | None, diag)`；模型缺失**硬失败**。
from skills.atomic.perception.pallet_detect_v2.detect_pose import (  # noqa: E402
    DEFAULT_POSE_ENGINE_PATH,
    DEFAULT_POSE_MODEL_PATH,
    _resolve_pose_session,
    detect_pallet_frame_pose,
)
from skills.atomic.perception.pallet_detect_v2.payload import (  # noqa: E402
    CHAIN,
    DEFAULT_TARGET_MM,
)
# ⚠️ `normal_from_tf` **不**从 skills 取：它要 import rospy，按分层约束只能待在
# 基础设施层，就在本脚本旁边（同目录 `tf_normal.py`）。本脚本开头已经把
# `Path(__file__).parent` 插进了 sys.path，所以直接 import 得到。
from tf_normal import normal_from_tf, transform_from_tf  # noqa: E402

# --------------------------------------------------------------------------- #
# 统一日志（`core/common/logger.py`）—— **带回退**
# --------------------------------------------------------------------------- #
# 仓库里那套 logger 给每行加 `[TRACE:xxxxxxxx] [file.py:行号]`，并把日志写进
# `log/` 下带时间戳的文件、支持"写满另开一个 + 按天清理"。
#
# ⚠️ **必须带回退**：本节点是 catkin 包里的独立 ROS 节点，**单独
# `roslaunch pallet_detection …` 时仓库根不一定在 `sys.path` 上**（一键脚本会 export
# PYTHONPATH，但不是所有跑法都会）。import 不到就退回 `rospy.log*` —— 那种情况下
# 日志仍然进 rosout / 终端，只是没有 trace id 与落盘轮转，**功能一点不少**。
try:
    from core.common.logger import init_logging as _init_logging
except Exception:                                    # noqa: BLE001
    _init_logging = None


def _console_output() -> bool:
    """统一 logger 还要不要往终端打一份。

    一键启动脚本会把它关掉（`LETOOLS_LOG_CONSOLE=0`）：那个脚本已经把本进程的
    stdout 重定向进**共用的日志文件**，而 logger 的控制台 handler 也写 stdout ——
    两份叠在一起，就是**每条日志在同一个文件里出现两次**（一次详细格式、一次
    控制台格式）。关掉之后每次只写一遍，文件才读得下去。

    手工 `roslaunch pallet_detection …` 时不设这个变量，终端照常有输出 ——
    交互调试不该被一键脚本的取舍牵连。
    """
    return os.environ.get('LETOOLS_LOG_CONSOLE', '1').strip().lower() not in (
        '0', 'false', 'no', 'off')


def _init_unified_logging(rospy, tag: str) -> None:
    """**必须在 `rospy.init_node()` 之后调**。

    `rospy.init_node()` 会往 root logger 上重装它自己的 handler，把我们配好的
    清掉 —— 顺序反了就白配。仓库里的既定写法见 `orchestration/nodes/boxcarry.py`
    与 `apps/test_upper_init/run_behavior_tree_json.py`（两处逐字相同）。
    """
    if _init_logging is None:
        rospy.logwarn('[pallet_detection_v2] 没找到 core.common.logger —— 日志只走 rospy，'
                      '不落盘、不轮转。注意仓库根**本节点自己会插进 sys.path**，'
                      '所以这多半不是路径问题，而是缺依赖（requirements.txt 里的 '
                      'colorlog）或部署的是旧代码')
        return
    try:
        _init_logging(force=True, log_tag=tag, console_output=_console_output())
    except Exception as exc:                         # noqa: BLE001
        rospy.logwarn('[pallet_detection_v2] 统一日志初始化失败，退回 rospy：%s', exc)


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
        from pallet_detection_v2_msgs.msg import PalletDetection
    except ImportError as exc:                       # pragma: no cover
        raise SystemExit(
            f'没找到消息包 pallet_detection_v2_msgs（{exc}）。先把 '
            'infrastructure/ros_packages/src/ros_vision/pallet_detection_v2_msgs 放进 catkin 工作区的 src/ 下 catkin_make，'
            '再 source devel/setup.bash：\n'
            '    cp -r infrastructure/ros_packages/src/ros_vision/pallet_detection_v2_msgs <工作区>/src/ && cd <工作区> && catkin_make')
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

    （与另一条链的同名函数逐字相同 —— 它跑过真机，直接抄。）
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
    """-> np.ndarray (H,W,3) BGR，喂给 `detect_pallet_frame_pose`。"""
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
    """-> np.ndarray (H,W) float32，**单位 mm**（检测需要 mm）。

    ⚠️ 常见坑：**Orbbec 头部相机的深度话题是 `/camera/depth/image_raw`**（默认值），
    RealSense 那套命名是 `aligned_depth_to_color/image_raw` —— 换相机时改
    `~image_depth`。Orbbec 发 **16UC1 且已经是毫米**；有些驱动却发 `32FC1` 的**米**。
    两种都支持，用 `depth_scale_mm` 兜底。

    ⚠️ 深度拟合链要浮点，这里返回 `float32`。
    """
    import numpy as np
    if msg.encoding in ('16UC1', 'mono16'):
        a = _image_to_array(msg, np.uint16).astype(np.float32)
        return a if scale == 1.0 else a * scale
    if msg.encoding == '32FC1':
        return _image_to_array(msg, np.float32) * 1000.0 * scale
    raise ValueError(f'不支持的深度编码 {msg.encoding!r}（要 16UC1 或 32FC1）')


# --------------------------------------------------------------------------- #
# POSE 关键点 / 选边诊断 -> 消息字段（固定长度、缺字段按契约填占位值）
# --------------------------------------------------------------------------- #
_N_KP = 12
_N_EDGE = 4
_NO_KP_UV = (-1.0, -1.0)
_NO_KP_CONF = -1.0
_NO_EDGE = -1


def _finite_float(value):
    """能变成有限 float 就返回它，否则 None。"""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _edge_debug_fields(diag):
    """`detect_pallet_frame_pose` 的 `diag` → 固定长度的诊断字段。

    **字段缺失一律按"没有"处理，绝不猜**：
      * 关键点 uv `(-1,-1)`、conf `-1`；
      * `selected_edges` 全是 `-1`；
      * `edge_score` 全是 `-1`（不可见/未知）；
      * `edge_visible` 全 `0`；
      * `dropped_edges` 全 `-1`；
      * `reason` 空串。

    ⚠️ `diag['edge_score']` 里允许有 `None`（那条边不可见）—— 这里转成 `-1`。
    这条函数是**纯搬运**，不做任何选边判断。
    """
    if not isinstance(diag, dict):
        diag = {}

    raw_uv = diag.get('keypoints_uv')
    raw_conf = diag.get('keypoints_conf')
    keypoints_uv = []
    keypoints_conf = []
    for i in range(_N_KP):
        u = v = _NO_KP_UV[0]
        conf = _NO_KP_CONF
        try:
            u = _finite_float(raw_uv[i][0])
            v = _finite_float(raw_uv[i][1])
            u = _NO_KP_UV[0] if u is None else u
            v = _NO_KP_UV[1] if v is None else v
        except (TypeError, IndexError, KeyError):
            pass
        try:
            c = _finite_float(raw_conf[i])
            conf = _NO_KP_CONF if c is None else c
        except (TypeError, IndexError, KeyError):
            pass
        keypoints_uv.append((u, v))
        keypoints_conf.append(conf)

    raw_score = diag.get('edge_score')
    if raw_score is None:
        raw_score = []
    edge_score = []
    for i in range(_N_EDGE):
        try:
            s = _finite_float(raw_score[i])
        except (TypeError, IndexError, KeyError):
            s = None
        edge_score.append(_NO_EDGE if s is None else s)

    # `visible_edges` 是选边算法的原始输出（下标列表）；没有它时退回
    # "score >= 0 就算可见"。两者都没有就是全不可见。
    raw_visible = diag.get('edge_visible')
    if raw_visible is None:
        raw_visible = diag.get('visible_edges')
    edge_visible = [0] * _N_EDGE
    if raw_visible is not None:
        try:
            for i in raw_visible:
                idx = int(i)
                if 0 <= idx < _N_EDGE:
                    edge_visible[idx] = 1
        except (TypeError, ValueError):
            pass
    else:
        edge_visible = [1 if s >= 0.0 else 0 for s in edge_score]

    selected = [_NO_EDGE] * 2
    raw_selected = diag.get('selected_edges')
    if raw_selected is not None:
        try:
            for i, value in enumerate(list(raw_selected)[:2]):
                idx = int(value)
                selected[i] = idx if 0 <= idx < _N_EDGE else _NO_EDGE
        except (TypeError, ValueError):
            pass

    dropped = [_NO_EDGE] * _N_EDGE
    try:
        for i, value in enumerate(list(diag.get('dropped_edges') or [])[:_N_EDGE]):
            idx = int(value)
            dropped[i] = idx if 0 <= idx < _N_EDGE else _NO_EDGE
    except (TypeError, ValueError):
        pass

    reason = (diag.get('edge_select_reason')
              or diag.get('edge_pair_note')
              or diag.get('reject')
              or '')
    return {
        'keypoints_uv': keypoints_uv,
        'keypoints_conf': keypoints_conf,
        'selected_edges': selected,
        'edge_score': edge_score,
        'edge_visible': edge_visible,
        'dropped_edges': dropped,
        'edge_select_reason': str(reason),
    }



class PalletDetectionNode:
    """订阅 彩色+深度+camera_info → 检测 → 发布 `PalletDetection`。"""

    def __init__(self, ros):
        self._ros = ros
        rospy = ros['rospy']
        self.image_color = rospy.get_param('~image_color', '/camera/color/image_raw')
        self.image_depth = rospy.get_param('~image_depth', '/camera/depth/image_raw')
        self.info_topic = rospy.get_param('~camera_info', '/camera/color/camera_info')
        self.out_topic = rospy.get_param('~detection_out', '/pallet_v2/detection')
        self.depth_scale = float(rospy.get_param('~depth_scale_mm', 1.0))
        self.sync_slop = float(rospy.get_param('~sync_slop_s', 0.05))
        self.queue_warn_ms = float(rospy.get_param('~queue_warn_ms', 1000.0))
        self.process_every = max(1, int(rospy.get_param('~process_every', 1)))
        self.target_mm = _parse_target_mm(
            rospy.get_param('~target_mm', list(DEFAULT_TARGET_MM)))
        # ★ call 0 记录里的尺寸优先于 board/launch 的 target_mm（2026-11-03）：
        #   上一次观测托盘时记下的尺寸跨开关机保留，生产不再需要手写
        #   pallet_size_mm。记录不存在/坏了就退回 target_mm，不阻断启动。
        self.pallet_record_path = str(rospy.get_param(
            '~pallet_record_path', '') or '').strip()
        # ★ call 0 记录（2026-11-03）：尺寸、base_link 系法向与 D 一次读进来。
        #   有记录时**不再每帧查 TF / 拟合法向 / 量 D** —— 这些量在 call 0
        #   已经定过，伺服过程中是常量；每帧重算只会把抖动带进位姿。
        #   没有记录（第一次跑、还没 call 0）才走原来的 TF + 深度自检。
        #   ⚠️ **整条记录一次替换**（`_record` 是一个 dict / None）：
        #      新文件缺字段时整条作废，绝不出现"新 D + 旧法向"的拼接。
        self._record = None
        self._frame_record = None      # 本帧快照（normal/D 必须同源）
        self._ignore_record = False    # 观测模式：忽略记录，走 TF+深度
        self._apply_pallet_record()
        # ★ 伺服节点 call 0 实测出的尺寸通过 latched 话题发来；同一个运行里
        #   覆盖 target_mm，让参考边和检测器用同一个尺寸（2026-11-03）。
        self.pallet_size_topic = str(rospy.get_param(
            '~pallet_size_topic', '/pallet_servo_v2/pallet_size_mm') or '').strip()
        self.pallet_size_sub = None
        if self.pallet_size_topic:
            try:
                from std_msgs.msg import Float32MultiArray

                self.pallet_size_sub = rospy.Subscriber(
                    self.pallet_size_topic, Float32MultiArray,
                    self._on_pallet_size, queue_size=1)
            except Exception as exc:                 # noqa: BLE001
                rospy.logwarn(
                    '[pallet_detection_v2] 订阅实测尺寸 %s 失败（%s）—— 只用启动时的 '
                    'target_mm/记录', self.pallet_size_topic, exc)
        # ★ 记录版本：call 0 写文件成功就发一次（与尺寸是否测出来无关）。
        self.pallet_record_epoch_topic = str(rospy.get_param(
            '~pallet_record_epoch_topic',
            '/pallet_servo_v2/pallet_record_epoch') or '').strip()
        self._last_record_epoch = None
        self.pallet_record_epoch_sub = None
        if self.pallet_record_epoch_topic:
            try:
                from std_msgs.msg import Int32

                self.pallet_record_epoch_sub = rospy.Subscriber(
                    self.pallet_record_epoch_topic, Int32,
                    self._on_record_epoch, queue_size=1)
            except Exception as exc:                 # noqa: BLE001
                rospy.logwarn(
                    '[pallet_detection_v2] 订阅记录版本 %s 失败（%s）—— '
                    '同一运行里 call 0 后不会重读记录',
                    self.pallet_record_epoch_topic, exc)
        # ★ 法向视角旋转（2026-11-03）：伺服节点按档位发 `[roll,pitch,yaw]`（度）。
        #   换到托盘对面时，call 0 记录在 base_link 里的法向要旋转后才能用；
        #   **俯仰/横滚同样会改变法向**，所以这里是完整的三轴旋转。
        self.pallet_normal_rot_topic = str(rospy.get_param(
            '~pallet_normal_rot_topic',
            '/pallet_servo_v2/pallet_normal_rot_deg') or '').strip()
        self._normal_rot_deg = (0.0, 0.0, 0.0)
        self.pallet_normal_rot_sub = None
        if self.pallet_normal_rot_topic:
            try:
                from std_msgs.msg import Float64MultiArray

                self.pallet_normal_rot_sub = rospy.Subscriber(
                    self.pallet_normal_rot_topic, Float64MultiArray,
                    self._on_normal_rot, queue_size=1)
            except Exception as exc:                 # noqa: BLE001
                rospy.logwarn(
                    '[pallet_detection_v2] 订阅法向视角旋转 %s 失败（%s）—— '
                    '换边后仍用 call 0 的原始法向',
                    self.pallet_normal_rot_topic, exc)
        self.long_side_parallel = self._as_bool(
            rospy.get_param('~long_side_parallel', False))
        # 法向：`~normal` 给 "x,y,z" 时**直接用，不查 TF**（真机 TF 没起来时能跑，
        # 也是测试路径）；给空串 = **从 TF 查**（部署默认）。
        self.normal_param = self._parse_normal(rospy.get_param('~normal', ''))
        # ---- 模型路径与推理参数 ----------------------------------------------
        # 模型留空 = 包内默认；缺失/加载失败在启动时抛。
        #
        # `~infer_backend`（2026-10-20）：
        #   auto   —— engine 文件在就用 TensorRT，不在就用 ONNX（默认）
        #   engine —— 强制 engine；不存在直接报错
        #   onnx   —— 强制 ONNX（旧的 onnxruntime 路径）
        # engine 是**预先导出好的静态文件**，启动不编译；比 ONNX CUDA EP 快且
        # 首帧没有 ~19s 的 cuDNN 选核。换机 / 换 TensorRT 版本要重导。
        self.infer_backend = str(
            rospy.get_param('~infer_backend', 'auto') or 'auto').strip().lower()
        if self.infer_backend not in ('auto', 'engine', 'onnx'):
            raise SystemExit(
                f"[pallet_detection_v2] ~infer_backend={self.infer_backend!r} 不认识；"
                f"允许值只有 'auto' / 'engine' / 'onnx'")
        self.imgsz = int(rospy.get_param('~imgsz', 640))
        self.nms_iou = float(rospy.get_param('~nms_iou', 0.45))
        self.pose_model_path = str(rospy.get_param('~pose_model_path', '') or '')
        self.pose_engine_path = str(rospy.get_param('~pose_engine_path', '') or '')
        self.pose_conf = float(rospy.get_param('~pose_conf', 0.25))
        self.pose_kconf = float(rospy.get_param('~pose_kconf', 0.5))
        self.pose_pick = str(rospy.get_param('~pose_pick', 'bottom_center'))
        # 法向自检：**TF 只当搜索初值，实际法向从当前帧深度拟合**（默认开）。
        # 现场事实（用户确认）：TF 链是通的，但 base_link 的 z 与托盘实际台面法向
        # 会有 **0~10°** 的静态偏差；而托盘解算把法向**锁死**成先验，
        # 偏了没有任何下游步骤会纠正它 —— 必须靠这一步纠偏（实测 12.4° 偏差下
        # 旧路线 too_few_edges 37/37 全拒，拟合后 37/37 全过）。
        # 关掉 = 只用 TF 先验（0~10° 会直接进位姿）。
        self.fit_normal = self._as_bool(rospy.get_param('~fit_normal', True))
        # 重新拟合：false（默认）= **只在首帧拟合一次，之后一直复用**。
        # ⚠️ 依据：伺服时机器人只做平面内平移（不动腰），相机相对 base 的朝向不变。
        # 真动了腰 / 换了工位要设成 true，否则拿过期法向静默继续算。
        self.normal_refit = self._as_bool(rospy.get_param('~normal_refit', False))
        # 首帧拟合的**合理性自检**（2026-10-22 现场事故）：那次 TF 初值本身
        # 歪到侧向，拟合结果与初值只差 0.05°、内点 487，却与相机光轴差 69.8°，
        # 于是被缓存 -> 之后每帧 D=241mm 被闸门全拒（pub=0，看起来像"检测坏了"）。
        # 两道闸见 `normals.normal_is_plausible`；不合格的拟合**绝不缓存**，
        # 并用**相机光轴**当初值重试一次（托盘台面必在俯视相机光轴附近）。
        # 0 = 关掉对应闸门。
        try:
            self.normal_max_tilt_deg = float(
                rospy.get_param('~normal_max_tilt_deg', 60.0))
        except (TypeError, ValueError):
            self.normal_max_tilt_deg = 60.0
        # POSE session：**启动时建一次；模型缺失/加载失败直接抛**（见 _build_session）
        self._sess = self._build_session()
        self.camera_frame = str(rospy.get_param('~camera_frame',
                                                'camera_color_optical_frame'))
        self.base_frame = str(rospy.get_param('~base_frame', 'base_link'))
        # 心跳周期（秒）。**必须有一个"成功时也打"的日志** —— 本节点其余的日志点
        # 全是一次性/只在失败时，正常工作时一片安静，与挂掉长得一模一样。
        # 0（或负）= 关掉。
        try:
            self.heartbeat_s = float(rospy.get_param('~heartbeat_s', 5.0))
        except (TypeError, ValueError):
            self.heartbeat_s = 5.0

        # ---- 检测门控（2026-10-18）------------------------------------------
        # 现场流程是"先抓箱、到位后才 call slot"，码放之前不一定看得见托盘。
        # `~gated=true` 时：**空闲不算、不发布**，只在服务器的门控话题说
        # "这一档要托盘"（`mode=2`）之后才算；但启动时**先跑一帧预热**
        # （首次检测实测 3.5s，含法向拟合与 ONNX 初始化），不能等 call 了
        # 服务才付这笔钱。
        # 默认 **false** = 一直算，旧的独立 roslaunch 跑法行为不变。
        self.gated = bool(rospy.get_param('~gated', False))
        self.detect_mode_topic = str(rospy.get_param(
            '~detect_mode_topic', '/pallet_servo_v2/detect_mode')).strip()
        self._gated_active = not self.gated
        # 未门控时按 mode=2（双路）处理交错；门控待命为 0。
        self._detect_mode = 0 if self.gated else 2
        self._warmed = False
        if self.gated:
            from std_msgs.msg import Int32
            self._mode_sub = rospy.Subscriber(
                self.detect_mode_topic, Int32, self._on_detect_mode,
                queue_size=1)
        # ---- D 只从 call 0 记录来 -----------------------------------------
        # 托盘一开始观测一次，D/法向写文件；之后检测只读文件记录。没有记录时
        # POSE 的 D 由 12 关键点四边几何自由解，深度只用于法向拟合。
        self._record_state = self._record is not None
        if self._record_state:
            rospy.loginfo(
                '[pallet_detection_v2] 有 call 0 记录（normal + D）—— '
                '位姿直接用记录里的 D/法向；不重算 D、不做 D 闸门')
        else:
            rospy.logwarn(
                '[pallet_detection_v2] 没有 call 0 记录 —— 本次 D 由 POSE 关键点'
                '自由解；建议先在观测位姿 call slot 0 记录托盘 D/法向')

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
        self._n_pub_reject = 0         # 其中以 `valid=false` 发布出去的诊断帧数
        self._last_reject = None        # 最近一次拒绝的 `format_rejects` 原文
        # 逐帧 POSE 摘要的日志门：状态（选边/拒绝码/D 档/法向来源）变化时 INFO，
        # 稳态时只打 DEBUG（每 5s 最多一条）。原来 0.5s 一条 INFO，是现场最大噪声源。
        self._last_pose_summary_key = None
        self._last_pose_summary_t = 0.0
        self._last_D_mm = None          # 最近一次记录/D 诊断值（心跳里显示；不参与拒绝）
        # 最近一帧实际用的法向（心跳里显示）—— 2026-10-22 事故的教训：
        # 只打首帧那一次的话，"法向被缓存成侧向、D 全被拒"在心跳里只剩一个
        # 孤零零的 D=241mm，看不出**为什么**。三个量都进心跳。
        self._last_normal = None
        self._last_normal_src = None
        self._last_normal_tilt_deg = None
        self._last_good_ms = None       # 最近一次成功那帧的耗时（**整帧**）
        self._last_detect_ms = None     # 其中 detect 段（输入转换 + 法向 + 推理 + 解算）
        # detect 段内部分项（2026-11-16）：预处理 / 推理 / 后处理算法。
        # 由 detect_pose 的 diag 提供，节点只负责搬运到心跳里。
        self._last_stage_ms = {}
        self._last_pub_wall = None      # 最近一次发布的墙钟时刻
        # 诊断：最近输入帧 / 已发布帧的图像时间戳（配对排查用）
        self._last_frame_stamp = None
        self._last_pub_stamp = None
        self._last_heartbeat = 0.0
        self._last_warn = 0.0
        self._last_tf_warn = 0.0
        self._normal_checked = False
        self._normal_fit = None            # 缓存的自检法向（只在工作线程读写）
        self._normal_fit_log = None        # 首帧自检结果的一行说明，写进日志
        # ★ **最后一次成功解析出来的 TF 法向**（2026-11-03 加）：TF 彻底查不到时
        #   用它兜底，而不是直接丢帧。与 `_normal_fit` 不是一回事 —— 后者是经深度
        #   拟合 + 合理性自检的答案，前者只是 TF 原始先验。只在 TF 失败时用。
        self._last_normal_prior = None

        self.tf_buffer = None
        self.tf_listener = None
        # TF 自愈（2026-11-02）：TCPROS 流坏掉后 `tf2_ros` 的订阅线程会死，
        # 旧的 Buffer 从此永远查不到新 TF。记录上次成功时刻，失败时**限频重建**
        # Buffer + TransformListener（不重建就得重启进程）。
        self._last_tf_ok_mono = None
        self._last_tf_rebuild_mono = 0.0
        self._tf_rebuild_min_s = 5.0
        self._worker_restarts = 0
        if self.normal_param is None:
            self.tf_buffer = ros['tf2_ros'].Buffer()
            self.tf_listener = ros['tf2_ros'].TransformListener(self.tf_buffer)

        self._info_sub = rospy.Subscriber(self.info_topic, ros['CameraInfo'],
                                          self._on_camera_info, queue_size=1)
        # ⚠️⚠️ **`buff_size` 必须显式给大**（2026-10-15 现场排查帧率时发现的）。
        # `message_filters.Subscriber` 把参数原样转给 `rospy.Subscriber`，而
        # **rospy 的默认 `buff_size` 只有 65536** —— 一条 640×480 的彩色图是
        # **921KB**，用 64KB 的缓冲去收要分 ~14 次读，每次都在 Python 层，
        # 抢的还是 worker 线程的 GIL。
        #
        # 实测代价（Orin AGX，同一份数据、同一个算法，只改喂帧率）：
        #     相机 6Hz  → 单帧 177ms
        #     相机 30Hz → 单帧 290ms（**慢 1.64 倍**），而且偶发 5~6 秒的停顿
        # 离线逐帧扫 816 帧、最慢一帧只要 202ms —— 说明**数据没问题，是收图的开销**。
        #
        # 仓库里**别的收图节点都设了**（`carton_box_yolo` / `node_pallet_servo`
        # 的 `queue_size=1, buff_size=2**24, tcp_nodelay=True` / `wrist_visual_servo`
        # 的 `buff_size=8MB`），**只有本节点漏了**。
        # `queue_size=1`：图像话题只要最新的一帧；不给的话 rospy 会退回**无界队列**。
        subs = [ros['Subscriber'](self.image_color, ros['Image'], queue_size=1,
                                  buff_size=2 ** 24, tcp_nodelay=True),
                ros['Subscriber'](self.image_depth, ros['Image'], queue_size=1,
                                  buff_size=2 ** 24, tcp_nodelay=True)]
        self.sync = ros['ApproximateTimeSynchronizer'](
            subs, queue_size=5, slop=self.sync_slop)
        self.sync.registerCallback(self._on_synced)

        import threading
        self._worker = threading.Thread(target=self._worker_main, daemon=True)
        self._worker.start()
        rospy.loginfo(f'[pallet_detection_v2] 订阅 {self.info_topic} + {self.image_color} '
                      f'+ {self.image_depth}  ->  发布 {self.out_topic}')
        # 日志落在哪个文件必须打出来：现场"日志里没东西"第一件要排除的，
        # 就是**根本没写进你翻的那个文件**（回退路径 / 别的目录 / 别的 tag）。
        try:
            from core.common.logger import get_active_log_file
            rospy.loginfo('[pallet_detection_v2] 日志文件：%s', get_active_log_file())
        except Exception:                            # noqa: BLE001
            pass                                     # 回退路径下没有它，不是错误
        rospy.loginfo('[pallet_detection_v2] 等第一帧 camera_info 建内参…')
        if self.normal_param is None:
            rospy.loginfo('[pallet_detection_v2] 法向来源 = TF（%s -> %s，%d 段链）'
                          '；缺任何一段都不发布',
                          self.base_frame, self.camera_frame, len(CHAIN))
        else:
            rospy.logwarn('[pallet_detection_v2] 法向来源 = 参数 ~normal=%s（**没查 TF**）。'
                          '部署时应给空串走 TF —— 参数法向是调试/无 TF 时的后路，'
                          '它的精度直接决定 detect 的初值质量'
                          '（差 5° 时 detect 单独偏 ~180mm）',
                          np.round(self.normal_param, 4).tolist())

    def _on_pallet_size(self, msg):
        """伺服节点 call 0 的 latched 尺寸话题：只更新 target_mm。

        记录重读由 `_on_record_epoch` 负责 —— 尺寸可能测不出来（话题不发），
        不能把"尺寸变化"当成"记录更新"的唯一信号。
        """
        try:
            data = [float(v) for v in msg.data]
        except (TypeError, ValueError):
            return
        if len(data) != 2 or not all(np.isfinite(v) and v > 0.0 for v in data):
            return
        new = (data[0], data[1])
        if new == tuple(self.target_mm):
            return
        self.target_mm = new
        self._ros['rospy'].loginfo(
            '[pallet_detection_v2] target_mm 改用伺服 call 0 实测尺寸：%s',
            list(new))

    def _on_record_epoch(self, msg):
        """call 0 写记录成功后的版本通知：重读整条记录。"""
        try:
            epoch = int(msg.data)
        except (TypeError, ValueError):
            return
        if epoch == self._last_record_epoch:
            return
        self._last_record_epoch = epoch
        self._apply_pallet_record()
        # 记录有无只决定 D/法向从哪里来；不再有逐帧 D 校验。
        # 这里必须跟着 `_apply_pallet_record()` 的整条替换一起刷新。
        self._ros['rospy'].loginfo(
            '[pallet_detection_v2] 收到记录版本 %d —— 已重读 %s（记录%s）',
            epoch,
            self.pallet_record_path or
            str(_REPO_ROOT / 'log' / 'pallet_servo_pallet_record.json'),
            '有效' if self._record_state else '无效')

    def _on_normal_rot(self, msg):
        """伺服切档时的法向视角旋转 `[roll, pitch, yaw]`（度）。"""
        try:
            rot = tuple(float(v) for v in msg.data)
        except (TypeError, ValueError):
            return
        if len(rot) != 3 or not all(math.isfinite(v) for v in rot):
            return
        if rot == self._normal_rot_deg:
            return
        self._normal_rot_deg = rot
        # 缓存的是"记录法向 → 相机系"的结果，旋转量变了必须作废。
        for rec in (self._record, self._frame_record):
            if isinstance(rec, dict):
                rec['cam'] = None
        self._ros['rospy'].loginfo(
            '[pallet_detection_v2] 法向视角旋转 [roll=%.1f°, pitch=%.1f°, yaw=%.1f°]'
            ' —— 已作废相机系法向缓存', rot[0], rot[1], rot[2])

    def _apply_pallet_record(self):
        """从 call 0 记录里加载尺寸 / 法向 / D（跨开关机保留）。

        * `pallet_size_mm` → 覆盖 `target_mm`；
        * `normal`（base_link 系单位向量）与 `D_mm`（base_link 原点到台面的
          有符号法向距离，`D = n·P0`）→ **整条记录一次替换**，供每帧换算到
          相机系。旧记录没有 `D_mm` 时退回 `plane_z_mm`（台面过 `(0,0,z)`）。

        ⚠️ **不允许半新半旧**：新文件缺 normal，或 D_mm / plane_z_mm 都缺或
        字段损坏时，整条记录作废并清空内存里的旧记录（回到 TF + 深度自检），
        绝不出现"新 D + 旧法向"。
        记录不存在 / 读坏时也清空旧内存记录，不阻断启动。
        """
        path = Path(self.pallet_record_path).expanduser() if self.pallet_record_path else (
            _REPO_ROOT / 'log' / 'pallet_servo_pallet_record.json')
        if not path.is_file():
            if self._record is not None:
                self._ros['rospy'].logwarn(
                    '[pallet_detection_v2] 托盘记录 %s 不存在 —— 清空内存记录，'
                    '回到 TF + 深度自检', path)
            self._record = None
            self._record_state = False
            return
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                record = json.load(stream)
        except Exception as exc:                     # noqa: BLE001
            self._ros['rospy'].logwarn(
                '[pallet_detection_v2] 读取托盘记录 %s 失败（%s）—— '
                '清空内存记录，回到 TF + 深度自检', path, exc)
            self._record = None
            self._record_state = False
            return

        raw_size = record.get('pallet_size_mm')
        if raw_size is not None:
            try:
                size = _parse_target_mm(raw_size)
                self.target_mm = size
                self._ros['rospy'].loginfo(
                    '[pallet_detection_v2] target_mm 改用 call 0 记录：%s（%s）',
                    list(size), path)
            except Exception as exc:                 # noqa: BLE001
                self._ros['rospy'].logwarn(
                    '[pallet_detection_v2] 记录里的 pallet_size_mm 不可用（%s）'
                    '—— 继续用 target_mm=%s', exc, list(self.target_mm))

        base_frame = str(record.get('base_frame') or 'base_link')
        normal = None
        raw_normal = record.get('normal')
        if raw_normal is not None:
            try:
                n = np.asarray(raw_normal, np.float64).reshape(3)
                norm = float(np.linalg.norm(n))
                if not np.all(np.isfinite(n)) or norm < 1e-9:
                    raise ValueError(f'normal={raw_normal!r}')
                normal = n / norm
            except (TypeError, ValueError) as exc:
                self._ros['rospy'].logwarn(
                    '[pallet_detection_v2] 记录里的 normal 不可用（%s）', exc)
        raw_z = record.get('plane_z_mm')
        if raw_z is None:
            # 兼容旧记录：旧文件只有 origin_mm[2]，且它可能是深度主平面锁出来
            # 的下层高度。能读，但要明确告警，现场必须重新 call 0。
            raw_origin = record.get('origin_mm')
            if isinstance(raw_origin, (list, tuple)) and len(raw_origin) >= 3:
                raw_z = raw_origin[2]
                self._ros['rospy'].logwarn(
                    '[pallet_detection_v2] 记录 %s 没有 plane_z_mm，退回 origin_mm.z=%s'
                    ' —— 旧记录的高度可能是深度主平面（下层/地面），'
                    '**请重新 call 0 观测托盘**', path, raw_z)
        z_mm = None
        if raw_z is not None:
            try:
                z = float(raw_z)
                if not np.isfinite(z):
                    raise ValueError(f'plane_z_mm={raw_z!r}')
                z_mm = z
            except (TypeError, ValueError) as exc:
                self._ros['rospy'].logwarn(
                    '[pallet_detection_v2] 记录里的 plane_z_mm 不可用（%s）', exc)

        # D_mm 是 base_link 原点到台面的**有符号法向距离**：D = n·P0
        # （P0 为台面上任意一点，n 已翻到从 base 指向台面，故 D>0）。
        # 新记录优先用它；plane_z_mm 仅在旧记录里没有 D_mm 时兜底。
        raw_d = record.get('D_mm')
        d_mm = None
        if raw_d is not None:
            try:
                d = float(raw_d)
                if np.isfinite(d) and d > 0.0:
                    d_mm = d
            except (TypeError, ValueError):
                pass

        if normal is None or (d_mm is None and z_mm is None):
            self._record = None
            self._record_state = False
            self._ros['rospy'].logwarn(
                '[pallet_detection_v2] 记录 %s 缺少/损坏 normal 或 D_mm/plane_z_mm —— '
                '**整条作废**，回到 TF + 深度自检', path)
            return
        # 一次性替换整条记录（normal/plane_z/D/base_frame/cam 同源）。
        self._record = {
            'normal': normal,
            'plane_z_mm': z_mm,    # 兼容/诊断；法向倾斜时不是真正的平面距离
            'D_mm': d_mm,          # 新口径：base 原点到台面的法向距离
            'base_frame': base_frame,
            'cam': None,
        }
        self._record_state = True
        if d_mm is not None:
            self._ros['rospy'].loginfo(
                '[pallet_detection_v2] 使用 call 0 记录：normal=%s D=%.1fmm '
                'base_frame=%s（%s）—— 平面高度用法向距离 D，plane_z=%s',
                np.round(normal, 4).tolist(), d_mm, base_frame, path,
                ('%.1fmm' % z_mm) if z_mm is not None else '无')
        else:
            # 旧记录只有 plane_z_mm：能跑，但法向有 x/y 分量时 D 会偏。
            self._ros['rospy'].logwarn(
                '[pallet_detection_v2] 旧记录 %s 只有 plane_z_mm=%.1f，没有 D_mm —— '
                '退回 plane_z 口径（法向倾斜时平面距离会有偏差），建议重新 call 0',
                path, z_mm)

    def _record_cam_state(self, record=None, stamp=None):
        """把记录的 base_link 法向 / 平面高度 z 变到相机系。

        * **距离口径**：优先用记录里的 `D_mm`（base 原点到台面的有符号法向
          距离，`D = n·P0`）。法向按 `normal_rot` 旋转，D 是标量、保持不变。
          旧记录没有 `D_mm` 时才退回 `plane_z_mm` 口径（台面过 `(0,0,z)`）。
        * **每帧用当前 TF**：call 0 是观测位姿，伺服时上半身角度可以不同；
          base 系的法向/z 不变，但“相机系里在哪”必须按当前 TF 重算。缓存只在
          同一图像 stamp 内复用，stamp 变了就重新拼 TF —— 绝不能把观测位姿的
          相机系平面一直吃到伺服。

        返回 `(normal_cam, D_cam_mm)` 或 `None`（还没解析出来）。
        """
        rec = self._record if record is None else record
        if rec is None:
            return None
        # stamp 只用于同一帧内复用；没有 stamp 时按旧行为返回已有缓存。
        try:
            key = (float(stamp.to_sec()) if hasattr(stamp, 'to_sec')
                   else float(stamp)) if stamp is not None else None
        except (TypeError, ValueError):
            key = None
        cached = rec.get('cam')
        if cached is not None:
            if len(cached) == 2:
                return cached
            if len(cached) == 3 and key is not None and cached[0] == key:
                return cached[1], cached[2]
        if self.tf_buffer is None:
            return None
        rec_base = rec.get('base_frame')
        if rec_base is not None and rec_base != self.base_frame:
            # 只记录、不按帧名拒绝：transform_from_tf 走固定的 13 段链。
            self._ros['rospy'].logwarn_throttle(
                5.0, '[pallet_detection_v2] 记录 base_frame=%s，节点 ~base_frame=%s '
                     '不一致 —— 仍按 TF 链换算，结果可能偏（检查部署配置）',
                rec_base, self.base_frame)
        try:
            missing: list = []
            shifted: list = []
            T_base_cam = transform_from_tf(
                self.tf_buffer, self.base_frame, self.camera_frame,
                stamp=stamp, missing=missing, shifted=shifted)
            if shifted and T_base_cam is not None:
                self._ros['rospy'].logwarn_throttle(
                    5.0, '[pallet_detection_v2] 记录换算：图像 stamp 无 TF 样本，'
                         '整条链改用 stamp-%.3fs（偏移 %.0fms）',
                    shifted[0], shifted[0] * 1000.0)
        except Exception as exc:                     # noqa: BLE001
            T_base_cam = None
            missing = [f'{type(exc).__name__}: {exc}']
        if T_base_cam is None:
            self._ros['rospy'].logwarn_throttle(
                5.0, '[pallet_detection_v2] 记录的法向/高度还变不到相机系'
                     '（TF 链缺边 %s）—— 暂用 TF + 深度自检；'
                     '注意：D/法向会改用 POSE 几何自由解', missing)
            return None
        R_base_cam = np.asarray(T_base_cam[:3, :3], np.float64)
        t_base_cam = np.asarray(T_base_cam[:3, 3], np.float64)
        R_cam_base = R_base_cam.T

        n_base = self._rotate_normal_base(
            np.asarray(rec['normal'], np.float64), self._normal_rot_deg)
        n_cam = R_cam_base @ n_base
        n_norm = float(np.linalg.norm(n_cam))
        if not np.isfinite(n_norm) or n_norm < 1e-9:
            return None
        n_cam = n_cam / n_norm

        d_mm = rec.get('D_mm')
        z_mm = rec.get('plane_z_mm')
        if d_mm is not None:
            # D 是 base 原点到台面的有符号法向距离（mm）；相机原点在 base 系
            # 的位置是 t_base_cam（m）。把平面 n·p = D 换到相机系：
            #   dot = D_m - n_base·t_base_cam
            # n_base 必须用 normal_rot 之后的法向；D 标量不随 base 旋转变化。
            dot = float(d_mm) * 0.001 - float(n_base @ t_base_cam)
        elif z_mm is not None:
            # 旧记录兼容：平面高度 z，base 下台面过 (0,0,z)。plane_point 的
            # 面内分量取 0（记录里不出现面内位姿）。
            plane_point_base = np.array([0.0, 0.0, float(z_mm) * 0.001],
                                        np.float64)
            origin_cam = R_cam_base @ (plane_point_base - t_base_cam)
            dot = float(n_cam @ origin_cam)
        else:
            return None
        if not np.isfinite(dot) or abs(dot) < 1e-6:
            return None
        # 检测器的 `normal` 约定是**从台面指向相机**（相机系 z<0）。
        if dot > 0.0:              # n_cam 从相机指向台面 -> 翻过来
            n_cam = -n_cam
        D_cam = abs(dot) * 1000.0
        if not np.isfinite(D_cam) or D_cam <= 0.0:
            self._ros['rospy'].logwarn_throttle(
                5.0, '[pallet_detection_v2] 记录换算出的相机系 D=%.1fmm 不合理'
                     ' —— 暂用 TF + 深度自检；没有记录时 D 由 POSE 几何自由解', D_cam)
            return None
        if key is None:
            cam = (n_cam, D_cam)
            rec['cam'] = cam
            if not rec.get('_logged'):
                rec['_logged'] = True
                self._ros['rospy'].loginfo(
                    '[pallet_detection_v2] call 0 记录已换算到相机系：normal=%s '
                    'D=%.1fmm（距离来源=%s base_frame=%s）',
                    np.round(n_cam, 4).tolist(), D_cam,
                    ('D_mm' if d_mm is not None
                     else 'plane_z=%.1fmm（旧记录兼容）' % float(z_mm)),
                    rec_base)
            return cam
        rec['cam'] = (key, n_cam, D_cam)
        if not rec.get('_logged'):
            rec['_logged'] = True
            self._ros['rospy'].loginfo(
                '[pallet_detection_v2] call 0 记录已按当前 TF 换算到相机系：'
                'normal=%s D=%.1fmm（plane_z=%s base_frame=%s）',
                np.round(n_cam, 4).tolist(), D_cam,
                ('%.1fmm' % float(z_mm)) if z_mm is not None else '旧D口径',
                rec_base)
        return n_cam, D_cam

    # ---- 参数工具 -------------------------------------------------------
    @staticmethod
    def _rotate_normal_base(n, rot_deg):
        """把记录法向从 call 0 的 base_link(B0) 转到当前 base_link(B1)。

        `rot_deg = [roll, pitch, yaw]`（度），应用顺序 `Rz @ Ry @ Rx`。
        全 0 / 非法值 → 原样返回（不阻断）。**俯仰/横滚同样改变法向**，
        不能只处理 yaw。
        """
        try:
            roll, pitch, yaw = (math.radians(float(v)) for v in rot_deg)
        except (TypeError, ValueError):
            return n
        if not (math.isfinite(roll) and math.isfinite(pitch)
                and math.isfinite(yaw)):
            return n
        if roll == 0.0 and pitch == 0.0 and yaw == 0.0:
            return n
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        return Rz @ Ry @ Rx @ np.asarray(n, np.float64)

    @staticmethod
    def _normal_tilt_deg(normal):
        """法向与**相机光轴**的夹角（度）；拿不到合法法向 -> `None`。

        俯视相机的托盘台面正常应接近 `-z`（现场实测 15.6°）。这个角度进日志/心跳
        —— 2026-10-22 事故里法向被缓存成 **69.8°（侧向）**，但日志只打首帧一次，
        心跳里只剩 `D=241mm`，肉眼看不出是法向歪了。
        """
        if normal is None:
            return None
        n = np.asarray(normal, float)
        if n.shape != (3,) or not np.all(np.isfinite(n)):
            return None
        norm = float(np.linalg.norm(n))
        if norm < 1e-9:
            return None
        c = abs(float(n[2] / norm))
        return float(np.degrees(np.arccos(np.clip(c, 0.0, 1.0))))

    def _normal_tilt_is_suspicious(self, tilt_deg) -> bool:
        """倾角超过自检阈值（阈值 <= 0 = 不做这项检查）就标异常。"""
        if tilt_deg is None or self.normal_max_tilt_deg <= 0.0:
            return False
        return tilt_deg > self.normal_max_tilt_deg

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

    # ---- 内参 -----------------------------------------------------------
    def _on_camera_info(self, msg):
        if self.k is not None:
            return
        K = list(msg.K)
        self.k = CameraIntrinsics(float(K[0]), float(K[4]), float(K[2]), float(K[5]))
        self._info_sub.unregister()          # 内参不变，收一帧就够
        self._ros['rospy'].loginfo(
            f'[pallet_detection_v2] 内参 fx={self.k.fx:.2f} fy={self.k.fy:.2f} '
            f'cx={self.k.cx:.2f} cy={self.k.cy:.2f}')

    # ---- 法向 -----------------------------------------------------------
    def _resolve_normal(self, color_msg):
        """本帧用的台面法向 + 来源标签。查不到返回 `(None, 原因)` —— **不回退**。

        ★ **过期 TF 的宽容**（2026-11-03）：`normal_from_tf` 先按图像 stamp 拼
        整条 13 段链；任一段缺就**整条**退回最新值重拼。退回成功时来源标签是
        `tf_stale`（不是 `tf`），并打一条节流 WARN —— 现场必须能看出"这一帧的
        法向不是按图像时刻取的"，否则机器人运动时会把过期关节位姿当成当前位姿。
        整条链都拼不出来时返回 `None`，由 `_normal_for_frame` 决定要不要用
        上一次成功的 TF 先验兜底。
        """
        if self.normal_param is not None:
            return self.normal_param, 'param'
        missing: list = []
        stale: list = []
        stamp = color_msg.header.stamp
        shifted: list = []
        n = normal_from_tf(self.tf_buffer, self.camera_frame,
                           source_frame=self.base_frame, stamp=stamp,
                           missing=missing, stale=stale, shifted=shifted)
        if n is None:
            # ★ 自愈：TF 的订阅线程可能已经死了（TCPROS 坏流），重建一次。
            self._maybe_rebuild_tf(f'TF 链缺边 {missing}')
            return None, f'TF 链缺边 {missing}'
        self._last_tf_ok_mono = time.monotonic()
        self._last_normal_prior = np.asarray(n, float).copy()
        if stale:
            self._ros['rospy'].logwarn_throttle(
                5.0, '[pallet_detection_v2] TF 按图像 stamp=%.3f 拼不齐，'
                     '**整条链退回最新值**（法向来源=tf_stale）；'
                     '按图像时刻缺的边=%s；TF 订阅可能已停更，下一次心跳会继续检查',
                float(stale[0]), missing)
            return n, 'tf_stale'
        if shifted:
            self._ros['rospy'].logwarn_throttle(
                5.0, '[pallet_detection_v2] TF 图像 stamp=%.3f 无样本，'
                     '整条链改用 stamp-%.3fs（偏移 %.0fms，缺边=%s）',
                float(stamp.to_sec()) if hasattr(stamp, 'to_sec') else 0.0,
                shifted[0], shifted[0] * 1000.0, missing)
            return n, 'tf_shift'
        return n, 'tf'

    def _normal_for_frame(self, color_msg, depth) -> tuple:
        """本帧实际用的法向 + 来源标签。**TF 只当搜索初值，答案从深度拟合。**

        返回 `(normal | None, 来源)`；`None` = 连初值都没有，调用方拒帧。

        ## call 0 记录优先（2026-11-03）

        记录里有 base_link 系法向与 D 时，直接换算到相机系并缓存（来源标签
        `record`），**不查 TF、不拟合、不量 D** —— 这两个量在 call 0 之后是
        常量，每帧重算只会把噪声带进位姿。记录只在第一次跑（还没 call 0）时
        不存在，那时才走下面的 TF + 深度自检。

        ## 法向是两级：TF 先验 + 深度自检（没有记录时）

        TF 链是通的，但 base_link 的 z 轴与托盘实际台面法向**会有 0~10° 的偏差**，
        所以必须用当前帧深度在它附近做一次拟合来纠正。这不是可选项：托盘解算把
        法向**锁死**成先验（6 自由度 → 4），偏了没有任何下游步骤会
        把它纠回来 —— 那 0~10° 会**直接进位姿**。

        实测（2026-09-29，test_jia4 37 帧）：地面/台面平面之间只差 1.14°，而
        **TF 与它们差 12.52°**，且 37 帧里 TF 一个数都没变（静态偏差，不是抖动）。
        偏 12.4° 时 `too_few_edges` **37/37 全拒**；换成拟合值、其它参数不动
        **37/37 全过**。

        ## 缓存（`~normal_refit`）

        ⚠️ **伺服时机器人只做平面内平移**（前后左右，不动腰），所以相机相对 base
        的朝向不变 —— 法向是这个过程中的**常量**。首帧花 ~10ms 拟合一次就够，
        之后每帧复用。缓存的**前提是"朝向不变"**，真动了腰（或换了工位）要把
        `~normal_refit` 设成 true 重新拟合，否则会拿过期法向继续算（**静默**）。

        ## 缓存的前提（2026-10-22 现场事故后加）

        **只有通过合理性自检的拟合才允许缓存**（`fit_floor_normal_checked`）：
        与相机光轴的夹角、拟合平面的距离都要说得通。首帧在机械臂姿态最不利时
        拟歪过一次（TF 初值本身侧向，拟合与初值只差 0.05°），旧代码照样缓存，
        于是之后每帧 D=241mm 被闸门全拒、一帧都不发 —— 只有重启才能恢复。
        不合格时**不写缓存**（下一帧继续重试），并改用户相机光轴当初值再拟一次。

        ## 过期 TF 的兜底（2026-11-03）

        已经有一份**通过合理性自检**的拟合法向、且 `normal_refit=false` 时，
        **先返回缓存、根本不去查 TF**。伺服过程中法向是常量，TF 停更/过期不应该
        让整条链一帧都出不来（旧顺序是先查 TF、失败就 `return None`，缓存形同
        虚设）。没有缓存（首帧或 `normal_refit=true`）时如果 TF 彻底查不到，
        再用**上一次成功解析的 TF 先验** `_last_normal_prior` 兜底，来源记
        `tf_last`，并打节流 WARN；仍然没有才返回 `None`。这条兜底只影响"用哪个
        初值"，后面的深度拟合与合理性自检一步都不少。
        """
        if self.normal_param is not None:
            return self.normal_param, 'param'
        if not self._ignore_record:
            # call 0 记录优先：法向/高度已经在 base 下定过；每帧按当前 TF
            # （图像 stamp）换到相机系，不再拟合法向 / 量深度 D。
            rec = getattr(self, '_frame_record', None)
            if rec is None:
                rec = getattr(self, '_record', None)
            stamp = getattr(getattr(color_msg, 'header', None), 'stamp', None)
            cached = self._record_cam_state(rec, stamp=stamp)
            if cached is not None:
                return cached[0], 'record'
            # ★ 缓存优先：法向是常量时，TF 停更不该拖垮已缓存的答案。
            if self._normal_fit is not None and not self.normal_refit:
                return self._normal_fit, 'cache'
        # 观测模式（call 0）：**不吃旧记录，也不吃旧拟合缓存** —— 这一趟就是
        # 在重新测托盘，旧值可能属于上一个工位。继续走 TF + 深度拟合。
        prior, why = self._resolve_normal(color_msg)
        if prior is None:
            # ★ 过期 TF 兜底：用最后一次成功解析的 TF 先验，而不是丢帧。
            if self._last_normal_prior is not None:
                self._normal_fit_log = (
                    f'TF 查不到（{why}）-> 用上一次 TF 先验兜底（tf_last）')
                self._ros['rospy'].logwarn_throttle(
                    5.0, '[pallet_detection_v2] 本帧 TF 查不到（%s）—— '
                         '**用上一次成功解析的 TF 先验兜底**（法向来源=tf_last）；'
                         '深度拟合与合理性自检照常执行', why)
                prior = np.asarray(self._last_normal_prior, float).copy()
                why = 'tf_last'
            else:
                return None, why
        if not self.fit_normal:
            return prior, why
        t0 = time.perf_counter()
        fitted, n_inl, plane_mm, check_why = fit_floor_normal_checked(
            depth, self.k, prior,
            max_tilt_deg=self.normal_max_tilt_deg)
        ms = (time.perf_counter() - t0) * 1000.0
        if fitted is None:
            # **回退策略 (A)**：拟合不合格就用 TF 的初值，而不是拒帧 ——
            # 初值本身在夹角窗内（窗是 ±25°），它只是"没被纠正"，不是"错的"。
            # ⚠️ **不写 `_normal_fit`**：这一帧没得到一个可信的拟合，缓存它
            # 就等于把"首帧拟歪永久生效"那条老路再走一遍。
            self._normal_fit_log = (f'拟合不合格（{check_why}）-> 回退 TF 初值 '
                                    f'（不缓存，下一帧重试） {ms:.0f}ms')
            self._ros['rospy'].logwarn_throttle(
                5.0, f'[pallet_detection_v2] 法向拟合不合格：{check_why} —— '
                     f'本帧退回 TF 初值（**不缓存**，下一帧继续重试）')
            # 把"这个初值本身是不是过期 TF"一起带出去：`tf_stale_bad_fit` /
            # `tf_last_bad_fit` 与普通的 `tf_bad_fit` 在现场是两件事。
            return prior, ('tf_bad_fit' if why == 'tf'
                           else f'{why}_bad_fit')
        ang = float(np.degrees(np.arccos(np.clip(abs(float(fitted @ prior)), -1, 1))))
        self._normal_fit = fitted
        extra = f'；{check_why}' if check_why else ''
        if check_why:
            self._ros['rospy'].loginfo_throttle(
                5.0, f'[pallet_detection_v2] 法向拟合自检：{check_why}'
                     f'（已采用重拟合结果并缓存）')
        self._normal_fit_log = (f'与 TF 初值差 {ang:.2f}°（内点 {n_inl}，'
                                f'平面 {plane_mm:.0f}mm，{ms:.0f}ms{extra}）')
        # 来源标签保留"初值从哪来"：`fit_stale` = 用最新 TF 纠正后的拟合，
        # `fit_tf_last` = 用上一次 TF 先验纠正后的拟合。不要一律写成 `fit`，
        # 否则心跳里看不出这一帧的 TF 已经停更。
        if why == 'tf':
            return fitted, 'fit'
        return fitted, f'fit_{why}'

    # ---- ONNX session（启动时建一次，**失败即抛**）--------------------------
    def _build_session(self):
        """按 `~infer_backend` 建 POSE 推理会话。**模型缺失 / 加载失败会让
        本节点起不来**（不是每帧静默失败，也没有第二条前端可回退）。

        后端选择与加载都在算法包里做（`detect_pose._resolve_pose_session`），
        本节点不再自己建 ORT session —— 否则两边会各写一份 provider/engine
        顺序，迟早静默分叉。
        """
        opts = dict(backend=self.infer_backend, imgsz=self.imgsz,
                    model_path=self.pose_model_path or str(DEFAULT_POSE_MODEL_PATH),
                    engine_path=self.pose_engine_path or str(DEFAULT_POSE_ENGINE_PATH))
        try:
            sess, src = _resolve_pose_session(opts)
        except Exception as exc:                 # noqa: BLE001
            raise SystemExit(
                f'[pallet_detection_v2] 托盘 POSE 模型不可用：{exc}\n'
                f'  backend={self.infer_backend}\n'
                f'  托盘检测**必须**有 POSE 模型（留空则用包内默认，'
                f'或用 ~pose_model_path / ~pose_engine_path 指定）。'
                f'导出见 pallet_detect/scripts/export_onnx_pose.py。') from exc

        self._ros['rospy'].loginfo(
            '[pallet_detection_v2] 后端=%s  模型=%s', self.infer_backend, src)
        if hasattr(sess, 'get_providers'):
            active = list(sess.get_providers())
            self._ros['rospy'].loginfo(
                '[pallet_detection_v2] onnxruntime providers=%s', active)
            if 'CUDAExecutionProvider' not in active:
                # 这里**只告警、不改环境**：ONNX Runtime 对 CUDA EP 缺库时会静默
                # 退回 CPU（症状是每帧慢一个数量级，日志里看不出来）。
                self._ros['rospy'].logwarn(
                    '[pallet_detection_v2] ⚠️ providers 里没有 CUDAExecutionProvider —— '
                    '可能已**静默退回 CPU**（每帧会慢一个数量级）。检查 '
                    'LD_LIBRARY_PATH、或用 ~infer_backend=engine。providers=%s', active)
        else:
            self._ros['rospy'].loginfo(
                '[pallet_detection_v2] TensorRT engine 输入=%s 输出=%s',
                sess.input_shape, sess.output_shape)
        return sess

    def _pose_detect_opts(self, stamp=None):
        """POSE 前端 `detect_pallet_frame_pose` 的 opts。

        2026-11-10 起 V2 **不再用当前帧深度锁 D**：
        * 有 call 0 记录：`fixed_d_mm` 用记录里的 base D 按当前 TF 换算到相机系，
          面内位姿仍由本帧关键点解；
        * 没有记录 / call 0 观测：`free_d=True`，D 只由 12 关键点四边 + 已知尺寸
          自由解。

        为什么去掉：深度主峰会选到箱顶/地面/木档（现场 D 实测从 625 到 1438mm
        都出现过），锁错层时即使关键点准确，3 自由度解也会被推到错误的平面/原点，
        表现为“点看着对、位姿飞了”。深度仍用于法向拟合和 D 诊断，但不再进入
        POSE 的 D 约束。
        """
        opts = dict(session=self._sess, imgsz=self.imgsz,
                    conf=self.pose_conf, nms_iou=self.nms_iou,
                    pick=self.pose_pick, kconf=self.pose_kconf,
                    d_lock=None,
                    free_d=True,
                    fit_normal=False)
        if not self._ignore_record:
            rec = getattr(self, '_frame_record', None)
            if rec is None:
                rec = getattr(self, '_record', None)
            cached = self._record_cam_state(rec, stamp=stamp)
            if cached is not None:
                # 记录里的 D 是 base_link 量、按当前 TF 换算到相机系；它不是
                # “当前帧深度主峰”，所以不属于这次要去掉的深度修正。
                opts['fixed_d_mm'] = cached[1]
                opts['free_d'] = False
        return opts

    def _pose_summary_key(self, diag, status, normal_source):
        """逐帧摘要的**去重键**：只留在排查时真正会变的粗粒度状态。

        ⚠️ **不要把 origin / 关键点坐标放进来**：它们是每帧都在动的小数，
        放进去等于每条都算"状态变了"，INFO 立刻回到 2Hz 刷屏。
        """
        def _tup(name):
            value = diag.get(name)
            return tuple(value) if isinstance(value, (list, tuple)) else value

        d_anchor = diag.get("d_anchor")
        try:
            d_anchor = round(float(d_anchor), -1)
        except (TypeError, ValueError):
            d_anchor = None
        return (
            str(status),
            str(diag.get("reject")),
            _tup("selected_edges"),
            _tup("visible_edges"),
            str(diag.get("edge_select_reason")),
            d_anchor,
            str(normal_source),
        )

    def _pose_summary_needed(self, key) -> bool:
        """只在**要打日志**时才构造 summary —— 它是字符串格式化 + numpy 投影，
        每帧无条件构造会白烧 CPU。"""
        if key != self._last_pose_summary_key:
            return True
        return time.time() - self._last_pose_summary_t >= 5.0

    def _log_pose_summary(self, summary, key):
        """状态变化 → INFO；稳态 → DEBUG（最多每 5s 一条）。"""
        now = time.time()
        # ⚠️ 这里**不许**用裸 `rospy`（2026-11-14 现场事故），必须经 `self._ros` 取。
        # 三个原因都是本文件既有的约定：
        #  1) 本文件**没有**模块级 `import rospy`：ROS 依赖由 `_ros()` 惰性加载、
        #     经 `self._ros['rospy']` 往下传（见本文件 :246-247 的注释），目的是让本模块
        #     在没有 ROS 的环境里也能被 import、跑离线自检（tests/ 下就是这种跑法）。
        #  2) `_run` / `_worker_main` / `_heartbeat` 三处都是先
        #     `rospy = self._ros['rospy']` 取到再用。本方法原来是在 `_run` 里内联的
        #     `rospy.loginfo_throttle(...)`；e1b64854（2026-11-13「日志降噪」）把它抽成
        #     独立方法时漏掉了 `_run` 里那个局部 `rospy`，于是每次打摘要都抛 NameError。
        #  3) 后果不是"线程死掉"，而是"每 1.4s 抽一次"：`_worker_main` 捕获后
        #     sleep(1.0) 重启 `_run`，检测被打成"跑 ~5 帧 + 停 1 秒"，出帧率 13/s→3.6/s，
        #     而 `input.pallet.age_s` 始终 < 1s —— 5s 的"检测器停摆"告警根本发现不了。
        # 也不要图省事改成模块顶 `import rospy`：`tests/test_pallet_detection_node.py:45`
        # 会在**没有 rospy** 的环境 import 本模块，一提就整份离线自检全跑不了。
        rospy = self._ros['rospy']
        if key != self._last_pose_summary_key:
            self._last_pose_summary_key = key
            self._last_pose_summary_t = now
            rospy.loginfo(summary)
            return
        if now - self._last_pose_summary_t >= 5.0:
            self._last_pose_summary_t = now
            rospy.logdebug(summary)

    def _pose_diag_summary(self, frame, diag, *, D_now=None, status='pose',
                           normal_source=None):
        """把一帧 POSE 的**选边/候选/深度/几何**压成一行日志。

        这是 2026-11-10 现场“关键点看着对、位姿却飞了”的排查补丁：
        原来的日志只有心跳和 `reject=`，看不到这一帧到底选了两条什么边、
        有几条可见边、D 从哪来、候选数是多少。没有这些字段，只能猜。
        """
        def _num(v, fmt='%.3f'):
            try:
                return fmt % float(v)
            except (TypeError, ValueError):
                return '?'

        def _seq(v):
            if v is None:
                return '-'
            try:
                return '[' + ','.join(_num(x, '%.1f') for x in v) + ']'
            except TypeError:
                return str(v)

        origin_m = None
        if frame is not None:
            try:
                origin_m = (np.asarray(frame.get('origin'), float) / 1000.0)
            except (TypeError, ValueError):
                origin_m = None
        origin_txt = _seq(origin_m)
        d_txt = '-' if D_now is None else f'{float(D_now):.1f}'
        # 关键点顺序 vs 解出的 frame origin：这是“点看着准、位姿翻到对角”
        # 的直接证据。frame origin 应投影到 12 点里的第 0 点（0/3/6/9 是角）。
        origin_kp_txt = ''
        corner_txt = ''
        if frame is not None and self.k is not None:
            kps = diag.get('keypoints_uv')
            try:
                o = np.asarray(frame.get('origin'), float) / 1000.0
                u = self.k.cx + self.k.fx * o[0] / max(o[2], 1e-6)
                v = self.k.cy + self.k.fy * o[1] / max(o[2], 1e-6)
                origin_kp_txt = f' origin_px=({u:.1f},{v:.1f})'
                if kps:
                    arr = np.asarray(kps, float)
                    d = np.linalg.norm(arr - np.array([u, v]), axis=1)
                    j = int(np.argmin(d))
                    origin_kp_txt += (f' nearest_kp={j} d={d[j]:.1f}px '
                                      f'kp0=({arr[0,0]:.1f},{arr[0,1]:.1f})')
                    # 四个 frame 角 -> 最近的 12 点序号；正确应为 [0,3,6,9]。
                    W = float(frame['W']); H = float(frame['H'])
                    E1 = np.asarray(frame['E1'], float)
                    E2 = np.asarray(frame['E2'], float)
                    om = np.asarray(frame['origin'], float) / 1000.0
                    E1m = E1 / 1000.0; E2m = E2 / 1000.0
                    cs = [om, om + W * E1m, om + W * E1m + H * E2m,
                          om + H * E2m]
                    idxs = []
                    for c in cs:
                        uu = self.k.cx + self.k.fx * c[0] / max(c[2], 1e-6)
                        vv = self.k.cy + self.k.fy * c[1] / max(c[2], 1e-6)
                        dd = np.linalg.norm(arr - np.array([uu, vv]), axis=1)
                        idxs.append(int(np.argmin(dd)))
                    corner_txt = f' corner_kp={idxs}'
            except (TypeError, ValueError, IndexError, AttributeError, KeyError):
                origin_kp_txt = ''
                corner_txt = ''
        size_txt = ''
        tgt = diag.get('target_mm')
        meas = diag.get('measured_size_mm')
        if tgt is not None or meas is not None:
            size_txt = f' target={_seq(tgt)} measured={_seq(meas)}'
        kpconf = diag.get('keypoints_conf')
        kp_txt = '-'
        if kpconf:
            try:
                arr = np.asarray(kpconf, float)
                kp_txt = f'min={np.nanmin(arr):.3f},mean={np.nanmean(arr):.3f}'
            except (TypeError, ValueError):
                pass
        cand_txt = ''
        if (diag.get('n_candidates') or 0) > 1:
            centers = diag.get('candidate_centers_uv') or []
            confs = diag.get('candidate_conf') or []
            cand_txt = f' candidates={list(zip(centers, confs))}'
        return (
            f'[pallet_detection_v2] POSE {status} '
            f'n_cand={diag.get("n_candidates", "?")} '
            f'pick={diag.get("pick", "?")} conf={_num(diag.get("conf"), "%.3f")} '
            f'{cand_txt}'
            f'visible={diag.get("visible_edges", "-")} '
            f'use={diag.get("use_pos", "-")} '
            f'selected={diag.get("selected_edges", "-")} '
            f'reason={diag.get("edge_select_reason", "-")} '
            f'edge_score={diag.get("edge_score", "-")} '
            f'd_anchor={_num(diag.get("d_anchor"), "%.1f")} '
            f'd_src={diag.get("d_anchor_src", "-")} '
            f'd_peak={_num(diag.get("d_peak"), "%.3f")} D_final={d_txt} '
            f'reproj={_num(diag.get("reproj_px"), "%.2f")} '
            f'inplane={_num(diag.get("inplane_deg"), "%.1f")} '
            f'origin_m={origin_txt}{origin_kp_txt}{corner_txt}{size_txt} '
            f'normal_src={normal_source or "-"} '
            f'kp_flip={diag.get("origin_kp_flip")} '
            f'kp_dist={diag.get("origin_kp_dist_px")}/'
            f'{diag.get("origin_kp_alt_dist_px")} '
            f'rebuilt={bool(diag.get("origin_rebuilt_from_pair", False))} '
            f'rebuild_kp0={diag.get("origin_rebuild_kp0_px", "-")} '
            f'kpconf={kp_txt}')

    # ---- 回调只入队，不做计算 -------------------------------------------
    def _on_detect_mode(self, msg):
        """伺服节点广播的检测门控（2026-10-18）。

        0 = 全停；1 = 只要纸箱（neighbor 档）；2 = 托盘+纸箱（pallet 档，
        使用 call 0 记录）；3 = **观测模式**（pallet 检测器出数，但忽略记录，
        走 TF + 深度自检 —— call 0 期间用）。
        """
        try:
            mode = int(msg.data)
        except (TypeError, ValueError):
            return
        active = mode in (2, 3)
        ignore_record = (mode == 3)
        if (active != self._gated_active or ignore_record != self._ignore_record
                or mode != self._detect_mode):
            self._ros['rospy'].loginfo(
                '[pallet_detection_v2] 检测门控：%s（mode=%d，忽略记录=%s）',
                '开 —— 开始出数' if active else '关 —— 待命，不出数',
                mode, ignore_record)
        self._detect_mode = mode
        self._gated_active = active
        self._ignore_record = ignore_record

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

    # ---- 工作线程：自愈 supervisor + 算 + 发布 --------------------------
    def _worker_main(self):
        """工作线程最外层：`_run` 抛出**任何**未捕获异常都不许静默带走检测。

        2026-11-02 现场：检测线程死后进程还在（`rospy.spin()` 活着），日志只剩
        TF 传输错误，现场看起来像"托盘检测自己退出了"。这里捕获、打完整
        traceback、等 1s 再重启 `_run`；持续失败时限频，不会刷屏。
        """
        rospy = self._ros['rospy']
        while not rospy.is_shutdown():
            try:
                self._run()
            except Exception as exc:                 # noqa: BLE001
                self._worker_restarts += 1
                # 只在第 1 次和每 10 次打完整栈：持续性异常不该淹没日志
                if self._worker_restarts == 1 or self._worker_restarts % 10 == 0:
                    rospy.logerr('[pallet_detection_v2] 检测线程异常退出后被重启'
                                 '（第 %d 次）：%s\n%s', self._worker_restarts,
                                 exc, traceback.format_exc())
                else:
                    rospy.logerr_throttle(
                        10.0, '[pallet_detection_v2] 检测线程异常退出后被重启（第 %d 次）：%s',
                        self._worker_restarts, exc)
                time.sleep(1.0)
            else:
                return                              # 只有 shutdown 时 `_run` 才会返回

    def _maybe_rebuild_tf(self, why: str) -> None:
        """TF 连续失败时**限频重建** Buffer + TransformListener。

        背景（2026-11-02 现场）：TF 的 TCPROS 流收到坏包后，`tf2_ros` 的接收
        线程会死；旧 Buffer 不会再更新，`normal_from_tf` 从此一直返回 None，
        而进程还活着 —— 检测静默停摆，只能重启。这里在新 Buffer 上重建订阅。
        """
        if self.normal_param is not None or self.tf_buffer is None:
            return
        now = time.monotonic()
        if now - self._last_tf_rebuild_mono < self._tf_rebuild_min_s:
            return
        self._last_tf_rebuild_mono = now
        rospy = self._ros['rospy']
        tf2_ros = self._ros['tf2_ros']
        try:
            old = self.tf_listener
            self.tf_listener = None
            if old is not None and hasattr(old, 'unregister'):
                try:
                    old.unregister()
                except Exception:                    # noqa: BLE001
                    pass
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            rospy.logwarn_throttle(
                30.0, '[pallet_detection_v2] TF 监听器已重建（原因：%s）', why)
        except Exception as exc:                     # noqa: BLE001
            rospy.logerr_throttle(
                10.0, '[pallet_detection_v2] TF 监听器重建失败：%s', exc)

    def _run(self):
        import queue as _queue
        rospy = self._ros['rospy']
        while not rospy.is_shutdown():
            try:
                color_msg, depth_msg = self.q.get(timeout=0.2)
            except _queue.Empty:
                continue
            self._n_processed += 1
            try:
                self._last_frame_stamp = float(color_msg.header.stamp.to_sec())
            except Exception:                        # noqa: BLE001
                self._last_frame_stamp = None
            if (self._n_processed - 1) % self.process_every != 0:
                continue                     # 不是处理槽
            # ---- 门控 / 预热（2026-10-18）----------------------------------
            # 空闲时**不算**（省 GPU、不刷拒绝日志）；启动后的第一帧要跑一次
            # 完整检测做预热（首次含法向拟合 + ONNX 初始化，实测 3.5s）——
            # 这笔钱必须在启动时付掉，不能等操作员 call 了服务才付。
            warm = False
            if not self._gated_active:
                if self._warmed:
                    self._heartbeat(rospy, 0.0)
                    continue
                warm = True
            t0 = time.perf_counter()
            try:
                color = _arr_to_bgr(color_msg)
                depth = _arr_to_depth_mm(depth_msg, self.depth_scale)
            except Exception as exc:         # noqa: BLE001
                rospy.logwarn_throttle(5.0, f'[pallet_detection_v2] 输入转换失败：{exc}')
                continue

            # ★ 本帧记录快照：normal 与 D 必须来自同一份记录。
            #   记录被 call 0 整体替换时，这一帧继续用旧快照，下一帧换新的。
            self._frame_record = self._record
            normal, src_or_why = self._normal_for_frame(color_msg, depth)
            if normal is None:
                # **法向缺失 = 不发布**。绝不回退到单位阵/竖直向下 ——
                # 那样它不会崩、不会报错，只是换个 score 继续"如实拒绝"，
                # 而"没有法向先验时检测会静默地算下去"是已知的坑（见 docstring）。
                now = time.time()
                if now - self._last_tf_warn > 5.0:
                    self._last_tf_warn = now
                    rospy.logwarn(f'[pallet_detection_v2] {src_or_why} —— 本帧**不发布**'
                                  f'（法向是必需先验，缺了就是静默乱算；检查 TF 树'
                                  f'或临时用 ~normal 参数）')
                continue
            # 心跳/拒绝日志要能独立说明"这一帧用的是什么法向"（2026-10-22）。
            self._last_normal = np.asarray(normal, float).copy()
            self._last_normal_src = src_or_why
            self._last_normal_tilt_deg = self._normal_tilt_deg(normal)

            try:
                found, diag = detect_pallet_frame_pose(
                    color, depth, self.k, normal=normal,
                    target_mm=self.target_mm,
                    long_side_parallel=self.long_side_parallel,
                    opts=self._pose_detect_opts(color_msg.header.stamp))
            except Exception as exc:         # noqa: BLE001
                rospy.logwarn_throttle(5.0, f'[pallet_detection_v2] 检测异常：{exc}')
                continue
            ms = (time.perf_counter() - t0) * 1000.0
            detect_ms = ms                    # 到这里为止：输入转换 + 法向 + detect
            # ---- 预热帧：算完就停，**不发布**（2026-10-18）------------------
            # 推理与法向拟合都已经跑过，CUDA/模型的钱付掉了；预热只负责把
            # 首帧的懒初始化（模型、缓存）摊掉，不必在这里发布。
            if warm:
                self._warmed = True
                rospy.loginfo(f'[pallet_detection_v2] 预热完成（{ms:.0f}ms）—— '
                              f'待命，call /pallet_servo_v2/slot 之后才开始出数')
                self._heartbeat(rospy, ms)
                continue
            if not self._normal_checked:
                self._normal_checked = True
                extra = f'  {self._normal_fit_log}' if self._normal_fit_log else ''
                # ⚠️ **来源标签必须看**：`fit` = 这一帧真的从深度拟合了，
                # `cache` = 复用首帧的拟合结果，`tf` = 拟合关掉了、直接用 TF 先验，
                # `tf_bad_fit` = 拟合没过合理性自检、退回 TF 先验且**没缓存**，
                # `param` = 用 ~normal 手给的。托盘解算把法向锁死，偏了没有下游纠正，
                # 所以"用的是哪个"必须可见。
                tilt = self._last_normal_tilt_deg
                tilt_txt = '?' if tilt is None else f'{tilt:.1f}°'
                suspicious = self._normal_tilt_is_suspicious(tilt)
                log = rospy.logwarn if suspicious else rospy.loginfo
                log(f'[pallet_detection_v2] 法向来源={src_or_why} '
                    f'normal={np.round(normal, 4).tolist()} 倾角={tilt_txt}'
                    f'{"" if not suspicious else "  ← 异常：与相机光轴夹角过大"}'
                    f'（fit_normal={self.fit_normal} '
                    f'normal_refit={self.normal_refit}）{extra}')
                if src_or_why == 'cache':
                    rospy.loginfo('[pallet_detection_v2] 法向走**缓存**（首帧拟合的结果）'
                                  '—— 伺服只做平面内平移时这是对的；'
                                  '**动了腰或换了工位**要把 ~normal_refit 设成 true')
            if found is None:
                self._n_reject += 1
                self._last_reject = format_rejects(diag)
                rospy.loginfo_throttle(
                    10.0, f'[pallet_detection_v2] 如实拒绝：{self._last_reject} '
                          f'（{ms:.0f}ms）')
                rej_key = self._pose_summary_key(
                    diag, f'reject:{diag.get("reject", "?")}', src_or_why)
                if self._pose_summary_needed(rej_key):
                    self._log_pose_summary(
                        self._pose_diag_summary(
                            None, diag,
                            status=f'reject:{diag.get("reject", "?")}',
                            normal_source=src_or_why),
                        rej_key)
                # ★ **拒绝帧也发一条 `valid=false` 的诊断消息**（2026-11-09）。
                #   目的只有一个：让 `/pallet_servo/overlay` 能看到**这一帧的
                #   关键点、选边与拒绝原因**（新选边闸门拒绝时尤其需要）。
                #   `valid=false` 的语义没变 —— 位姿仍是占位值，消费端
                #   （`NodePalletObs`）照旧把它挡在黑板之外。
                #
                #   只在 `diag` 真的带关键点时发：完全没检出托盘的帧（没有
                #   关键点可画）继续沿用旧行为，不制造无信息量的 valid=false 流量。
                if diag.get('keypoints_uv') is not None:
                    try:
                        rej_payload = build_payload(
                            None, diag, self.k, latency_ms=ms,
                            normal_source=src_or_why, normal=normal)
                        self.pub.publish(self._to_msg(
                            rej_payload, color_msg.header, diag=diag))
                        self._n_pub_reject += 1
                    except Exception as exc:     # noqa: BLE001
                        rospy.logwarn_throttle(
                            5.0, f'[pallet_detection_v2] 拒绝诊断帧发布失败：{exc}')
                self._heartbeat(rospy, ms)
                continue

            # POSE 初解就是最终位姿，直接发布。
            final = found
            ms = (time.perf_counter() - t0) * 1000.0
            self._last_detect_ms = detect_ms

            # D 只看记录（call 0 记录的 base D 换算到相机系）；不重算、不拒绝。
            self._last_D_mm = diag.get('d_anchor')
            self._last_stage_ms = {
                k: diag.get(k) for k in ('t_pre_ms', 't_infer_ms', 't_algo_ms')
                if diag.get(k) is not None}
            payload = build_payload(final, diag, self.k, latency_ms=ms,
                                    normal_source=src_or_why, normal=normal)
            self.pub.publish(self._to_msg(payload, color_msg.header, diag=diag))
            self._last_pub_stamp = self._last_frame_stamp
            self._n_pub += 1
            pub_key = self._pose_summary_key(diag, 'pub', src_or_why)
            if self._pose_summary_needed(pub_key):
                self._log_pose_summary(
                    self._pose_diag_summary(
                        final, diag, D_now=diag.get('d_anchor'), status='pub',
                        normal_source=src_or_why),
                    pub_key)
            self._last_pub_wall = time.time()
            self._last_good_ms = ms
            if ms > self.queue_warn_ms and time.time() - self._last_warn > 5.0:
                self._last_warn = time.time()
                rospy.logwarn(f'[pallet_detection_v2] 本帧 {ms:.0f}ms，超过 '
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

            proc=123 pub=120 rej=3 D=902mm(record/geom) 法向=cache 倾角=15.6° tf=0.2s前 drop=456  最近一帧 880ms（detect 70ms）  最近一次发布 0.9s 前
              最近一次拒绝：reject=too_few_edges,n_observed_edges=0<2,score=0.4135

        ⚠️ **`法向=` / `倾角=` 是 2026-10-22 事故后加的**。
        `倾角` 是法向与**相机光轴**的夹角（正常俯视台面 ~15°），超过
        `~normal_max_tilt_deg`（默认 60°）时标 `!!`。`D=` 后面现在直接跟
        相对 `ref` 的偏差百分比，不用再去心算。

        ⚠️ **`tf=` 是最后一次成功查 TF 距今多久**（2026-11-02 加）。TF 的
        TCPROS 订阅线程死掉时，`法向=` 可能还停在旧值，但 `tf=` 会一直涨 ——
        这两格必须分开看。`线程重启=` 只在工作线程抛出未捕获异常并被 supervisor
        重启后出现；它不为 0 就说明有代码异常，去看同时间段的 ERROR traceback。

        ⚠️ **括号里的 `detect` 分项不是装饰**（2026-10-15 加）：
        `最近一帧` 是**整帧**（输入转换 + 法向 + 推理 + 解算），`detect`
        段有 GPU 推理；没有分项就不知道该优化哪一段。

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
        # ⚠️ **detect 耗时必须打**（2026-10-15）。只打整帧的话，优化起来是瞎的。
        # 2026-11-16 起再拆成 预处理 / 推理 / 后处理算法 三段：整帧 120~200ms 里
        # 到底哪一段贵，只有拆开才看得出来（`detect_pose` 的 `t_pre/t_infer/t_algo`）。
        split = (f'（detect {self._last_detect_ms:.0f}ms）'
                 if self._last_detect_ms is not None else '')
        stage = ''
        for name, key in (('pre', 't_pre_ms'), ('infer', 't_infer_ms'),
                          ('algo', 't_algo_ms')):
            value = self._last_stage_ms.get(key)
            if value is not None:
                stage += f' {name}={value:.0f}'
        if stage:
            split = split + ' 分段:' + stage
        # D 只显示记录/几何解出来的值，**不参与任何拒绝**（2026-11-10 移除闸门）。
        gate = (f' D={self._last_D_mm:.0f}mm(record/geom)'
                if self._last_D_mm is not None else '')
        # ★ 法向也进心跳（2026-10-22）：事故那次心跳只有 `D=241mm`，而
        #   原因是"法向被缓存成侧向、与光轴 69.8°"。把来源/倾角放在 D 旁边，
        #   一眼就能对上因果；倾角超过自检阈值时打 `!!`。
        normal_tag = ''
        if self._last_normal_src:
            tilt = self._last_normal_tilt_deg
            tilt_txt = '?' if tilt is None else f'{tilt:.1f}°'
            mark = '!!' if self._normal_tilt_is_suspicious(tilt) else ''
            normal_tag = (f' 法向={self._last_normal_src}{mark}'
                          f' 倾角={tilt_txt}')
            if mark and self._last_normal is not None:
                # 异常时把法向本身也打出来（grep 心跳就能看到具体值）
                normal_tag += f' {np.round(self._last_normal, 3).tolist()}'
        # TF 健康度：最后一次成功查 TF 距今多久。TF 订阅线程死掉时这里会一直涨，
        # 而 `法向=` 标签可能还停在最后一次成功的值上 —— 必须分开看。
        # 心跳里主动查一次 latest TF，**空闲待命时也能发现订阅已死**（否则
        # `_last_tf_ok_mono` 只在真正算帧时更新，空闲时看起来像 TF 断了）。
        if self.normal_param is None and self.tf_buffer is not None:
            missing: list = []
            tf_now = normal_from_tf(self.tf_buffer, self.camera_frame,
                                    source_frame=self.base_frame, stamp=None,
                                    missing=missing)
            if tf_now is not None:
                self._last_tf_ok_mono = time.monotonic()
            else:
                self._maybe_rebuild_tf(f'心跳自查 TF 失败 {missing}')
        tf_tag = ''
        if self.normal_param is None:
            if self._last_tf_ok_mono is None:
                tf_tag = ' tf=从未'
            else:
                tf_tag = f' tf={time.monotonic() - self._last_tf_ok_mono:.1f}s前'
        worker_tag = (f' 线程重启={self._worker_restarts}'
                      if self._worker_restarts else '')
        # 门控状态必须进心跳（2026-10-18）：空闲时 `proc` 不涨、`pub` 不涨，
        # 与"线程死了 / 相机停了"在日志上长得一模一样 —— 这一格把它们分开。
        mode_tag = ''
        if self.gated:
            mode_tag = (' 门控=' + ('开' if self._gated_active
                                    else ('关（已预热，待命）' if self._warmed
                                          else '关（预热中）')))
        rec_extra = ''
        rec = getattr(self, '_record', None)
        if isinstance(rec, dict) and rec.get('plane_z_mm') is not None:
            try:
                rec_extra = f' z={float(rec["plane_z_mm"]):.1f}mm'
            except (TypeError, ValueError):
                rec_extra = ''
        rospy.loginfo(
            f'[pallet_detection_v2] proc={self._n_processed} pub={self._n_pub} '
            f'rej={self._n_reject}{gate}{normal_tag}{tf_tag}{worker_tag} '
            f'drop={self._n_dropped}{mode_tag}'
            f' 帧stamp={self._last_frame_stamp} 发stamp={self._last_pub_stamp}'
            f' 记录={"有" if self._record_state else "无"}{rec_extra}'
            f'{"/忽略记录" if self._ignore_record else ""}  '
            f'最近一帧 {last_ms:.0f}ms'
            f'{split}  最近一次发布 {stale} 前')
        if self._last_reject:
            rospy.loginfo(
                f'[pallet_detection_v2]   最近一次拒绝：{self._last_reject}')

    def _to_msg(self, p: dict, header, diag=None):
        """payload -> 消息。**这里只搬运，不含任何判断。**

        ⚠️ `header.stamp` **就是传进来的那个**（= 彩色图那帧的采集时刻），
        这里**不碰**它 —— 换 `rospy.Time.now()` 就是契约里那条硬要求说的错。

        `diag` 是 `detect_pallet_frame_pose` 的**原始 dict**（不是 payload 里
        那个格式化后的字符串），关键点 / 选边这些诊断字段从它取。不给
        `diag` 时全部按"没有"填固定占位值 —— 旧调用方（测试 / 离线脚本）
        行为不变。
        """
        ros = self._ros
        m = ros['PalletDetection']()
        m.header = ros['Header'](stamp=header.stamp, frame_id=header.frame_id)
        m.T_cam_pallet = [float(v) for v in p['T_cam_pallet']]
        m.det = float(p['det'])
        m.valid = bool(p['valid'])
        m.source = p['source']
        m.size_mm = [float(v) for v in p['size_mm']]
        # `corners_uv` 是**固定长度 4** 的字段。拒绝帧的 payload 里是空列表
        # （契约：`valid=false` 时下面全是占位值），直接赋 `[]` 会让序列化
        # 长度对不上 —— 所以补成 4 个 `(0,0)`。有效帧永远正好 4 个。
        corners = list(p.get('corners_uv') or [])
        if len(corners) != 4:
            corners = [(0.0, 0.0, 0.0)] * 4
        m.corners_uv = [ros['Point32'](float(u), float(v), 0.0)
                        for u, v, _ in corners]
        m.n_used = int(p['n_used'])
        m.n_slots = int(p['n_slots'])
        m.n_failed = int(p['n_failed'])
        m.spread_mm = float(p['spread_mm'])
        m.spread_deg = float(p['spread_deg'])
        m.latency_ms = float(p['latency_ms'])
        m.diag = p['diag']
        m.rejects = p['rejects']

        # ---- POSE 关键点 / 选边诊断（固定长度，缺字段填占位值）------------
        dbg = _edge_debug_fields(diag)
        m.keypoints_uv = [ros['Point32'](float(u), float(v), 0.0)
                          for u, v in dbg['keypoints_uv']]
        m.keypoints_conf = [float(c) for c in dbg['keypoints_conf']]
        m.selected_edges = [int(i) for i in dbg['selected_edges']]
        m.edge_score = [float(s) for s in dbg['edge_score']]
        m.edge_visible = [int(v) for v in dbg['edge_visible']]
        m.dropped_edges = [int(i) for i in dbg['dropped_edges']]
        m.edge_select_reason = dbg['edge_select_reason']
        return m




def main():
    ros = _ros()
    ros['rospy'].init_node('pallet_detection_v2')
    _init_unified_logging(ros['rospy'], '_pallet')
    PalletDetectionNode(ros)
    ros['rospy'].spin()


if __name__ == '__main__':
    main()
