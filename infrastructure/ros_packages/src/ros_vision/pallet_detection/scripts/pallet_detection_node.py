#!/usr/bin/env python3
"""ROS1 节点：`/pallet/detection` —— 木托盘台面坐标系的实时检测结果。

**两种前端**（`~detector` 切换，默认 `obb`）：

  * `obb`  —— YOLO-OBB 初值 + 全 refine（现行路径，~5Hz）
  * `pose` —— YOLO-pose 12 关键点 → 四边解算 + 轻量 edge_refine（15Hz 路径）

两条前端的输出契约完全相同（`frame` / `diag` 同一套），本节点只做分发。
`pose` 路线建议 `~refine=false`（edge_refine 已在 detect 内做完）。

**分层**（与仓库里其他检测节点同一结构，架构硬约束）：

    skills/atomic/perception/pallet_detect/   纯函数、零状态、**不 import ROS**  ← 算法核心
        detect.py          detect_pallet_frame()  YOLO-OBB 推理 -> frame_from_obb
        detect_pose.py     detect_pallet_frame_pose()  YOLO-pose -> 四边 -> solve_edges
        solve_obb.py       frame_from_obb()       锁法向 + 四条边直线 -> 4 自由度初值
        refine_depth.py    refine_pallet_frame()  用户规则开关 -> 脊线/角点/深度三级细化
        payload.py         build_payload() / format_*()     纯函数搬运层（不 import ROS）
    pallet_detection_node.py                  本文件：节点类只做搬运
                                              （订阅 / 推理会话 / 出队 / 填消息 / 发布）
    tf_normal.py       normal_from_tf()                   要查 TF，在 infrastructure 层

算法层在 LeTools 的 `skills/` 里、**不 import ROS**，所以它能脱离 ROS 单独跑和测；
本文件里**没有业务逻辑**（推理会话在启动时建一次，OBB 解码/选框/细化全在算法层）。

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
  | `image_depth` | `/camera/depth/image_raw` | 深度话题（**只用于退化帧锁 D**） |
  | `camera_info` | `/camera/color/camera_info` | 内参话题（用它的 K 矩阵） |
  | `detection_out` | `/pallet/detection` | 输出话题 |
  | `target_mm` | `[1200.0, 1000.0]` | 托盘实际尺寸（长, 短） |
  | `long_side_parallel` | `false` | **与画面近平行的托盘边是不是长边**。定死面内转角（短边近平行 -> `false`） |
  | `normal` | `''` | `"x,y,z"`；**给空串 = 从 TF 查**（部署默认） |
  | `fit_normal` | `true` | **TF 只当搜索初值，实际法向从当前帧深度拟合** |
  | `normal_refit` | `false` | `false` = 只在首帧拟合一次、之后复用；动腰/换工位要设 true |
  | `normal_max_tilt_deg` | `60.0` | 首帧拟合的**合理性自检**：法向与相机光轴的最大夹角，`0` = 关。拦住"TF 初值本身侧向"导致的歪拟合 |
  | `normal_d_check_tol` | `0.35` | 同上：拟合平面距离与 `d_mm` 的相对容差（比发布闸门宽，因为拟合到的常是地面）。`0` = 关 |
  | `normal_refit_on_d_reject` | `true` | D 闸门拒绝时丢弃缓存法向、下一帧重拟合（自愈）；关掉 = 拟歪一次只能重启 |
  | `camera_frame` | `camera_color_optical_frame` | TF 里的相机系 |
  | `base_frame` | `base_link` | TF 里的基座系 |
  | `model_path` | `''` | 托盘 OBB 的 `.onnx`；空 = 包内 `models/pallet_obb.onnx` |
  | `engine_path` | `''` | 托盘 OBB 的 `.engine`；空 = 包内 `models/pallet_obb_fp32.engine` |
  | `infer_backend` | `auto` | `auto`（engine 文件在就用它）/ `engine` / `onnx`。**只认这三个值** |
  | `imgsz` | `640` | 输入边长，**必须与导出时一致** |
  | `conf` | `0.25` | 置信度阈值（只用于过滤与 NMS，**不参与选框打分**） |
  | `nms_iou` | `0.45` | 旋转框 NMS 阈值（导出图没有 NMS，**必须自己去重**） |
  | `obb_pick` | `bottom_center` | 多候选怎么挑：`bottom_center` / `conf` / `area` |
  | `detector` | `obb` | 前端：`obb`（现行）/ `pose`（15Hz）。**只认这两个值**，其它值启动即 SystemExit |
  | `pose_model_path` | `''` | POSE 的 `.onnx`；空 = 包内 `models/pallet_pose.onnx` |
  | `pose_engine_path` | `''` | POSE 的 `.engine`；空 = 包内 `models/pallet_pose_fp32.engine` |
  | `pose_conf` | `0.25` | POSE 置信度阈值 |
  | `pose_kconf` | `0.5` | POSE 参与边拟合的最低关键点置信度 |
  | `pose_edge_refine` | `true` | POSE 初解后的局部梯度边修正（失败时静默保留初解） |
  | `pose_pick` | `bottom_center` | POSE 多候选怎么挑：`bottom_center` / `conf` / `area` |
  | `sync_slop_s` | `0.05` | 两路时间戳允许的最大差 |
  | `depth_scale_mm` | `1.0` | 深度单位换算到 mm 的系数 |
  | `process_every` | `1` | 每几帧处理一次 |
  | `queue_warn_ms` | `1000.0` | 处理慢于这个就告警一次（节流） |
  | `refine` | `true` | 检出后是否跑 `refine_pallet_frame` |
  | `d_mm` | `890.0` | **D 闸门**的参考平面距离（相机原点到**台面**、沿法向，mm）。**现场实测值**，换机位 / 动腰后要重量。`<=0` = 关闸 |
  | `d_tol` | `0.1` | D 闸门容差：`\|D/d_mm-1\|` 超过它就**不发布这一帧**。`<=0` = 关闸 |
  | `heartbeat_s` | `5.0` | 心跳周期（秒）。**成功时也打**，见 §心跳 |

  `long_side_parallel` 是**面内转角的先验、必须给对**：OBB 的宽高比 ≈ 1，
  "哪条边是长边"在框上不可分，所以**不能靠数据去挑**（试两个转角候选、用残差选，
  同一场景不同帧会投出不同结果）。它直接定死转角种子 —— ⚠️ **给错就明确偏 90°**，
  这是刻意的（错误应当可见，而不是被兜底掩盖）。现场：短边近平行 -> `false`。

## ⚠️ 托盘检测**必须**有模型（缺失即起不来，不前端回退）

按 `~detector` 选模型：`obb` 用 `model_path` / `engine_path`（或包内
`models/pallet_obb.{onnx,fp32.engine}`），`pose` 用 `pose_model_path` /
`pose_engine_path`（或包内 `models/pallet_pose.{onnx,fp32.engine}`）。拿不到可用
session 时 `__init__` **直接抛**、节点起不来 —— 与 `carton_obb_detect` 同一做法。
理由：模型缺失若只是每帧静默返回 `None`，症状是"一直检不到"，与"现场真没有托盘"
在日志上**无法区分**。两条前端的模型**不通用**，pose 缺失时也**不会**回退到 OBB。
导出分别见 `pallet_detect/scripts/export_onnx.py` 与 `export_onnx_pose.py`；
engine 用 `trtexec --onnx=... --saveEngine=... --noTF32 --buildOnly` 在**目标机**上导。
`~infer_backend=auto` 时 engine 文件在就用 engine（预先导出、启动不编译），
否则用 ONNX；engine 存在但加载失败**直接报错**，不静默退回 ONNX。

## ⚠️ 心跳：分不清"在跑"和"挂了"是这个节点的老毛病

本节点的日志点大多是**一次性的**（`法向来源` 只打首帧）、**只在失败时**
（`如实拒绝`）或**只在超时时**（`本帧 Xms 超过`）。于是「一切正常」与「工作线程
死了 / 一直在拒但被节流吞了」在日志上**长得一模一样**：都是什么都不打。
2026-09-29 现场就这么卡过一次。现在每 `~heartbeat_s` 秒**无条件**打一行：

```
[pallet_detection] proc=123 pub=120 rej=3 drop=456  本帧 40ms  最近一次发布 0.9s 前
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
算得出来**。契约原文见 `pallet_detection_msgs/msg/PalletDetection.msg` 末尾。

## ⚠️ 法向是**两级**的，第二级（深度自检）不能跳

1. **TF 先验**：TF 链通了、`base_link` 的 z 转进相机系 —— 但它与托盘实际台面法向
   会有 **0~10° 的静态偏差**（现场事实）。
2. **深度自检**：`fit_floor_normal` 在当前帧深度上、以先验为初值（夹角窗 ±25°）
   拟合"画面最大的面"，把这 0~10° 纠正回来（`~fit_normal`，默认开）。

⚠️ **为什么必须做第二级**：`frame_from_obb` 把法向**锁死**成先验（自由度 6→4），
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

⚠️ 需要先编译消息包 `infrastructure/ros_packages/src/ros_vision/pallet_detection_msgs`
（catkin 工作区里 `catkin_make`），并把生成的 Python 模块放进 `PYTHONPATH`。
没编译的话本文件会给出明确报错。
"""
from __future__ import annotations

import os
import sys
import time
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
from skills.atomic.perception.pallet_detect import (   # noqa: E402
    CameraIntrinsics,
    D_from_frame,
    build_payload,
    detect_pallet_frame,
    fit_floor_normal_checked,
    format_rejects,
    refine_pallet_frame,
)
# ⚠️ `DEFAULT_TARGET_MM` 与 `CHAIN` **必须显式 import**：它们定义在纯函数层
# `payload.py` 里，本文件只是使用者。漏了不会在 import 时报错 ——
# `PalletDetectionNode.__init__` 里才 NameError，而**单元测试从来不构造这个类**
# （只调静态方法），所以整条 ROS 路径零覆盖。测试里有一条 AST 静态检查钉住这点。
from skills.atomic.perception.pallet_detect.detect import (  # noqa: E402
    DEFAULT_ENGINE_PATH,
    DEFAULT_MODEL_PATH,
    _resolve_session,
)
# POSE 前端（2026-10-18）。与 OBB 同一入口契约：`(frame | None, diag)`；
# 模型缺失同样**硬失败**，不回退到 OBB。
from skills.atomic.perception.pallet_detect.detect_pose import (  # noqa: E402
    DEFAULT_POSE_ENGINE_PATH,
    DEFAULT_POSE_MODEL_PATH,
    _resolve_pose_session,
    detect_pallet_frame_pose,
)
from skills.atomic.perception.pallet_detect.payload import (  # noqa: E402
    CHAIN,
    DEFAULT_TARGET_MM,
)
# ⚠️ `normal_from_tf` **不**从 skills 取：它要 import rospy，按分层约束只能待在
# 基础设施层，就在本脚本旁边（同目录 `tf_normal.py`）。本脚本开头已经把
# `Path(__file__).parent` 插进了 sys.path，所以直接 import 得到。
from tf_normal import normal_from_tf  # noqa: E402

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
        rospy.logwarn('[pallet_detection] 没找到 core.common.logger —— 日志只走 rospy，'
                      '不落盘、不轮转。注意仓库根**本节点自己会插进 sys.path**，'
                      '所以这多半不是路径问题，而是缺依赖（requirements.txt 里的 '
                      'colorlog）或部署的是旧代码')
        return
    try:
        _init_logging(force=True, log_tag=tag, console_output=_console_output())
    except Exception as exc:                         # noqa: BLE001
        rospy.logwarn('[pallet_detection] 统一日志初始化失败，退回 rospy：%s', exc)


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

    ⚠️ 那条链的同名函数返回的是 `uint16`（它的深度拟合要整数），
    这里返回 `float32` —— `detect_pallet_frame`
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
        # ---- 前端选择（2026-10-18）-----------------------------------------
        # `obb` = 现行路径；`pose` = 15Hz 路径。**只认这两个值** ——
        # 拼错时静默退回 obb 会让人以为"切过去了"，却仍在跑旧后端。
        self.detector = str(rospy.get_param('~detector', 'obb') or '').strip().lower()
        if self.detector not in ('obb', 'pose'):
            raise SystemExit(
                f"[pallet_detection] ~detector={self.detector!r} 不认识；"
                f"允许值只有 'obb'（现行路径）或 'pose'（15Hz 路径）")
        # ---- 模型路径与推理参数 ----------------------------------------------
        # 两条前端的模型**不通用**。留空 = 各自包内默认；缺失/加载失败在启动时抛。
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
                f"[pallet_detection] ~infer_backend={self.infer_backend!r} 不认识；"
                f"允许值只有 'auto' / 'engine' / 'onnx'")
        self.model_path = str(rospy.get_param('~model_path', '') or '')
        self.engine_path = str(rospy.get_param('~engine_path', '') or '')
        self.obb_imgsz = int(rospy.get_param('~imgsz', 640))
        self.obb_conf = float(rospy.get_param('~conf', 0.25))
        self.obb_nms_iou = float(rospy.get_param('~nms_iou', 0.45))
        self.obb_pick = str(rospy.get_param('~obb_pick', 'bottom_center'))
        # POSE 前端（detector=pose 才用；obb 路线照样读，便于一键切换时参数齐全）
        self.pose_model_path = str(rospy.get_param('~pose_model_path', '') or '')
        self.pose_engine_path = str(rospy.get_param('~pose_engine_path', '') or '')
        self.pose_conf = float(rospy.get_param('~pose_conf', 0.25))
        self.pose_kconf = float(rospy.get_param('~pose_kconf', 0.5))
        self.pose_edge_refine = self._as_bool(
            rospy.get_param('~pose_edge_refine', True))
        self.pose_pick = str(rospy.get_param('~pose_pick', 'bottom_center'))
        # 法向自检：**TF 只当搜索初值，实际法向从当前帧深度拟合**（默认开）。
        # 现场事实（用户确认）：TF 链是通的，但 base_link 的 z 与托盘实际台面法向
        # 会有 **0~10°** 的静态偏差；而 frame_from_obb 把法向**锁死**成先验，
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
            self.normal_d_check_tol = float(
                rospy.get_param('~normal_d_check_tol', 0.35))
        except (TypeError, ValueError):
            self.normal_max_tilt_deg, self.normal_d_check_tol = 60.0, 0.35
        # D 闸门拒绝时丢掉缓存法向（默认 true）：拟歪一次不该永久生效。
        self.normal_refit_on_d_reject = self._as_bool(
            rospy.get_param('~normal_refit_on_d_reject', True))
        # OBB session：**启动时建一次；模型缺失/加载失败直接抛**（见 _build_session）
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
            '~detect_mode_topic', '/pallet_servo/detect_mode')).strip()
        self._gated_active = not self.gated
        self._warmed = False
        if self.gated:
            from std_msgs.msg import Int32
            self._mode_sub = rospy.Subscriber(
                self.detect_mode_topic, Int32, self._on_detect_mode,
                queue_size=1)
        # refine：检出之后**要不要跑** `refine_pallet_frame`。
        #
        # ⚠️ OBB 路线下 refine 不再有"平面细化"这个开关 —— 法向是**锁死**的
        # （`frame_from_obb` 的 4 自由度），`refine_pallet_frame` 里
        # `plane` / `edge_support_min` 两个旧参数**收下不用**（保留只为签名兼容）。
        # 旧的 `~plane` 之所以"部署必须开"，是因为旧 detect 不细化法向、要靠
        # refine 的 SVD 平面拟合修 TF 的 5.6° 偏差；OBB 路线没有这个问题。
        self.use_refine = self._as_bool(rospy.get_param('~refine', True))

        # ---- D 闸门 ----------------------------------------------------
        # 最终帧的平面距离 D（相机原点到**台面**、沿台面法向，毫米）与板上参考值的
        # 最大相对偏差。超过就不发布这一帧（宁可不发，也不要送一个明显歪的位姿）。
        #
        # ⚠️ **D 是"看图看不出来"的量**：给定法向之后，托盘的远近只表现为投影尺寸，
        # 而投影尺寸与"托盘多大"不可分 —— 远的大托盘和近的小托盘在图像上完全一样。
        # 所以必须靠这个外部先验兜住。`pallet_d_mm` 是**现场实测值**，
        # **换机位 / 动腰之后要重新量**（实测不同机位从 ~650 到 ~1650mm 都有）。
        #
        # `d_mm <= 0` 或 `d_tol <= 0` = **关掉闸门**（现场调参时不用改代码就能停用，
        # 启动日志会明说）。反解走单一真相源 `dsearch.D_from_frame`。
        try:
            self.d_mm = float(rospy.get_param('~d_mm', 890.0))
            self.d_tol = float(rospy.get_param('~d_tol', 0.1))
        except (TypeError, ValueError):
            self.d_mm, self.d_tol = 890.0, 0.1
        self.d_gate_on = (self.d_mm > 0.0) and (self.d_tol > 0.0)
        if self.d_gate_on:
            rospy.loginfo('[pallet_detection] D 闸门：ref=%.1fmm tol=%.1f%%'
                          '（|D/ref-1| 超过它的帧**不发布**；D 的实测值见心跳）',
                          self.d_mm, self.d_tol * 100.0)
        else:
            rospy.logwarn('[pallet_detection] D 闸门**关闭**（d_mm=%.1f d_tol=%.3f）'
                          '—— 不校验平面距离，位姿的远近没有任何外部约束',
                          self.d_mm, self.d_tol)

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
        self._n_d_reject = 0            # 其中被 D 闸门挡下的（心跳里单列，好区分）
        self._last_reject = None        # 最近一次拒绝的 `format_rejects` 原文
        self._last_D_mm = None          # 最近一次走到闸门的帧的 D（心跳里显示实测值）
        # 最近一帧实际用的法向（心跳里显示）—— 2026-10-22 事故的教训：
        # 只打首帧那一次的话，"法向被缓存成侧向、D 全被拒"在心跳里只剩一个
        # 孤零零的 D=241mm，看不出**为什么**。三个量都进心跳。
        self._last_normal = None
        self._last_normal_src = None
        self._last_normal_tilt_deg = None
        self._d_help_shown = False      # "pub=0 且全是 D 拒绝"的长提示只打一次
        self._last_good_ms = None       # 最近一次成功那帧的耗时（**整帧**）
        self._last_detect_ms = None     # 其中 detect 段（输入转换 + 法向 + 推理 + 解算）
        self._last_refine_ms = None     # 其中 refine 段（三级细化）
        self._last_pub_wall = None      # 最近一次发布的墙钟时刻
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
        rospy.loginfo(f'[pallet_detection] 订阅 {self.info_topic} + {self.image_color} '
                      f'+ {self.image_depth}  ->  发布 {self.out_topic}')
        # 日志落在哪个文件必须打出来：现场"日志里没东西"第一件要排除的，
        # 就是**根本没写进你翻的那个文件**（回退路径 / 别的目录 / 别的 tag）。
        try:
            from core.common.logger import get_active_log_file
            rospy.loginfo('[pallet_detection] 日志文件：%s', get_active_log_file())
        except Exception:                            # noqa: BLE001
            pass                                     # 回退路径下没有它，不是错误
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
            f'[pallet_detection] 内参 fx={self.k.fx:.2f} fy={self.k.fy:.2f} '
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
        n = normal_from_tf(self.tf_buffer, self.camera_frame,
                           source_frame=self.base_frame, stamp=stamp,
                           missing=missing, stale=stale)
        if n is None:
            # ★ 自愈：TF 的订阅线程可能已经死了（TCPROS 坏流），重建一次。
            self._maybe_rebuild_tf(f'TF 链缺边 {missing}')
            return None, f'TF 链缺边 {missing}'
        self._last_tf_ok_mono = time.monotonic()
        self._last_normal_prior = np.asarray(n, float).copy()
        if stale:
            self._ros['rospy'].logwarn_throttle(
                5.0, '[pallet_detection] TF 按图像 stamp=%.3f 拼不齐，'
                     '**整条链退回最新值**（法向来源=tf_stale）；'
                     'TF 订阅可能已停更，下一次心跳会继续检查',
                float(stale[0]))
            return n, 'tf_stale'
        return n, 'tf'

    def _normal_for_frame(self, color_msg, depth) -> tuple:
        """本帧实际用的法向 + 来源标签。**TF 只当搜索初值，答案从深度拟合。**

        返回 `(normal | None, 来源)`；`None` = 连初值都没有，调用方拒帧。

        ## 法向是两级：TF 先验 + 深度自检（现场事实，别跳第二级）

        TF 链是通的，但 base_link 的 z 轴与托盘实际台面法向**会有 0~10° 的偏差**，
        所以必须用当前帧深度在它附近做一次拟合来纠正。这不是可选项：OBB 路线把
        法向**锁死**成先验（`frame_from_obb` 的 4 自由度），偏了没有任何下游步骤会
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
        # ★ 缓存优先：法向是常量时，TF 停更不该拖垮已缓存的答案。
        if self._normal_fit is not None and not self.normal_refit:
            return self._normal_fit, 'cache'
        prior, why = self._resolve_normal(color_msg)
        if prior is None:
            # ★ 过期 TF 兜底：用最后一次成功解析的 TF 先验，而不是丢帧。
            if self._last_normal_prior is not None:
                self._normal_fit_log = (
                    f'TF 查不到（{why}）-> 用上一次 TF 先验兜底（tf_last）')
                self._ros['rospy'].logwarn_throttle(
                    5.0, '[pallet_detection] 本帧 TF 查不到（%s）—— '
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
            max_tilt_deg=self.normal_max_tilt_deg,
            d_ref_mm=self.d_mm if self.d_gate_on else None,
            d_tol=self.normal_d_check_tol)
        ms = (time.perf_counter() - t0) * 1000.0
        if fitted is None:
            # **回退策略 (A)**：拟合不合格就用 TF 的初值，而不是拒帧 ——
            # 初值本身在夹角窗内（窗是 ±25°），它只是"没被纠正"，不是"错的"。
            # ⚠️ **不写 `_normal_fit`**：这一帧没得到一个可信的拟合，缓存它
            # 就等于把"首帧拟歪永久生效"那条老路再走一遍。
            self._normal_fit_log = (f'拟合不合格（{check_why}）-> 回退 TF 初值 '
                                    f'（不缓存，下一帧重试） {ms:.0f}ms')
            self._ros['rospy'].logwarn_throttle(
                5.0, f'[pallet_detection] 法向拟合不合格：{check_why} —— '
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
                5.0, f'[pallet_detection] 法向拟合自检：{check_why}'
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
        """按 `~detector` + `~infer_backend` 建推理会话。**模型缺失 / 加载失败会让
        本节点起不来**（不是每帧静默失败，也不回退到另一条前端）。

        * `obb`  -> `~model_path` / `~engine_path`，空则包内 `models/pallet_obb.*`
        * `pose` -> `~pose_model_path` / `~pose_engine_path`，空则 `models/pallet_pose.*`

        后端选择与加载都在算法包里做（`detect._resolve_session` /
        `detect_pose._resolve_pose_session`），本节点不再自己建 ORT session ——
        否则两边会各写一份 provider/engine 顺序，迟早静默分叉。
        """
        if self.detector == 'pose':
            kind = 'POSE'
            opts = dict(backend=self.infer_backend, imgsz=self.obb_imgsz,
                        model_path=self.pose_model_path or str(DEFAULT_POSE_MODEL_PATH),
                        engine_path=self.pose_engine_path or str(DEFAULT_POSE_ENGINE_PATH))
            resolver = _resolve_pose_session
            export_hint = ('cd <本包>/scripts && OMP_NUM_THREADS=1 python3 '
                           'export_onnx_pose.py --weights best.pt --verify '
                           '（engine 用 trtexec --onnx=pallet_pose.onnx '
                           '--saveEngine=pallet_pose_fp32.engine --noTF32 --buildOnly）')
        else:
            kind = 'OBB'
            opts = dict(backend=self.infer_backend, imgsz=self.obb_imgsz,
                        model_path=self.model_path or str(DEFAULT_MODEL_PATH),
                        engine_path=self.engine_path or str(DEFAULT_ENGINE_PATH))
            resolver = _resolve_session
            export_hint = ('pallet_detect/scripts/export_onnx.py（engine 用 trtexec '
                           '--onnx=pallet_obb.onnx --saveEngine=pallet_obb_fp32.engine '
                           '--noTF32 --buildOnly）')
        try:
            sess, src = resolver(opts)
        except Exception as exc:                 # noqa: BLE001
            raise SystemExit(
                f'[pallet_detection] 托盘 {kind} 模型不可用：{exc}\n'
                f'  前端 detector={self.detector}  backend={self.infer_backend}\n'
                f'  托盘检测**必须**有{kind}模型（留空则用包内默认，'
                f'或用 ~{"pose_model_path" if self.detector == "pose" else "model_path"} '
                f'/ engine_path 指定）。导出见 {export_hint}。') from exc

        self._ros['rospy'].loginfo(
            '[pallet_detection] 前端=%s  后端=%s  模型=%s',
            self.detector, self.infer_backend, src)
        if hasattr(sess, 'get_providers'):
            active = list(sess.get_providers())
            self._ros['rospy'].loginfo(
                '[pallet_detection] onnxruntime providers=%s', active)
            if 'CUDAExecutionProvider' not in active:
                # 这里**只告警、不改环境**：ONNX Runtime 对 CUDA EP 缺库时会静默
                # 退回 CPU（症状是每帧慢一个数量级，日志里看不出来）。
                self._ros['rospy'].logwarn(
                    '[pallet_detection] ⚠️ providers 里没有 CUDAExecutionProvider —— '
                    '可能已**静默退回 CPU**（每帧会慢一个数量级）。检查 '
                    'LD_LIBRARY_PATH、或用 ~infer_backend=engine。providers=%s', active)
        else:
            self._ros['rospy'].loginfo(
                '[pallet_detection] TensorRT engine 输入=%s 输出=%s',
                sess.input_shape, sess.output_shape)
        return sess

    def _detect_opts(self):
        """OBB 前端 `detect_pallet_frame` 的 opts —— 把启动时建好的 session 传进去。

        ⚠️ `fit_normal=False`：法向第二级（深度自检）**由本节点做**（`_normal_for_frame`，
        带首帧缓存、`~normal_refit` 控制），所以**不让 detect 再拟合一遍** ——
        否则同一帧会拟合两次，且第二次用的是没缓存的结果。
        """
        return dict(session=self._sess, imgsz=self.obb_imgsz, conf=self.obb_conf,
                    nms_iou=self.obb_nms_iou, pick=self.obb_pick, fit_normal=False)

    def _pose_detect_opts(self):
        """POSE 前端 `detect_pallet_frame_pose` 的 opts。

        `imgsz` / `nms_iou` 与 OBB 共用（同一个输入规范）；`d_lock='depth'` 是
        POSE 的默认口径（实机 bag 已证明几何解出的 D 不稳）；`fit_normal=False`
        同 OBB —— 法向第二级由本节点做。
        """
        return dict(session=self._sess, imgsz=self.obb_imgsz,
                    conf=self.pose_conf, nms_iou=self.obb_nms_iou,
                    pick=self.pose_pick, kconf=self.pose_kconf,
                    edge_refine=self.pose_edge_refine,
                    d_lock='depth', fit_normal=False)

    # ---- 回调只入队，不做计算 -------------------------------------------
    def _on_detect_mode(self, msg):
        """伺服节点广播的检测门控（2026-10-18）。

        只有 `mode == 2`（pallet 档）才需要托盘；`1` 是 neighbor 档（只要纸箱）。
        """
        try:
            mode = int(msg.data)
        except (TypeError, ValueError):
            return
        active = (mode == 2)
        if active != self._gated_active:
            self._ros['rospy'].loginfo('[pallet_detection] 检测门控：%s（mode=%d）',
                                       '开 —— 开始出数' if active
                                       else '关 —— 待命，不出数', mode)
        self._gated_active = active

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
                    rospy.logerr('[pallet_detection] 检测线程异常退出后被重启'
                                 '（第 %d 次）：%s\n%s', self._worker_restarts,
                                 exc, traceback.format_exc())
                else:
                    rospy.logerr_throttle(
                        10.0, '[pallet_detection] 检测线程异常退出后被重启（第 %d 次）：%s',
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
                30.0, '[pallet_detection] TF 监听器已重建（原因：%s）', why)
        except Exception as exc:                     # noqa: BLE001
            rospy.logerr_throttle(
                10.0, '[pallet_detection] TF 监听器重建失败：%s', exc)

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
            # 心跳/拒绝日志要能独立说明"这一帧用的是什么法向"（2026-10-22）。
            self._last_normal = np.asarray(normal, float).copy()
            self._last_normal_src = src_or_why
            self._last_normal_tilt_deg = self._normal_tilt_deg(normal)

            try:
                if self.detector == 'pose':
                    found, diag = detect_pallet_frame_pose(
                        color, depth, self.k, normal=normal,
                        target_mm=self.target_mm,
                        long_side_parallel=self.long_side_parallel,
                        opts=self._pose_detect_opts())
                else:
                    found, diag = detect_pallet_frame(
                        color, depth, self.k, normal=normal,
                        target_mm=self.target_mm,
                        long_side_parallel=self.long_side_parallel,
                        opts=self._detect_opts())
            except Exception as exc:         # noqa: BLE001
                rospy.logwarn_throttle(5.0, f'[pallet_detection] 检测异常：{exc}')
                continue
            ms = (time.perf_counter() - t0) * 1000.0
            detect_ms = ms                    # 到这里为止：输入转换 + 法向 + detect
            # ---- 预热帧：算完就停，**不发布**（2026-10-18）------------------
            # 推理与法向拟合都已经跑过，CUDA/模型的钱付掉了。refine 是纯 CPU
            # 的稳态开销（没有懒初始化），不必在这里再花 ~335ms。
            if warm:
                self._warmed = True
                rospy.loginfo(f'[pallet_detection] 预热完成（{ms:.0f}ms）—— '
                              f'待命，call /pallet_servo/slot 之后才开始出数')
                self._heartbeat(rospy, ms)
                continue
            if not self._normal_checked:
                self._normal_checked = True
                extra = f'  {self._normal_fit_log}' if self._normal_fit_log else ''
                # ⚠️ **来源标签必须看**：`fit` = 这一帧真的从深度拟合了，
                # `cache` = 复用首帧的拟合结果，`tf` = 拟合关掉了、直接用 TF 先验，
                # `tf_bad_fit` = 拟合没过合理性自检、退回 TF 先验且**没缓存**，
                # `param` = 用 ~normal 手给的。OBB 路线把法向锁死，偏了没有下游纠正，
                # 所以"用的是哪个"必须可见。
                tilt = self._last_normal_tilt_deg
                tilt_txt = '?' if tilt is None else f'{tilt:.1f}°'
                suspicious = self._normal_tilt_is_suspicious(tilt)
                log = rospy.logwarn if suspicious else rospy.loginfo
                log(f'[pallet_detection] 法向来源={src_or_why} '
                    f'normal={np.round(normal, 4).tolist()} 倾角={tilt_txt}'
                    f'{"" if not suspicious else "  ← 异常：与相机光轴夹角过大"}'
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

            # ---- refine（**默认开**）---------------------------------------
            # 用户规则开关（D 异常 + 出画 -> "1 个可见角 + 两条相邻边方向"）
            # + 脊线级/角点级/深度级三级细化，见 `refine_depth.refine_pallet_frame`。
            # ⚠️ 它的契约是**永不返回 None**（输入合法时）—— 细化没成就交回初值、
            # 照常发布；这里仍然防御性地判一次 None（输入非法时它会给 (None, ...)）。
            refine_diag = None
            refined = None
            final = found                      # 发布用的位姿：refine 过就用 refine 的
            refine_ms = 0.0
            if self.use_refine:
                _tr = time.perf_counter()
                try:
                    final, refine_diag = refine_pallet_frame(
                        found, color, depth, self.k, target_mm=self.target_mm)
                except Exception as exc:     # noqa: BLE001
                    rospy.logwarn_throttle(
                        5.0, f'[pallet_detection] refine 异常：{exc} —— 退回 detect '
                             f'的结果')
                    final, refine_diag, refined = found, None, False
                else:
                    if final is None:
                        rospy.loginfo_throttle(
                            5.0, f'[pallet_detection] refine 如实拒绝：'
                                 f"reject={refine_diag.get('reject')} （{ms:.0f}ms）")
                        self._n_reject += 1
                        self._last_reject = f"refine:{refine_diag.get('reject')}"
                        self._heartbeat(rospy, ms)
                        continue
                    refined = True
                finally:
                    refine_ms = (time.perf_counter() - _tr) * 1000.0
            # ⚠️ **`ms` 在 refine 跑完之后再取一次，它才是"这一帧总共花了多久"。**
            # 从前它停在 detect 返回处，于是 335ms 的 refine 在日志里**完全不可见**
            # ——操作员看到 `本帧 51ms` 却只有 1.8Hz，两个数都是真的、只是量的不是
            # 同一件事。2026-10-15 现场排帧率时踩的正是这个。
            # 实测（Orin AGX / 现场 bag）：detect 段 ~70ms、refine 段 ~335ms，
            # 整帧 ~406ms。**先看 `detect`/`refine` 两个分项再决定优化哪一段。**
            ms = (time.perf_counter() - t0) * 1000.0
            self._last_detect_ms = detect_ms
            self._last_refine_ms = refine_ms

            # ---- D 闸门：最终帧的平面距离与板上参考值差太多就不发布 --------
            # 位置选在这里：`final` 已经定下来（refine 返回 None 的帧在上面的
            # 分支里 `continue` 掉了；refine 抛异常会退回 `found`；`~refine=false`
            # 时 `final` 就是 detect 的结果）。所以**闸门只见"本来会发布"的帧** ——
            # 这正是想要的语义：detect/refine 阶段就失败的帧走各自的老路径。
            if self.d_gate_on:
                D_now = D_from_frame(final)
                self._last_D_mm = D_now
                dev = abs(D_now / self.d_mm - 1.0)
                if dev > self.d_tol:
                    self._n_reject += 1
                    self._n_d_reject += 1
                    # 拒绝串交给 `format_rejects` 渲染 —— "必须写实际值与阈值"的
                    # 契约只该有一处实现，不在这儿手写一遍。
                    self._last_reject = format_rejects({
                        'reject': 'D_out_of_tol', 'D_mm': D_now,
                        'D_ref_mm': self.d_mm, 'D_tol': self.d_tol})
                    # ★ **自愈**（2026-10-22）：D 超限往往不是"托盘真的远了"，
                    #   而是缓存法向歪了（17:44 那次 D=241 vs 990，全部帧被拒）。
                    #   丢掉缓存，下一帧重新拟合并重新过自检 —— 否则只能重启进程。
                    if (self.normal_refit_on_d_reject
                            and self._normal_fit is not None):
                        discarded = np.round(self._normal_fit, 4).tolist()
                        tilt = self._last_normal_tilt_deg
                        tilt_txt = '?' if tilt is None else f'{tilt:.1f}°'
                        self._normal_fit = None
                        rospy.logwarn_throttle(
                            5.0, f'[pallet_detection] D 闸门拒绝：丢弃缓存法向 '
                                 f'normal={discarded}（与光轴 {tilt_txt}），'
                                 f'下一帧重新拟合（若持续超限，检查 normal_refit/'
                                 f'TF 或现场 D 参考值）')
                    tilt = self._last_normal_tilt_deg
                    tilt_txt = '?' if tilt is None else f'{tilt:.1f}°'
                    guess = ('；**法向与光轴夹角异常** —— 多半是拟合歪了，'
                             'D 只是它的后果' if self._normal_tilt_is_suspicious(tilt)
                             else '')
                    rospy.logwarn_throttle(
                        5.0, f'[pallet_detection] D 闸门：{self._last_reject} '
                             f'（法向来源={src_or_why} 倾角={tilt_txt}，{ms:.0f}ms）'
                             f'—— 这一帧**不发布**{guess}')
                    self._heartbeat(rospy, ms)
                    continue

            payload = build_payload(final, diag, self.k, latency_ms=ms,
                                    normal_source=src_or_why, normal=normal,
                                    refine_diag=refine_diag, refined=refined)
            self.pub.publish(self._to_msg(payload, color_msg.header))
            self._n_pub += 1
            self._d_help_shown = False      # 又开始正常发布：下次再出问题时还能提示
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

            proc=123 pub=120 rej=3 d闸=1 D=902mm 偏1.3%(ref=890mm) 法向=cache 倾角=15.6° tf=0.2s前 drop=456  最近一帧 880ms（detect 70ms + refine 810ms）  最近一次发布 0.9s 前
              最近一次拒绝：reject=too_few_edges,n_observed_edges=0<2,score=0.4135

        ⚠️ **`法向=` / `倾角=` 是 2026-10-22 事故后加的**：那次心跳只有
        `d闸=412 D=241mm`，看不出原因是"缓存法向歪到侧向（倾角 69.8°）"。
        `倾角` 是法向与**相机光轴**的夹角（正常俯视台面 ~15°），超过
        `~normal_max_tilt_deg`（默认 60°）时标 `!!`。`D=` 后面现在直接跟
        相对 `ref` 的偏差百分比，不用再去心算。

        ⚠️ **`tf=` 是最后一次成功查 TF 距今多久**（2026-11-02 加）。TF 的
        TCPROS 订阅线程死掉时，`法向=` 可能还停在旧值，但 `tf=` 会一直涨 ——
        这两格必须分开看。`线程重启=` 只在工作线程抛出未捕获异常并被 supervisor
        重启后出现；它不为 0 就说明有代码异常，去看同时间段的 ERROR traceback。

        ⚠️ **括号里的 `detect` / `refine` 分项不是装饰**（2026-10-15 加）：
        `最近一帧` 是**整帧**（输入转换 + 法向 + 推理 + 解算 + 三级细化），
        detect 段有 GPU 推理、refine 段是纯 CPU，现场实测差一个量级
        （70ms vs 335ms）。**没有分项就不知道该优化哪一段。**

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
        # ⚠️ **分项必须打**（2026-10-15）。只打整帧的话，优化起来是瞎的：
        # detect 段（有 GPU 推理）与 refine 段（纯 CPU）差一个量级，
        # 现场实测 70ms vs 335ms —— 不打分项就不知道该动哪一段。
        if self._last_detect_ms is not None and self._last_refine_ms is not None:
            split = (f'（detect {self._last_detect_ms:.0f}ms + '
                     f'refine {self._last_refine_ms:.0f}ms）')
        else:
            split = ''
        # ⚠️ **D 闸门的两个量也要打**：`d闸` 把"被 D 挡下的"从"被检测挡下的"里
        # 分出来；`D=` 是**现场标定 890 那个值靠不靠谱的唯一证据** —— 跑一次就能从
        # 日志读到真实的 D，不必再单独搭离线回放去量（D 是"看图看不出来"的量，
        # 换机位 / 动腰之后它会变）。
        gate = ''
        if self.d_gate_on:
            gate = f' d闸={self._n_d_reject}'
            if self._last_D_mm is not None:
                dev_txt = ''
                if self.d_mm > 0.0:
                    dev = abs(self._last_D_mm / self.d_mm - 1.0)
                    dev_txt = f' 偏{dev * 100:.1f}%'
                gate += (f' D={self._last_D_mm:.0f}mm{dev_txt}'
                         f'(ref={self.d_mm:.0f}mm)')
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
        rospy.loginfo(
            f'[pallet_detection] proc={self._n_processed} pub={self._n_pub} '
            f'rej={self._n_reject}{gate}{normal_tag}{tf_tag}{worker_tag} '
            f'drop={self._n_dropped}{mode_tag}  '
            f'最近一帧 {last_ms:.0f}ms'
            f'{split}  最近一次发布 {stale} 前')
        # ★ 一次性的"病因提示"：pub 一直是 0 且拒绝全来自 D 闸门时，最容易被
        #   误判成"检测不到托盘"。D 拒绝的帧**检测是成功的**（出框了才走到闸门）。
        if (self._n_pub == 0 and self._n_d_reject >= 10
                and not self._d_help_shown):
            self._d_help_shown = True
            d_txt = '?' if self._last_D_mm is None else f'{self._last_D_mm:.0f}mm'
            t_txt = ('?' if self._last_normal_tilt_deg is None
                     else f'{self._last_normal_tilt_deg:.1f}°')
            rospy.logwarn(
                '[pallet_detection] ★ 至今 pub=0，拒绝全部来自 D 闸门（d闸=%d，'
                'D=%s，ref=%.0fmm）。**不是"没检测到托盘"** —— 检测已经出框，'
                '是位姿的平面距离与参考值不符。先看 法向=%s 倾角=%s：'
                '倾角接近 90° = 法向拟合歪了（TF 初值/首帧深度），不是模型坏了。',
                self._n_d_reject, d_txt, self.d_mm,
                self._last_normal_src or '?', t_txt)
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
    _init_unified_logging(ros['rospy'], '_pallet')
    PalletDetectionNode(ros)
    ros['rospy'].spin()


if __name__ == '__main__':
    main()
