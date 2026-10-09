# 纸箱检测（YOLO / ONNX → `/box/yolo_box`）

从相机彩色图里检出纸箱的**轴对齐外接框**，发到 `/box/yolo_box`。
**只发框，不恢复旋转角、不算位姿。**

这是 `/box/detection` 那条链路的**上游** —— 在它之前，这个话题全仓没有真生产者。

```text
rosbag / 相机 ──/camera/color/image_raw──→  carton_box_yolo ──/box/yolo_box──┐
              ──/camera/depth/image_raw──────────────────────┐               │
                                                              │              │
                                  （深度只用于**选箱打分**，    │              │
                                    不参与框的坐标计算）        │              │
                                                               ↓              ↓
                                                    box_detection（本仓库）
                                                               ↓
                                                       /box/detection
                                                 （箱子顶面四角 + valid + angle）
```

下游 `box_detection` 的算法在 `skills/atomic/perception/box_frame/`，
ROS 壳在 `infrastructure/.../detection_industrial_yolo/box_detection/`。

| 文件 | 作用 |
|---|---|
| `scripts/carton_detector.py` | **纯函数**：letterbox、坐标反变换、**选箱打分**、话题契约。不 import ROS / ultralytics |
| `scripts/carton_box_detect.py` | ROS 节点：订阅图像 + 深度 → ONNX 推理 → 打分选箱 → 发话题。只做搬运 |
| `scripts/export_onnx.py` | 开发机上跑一次，把 `.pt` 导成 `.onnx` |
| `tests/test_carton_detector.py` | 纯函数自检（不需要 ROS / 模型 / onnxruntime） |
| `config/carton_box_yolo.yaml` | 参数默认值 |
| `launch/carton_box_yolo.launch` | 起节点的 launch |
| `models/` | 权重放这里，**不进 git**，见 `models/README.md` |

# 1. 接口

## 输入

### `~image`（`sensor_msgs/Image`，默认 `/camera/color/image_raw`）

彩色图，认 `bgr8` / `rgb8` / `mono8`。**编码不认识会直接报错，不猜** —— 猜错了
通道顺序反了框会偏，而且看不出来。

### `~image_depth`（`sensor_msgs/Image`，默认 `/camera/depth/image_raw`）

深度图，认 `16UC1`（**已经是毫米**）与 `32FC1`（**米**，节点自动 ×1000）。
**只用于选箱打分**（见 §3），不参与框的坐标计算 —— 所以取不到深度**不影响
输出框的正确性**，只影响「挑哪一个」。

⚠️ **要求深度已配准到彩色**（`config/camera_config.yaml` 的
`head_depth_registration: true`）。没配准的话采样到的深度不是箱子的，打分就是
噪声。**给空串 = 不订阅深度**，全部候选走"无深度"降级。

## 输出

### `/box/yolo_box`（`geometry_msgs/PolygonStamped`）

| 字段 | 说明 |
|---|---|
| `header` | **沿用输入图像的时间戳与 `frame_id`**（不做时间换算，下游靠它对齐） |
| `polygon.points[0]` | **左上** `(u_min, v_min)`，`z` 恒为 0 |
| `polygon.points[1]` | **右下** `(u_max, v_max)`，`z` 恒为 0 |

契约由下游 `box_detection_node.py` 的 `_box_uv_from_msg` 定死。那边取两点的
**外接框**，所以顺序给反了也不会出错，但契约照写。

**没检到箱子时一个字都不发**（`publish_empty: false`，默认）。这与 `NodeBoxObs`
对 `valid=false` 的处置同构：宁可让下游继续用上一帧的好值，也不要喂空的进去。

## 节点参数

| 名字 | 默认 | 说明 |
|---|---|---|
| `image` | `/camera/color/image_raw` | 输入彩色图 |
| `image_depth` | `/camera/depth/image_raw` | 深度图，**只用于选箱打分**；空串 = 不订阅 |
| `depth_scale_mm` | `1.0` | 深度单位→毫米系数（`16UC1` 已是毫米填 1.0） |
| `box_out` | `/box/yolo_box` | 输出框话题 |
| `model_path` | `<包>/models/best_cartonbb.onnx` | ONNX 权重（**留空 = 用默认值**） |
| `imgsz` | `640` | 输入边长，**必须与导出时一致** |
| `conf` | `0.25` | 置信度阈值 |
| `target_class` | `-1` | 要挑的类别 id；`-1` = 不筛（单类模型只有 0） |
| `publish_empty` | `false` | 没检到时要不要发空消息 |
| `max_rate_hz` | `0.0` | >0 时降频（相机 30Hz、YOLO 10Hz 就够） |
| `queue_size` | `1` | 订阅队列，只要最新帧 |

**选箱打分**（见 §3）：`score_w_conf` `0.5`、`score_w_area` `0.2`、
`score_w_depth` `0.3`、`z_near_mm` `500`、`z_far_mm` `1500`、`depth_patch_px` `15`。

⚠️ **所有参数都是私有参数**（`~name`，即 `/carton_box_detect/<name>`）。
launch 里 `<rosparam>` / `<param>` **必须写在 `<node>` 标签内部** —— 写在
`<launch>` 顶层会落到全局 `/name`，节点一个都读不到，**静默用代码里的默认值**。
（2026-09-24 修：原来就是写在顶层，改 yaml 完全没反应。）

# 2. 快速测试

## 2.1 纯函数自检（最快，什么都不需要）

```bash
cd <本包>
python3 tests/test_carton_detector.py        # 退出码 0 通过 / 1 失败
```

装了 ultralytics 的话会多跑一条**逐像素比对**（跟 `ultralytics.data.augment.LetterBox`
比 5 组不同尺寸），没装自动跳过。**这条才是真正的判据** —— 手算的断言只能证明
「跟我以为的一样」。

## 2.2 离线验一遍推理（不需要 ROS）

```bash
cd <本包>/scripts
export OMP_NUM_THREADS=1
python3 -c "
import sys; sys.path.insert(0,'.')
import cv2, onnxruntime as ort
from carton_detector import to_input_tensor, decode, score_candidates
s = ort.InferenceSession('../models/best_cartonbb.onnx',
                         providers=['CUDAExecutionProvider','CPUExecutionProvider'])
img = cv2.imread('/path/to/color_frames/xxx.png')
x, r, pad = to_input_tensor(img, 640)
raw = s.run(None, {'images': x})[0]
dets = decode(raw[0], r, pad, conf_thr=0.25, orig_shape=img.shape[:2])
for d, score, parts in score_candidates(dets):   # 不给 z_list = 全部按无深度
    print(round(score, 3), parts, d['box_uv'])
"
```

## 2.3 端到端（rosbag 回放，不需要真机）

**这是最接近真机的一条**：`rosbag play` 出图 → 本节点 → `box_detection_node` → `/box/detection`。

```bash
# 终端 1
roscore

# 终端 2：回放（循环）。**注意深度话题是 Orbbec 的命名**
rosbag play -l <你的包>.bag

# 终端 3：本节点（/box/yolo_box 的生产者）
roslaunch carton_box_yolo carton_box_yolo.launch

# 终端 4：下游（本仓库）。深度话题默认已是 /camera/depth/image_raw
roslaunch box_detection box_detection.launch

# 终端 5：看结果
rostopic echo /box/yolo_box        # 本节点发的框
rostopic echo /box/detection       # 四角 + valid + angle_deg
```

判据是 `/box/detection` 里 **`valid: true` 且 `source: "window"`** —— 那说明
窗里 5 帧全成功，四角是它们的逐点平均。`source: "yolo_fallback"` 说明窗全失败、
四角退回原始 YOLO 框，**不能拿去用**。

## 2.4 没有 rosbag 时用假相机

`maduo/tools/pub_test_frames.py` 从离线数据集发图。它**同时发图、深度、内参和框**，
而框那一路会和本节点抢 `/box/yolo_box` —— 用 `--box-topic` 把它挪开：

```bash
python3 tools/pub_test_frames.py --sequence test_data/9_test --box-topic /unused/yolo_box
```

**别去改 `pub_test_frames.py`** —— 它是 `maduo` 侧的工具，改它会把两边搞混。

## 2.5 导出权重

在**有 ultralytics 的开发机**上跑一次（机器人上不需要 ultralytics）：

```bash
cd <本包>/scripts
OMP_NUM_THREADS=1 python3 export_onnx.py \
    --weights /path/to/best_cartonbb.pt --verify
```

# 3. 注意事项

**① 推理必须在独立进程里。** YOLO 一帧几毫秒到几十毫秒，放进行为树的 tick 里会把
整棵树拖住。本包是标准 ROS 节点，与行为树**只通过话题相连**，不 import LeTools
的任何东西，单独 `rosrun` 就能跑。

**①b 选箱是打分，不是"面积最大"或"置信度最高"。** 两个单指标各自都有反例：
邻箱可能更大，邻箱的置信度也常常很高（实测都 >0.98）。判据是三项归一化加权求和 ——

```text
score = w_conf  * conf                              # 已在 [0,1]
      + w_area  * (area_px / max_area_px_in_frame)  # 同帧相对归一
      + w_depth * clamp((z_far - z) / (z_far - z_near), 0, 1)
```

面积用**同帧最大值**归一，所以不需要"面积满分对应多少像素"那种常量 ——
代价是**单候选时该项恒为 1**（此时本来也不需要它区分）。深度取框中心
`(2r+1)²` 方块的**中位数**，不是中心那一个像素（深度图到处是空洞与飞点，
单点会一帧有值一帧没值，让排序跳）。

⚠️ 取不到深度时**把深度项权重按比例让给另两项**（不罚、不丢候选）：
`score = (w_conf*conf + w_area*area_norm) / (w_conf + w_area)`。深度空洞往往是
近物遮挡造成的，罚它会适得其反。代价是归一化到 `[0,1]`，所以深度项**只能压低
远处的候选，不能额外抬高近处的**。

⚠️ **`~pick` 已废弃**（2026-09-24）。它以前是 `largest`/`best` 二选一，但那个参数
**从来没有被传给选框函数** —— 读了、打了日志、没生效。现在选框没有模式可切，
配置里还写着它会得到一条 WARNING。

**①c 选箱的权重与距离边界是拍的，没有数据支撑。** `score_w_*` / `z_near_mm` /
`z_far_mm` 的默认值只是起点（`z_near/z_far` 依据是"头部相机到手上的箱子约
0.5~1.5 m"）。**落地后应当拿一帧多箱子的真实数据核一遍。** 节点的 INFO 日志
（2 秒节流）里带 `score=` 与各分项，现场"为什么挑了这个"靠它回答。

**② 发 ONNX 不发 `.pt`。** `.pt` 把网络结构存成 yaml 描述，加载时由**当前装的
ultralytics** 去实例化，于是同一个权重在不同版本下给出**不同的数**：

```text
ultralytics 8.4.41（训练用的）  conf=0.986  框=[212.98, 243.79, 470.48, 422.62]
ultralytics 8.3.163（仓库里现成）conf=0.911  框=[213.22, 244.19, 468.08, 422.98]
```

**不报错，只是数字悄悄变了。** 而 LeTools 里现成的 pin 正是 `ultralytics==8.3.163`
（`third_party/basket_vision` 的 jetpack5 运行时清单），与训练用的 8.4.41 不一致。
**解决**：导成 ONNX，结构冻结进图里，推理端只认 onnxruntime。实测同一份输入张量下
PyTorch 与 ONNX 的原始输出**最大差 6.1e-05**。

**③ letterbox 必须用「方形 640×640 + 补边」，不能用 rect / 不补边。** 训练时
ultralytics 用的是方形补边，推理要跟训练一致。实测 8_test 上两种模式差 **6.7px**
（v 方向）—— 根因是那批数据里有一部分的标注由**无补边的模型**产出（labels 的 v 正好
在 0/480 边界）。6_test 上有四角真值，两种模式的**最终精度一样**（11.6 vs 11.7px），
所以按「与训练一致」走不吃亏。`tests/` 里那条逐像素比对就是钉这个的。

**④ 深度话题是 Orbbec 命名。** `box_detection_node.py` 的 `~image_depth` 默认值已从
RealSense 的 `aligned_depth_to_color/image_raw` 改成 **`/camera/depth/image_raw`**
（2026-09-23）。换回 RealSense 时改这一个参数即可。格式上两种都支持：16UC1 已经是
毫米，32FC1 是米（用 `depth_scale_mm` 兜底）。

**⑤ GPU 要显式给 cuDNN。** onnxruntime 的 CUDA provider **不会**自己去 conda 环境的
`site-packages/nvidia/` 里找库。不给的话**不报错，只是静默退回 CPU**：

```bash
L=<conda-env>/lib/python3.*/site-packages/nvidia
export LD_LIBRARY_PATH=$L/cudnn/lib:$L/cublas/lib:$L/cuda_runtime/lib:$LD_LIBRARY_PATH
```

同机实测：给了 **4.0ms**，静默退回 CPU **9.7ms**。启动日志里那行
`onnxruntime x.y.z，provider=CUDAExecutionProvider` 就是判据 —— **看这一行确认真的用上了**。

**⑥ `onnxruntime-gpu` 对 python 3.8 只到 `1.16.3`。** 更高版本（1.17+）没有 cp38
轮子。而 1.16.3 要 **CUDA 11** 的 cublas/cudnn；1.17+ 才要 CUDA 12。**换机器前先确认
CUDA 版本再选 onnxruntime 版本**，装错了是静默退回 CPU（同 ⑤）。

```bash
python3 -m pip install onnxruntime-gpu==1.16.3 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

**⑦ 补边区域里的框会被丢掉。** `decode()` 给了 `orig_shape` 就把**完全跑出图外**的框
滤掉 —— 那是真模型偶尔会吐的噪声（letterbox 的灰边上没有目标）。

**⑧ `bool("false") is True`。** `publish_empty` 这类参数走的是 `_as_bool()`，不是
`bool()` —— rosparam / 场景 JSON 真会传字符串。

**⑨ `OMP_NUM_THREADS=1` 是必须的。** onnxruntime / OpenBLAS 在这个规模的模型上开多线程
是纯开销，而且会和 ROS 的回调线程抢核。`launch` 里已经用 `<env>` 设好。

**⑩ 模型尺度要匹配。** 权重用 640×480 的数据训（`maduo/test_data/横竖抓_rgb`）。
喂 1280×800 的图时 letterbox 之后箱子只占很小一块，**实测几乎检不到** —— 那不是链路
断了，是模型没见过这个尺度。换分辨率要重训或补数据。

**⑪ 权重不进 git。** `models/` 在 git 里只有 `.gitignore` 和 `README.md`，与
`module_internal/zhaofeng_feeding/yolo_weights/` 同一做法。导出与拷贝见 `models/README.md`。

**⑫ 解释器由 `launch` 的 `yolo_python` arg 决定，默认空串 = 系统 python3。**

空串 = **不挂 `launch-prefix`**，就用 catkin 生成的 node wrapper 自带的解释器
（`#!/usr/bin/python3`）。**那个 `#!/usr/bin/python3` 就是系统 python3**，
也就是"基础环境"里那份 numpy / cv2 / onnxruntime —— 不是任何 conda / venv。

⚠️ **`activate` 对节点无效。** catkin 生成的 node wrapper 顶上写死了
`#!/usr/bin/python3`，而 `roslaunch/node_args.py` 是**直接 exec 那个文件**的
（`_launch_prefix_args(node) + cmd + args`）—— 不查 PATH，也不看你当前激活了什么。
**换解释器只有 `launch-prefix` 这一条路**，而这个 arg 就是它的出口。

```bash
# 挂一个 venv（绝对路径）
roslaunch carton_box_yolo carton_box_yolo.launch yolo_python:=/path/to/venv/bin/python
# 回到基础环境
roslaunch carton_box_yolo carton_box_yolo.launch yolo_python:=/usr/bin/python3
```

⚠️ **回退只能改 `launch` 里 `yolo_python` 的 default，命令行传空串无效。**
roslaunch 的 `load_mappings` 会把 `x:=` 这种空值从映射里丢掉（实测
`load_mappings(['yolo_python:='])` 返回 `{}`），arg 保持默认值 —— 你以为切回去了，
其实没有，**没有任何提示**。命令行要覆盖就写个真解释器。

**挂 venv 时那个环境需要什么**（`source devel/setup.bash` 只把 ROS 那几个目录放进
`PYTHONPATH`，**不会**装进 venv）：

| 包 | 为什么 |
|---|---|
| `rospkg` | `rospy` → `roslib` → `rospkg` 是硬链，缺了整个节点起不来 |
| `onnxruntime-gpu` | 推理 |
| `opencv` / `numpy` | 纯函数层 `carton_detector.py` 顶层就 import |

⚠️ **onnxruntime 与 numpy 的版本必须配套**（实测，不是推断）：

```text
numpy 1.x  + onnxruntime 1.15.1  →  ✅ 推理正常
numpy 2.x  + onnxruntime 1.15.1  →  ❌ import 就 _ARRAY_API not found，继续跑是段错误
numpy 2.x  + onnxruntime 1.19.2  →  ✅ 推理正常
```

1.15 / 1.16 / 1.17 都是拿 numpy 1.x 编的。

**怎么确认跑的是哪个解释器**：节点启动那行 `onnxruntime x.y.z，provider=…` ——
版本号能对上哪个环境里的 onnxruntime，就是跑了哪个环境。基础环境那份是
`~/.local/lib/python3.8/site-packages`（本机是 **1.16.3**），也是**唯一在 CPU 上跑**
的那份（它要 CUDA 11 的 cublas/cudnn，而本机只有 CUDA 12/13）—— 看到
`provider=CPUExecutionProvider` 且版本 1.16.3，就说明走的是基础环境。

# 4. 相关文档

- 下游消费方：`skills/atomic/perception/box_frame/`（消费 `/box/yolo_box`，发 `/box/detection`）
- 再下游：`skills/atomic/perception/pallet_servo/NOTES.md` §11、§13.5
- 权重来源与训练数据：`maduo/test_data/横竖抓_rgb/`（yolo26n，单类 `carton`）
