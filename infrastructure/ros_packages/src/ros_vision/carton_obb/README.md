# 纸箱顶面四角（OBB → `/box/detection`）

从相机彩色图里检出**手里那个纸箱的顶面四角**（旋转四边形），发到 `/box/detection`。

这是「纸箱」那条链路的**唯一一环** —— 一个节点直接从网络出四角，
不需要「轴对齐框 → 框内深度边缘拟合」两段。

```text
bag / 相机 ──/camera/color/image_raw──→  carton_obb  ──/box/detection────────┐
           ──/camera/depth/image_raw──┘   (ONNX OBB)                         │
                            （深度只用于**选箱打分**）                        ↓
                                                                    NodeBoxObs
                                                                        ↓
                                                              latest_box_obs（黑板）
                                                                        ↓
                                                                NodePalletServo
                                                     （叠加图上的灰框也从它算）
```

⚠️ **只发一条话题。** 从前还发一路 `/box/yolo_box`（`PolygonStamped`，给叠加图画
灰框），2026-10-15 删了：它就是这四角的**轴对齐包围盒**，而黑板上的
`BoxObservation.u1/v1/u2/v2` 本来就是同一个数（`node_box_obs` 用 `min/max` 填的），
所以删掉零信息损失，伺服那边也不用加订阅。

| 文件 | 作用 |
|---|---|
| `scripts/carton_obb_detect.py` | ROS 节点：订阅图像 + 深度 → ONNX 推理 → 选箱 → 时间窗平均 → 发话题。只做搬运 |
| `tests/test_node_static.py` | 节点静态自检（读源码，不需要 ROS / 模型 / onnxruntime） |
| `config/carton_obb.yaml` | 参数默认值 |
| `launch/carton_obb.launch` | 起节点的 launch |

⚠️ **算法和权重都不在这里。** 纯函数核心（letterbox / OBB 解码 / 旋转 NMS / 选箱打分 /
角度）2026-10-15 搬到了 `skills/atomic/perception/carton_detect/` —— 与托盘那条链
同构：**算法进 `skills/`、ROS 壳进 `infrastructure/`**。本包只订阅话题、跑推理、发话题。

**权重与导出脚本也跟着算法走**（2026-10-19）：`carton_detect/models/carton_obb.onnx`
与 `carton_detect/scripts/export_onnx.py`。默认权重路径由算法包给出
（`carton_detect.DEFAULT_MODEL_PATH`），本节点只取用 —— 与托盘那条链一致
（那边是 `pallet_detect.detect.DEFAULT_MODEL_PATH`）。

## TensorRT 共享推理服务

`carton_obb_fp16.engine` 由独立节点常驻加载。节点直接订阅公共头部相机并缓存
最新彩色帧，多个调用方通过 `/carton_obb/infer` 请求推理，避免每个业务进程重复
加载 engine、占用显存或自行启动相机：

```bash
catkin build box_detection_msgs carton_obb
source devel/setup.bash
roslaunch carton_obb carton_obb_trt_service.launch \
  engine_path:=<LeTools 仓库根>/skills/atomic/perception/carton_detect/models/carton_obb_fp16.engine
```

服务类型是 `box_detection_msgs/InferCartonObb`。请求只包含置信度和 NMS 阈值；
服务端强制使用 `/camera/color/image_raw` 中接收的最新帧，并拒绝陈旧帧。响应返回
实际推理帧的 `source_header`、图像尺寸、原图坐标下的旋转框数组
`center_x/center_y/width/height/angle_deg/scores`，以及纯 TensorRT 推理耗时。
服务端只负责模型前处理、推理、NMS，不负责深度采样或选箱，因此不同业务可以使用
自己的目标选择策略。调用方应使用 `source_header.stamp` 匹配同一时刻的彩色图和
配准深度图，不能假定请求发起时本地看到的帧就是服务端推理帧。

服务端收到头部相机首帧后才加载并预热模型，退出时显式注销相机订阅并释放 CUDA
buffer。调用由 backend 内部锁串行化，因为同一个 TensorRT execution context 和
CUDA buffer 不能并发复用。

### 本节点怎么用它（`~infer_mode`，2026-10-20）

`carton_obb_detect` 启动时探测 `/carton_obb/infer`：

| `~infer_mode` | 行为 |
|---|---|
| `auto`（默认） | 服务在 → 用它的输出；不在 → 自己加载 `models/carton_obb.onnx` |
| `shared` | 强制用服务；探测不到打 ERROR 并退回本地 |
| `local` | 强制本地 ONNX，不探测 |

⚠️ **两条路的输出必须逐位同口径**，所以分工是刻意划死的：

* 请求固定传 `nms_iou=1.0` —— 让服务端**不要 NMS**，去重只在本地做一次
  （`nms_rotated` 的 IoU + IoMin 两条判据；服务端只有 IoU，栅格化口径也不同）。
* 服务响应 → `carton_detect.dets_from_rotated_rects()` → 与 `decode_obb` 同字段的
  候选列表（置信度过滤、降序截断 `max_det`、四角排序、越界过滤都在里面，
  有与 `decode_obb` 的逐字段等价性测试）。
* 时间窗、选箱、发布**是同一段代码**，不按后端分叉。
* 发布 header 用**响应里的 `source_header`**（服务端推理的是它缓存的最新帧，不是
  本节点队列里那一帧），深度按该 stamp 精确配对，配不上才退回最新一帧。
* 服务端帧率低于本节点处理节奏时，同一 stamp 不重复进时间窗。

⚠️ 探测只在启动时做一次；服务中途挂掉，连续失败 `~shared_fail_limit`（默认 3）
次后切回本地 ONNX，**不自动切回**（来回切会让时间窗里混着两个来源的帧）。
`srv` 没编译进工作空间时只告警、退回本地，节点照常能起。

# 1. 接口

## 输入

### `~image_color`（`sensor_msgs/Image`，默认 `/camera/color/image_raw`）

彩色图，认 `bgr8` / `rgb8` / `mono8`。**编码不认识会直接报错，不猜** —— 猜错了
通道顺序反了框会偏，而且看不出来。

### `~image_depth`（`sensor_msgs/Image`，默认 `/camera/depth/image_raw`）

深度图，认 `16UC1`（**已经是毫米**）与 `32FC1`（**米**，节点自动 ×1000）。
用于**选箱打分**（见 §3 ⑥）与**深度下沿修正**（`depth_refine_bottom`，默认开，
只在明确跳变时收缩下边）。取不到深度**不影响四角的正确性** —— 打分退回无深度
口径，下沿修正也不执行。

⚠️ **要求深度已配准到彩色**。没配准的话采样到的不是这个箱子的深度，打分就是噪声。
**给空串 = 不订阅深度**，全部候选走"无深度"降级。

### `~camera_info`（`sensor_msgs/CameraInfo`，默认 `/camera/color/camera_info`）

**OBB 推理不需要内参**，节点只用它打一行启动日志（换相机时能对上）。仍然订阅，
免得换了相机/分辨率之后没有任何地方留下痕迹。

## 输出

### `/box/detection`（`box_detection_msgs/BoxDetection`）—— **伺服吃这个**

| 字段 | 说明 |
|---|---|
| `header` | **沿用输入图像的时间戳与 `frame_id`**（不做时间换算，下游靠它跟托盘位姿配对） |
| `corners_uv[4]` | **右下 → 左下 → 左上 → 右上**（`BoxDetection.msg` 定死的契约）。z 恒为 0 |
| `valid` | **恒 `true`** —— 本节点只在有结果时发（`publish_empty=false` 时没检到就一个字都不发） |
| `source` | `'single'`（窗里只有 1 帧）/ `'window'`（多帧平均）。**没有兜底档** |
| `angle_deg` | **下边**（左下 → 右下）的方向角，折进 (−90, 90]。见 §3 ⑤ |
| `box_uv[2]` | `[0]`=左上 `(u_min,v_min)`、`[1]`=右下 `(u_max,v_max)`，是四角的**外接框** |
| `spread_px` | 窗内各帧四角到平均值的最远距离（判断这一窗稳不稳） |
| `n_used` / `n_slots` / `n_failed` | 参与平均的帧数 / 窗长 / 失败槽数 |
| `orientation` | **恒空串**（见 §3 ⑯） |
| `rejects` | 恒空串（没有兜底档，也就没有失败原因可报） |

⚠️ **没有「退回外接框」这一档。** 拿外接框当四角发出来是个静默陷阱：四角看着
规规矩矩是个矩形，`theta` 却是错的，下游照样算得出三个数。本节点**没检到就不发**，
下游继续用上一帧的好值。


## 节点参数

| 名字 | 默认 | 说明 |
|---|---|---|
| `image_color` | `/camera/color/image_raw` | 输入彩色图 |
| `image_depth` | `/camera/depth/image_raw` | 深度图，用于选箱打分与可选的深度下沿修正；空串 = 不订阅 |
| `camera_info` | `/camera/color/camera_info` | 内参（只记日志） |
| `detection_out` | `/box/detection` | 四角输出 |
| `model_path` | `<算法包>/models/carton_obb.onnx` | ONNX 权重（**留空 = 用默认值**，见 `carton_detect.DEFAULT_MODEL_PATH`） |
| `imgsz` | `640` | 输入边长，**必须与导出时一致** |
| `conf` | `0.25` | 置信度阈值。**只用于过滤与 NMS 排序，不参与选箱打分** |
| `nms_iou` | `0.45` | 旋转框 NMS 阈值：两个框**互相错着盖**。见 §3 ② |
| `nms_iomin` | `0.85` | 旋转框 NMS 的第二条：**小框整个套在大框里**（`交/较小面积`）。见 §3 ② |
| `window` | `3` | 滑动窗长（处理槽数） |
| `process_every` | `1` | 每几帧进一个处理槽（相机 30Hz → 30Hz）。**它决定 `/box/detection` 的上限，而 `/pallet/dis` 被 min(托盘率, 纸箱率) 封顶**（2026-10-19：默认 3 时纸箱 7.8Hz，把 `/pallet/dis` 拖到 7~8Hz）。要降载改这里：2 = 15Hz、3 = 10Hz |
| `agg` | `mean` | 多帧聚合：`mean` / `median` |
| `max_rate_hz` | `0.0` | >0 时降频 |
| `publish_empty` | `false` | 没检到时要不要发一帧空消息 |
| `queue_size` | `1` | 订阅队列，只要最新帧 |
| `infer_mode` | `auto` | `auto` / `shared` / `local`，见「TensorRT 共享推理服务」 |
| `infer_service` | `/carton_obb/infer` | 公共服务名 |
| `infer_probe_s` | `2.0` | 启动时等服务出现的秒数 |
| `shared_fail_limit` | `3` | 共享服务连续失败几次后切回本地 |

**选箱**（见 §3 ⑥）：`score_w_dcenter` `0.5`、`score_w_area` `0.2`、`score_w_depth` `0.3`、
`d_near_px` `100.0`、`d_far_px` `450.0`、`z_near_mm` `500.0`、`z_far_mm` `1500.0`、
`depth_patch_px` `15`。

**深度下沿修正**（2026-11-06，默认开）：`depth_refine_bottom` `true` →
`depth_refine_step` `2`、`depth_refine_band_px` `45`、`depth_refine_jump_mm` `150.0`、
`depth_refine_min_shrink_px` `3.0`。只在「框内箱体 → 下方远背景」有持续深度跳变时
把下边收回去；没有跳变（箱子放在料垛上、下沿出画）保持模型框，只收缩不放大。
用于修手持箱被拉长到下方背景的情况，见 `obb_decoder.refine_bottom_by_depth`。

⚠️ **关闭方式**：`config/carton_obb.yaml` 里的同名键会被 `carton_obb.launch`
的 `<param>`（arg 默认 `true`）覆盖。在启动命令或场景脚本里传
`depth_refine_bottom:=false`，或把 launch 里的 arg 默认值改成 `false`。

⚠️ **所有参数都是私有参数**（`~name`，即 `/carton_obb_detect/<name>`）。
launch 里 `<param>` / `<rosparam>` **必须写在 `<node>` 标签内部** —— 写在
`<launch>` 顶层会落到全局 `/name`，节点一个都读不到，**静默用代码里的默认值**。

# 2. 快速测试

## 2.1 纯函数自检（最快，什么都不需要）

算法已经搬到 `skills/atomic/perception/carton_detect/`，所以自检也跟着走：

```bash
cd <仓库根>
python3 skills/atomic/perception/carton_detect/tests/test_obb_decoder.py   # 0 通过 / 1 失败
```

装了 ultralytics 的话会多跑一条**逐像素比对**（跟 `ultralytics.data.augment.LetterBox`
比 5 组不同尺寸）。**那条才是真正的判据** —— 手算的断言只能证明「跟我以为的一样」。

**本包自己的自检**（读源码，不需要 ROS / 模型）：

```bash
cd <仓库根>
python3 infrastructure/ros_packages/src/ros_vision/detection_industrial_yolo/\
carton_obb/tests/test_node_static.py
```

## 2.2 离线验一遍推理（不需要 ROS）

```bash
cd <仓库根>
export OMP_NUM_THREADS=1
python3 -c "
import sys; sys.path.insert(0,'.')
import cv2, onnxruntime as ort
from skills.atomic.perception.carton_detect import (
    to_input_tensor, decode_obb, nms_rotated, pick_best)
s = ort.InferenceSession('skills/atomic/perception/carton_detect/'
                         'models/carton_obb.onnx',
                         providers=['CPUExecutionProvider'])
img = cv2.imread('/path/to/color_frames/xxx.png'); h, w = img.shape[:2]
x, r, pad = to_input_tensor(img, 640)
raw = s.run(None, {'images': x})[0][0].T          # ★ 转置成 (8400, 6)
dets = nms_rotated(decode_obb(raw, r, pad, conf_thr=0.25, orig_shape=(h, w)), 0.45, (h, w))
# 第三个参数之后是关键字参数；默认 iomin_thr=0.85 已经开着（见 §3 ②）
best = pick_best(dets, ref_uv=(w/2.0, float(h)))
print(best['corners_uv'], best['confidence'])
"
```

## 2.3 端到端（rosbag 回放，不需要真机）

**这是最接近真机的一条**：`rosbag play` 出图 → 本节点 → `NodeBoxObs` → 伺服。

```bash
# 终端 1
roscore

# 终端 2：回放（循环）。注意深度话题是 Orbbec 的命名
rosbag play -l <你的包>.bag

# 终端 3：本节点
roslaunch carton_obb carton_obb.launch

# 终端 4：看结果
rostopic echo /box/detection          # 四角 + valid + angle_deg
```

判据是 `/box/detection` 的 `source` 是 `window`（3 帧平均）或 `single`（窗里只有 1 帧）。

## 2.4 导出权重

在**有 ultralytics 的开发机**上跑一次（机器人上不需要 ultralytics）：

```bash
cd skills/atomic/perception/carton_detect/scripts
OMP_NUM_THREADS=1 python3 export_onnx.py --weights best.pt --verify
```

# 3. 注意事项

**① ONNX 的输出格式是实测定的，换模型要重验。** 导出图 `output0` 形状
`(1, 6, 8400)` —— **6 在 8400 前面，解码前要转置**。每行
`[cx, cy, w, h, conf, angle]`：前四列是**中心 + 宽高**（不是角点），单位是
letterbox 之后的 640 空间；第 6 列是**原始弧度**。

三种解码假设在同一批帧上实测过（对真值的旋转 IoU 中位）：直接把第 6 列当弧度
**0.8728**；按 `(a-0.25)*pi/2` 解 **0.6812**；按 `(sigmoid(a)-0.25)*pi/2` 解
**0.6524**。第一种是对的。

⚠️ **yolo26 的 `OBB26` 头对角度做 sigmoid**，导出的图第 6 列会落在 [0,1] ——
换模型一定要看 `export_onnx.py --verify` 打出来的样例值。

**② 导出图里没有 NMS，也没有 query 上限。** 8400 个锚点行全在，同一目标会出
2~3 行 IoU>0.9 的重复框。实测 val 330 帧：NMS 前中位 **4** 个候选、最大 **22** 个；
NMS 后中位 1 个。**不去重下游会把一个箱子当成好几个。**

⚠️ **光有 IoU 不够：模型还会出「只框了箱子一角」的小框**（整个套在大框里）。
那类 `IoU = 交/并 ≈ 小面积/大面积`，实测碎框 IoU **0.25**、IoMin **1.0** ——
**IoU 阈值给多低都放它过去**。所以 NMS 有第二条判据 `~nms_iomin`
（`交/较小面积`）。抑制按置信度降序，留下的是**置信度最高**的那个；
`>= 1.0` = 关掉。阈值 0.85 的实测依据写在 `config/carton_obb.yaml` 里。

**③ 旋转 IoU 必须真的算旋转交集，不能用轴对齐近似。** 同中心、同大小的正方形
差 45°：旋转 IoU = **0.7071**，而 AABB 近似只给 **0.5** —— 差 0.2，足以让 NMS
判错。`nms_rotated` 按栅格化算（640×480 上够快也够准），且**掩码每个候选只算
一次**再在循环里复用 —— 逐对重算会慢一倍多（实测 745 个候选：15.4 → 10.1 ms/张），
而 IoU 与 IoMin 两条判据共用同一次求交。

**④ 坐标反变换是「先转四角、再减补边」。** 反过来（先在 640 空间减补边再转四角）
会得到错的框 —— 同一批帧上实测 IoU 从 0.99 掉到 **0.66**。`decode_obb` 里那两行
别动。

**⑤ `angle_deg` 用的是下边，不是 OBB 的角度。** 契约是「左下 → 右下」的方向角；
OBB 输出的是「长边相对水平」的角度 —— 箱子转 90° 时两者差一个直角。
`carton_detect.angle_deg_from_corners` 就是这条契约（在
`skills/atomic/perception/carton_detect/`）。

**⑥ 选箱：到「画面下方中心」的距离是主判据，置信度不参与。** 相机固定俯视，
手里的箱子永远在画面下方中心（操作员 2026-10-13 确认）。三项归一化后加权求和：

```text
score = w_dcenter * d_center_score(d_px)          # 绝对刻度，越近越高
      + w_area    * (area_px / 同帧最大面积)        # 同帧相对归一
      + w_depth   * depth_score(z_mm)             # 越近越高
```

置信度只用于**过滤**（`conf`）和**NMS 排序**，不进公式。实测反例：现场 val 的
36 个多框帧里有一帧手箱 conf 0.9773、邻箱 **0.9642**（分不开），而**邻箱面积还
更大**（46147 > 45118）——「conf 最高」和「面积最大」都会挑错，「离下方中心最近」
挑对了。

距离的分离度实测：手标双箱 10 帧中位 **192.9px**（最小 191.5）；模型预测双箱帧
中位 127.7px。⚠️ **`d_near_px` / `d_far_px` 是 640×480 口径的绝对刻度**，
换分辨率要跟着改。

**⑦ 取不到深度时把深度项的权重按比例让给另两项**（不罚、不丢候选）：

```text
score = (w_dcenter*d + w_area*a) / (w_dcenter + w_area)
```

深度空洞往往是近物遮挡造成的，罚它会适得其反。代价是归一化到 `[0,1]`，
所以深度项**只能压低远处的候选，不能额外抬高近处的**。

**⑧ 越界的框会被丢掉。** `decode_obb` 给了 `orig_shape` 就把**完全跑出图外**的
框滤掉。OBB 会把被画面边界裁掉的目标拟合成完整矩形，四角常常伸到画面外 ——
val 330 帧实测 **145 个框越界**。

**⑨ 中心深度取方块中位数，不是中心那一个像素。** 深度图到处是空洞与飞点
（黑箱子、反光、遮挡边缘），单点很容易落空，一帧有值一帧没值会让排序跳。

**⑩ `bool("false") is True`。** `publish_empty` 这类参数走的是 `_as_bool()`，
不是 `bool()` —— rosparam / 场景 JSON 真会传字符串。

**⑪ `OMP_NUM_THREADS=1` 是必须的。** onnxruntime / OpenBLAS 在这个规模的模型上
开多线程是纯开销，而且会和 ROS 回调线程抢核。`launch` 里已经用 `<env>` 设好。

**⑫ GPU 的 cuDNN 要自己给。** onnxruntime 的 CUDA provider **不会**自己去 conda
环境的 `site-packages/nvidia/` 里找库。不给的话**不报错，只是静默退回 CPU**
（同机实测 4.0ms vs 9.7ms）：

```bash
L=<conda-env>/lib/python3.*/site-packages/nvidia
export LD_LIBRARY_PATH=$L/cudnn/lib:$L/cublas/lib:$L/cuda_runtime/lib:$LD_LIBRARY_PATH
```

**本仓库不设这个变量**（它是机器相关的）。判据是启动日志里那行
`onnxruntime x.y.z，provider=...` —— 不是 `CUDAExecutionProvider` 就是退回 CPU 了。

**⑬ `onnxruntime-gpu` 对 python 3.8 只到 `1.16.3`**（更高版本没有 cp38 轮子）。
1.16.3 要 **CUDA 11**；1.17+ 要 CUDA 12。**换机器前先确认 CUDA 版本再选版本**，
装错了是静默退回 CPU（同 ⑫）。另外 **onnxruntime 与 numpy 的版本必须配套**：
numpy 2.x + onnxruntime 1.15/1.16/1.17 → import 就 `_ARRAY_API not found`。

**⑭ 权重不进 git。** `models/` 在 git 里只有 `.gitignore` 和 `README.md`。
导出与拷贝见 `models/README.md`。

**⑮ 本节点跑在哪个解释器里。** 由 `launch` 的 `box_python` arg 经 `launch-prefix`
指定，**空串 = 系统 python3**（默认）。要挂 venv 就填解释器的**绝对路径**：

```bash
roslaunch carton_obb carton_obb.launch box_python:=/path/to/env/bin/python
```

⚠️ **命令行传空串是无效的** —— roslaunch 的 `load_mappings` 会把 `x:=` 这种空值
从映射里丢掉，arg 保持默认值，**没有任何提示**。

⚠️ **`activate` 对节点无效。** catkin 生成的 node wrapper 顶上写死了
`#!/usr/bin/python3`，而 `roslaunch/node_args.py` 是**直接 exec 那个文件**的，
不查 PATH，也不看你当前激活了什么。

挂 venv 时那个环境里要有：`rospkg`（`rospy` → `roslib` → `rospkg` 是硬链）、
`onnxruntime`、`opencv`、`numpy`。

**⑯ `orientation` 恒为空串。** 它是给「矩形搜索里整框转 90°」那个歧义留的痕迹位。
OBB 直接给出角度，没有那个歧义 —— 上游的朝向先验服务（`/infer_carton_pose`）
本节点**不接**。

# 4. 相关文档

- 消费方：`orchestration/nodes/node_box_obs.py`（写黑板 `latest_box_obs`）
- 再下游：`orchestration/nodes/node_pallet_servo.py`（算 `e_bottom_px` / `e_right_px` / `theta`）
- 消息定义：`infrastructure/ros_packages/src/ros_vision/box_detection_msgs/`
- 训练数据格式：`data_obb.yaml`，单类 `carton`，四点格式
- 链路总览：`orchestration/scenarios/pallet_servo_real_v1/README.md`
