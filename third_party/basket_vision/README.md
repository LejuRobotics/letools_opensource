# Basket Vision — 5 类箱体 6D 位姿推理

基于 **YOLO + GDRN++** 的箱体检测与位姿估计，通过 ROS Service 接口暴露推理结果，同时以伪 AprilTag 形式发布到 `/tag_detections` 兼容现有二维码作业流程，并通过 `/basket_vision/viz_image` 发布带坐标轴的可视化图像。

支持 **5 类箱体**：basket_4322 / basket_4622 / basket_4611 / basket_4633 / basket_4311，**自由摆放**（不依赖预设槽位）。

---

## 1. 文件结构

```
third_party/basket_vision/
├── README.md
├── __init__.py                          # 导出 BasketVisionMixin
├── hardware_mixin.py                    # LejuWheeledArmHardware 混入
├── sdk/
│   ├── __init__.py                      # 导出 BasketVisionClient
│   └── basket_vision_client.py          # ROS SDK 客户端
├── basket_vision_ws/src/basket_vision_msgs/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/InferBasketPose.srv          # ROS 服务定义
├── basket_vision_module/scripts/
│   ├── start_gdrn_inference.sh          # 启动 GDRN++ 推理节点
│   └── start_basket_tf_fallback.sh      # TF 断链 fallback
├── basket_gdrnpp/                       # 推理引擎根目录（proj_root）
│   ├── core/
│   │   ├── gdrn_modeling/               # GDRN++ 模型定义 + 引擎
│   │   │   ├── engine/                  # 推理、评估引擎
│   │   │   ├── models/                  # 网络结构定义
│   │   │   ├── datasets/               # 数据集注册
│   │   │   ├── losses/                  # 训练 loss（模型构建依赖）
│   │   │   ├── main_gdrn.py            # Lite 入口
│   │   │   └── demo/
│   │   │       ├── inference_service_vis_2_mult_inst_10_shared.py  # 主推理节点 ★
│   │   │       ├── predictor_gdrn.py    # GDRN++ 预测器 ★
│   │   │       └── box_configs/
│   │   │           └── basket_5.yaml    # 5类箱体配置文件
│   │   ├── utils/                       # 工具函数
│   │   └── csrc/                        # C 扩展（懒加载）
│   ├── lib/                             # 底层库（pysixd, 渲染, 可视化, 工具）
│   │   ├── pysixd/                      # BOP 数据 IO
│   │   ├── render_vispy/               # vispy 模型渲染
│   │   ├── vis_utils/                   # 可视化工具
│   │   └── utils/                       # 通用工具
│   ├── ref/                             # 物体映射
│   │   └── basket.py                    # 5类 id2obj 映射
│   ├── output_basket_5/                 # GDRN 权重 + config
│   ├── yolo_basket_5_weights/           # YOLO 权重
│   └── datasets/                        # CAD 模型 + 相机参数
├── docs/
│   └── basket_vision_migration_and_test_guide.md
└── logs/
```

### 核心文件说明

| 文件 | 作用 |
|------|------|
| `inference_service_vis_2_mult_inst_10_shared.py` | 主推理节点：加载 YOLO+GDRN++，暴露 2 个 ROS Service + 2 个 Topic |
| `predictor_gdrn.py` | GDRN++ 预测器，管理模型加载、前处理、推理、后处理 |
| `ref/basket.py` | 5 类箱体映射 `id2obj = {1: "basket_4322", ..., 5: "basket_4311"}`（位于 `basket_gdrnpp/ref/`） |
| `basket_5.yaml` | 推理配置：模型路径、置信度阈值、过滤参数 |
| `basket_vision_client.py` | Python SDK 客户端，封装 ROS Service 调用 |
| `start_gdrn_inference.sh` | 启动脚本：环境变量 → 加载模型 → 启动 ROS 节点 |
| `hardware_mixin.py` | SDK 方法提供者，嵌入 `LejuWheeledArmHardware` |

---

## 2. 推理流程

```
相机话题 (/camera/color/image_raw + /camera/depth/image_raw)
       │
       ▼
  RGB/Depth 回调缓存（线程安全 lock）
       │
       ▼  ROS Service 调用触发
┌──────┴──────────────────────────────────────┐
│  run_shared_pipeline(rgb, depth, cfg)        │
│                                              │
│  1. YOLO 检测 → bbox + cls_id + conf        │
│  2. bbox 尺寸过滤 (min_w/min_h)             │
│  3. 模式过滤:                                │
│     • single: filter_center_column(按中心列) │
│     • top:    filter_by_area_only(按面积)    │
│  4. 逐个实例 GDRN++ 推理:                    │
│     ├─ YOLO cls_id → GDRN obj_id 映射       │
│     ├─ preprocessing → inference → post     │
│     ├─ 对称性最优方向选择                     │
│     ├─ 坐标变换 (camera→base_link)     │
│     └─ base 偏移修正                         │
│  5. roll/pitch 合理性检查                    │
│  6. 位置范围过滤 (x_only / xy)              │
└──────────────────────────────────────────────┘
       │
       ├─► /infer_basket_pose (single)
       │     • 选最高层 z 的最远 (min xy) 实例
       │     • 返回 1 个 Pose
       │
       └─► /infer_top_basket_ids (top)
             • 选最高层 z 的所有实例
             • 估算层数/总箱数
             • 返回 N 个 Pose（return_single_target 时返回 1 个）
       │
       ▼  同时
  ┌─────────────────────────┐
  │ AprilTag 变换管道        │
  │ 1. correct_box_pose     │ 点积判断正反 → 绕局部 Y 翻转 180°
  │ 2. apply_vision_to_tag  │ 绕局部 Y 旋转 -90° → 伪装 AprilTag 坐标系
  │ 3. 组合 data_id+flag    │ 编码为 5 位 ID (data_id[3位]+class[2位])
  └────────┬────────────────┘
           ▼
  /tag_detections (AprilTagDetectionArray)
```

---

## 3. 模型部署

### 需要准备的文件

| 文件 | 放在哪里 | 说明 |
|------|---------|------|
| GDRN 配置 `.py` | `basket_gdrnpp/output_basket_5/` | config 文件 |
| GDRN 权重 `.pth` | `basket_gdrnpp/output_basket_5/model_final.pth` | 训练好的 checkpoint |
| YOLO 权重 `.pt` | `basket_gdrnpp/yolo_basket_5_weights/best.pt` | 5 类 YOLO 检测器 |
| CAD 模型 `.ply` | `basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_00000{1,2,3,4,5}.ply` | 5 个箱体 3D 模型 |
| `models_info.json` | 同上目录 | 物体尺寸信息 |
| `camera.json` | 启动时自动生成 | 相机内参 |

proj_root 默认指向 `third_party/basket_vision/basket_gdrnpp/`，所有相对路径由此解析。

### Python 环境依赖

> 以下 pip/conda 包需要在运行环境中提前安装：

```bash
pip install empy==3.3.4      # ROS catkin_make 代码生成依赖（必须 3.x，4.x 不兼容 noetic）
```

### ROS 包依赖

```bash
sudo apt install ros-noetic-apriltag-ros    # AprilTag 消息类型（/tag_detections 话题发布）
```

### 模型权重来源

训练好的 5 类模型位于：
```
/data/Real_Downloads/12_29_occ_5/12_29_near_occ_5/gdrnpp_est/output/gdrn/basket/
├── convnext_a6_..._basket/   ← GDRN model_final.pth, config
└── best_5.pt                 ← YOLO 5-class detector
```

部署时将上述文件复制到 `basket_gdrnpp/` 下对应子目录即完成。

---

## 4. 输入输出

### 输入（订阅）

| Topic | 类型 | 说明 |
|-------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参（启动时读取一次） |

### 输出 1: ROS Service

**`/infer_basket_pose`** — 单箱抓取

```
Request:  {}
Response: InferBasketPoseResponse
  bool  success
  string message
  Pose[] poses_camera_link    # camera_color_optical_frame
  Pose[] poses_base_link      # base_link
  float32[] bbox_xyxy         # [x1, y1, x2, y2, ...]
  uint32 num_instances         # =1
  float32[] yaw                # [yaw_rad]
```

**`/infer_top_basket_ids`** — 顶层识别 + 层数估算

```
Request:  {}
Response: InferBasketPoseResponse
  bool  success
  string message              # 含 basket_type, total_count, layer_count
  Pose[] poses_*              # 同上
  uint32 num_instances         # 返回数量
```

### 输出 2: ROS Topic（AprilTag 兼容）

**`/tag_detections`** — `apriltag_ros/AprilTagDetectionArray`

```
header:
  frame_id: camera_color_optical_frame
detections[]:
  id: [combined_id]           # data_id(3位) + basket_class(2位), 如 60102
  size: [10]
  pose:
    header.frame_id: camera_color_optical_frame
    pose.pose:
      position: {x, y, z}     # 相机坐标系
      orientation: {x,y,z,w}  # 归正+旋转后的 AprilTag 坐标系
```

### 输出 3: SDK (Python)

```python
from module_internal.basket_vision import BasketVisionMixin
# 通过 LejuWheeledArmHardware 调用
hw.wait_basket_vision_ready()
result = hw.infer_basket_pose()   # Result{success, message, data}
result = hw.infer_top_basket()    # Result{success, message, data}

# result.data 结构:
{
  "success": true,
  "num_instances": 1,
  "poses_camera_link": [...],
  "poses_base_link": [...],
  "bbox_xyxy": [...],
  "yaw": [...],
  "baskets": [{index, pose6d, bbox, ...}],
  "embodied_compat": {source, t_base, q_base, t_camera, yaw_rad, ...},
  "target": {...},            # 仅 infer_top_basket
}
```

### 坐标约定

| 坐标系 | x | y | z |
|--------|---|---|---|
| `camera_color_optical_frame` | 图像右方 | 图像下方 | 相机前方 |
| `base_link` | 机器人前方 | 机器人左侧 | 机器人上方 |

---

## 5. 配置

### 环境变量（启动脚本）

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `KUAVO_STUDIO_DIR` | 从脚本路径自动推算 | 项目根目录。若推算错误需手动指定 |
| `UV_ACTIVATE_SCRIPT` | `.../basket_gdrnpp/gdrn/bin/activate` | Python 虚拟环境 activate 脚本路径 |
| `BASKET_BOX_CONFIG_YAML` | `core/gdrn_modeling/demo/box_configs/basket_5.yaml` | 配置文件路径 |
| `BASKET_INFERENCE_IMAGE_WIDTH` | `640` | 推理缩放目标宽度（设为 `0` 禁用缩放） |
| `BASKET_INFERENCE_IMAGE_HEIGHT` | `480` | 推理缩放目标高度（设为 `0` 禁用缩放） |
| `BASKET_VISION_RUN_ID` | 时间戳 | 运行 ID |
| `BASKET_VISION_LOG_ROOT` | `logs/` | 日志根目录 |
| `TORCH_HOME` | `.../torch` | PyTorch 缓存 |

> **关于 `UV_ACTIVATE_SCRIPT`**：启动脚本期望 `basket_gdrnpp/gdrn/bin/activate` 路径存在。如果你用的是 conda 环境（如 `conda activate gdrn`），可以设置 `export UV_ACTIVATE_SCRIPT=/path/to/conda/envs/gdrn/bin/activate` 来覆盖。或者直接 source ROS 环境和 `basket_vision_ws/devel/setup.bash` 后手动运行 Python 脚本（见下方 6.2 节"手动启动"）。
>
> **关于 `KUAVO_STUDIO_DIR`**：脚本默认通过 `$(cd ../../../.. && pwd)` 推算。如果文件夹层级不符合预期，手动 `export KUAVO_STUDIO_DIR=/your/project/root` 即可。
>
> **关于推理分辨率**：默认 `640×480`（4:3），与 GDRN 模型训练时使用的图像分辨率一致。该模块使用 letterbox 策略：等比例缩放原始图像使其不超出目标尺寸，然后用黑边填充至 640×480，相机内参同步缩放并加偏移。这样不会拉伸变形，同时模型看到的内参量和训练时一致。如果相机本身就是 640×480，该选项自动跳过不做任何处理。

### 主要 YAML 参数（basket_5.yaml）

| 参数 | single | top | 说明 |
|------|--------|-----|------|
| `conf_thr` | 0.01 | 0.01 | YOLO 置信度阈值 |
| `max_keep_instances` | 2 | 5 | 进入 GDRN++ 的候选数上限 |
| `min_bbox_area_abs` | 7000 | 2000 | bbox 最小面积 (px²) |
| `base_x_min / base_x_max` | 0.55/2.0 | 0.5/2.3 | base_link x 范围过滤 |
| `base_z_offset_m` | - | -0.10 | base z 偏移 |
| `max_layers` | - | 4 | 最大堆叠层数 |
| `layer_z_threshold` | 0.1 | 0.20 | 顶层 z 判定阈值 |

---

## 6. 使用方法

### 6.1 编译 ROS 消息包

```bash
cd third_party/basket_vision/basket_vision_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
```

> **关于 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`**：新版 CMake（≥3.5）移除了对 `< 3.5` 的兼容。ROS noetic 的 `catkin` 默认生成 `cmake_minimum_required(VERSION 3.0.2)`，不加此参数会报错 `Compatibility with CMake < 3.5 has been removed`。
>
> **关于 `empy`**：`catkin_make` 需要 `empy` 包来生成 ROS 消息代码。必须安装 `3.3.4` 版本（`pip install empy==3.3.4`），因为 4.x 移除了 `em.RAW_OPT` 等 API，与 ROS noetic 的 `gencpp` 不兼容。报错信息为 `AttributeError: module 'em' has no attribute 'RAW_OPT'`。

### 6.2 启动推理节点

**方式一：使用启动脚本（推荐）**

```bash
cd /path/to/kuavo-studio
source /opt/ros/noetic/setup.bash
source third_party/basket_vision/basket_vision_ws/devel/setup.bash

# 如果 KUAVO_STUDIO_DIR 推算不对，手动指定
export KUAVO_STUDIO_DIR=/path/to/kuavo-studio

# 如果用的不是脚本默认的 venv 路径，指定 conda/env 路径
export UV_ACTIVATE_SCRIPT=/path/to/conda/envs/gdrn/bin/activate

# 环境设置（仅 ROS 多机场景需要）
export ROS_MASTER_URI=http://kuavo_master:11311
export ROS_IP=192.168.26.12

# 启动
bash third_party/basket_vision/basket_vision_module/scripts/start_gdrn_inference.sh
```

**方式二：手动启动（conda 环境）**

如果你已经 `conda activate gdrn`，可以直接运行 Python 脚本：

```bash
cd /path/to/kuavo-studio
source /opt/ros/noetic/setup.bash
source third_party/basket_vision/basket_vision_ws/devel/setup.bash

export PYTHONPATH=$(pwd)/third_party/basket_vision/basket_gdrnpp:$PYTHONPATH
export ROS_MASTER_URI=http://localhost:11311   # 按实际情况设置

cd third_party/basket_vision/basket_gdrnpp
python -u core/gdrn_modeling/demo/inference_service_vis_2_mult_inst_10_shared.py \
  _proj_root:=$(pwd) \
  _box_config_yaml:=core/gdrn_modeling/demo/box_configs/basket_5.yaml \
  _save_outputs:=true
```

成功日志：
```
[RESOLUTION] letterbox 1280x800 → 640x480 (content=640x400, scale=0.5000, pad=[40,40,0,0])
[RESOLUTION] scaled intrinsics: fx=304.72, fy=304.71, cx=323.23, cy=201.03
[BOX_CFG] loaded box config: .../basket_5.yaml
[BOX_CFG] gdrn_ckpt .../output_basket_5/model_final.pth
[BOX_CFG] yolo_weights .../yolo_basket_5_weights/best.pt
SharedBasketPoseServiceNode ready.
  service: /infer_basket_pose
  service: /infer_top_basket_ids
  publisher: /tag_detections (AprilTagDetectionArray)
  publisher: /basket_vision/viz_image
[gdrn] inference service is ready
```

> **日志解读**：`[RESOLUTION]` 行表示检测到输入分辨率与训练分辨率不一致，自动做了缩放。如果 native 分辨率就是 640×480，则显示 `[RESOLUTION] using native resolution 640x480`。

### 6.3 启动 TF 变换（如需要）

推理节点需要 `base_link` → `camera_color_optical_frame` 的 TF 变换。如果该变换未由机器人 URDF 发布，使用 fallback 脚本：

```bash
cd /path/to/kuavo-studio
source /opt/ros/noetic/setup.bash
bash third_party/basket_vision/basket_vision_module/scripts/start_basket_tf_fallback.sh
```

> fallback 外参默认值来自 biped_s42 机器人。如果你的机器人不同，需通过环境变量覆盖（`BASKET_CAMERA_TF_X` / `BASKET_CAMERA_TF_Y` / `BASKET_CAMERA_TF_Z` / `BASKET_CAMERA_TF_YAW` / `BASKET_CAMERA_TF_PITCH` / `BASKET_CAMERA_TF_ROLL`）。如果 URDF 已发布正确的 TF 树，此脚本会自动跳过不做任何事。

### 6.4 调用推理

```bash
# 单箱抓取
rosservice call /infer_basket_pose "{}"

# 顶层识别
rosservice call /infer_top_basket_ids "{}"

# 查看 AprilTag 输出
rostopic echo /tag_detections -n 1

# 查看可视化图像（保存到本地，Ctrl-C 停止）
rostopic echo /basket_vision/viz_image -n 1  # 或使用 rqt_image_view 实时查看
```

### 6.5 通过 SDK 调用

```python
from adapters.hardware.factory import HardwareFactory

hw = HardwareFactory.create_hardware({
    "robot_type": "leju_wheeled",
    "basket_vision": {
        "timeout": 10.0,
        "basket_pose_service": "/infer_basket_pose",
        "top_basket_service": "/infer_top_basket_ids",
    },
})

hw.wait_basket_vision_ready()
result = hw.infer_top_basket()
print(result.data["embodied_compat"]["t_base"])  # [x, y, z] in base_link
```

---

## 7. 管道验证检查点

部署后按以下步骤验证：

1. **服务可用**: `rosservice list | grep infer_basket`
2. **相机通**: `rostopic echo -n 1 /camera/color/image_raw`
3. **TF 通**: `rosrun tf tf_echo base_link camera_color_optical_frame`
4. **推理返回**: `rosservice call /infer_top_basket_ids "{}"` 返回 `success: True`
5. **AprilTag**: `rostopic echo /tag_detections -n 1` 有检测结果
6. **多类**: 日志中可见不同 `obj_name`（如 `basket_4322`, `basket_4622`）

### 日志位置

```
logs/gdrn_inference/YYYYMMDD_HHMMSS/
├── start_gdrn_inference.log         # 服务启动和推理日志
├── run_info.txt                     # 本次运行配置
├── images/*_input.jpg               # 输入图
├── images/*_bbox.jpg                # bbox 标注
├── internal/*_response.json         # 每次 service 返回
├── service_outputs/infer_basket_pose/<time>/
└── service_outputs/infer_top_basket_ids/<time>/
```
