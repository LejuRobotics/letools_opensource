# 箱体视觉模块部署与测试指南

> **客户部署入口已更新。** 环境安装和依赖版本以
> [Python 3.8 客户部署基线](basket_vision_python38_deployment.md) 为准；本文保留 ROS 接口、运行测试和故障排查说明，文中的旧目录示例不作为固定安装路径。

本文档分为 4 个部分：

1. **环境准备** — Python 虚拟环境、ROS 消息包、模型权重
2. **配置说明** — basket_5.yaml 参数、环境变量
3. **运行测试** — 启动服务、调用 ROS Service/SDK、检查输出
4. **常见问题** — 排查思路

---

## 一、环境准备

### 1. Python 虚拟环境

推理需要独立的虚拟环境，包含 PyTorch、ultralytics(YOLO)、scipy、opencv-python 等：

```bash
# 在目标机上创建虚拟环境
python3 -m venv /media/data/basket_gdrnpp/gdrn
source /media/data/basket_gdrnpp/gdrn/bin/activate

# 安装核心依赖
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install ultralytics scipy opencv-python pyyaml pillow
```

也可通过 uv 管理（启动脚本支持 `uv run` fallback）。

### 2. 编译 ROS 消息包

```bash
cd /media/data/kuavo-studio/third_party/basket_vision/basket_vision_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如果 CMake cache 指向旧路径：

```bash
rm -rf build devel
catkin_make
source devel/setup.bash
```

### 3. 安装 apriltag_ros

```bash
sudo apt install ros-noetic-apriltag-ros
```

### 4. 部署模型权重

将训练好的权重文件放到 `basket_gdrnpp/` 对应目录：

```
basket_gdrnpp/
├── output_basket_5/                       # GDRN 模型目录
│   ├── convnext_a6_..._basket.py          # GDRN 配置文件
│   └── model_best.pth                     # GDRN 权重
├── yolo_basket_5_weights/                 # YOLO 模型目录
│   └── best.pt                            # 5类 YOLO 检测器
└── datasets/BOP_DATASETS/basket/models/   # CAD 模型
    ├── models_info.json                   # 物体尺寸（5类）
    ├── obj_000001.ply
    ├── obj_000002.ply
    ├── obj_000003.ply
    ├── obj_000004.ply
    └── obj_000005.ply
```

也可创建 symlink 指向已有模型位置。

### 5. 验收检查

```bash
cd /media/data/kuavo-studio

# 轻量文件（已合入仓库）
ls third_party/basket_vision/hardware_mixin.py
ls third_party/basket_vision/sdk/basket_vision_client.py
ls third_party/basket_vision/basket_vision_module/scripts/start_gdrn_inference.sh
ls third_party/basket_vision/basket_gdrnpp/core/gdrn_modeling/demo/box_configs/basket_5.yaml

# 大文件（手动部署）
ls third_party/basket_vision/basket_gdrnpp/output_basket_5/model_best.pth
ls third_party/basket_vision/basket_gdrnpp/yolo_basket_5_weights/best.pt
ls third_party/basket_vision/basket_gdrnpp/datasets/BOP_DATASETS/basket/models/models_info.json
ls /media/data/basket_gdrnpp/gdrn/bin/activate

# 检查模型信息（5类箱体）
python3 - <<'PY'
import json
p = "third_party/basket_vision/basket_gdrnpp/datasets/BOP_DATASETS/basket/models/models_info.json"
data = json.load(open(p))
for obj_id, info in data.items():
    print(f"obj_{obj_id}: size=({info['size_x']:.1f}, {info['size_y']:.1f}, {info['size_z']:.1f})mm, diam={info['diameter']:.1f}mm")
PY

# Python 静态检查
python3 -m py_compile \
  third_party/basket_vision/hardware_mixin.py \
  third_party/basket_vision/sdk/basket_vision_client.py \
  third_party/basket_vision/basket_gdrnpp/core/gdrn_modeling/demo/inference_service_vis_2_mult_inst_10_shared.py
```

---

## 二、配置说明

### basket_5.yaml 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `common.box_height_m` | 0.40 | 单层箱体高度，用于估算层数 |
| `top.conf_thr` | 0.01 | YOLO 置信度阈值 |
| `top.max_keep_instances` | 5 | 进入 GDRN++ 的候选数上限 |
| `top.min_bbox_area_abs` | 2000 | bbox 最小面积 (px²) |
| `top.max_layers` | 4 | 最大堆叠层数 |
| `top.layer_z_threshold` | 0.20 | 顶层 z 判定阈值 (m) |
| `top.return_single_target` | true | 只返回 min\|base_y\| 的实例 |
| `top.base_z_offset_m` | -0.10 | base_link z 偏移 |
| `top.disable_position_filter` | true | 自由摆放，不过滤 xy 位置 |
| `single.conf_thr` | 0.01 | YOLO 置信度阈值 |
| `single.max_keep_instances` | 2 | 进入 GDRN++ 的候选数上限 |
| `single.min_bbox_area_abs` | 7000 | bbox 最小面积 |
| `single.base_x_min/max` | 0.55/2.0 | 筛选 x 范围 |

修改 yaml 或模型后必须重启 GDRN++ 服务。

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `BASKET_BOX_CONFIG_YAML` | `core/gdrn_modeling/demo/box_configs/basket_5.yaml` | 配置文件 |
| `BASKET_VISION_RUN_ID` | 时间戳 | 运行 ID |
| `BASKET_VISION_LOG_ROOT` | `logs/` | 日志根目录 |
| `BASKET_GDRN_USE_PNP` | (未设置) | 启用 PnP 深度优化 |
| `UV_ACTIVATE_SCRIPT` | `...basket_gdrnpp/gdrn/bin/activate` | 虚拟环境路径 |

---

## 三、运行测试

### 1. 每个新终端先 source

```bash
cd /media/data/kuavo-studio
source /opt/ros/noetic/setup.bash
source /media/data/kuavo_ros_application/devel/setup.bash
source /media/data/kuavo-studio/third_party/basket_vision/basket_vision_ws/devel/setup.bash
export ROS_MASTER_URI=http://kuavo_master:11311
export ROS_IP=192.168.26.12
export PYTHONPATH=/media/data/kuavo-studio:/media/data/kuavo-studio/src:$PYTHONPATH
```

不要把 `ROS_MASTER_URI` 写成连在一起的错误值：
```text
http://kuavo_master:11311ROS_MASTER_URI=http://kuavo_master:11311  ← 错误
```

检查导入：
```bash
python3 - <<'PY'
from basket_vision_msgs.srv import InferBasketPose
print("basket vision import ok")
PY
```

### 2. 检查相机和 TF

```bash
# 相机
rostopic list | grep -E "/camera/color/image_raw|/camera/depth/image_raw"
timeout 10 rostopic echo -n 1 /camera/color/image_raw >/dev/null && echo "camera ok"

# TF
rosrun tf tf_echo base_link camera_color_optical_frame
rosrun tf tf_echo base_link waist_yaw_link
```

要求：
- `base_link -> camera_color_optical_frame` 连通
- `base_link -> waist_yaw_link` 最好来自真实机器人 TF（fallback 只作临时排查）

```bash
# TF fallback（临时）
bash /media/data/kuavo-studio/third_party/basket_vision/basket_vision_module/scripts/start_basket_tf_fallback.sh
```

### 3. 启动 GDRN++ 推理服务

先停旧服务：
```bash
pkill -f inference_service_vis_2_mult_inst_10_shared.py
pkill -f start_gdrn_inference.sh
```

启动：
```bash
cd /media/data/kuavo-studio
source /opt/ros/noetic/setup.bash
source /media/data/kuavo_ros_application/devel/setup.bash
source /media/data/kuavo-studio/third_party/basket_vision/basket_vision_ws/devel/setup.bash
export ROS_MASTER_URI=http://kuavo_master:11311
export ROS_IP=192.168.26.12
export PYTHONPATH=/media/data/kuavo-studio:/media/data/kuavo-studio/src:$PYTHONPATH
export UV_ACTIVATE_SCRIPT=/media/data/basket_gdrnpp/gdrn/bin/activate
export UV_CACHE_DIR=/media/data/uv_cache

bash third_party/basket_vision/basket_vision_module/scripts/start_gdrn_inference.sh
```

成功日志应包含：
```text
[BOX_CFG] loaded box config: .../basket_5.yaml
[BOX_CFG] gdrn_ckpt .../output_basket_5/model_best.pth
[BOX_CFG] yolo_weights .../yolo_basket_5_weights/best.pt
SharedBasketPoseServiceNode ready.
  service: /infer_basket_pose
  service: /infer_top_basket_ids
  publisher: /tag_detections (AprilTagDetectionArray)
[gdrn] inference service is ready
```

### 4. 直接测试 ROS service

```bash
rosservice list | grep infer

# 单箱抓取
rosservice call /infer_basket_pose "{}"

# 顶层识别
rosservice call /infer_top_basket_ids "{}"
```

不要漏掉 `"{}"`，否则交互 shell 可能卡住等待输入。

### 5. 检查 AprilTag 输出

```bash
rostopic echo /tag_detections -n 1
```

输出示例：
```text
header:
  frame_id: "camera_color_optical_frame"
detections:
  - id: [60102]          # data_id=601, basket_class=2 (basket_4622)
    size: [10]
    pose:
      pose:
        position: {x: 0.523, y: -0.142, z: 1.203}
        orientation: {x: ..., y: ..., z: ..., w: ...}
```

### 6. 测试 SDK

```bash
cd /media/data/kuavo-studio
source /opt/ros/noetic/setup.bash
source /media/data/kuavo_ros_application/devel/setup.bash
source /media/data/kuavo-studio/third_party/basket_vision/basket_vision_ws/devel/setup.bash
export ROS_MASTER_URI=http://kuavo_master:11311
export ROS_IP=192.168.26.12
export PYTHONPATH=/media/data/kuavo-studio:/media/data/kuavo-studio/src:$PYTHONPATH

# 所有箱子位姿
python3 - <<'PY'
from adapters.hardware.factory import HardwareFactory

hw = HardwareFactory.create_hardware({
    "robot_type": "leju_wheeled",
    "basket_vision": {
        "timeout": 10.0,
        "basket_pose_service": "/infer_basket_pose",
        "top_basket_service": "/infer_top_basket_ids",
    },
})

result = hw.infer_basket_pose()
print("success:", result.success)
if result.data:
    print("num_instances:", result.data["num_instances"])
    print("baskets:", result.data["baskets"][0]["pose6d_list"] if result.data["baskets"] else None)
    print("embodied_compat:", result.data["embodied_compat"]["t_base"])
PY

# 顶层目标
python3 - <<'PY'
from adapters.hardware.factory import HardwareFactory

hw = HardwareFactory.create_hardware({
    "robot_type": "leju_wheeled",
    "basket_vision": {
        "timeout": 10.0,
        "basket_pose_service": "/infer_basket_pose",
        "top_basket_service": "/infer_top_basket_ids",
    },
})

result = hw.infer_top_basket()
print("success:", result.success)
if result.data:
    print("num_instances:", result.data["num_instances"])
    print("target:", result.data["target"])
    print("embodied_compat:", result.data["embodied_compat"]["t_base"])
PY
```

### 7. 输出结构

`result.data` 主要字段：

```text
success                  # ROS service 原始 success
message                  # 含 basket_type, total_count, layer_count
num_instances            # 返回实例数量
poses_camera_link        # camera_color_optical_frame 下的位姿列表
poses_base_link          # base_link 下的位姿列表
poses_waist_link         # waist_yaw_link 下的位姿列表
bbox_xyxy                # 图像像素框 [x1,y1,x2,y2,...]
yaw                      # 每个实例 yaw，单位 rad
baskets                  # SDK 整理后的实例列表
embodied_compat          # embodied 兼容结构
target                   # 仅 infer_top_basket() 有
```

坐标约定：

| 坐标系 | x | y | z |
|--------|---|---|---|
| `camera_color_optical_frame` | 图像右方 | 图像下方 | 相机前方 |
| `base_link` | 机器人前方 | 机器人左侧 | 机器人上方 |
| `waist_yaw_link` | 旋转后前方 | 旋转后左侧 | 上方 |

`embodied_compat` 结构：
```text
source                  # service_selected_pose 或 no_valid_instance
t_base                  # 选中目标在 base_link 下的位置 [x,y,z]
q_base                  # 选中目标在 base_link 下的四元数 [x,y,z,w]
t_camera                # 选中目标相机坐标 [x,y,z]
q_camera                # 选中目标相机四元数 [x,y,z,w]
yaw_rad                 # 选中目标 yaw
filter                  # embodied 选择时使用的范围
```

### 8. 日志和输出文件

```bash
/media/data/kuavo-studio/third_party/basket_vision/logs/gdrn_inference/YYYYMMDD_HHMMSS/
```

常用文件：
```text
start_gdrn_inference.log                         # 服务启动和推理主日志
run_info.txt                                     # 本次运行路径和配置
images/*_input.jpg                               # 输入图
images/*_bbox.jpg                                # bbox 标注图
images/*_pose6d.jpg                              # 中心点和 pose6d 图
internal/*_response.json                         # SDK 每次 service 返回
service_outputs/infer_basket_pose/<time>/        # 单箱服务输出
service_outputs/infer_top_basket_ids/<time>/     # 顶层服务输出
```

查看最新日志：
```bash
LATEST=$(ls -td /media/data/kuavo-studio/third_party/basket_vision/logs/gdrn_inference/* | head -1)
echo "$LATEST"
grep -R "PIPELINE\|CAM_POSE\|CANDIDATE\|RESULT\|FREE_FORM\|APRILTAG\|WARN" "$LATEST" | tail -80
```

---

## 四、常见问题

**`ModuleNotFoundError: basket_vision_msgs`**
```bash
source /media/data/kuavo-studio/third_party/basket_vision/basket_vision_ws/devel/setup.bash
```
仍失败则重新编译 `basket_vision_ws`。

**`ModuleNotFoundError: predictor_gdrn`**
```bash
export PYTHONPATH=/media/data/kuavo-studio/third_party/basket_vision/basket_gdrnpp:$PYTHONPATH
```

**`No detection from YOLO`**
- conf_thr 过高，降低 `basket_5.yaml` 中的 `conf_thr`（当前 0.01）
- 相机遮挡或画面太亮/太暗
- 检查 `images/*_input.jpg` 是否正常

**`No valid boxes after bbox size filtering`**
- bbox 贴边窄框被过滤：让目标完整进入画面，边缘留 50~100 px
- min_bbox_area_abs 太高：降低 `basket_5.yaml` 中的值

**`GDRNPP inference failed for all candidates`**
- obj_name 不匹配：检查 `ref/basket.py` 中 `id2obj` 是否正确
- 模型权重不匹配：确认 `output_basket_5/` 下的 `model_best.pth` 和 config 一致

**`embodied_compat` 为空**
检测有结果但未通过范围过滤。先看 `poses_base_link`、`poses_camera_link`、`bbox_xyxy`，确认 base 坐标系和相机坐标系的平移在合理范围内。

**TF 不连通**
```text
Could not find a connection between 'base_link' and 'camera_color_optical_frame'
```
确认下位机、机器人状态发布、相机节点都已启动。可用 `start_basket_tf_fallback.sh` 临时排查（不作为最终依据）。

---

## 五、最小验收标准

```bash
# 1. ROS Service 可见
rosservice list | grep infer_basket

# 2. 两个 service 能返回
rosservice call /infer_basket_pose "{}"      # success: True
rosservice call /infer_top_basket_ids "{}"   # success: True

# 3. AprilTag topic 有输出
rostopic echo /tag_detections -n 1 | grep "id:"

# 4. SDK 可调用
python3 -c "
from adapters.hardware.factory import HardwareFactory
hw = HardwareFactory.create_hardware({'robot_type':'leju_wheeled'})
print(hw.infer_top_basket().success)
"
```
