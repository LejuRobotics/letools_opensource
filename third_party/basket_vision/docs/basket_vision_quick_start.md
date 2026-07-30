# Basket Vision 新用户 Quick Start

本文适用于第一次在客户 Jetson AGX Orin 上部署 Basket Vision 的用户。按照顺序执行即可；需要了解版本约束和故障细节时，再阅读[完整 Python 3.8 部署基线](basket_vision_python38_deployment.md)。

本流程只启动相机和感知节点，不启动机器人运动组件。最终用于抓取前，必须由机器人集成侧提供真实且经过标定的相机 TF。

## 0. 部署前先拿齐交付物

仅克隆 Git 仓库不能完成部署。开始前必须具备：

1. Basket Vision Git 仓库；
2. 三个 Jetson aarch64 wheel；
3. GDRN、YOLO 和 CAD 模型资产；
4. 可用的 apt 源和 pip 源。

三个 wheel 必须放在 Basket Vision 根目录的 `wheels/` 中：

```text
wheels/
├── torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
├── torchvision-0.15.1-cp38-cp38-linux_aarch64.whl
└── detectron2-0.6-cp38-cp38-linux_aarch64.whl
```

模型资产必须放在以下固定相对路径：

```text
basket_gdrnpp/output_basket_5/model_final_5.pth.1
basket_gdrnpp/yolo_basket_5_weights/best_5.pt
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/models_info.json
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_000001.ply
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_000002.ply
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_000003.ply
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_000004.ply
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_000005.ply
```

这些大文件不进入普通 Git。如果客户机不能访问 pip 源，还需要另行提供完整的 Python 离线 wheelhouse；当前仓库本身不是全离线安装包。

## 1. 确认支持的平台

本方案只支持下面这套已经实机验证的平台：

| 项目 | 要求 |
|---|---|
| 设备 | Jetson AGX Orin，aarch64，算力 8.7 |
| Ubuntu | 20.04.6 |
| JetPack / L4T | 5.1.4 / 35.6.0 |
| CUDA / cuDNN | 11.4 / 8.6 |
| ROS | ROS 1 Noetic |
| Python | 3.8.10 |

其他 JetPack、Python 或架构不能直接复用这里的二进制 wheel。

## 2. 设置工程路径

下文把 Basket Vision 仓库根目录记为 `BASKET_ROOT`。必须使用绝对路径：

```bash
export BASKET_ROOT=/absolute/path/to/basket_vision
export BASKET_VISION_ENV="$BASKET_ROOT/basket_gdrnpp/.venv"
cd "$BASKET_ROOT"
```

新开终端后需要重新执行这三个环境设置命令。

## 3. 只读采集实机环境

先检查，暂时不要安装：

```bash
bash basket_vision_module/scripts/collect_jetson_env.sh /usr/bin/python3.8 \
  | tee basket_vision_host_report.txt
```

只有报告确认以下内容全部匹配才继续：

- `aarch64`、Ubuntu 20.04；
- JetPack 5.1.4 / L4T 35.6.0；
- CUDA 11.4、cuDNN 8.6；
- ROS Noetic、Python 3.8；
- `cv_bridge` 链接 Python 3.8。

如有不匹配，请停止部署并保留 `basket_vision_host_report.txt`。不要通过升级客户机 CUDA、ROS 或系统 Python 强行匹配。

## 4. 安装系统和 ROS 依赖

在有管理员权限的维护窗口执行一次：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3.8-venv \
  python3-catkin-tools \
  python3-empy \
  libopenblas-dev \
  liblapack-dev \
  libeigen3-dev \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libgl1 \
  libglib2.0-0 \
  ros-noetic-rospy \
  ros-noetic-cv-bridge \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-image-transport \
  ros-noetic-sensor-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-std-msgs \
  ros-noetic-apriltag-ros
```

这一步不升级 JetPack，也不要额外执行 `pip install -U torch torchvision detectron2`。

## 5. 校验 wheel 和模型资产

确认交付方提供的文件已经放到第 0 节列出的路径，然后从工程根目录执行：

```bash
cd "$BASKET_ROOT/wheels"
sha256sum -c ../basket_gdrnpp/requirements/jetpack5_py38_binary_wheels.sha256

cd "$BASKET_ROOT"
sha256sum -c basket_gdrnpp/requirements/basket_model_assets.sha256
```

所有文件都必须显示 `OK`。任何文件缺失或哈希不一致都应停止部署并重新获取交付物。

## 6. 创建独立 Python 3.8 环境

```bash
cd "$BASKET_ROOT"

bash basket_vision_module/scripts/install_jetpack5_py38_env.sh \
  "$BASKET_VISION_ENV" \
  "$BASKET_ROOT/wheels"
```

成功时最后显示：

```text
[PASS] Basket Vision Python environment installed
```

安装器只创建本模块自己的 venv，不修改系统 Python。普通 Python 依赖会从客户配置的 pip 源下载。

## 7. 编译 ROS Service 消息

catkin 生成物包含绝对路径，必须在客户机的最终工程路径重新生成：

```bash
source /opt/ros/noetic/setup.bash
cd "$BASKET_ROOT/basket_vision_ws"

catkin_make \
  -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5

source devel/setup.bash
rossrv show basket_vision_msgs/InferBasketPose
```

`rossrv show` 能显示 Service 字段，表示消息包已经生成并进入当前 ROS 环境。

## 8. 执行安装后验收

仍在同一终端执行：

```bash
cd "$BASKET_ROOT"
source /opt/ros/noetic/setup.bash
source "$BASKET_ROOT/basket_vision_ws/devel/setup.bash"

bash basket_vision_module/scripts/verify_jetpack5_py38_runtime.sh \
  "$BASKET_VISION_ENV/bin/python"

BASKET_VERIFY_HASHES=1 \
  bash basket_vision_module/scripts/verify_jetpack5_py38_runtime.sh \
  "$BASKET_VISION_ENV/bin/python"

BASKET_VERIFY_MODEL_LOAD=1 \
  bash basket_vision_module/scripts/verify_jetpack5_py38_runtime.sh \
  "$BASKET_VISION_ENV/bin/python"
```

三次验收都必须通过。权重加载验收会使用 GPU 显存，但不会连接 ROS master、订阅相机或启动机器人运动。

## 9. 启动相机和推理服务

### 终端 A：只启动 Gemini 335L 相机

```bash
export KUAVO_APP=/home/leju_kuavo/kuavo_ros_application
cd "$KUAVO_APP"
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch dynamic_biped orbbec_sensor_only_enable.launch \
  with_wrist_camera:=false
```

这个 launch 不包含机器人运动组件。保持终端 A 运行。

### 终端 B：检查相机话题

```bash
source /opt/ros/noetic/setup.bash

timeout 10 rostopic echo -n 1 /camera/color/image_raw >/dev/null
timeout 10 rostopic echo -n 1 /camera/depth/image_raw >/dev/null
timeout 10 rostopic echo -n 1 /camera/color/camera_info >/dev/null
```

三个命令都返回退出码 `0` 后，在同一终端启动 Basket Vision：

```bash
export BASKET_ROOT=/absolute/path/to/basket_vision
export BASKET_VISION_ENV="$BASKET_ROOT/basket_gdrnpp/.venv"
source "$BASKET_ROOT/basket_vision_ws/devel/setup.bash"

bash "$BASKET_ROOT/basket_vision_module/scripts/start_gdrn_inference.sh"
```

如果 Basket Vision 位于 LeTools 工程之外，并且需要叠加 LeTools 的 ROS workspace，请在启动前设置：

```bash
export KUAVO_STUDIO_DIR=/absolute/path/to/LeTools
```

这里的 LeTools 根目录应包含 `infrastructure/ros_packages/devel/setup.bash`。

启动成功会显示：

```text
[gdrn] inference service is ready
```

### 终端 C：调用服务

```bash
export BASKET_ROOT=/absolute/path/to/basket_vision
source /opt/ros/noetic/setup.bash
source "$BASKET_ROOT/basket_vision_ws/devel/setup.bash"

rosservice call /infer_basket_pose "{}"
rosservice call /infer_top_basket_ids "{}"
rostopic echo -n 1 /tag_detections
```

## 10. 判断是否部署成功

至少确认：

- 三次严格运行时验收全部通过；
- 三个相机输入话题都有消息；
- `/infer_basket_pose` 和 `/infer_top_basket_ids` 均已注册；
- 推理日志没有 CUDA、TorchVision NMS 或模型加载错误；
- 实际调用能识别箱体并输出相机坐标位姿；
- 用于机器人抓取前，存在真实标定的 `base_link <- camera_color_optical_frame` TF。

仅使用 sensor-only 相机 launch 时通常没有真实的 `base_link` TF。此时日志中的 `[CAM_POSE_6D]` 可以证明相机坐标推理已经执行，但 Service 可能按设计返回 `success: False`。不要使用临时 TF fallback 代替抓取精度验收。

## 11. 停止与恢复

测试完成后：

1. 在终端 B 按 `Ctrl-C` 停止 Basket Vision；
2. 在终端 A 按 `Ctrl-C` 停止相机；
3. 使用 `rosnode list` 确认没有测试节点残留。

Python 安装需要回滚时，只处理 `BASKET_VISION_ENV` 指向的模块 venv。不要修改或删除系统 Python、CUDA、ROS 和 JetPack。

## 常见失败

| 现象 | 处理 |
|---|---|
| `required file is missing` | 检查三个 wheel 是否使用精确文件名并放入 `wheels/` |
| wheel 或模型哈希失败 | 停止安装，重新获取交付物，不要忽略校验 |
| `Python ensurepip is unavailable` | 安装 `python3.8-venv` 后重新运行安装器 |
| pip 下载失败 | 检查客户 pip 镜像；离线客户机需要完整 wheelhouse |
| `rossrv show` 找不到消息 | 重新执行第 7 节并 source 当前 `devel/setup.bash` |
| 启动脚本一直等待相机 | 检查第 9 节三个相机话题 |
| 已识别箱体但 Service 返回失败 | 检查真实的 `base_link <- camera_color_optical_frame` 标定 TF |
