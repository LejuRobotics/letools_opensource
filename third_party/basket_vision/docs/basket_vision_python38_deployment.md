# Basket Vision Python 3.8 客户部署基线

本文给出已经在 AGX Orin 实机验证的部署方案。安装脚本只创建模块自己的 Python 环境，不安装 apt 包、不修改系统 Python，也不升级 JetPack。它只适用于下列固定平台指纹。

## 1. 支持的平台

| 项目 | 已验证值 |
|---|---|
| 设备 | Jetson AGX Orin，aarch64，算力 8.7 |
| Ubuntu | 20.04.6 |
| JetPack / L4T | 5.1.4 / 35.6.0 |
| CUDA / cuDNN | 11.4 / 8.6 |
| ROS | ROS 1 Noetic |
| Python | 3.8.10 |
| GCC | 9.4 |

这套方案不复用客户已有的 Python 包环境。ROS 和 `cv_bridge` 使用系统 Noetic 包，视觉依赖安装到独立 Python 3.8 虚拟环境。

## 2. 先采集，不要先安装

从 Basket Vision 根目录执行：

```bash
bash basket_vision_module/scripts/collect_jetson_env.sh /usr/bin/python3.8 \
  | tee basket_vision_host_report.txt
```

只有满足以下条件才继续使用本方案：

- `aarch64`；
- Ubuntu 20.04；
- JetPack 5.1.4 / L4T R35；
- CUDA 11.4、cuDNN 8.6；
- ROS Noetic；
- `/usr/bin/python3.8` 可用；
- `cv_bridge_boost.so` 链接 `libpython3.8` 和 `libboost_python38`。

不满足时保留采集报告，为该平台建立独立 profile。不要升级客户系统的 CUDA、ROS 或系统 Python 来强行匹配。

## 3. 依赖分层

部署依赖分为三层，不能合并成一个不受控的 `pip install -U`：

1. 系统/ROS：通过 Ubuntu 和 ROS Noetic apt 包提供。
2. Jetson 二进制：PyTorch、TorchVision、Detectron2 使用与 Python 3.8、aarch64、CUDA 11.4 匹配的 wheel。
3. Python 运行库：直接依赖记录在 `jetpack5_py38_runtime.in`，客户安装使用完整精确锁定的 `jetpack5_py38_runtime.txt`。

禁止执行：

```bash
pip install -U torch torchvision detectron2 mmcv
```

它可能安装 CPU wheel 或覆盖已经验证的 Jetson CUDA 构建。

## 4. 系统和 ROS 包

在有管理员权限的维护窗口安装缺失项：

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

安装前可用 `dpkg -s <package>` 判断是否已经存在。不要为了部署视觉模块升级整套 JetPack。

## 5. 准备 Jetson 二进制 wheel

无论后续选择制品服务器、Git LFS 还是离线包，安装前都要将以下三个文件放入 Basket Vision 根目录的 `wheels/`。这些文件不提交普通 Git：

```text
torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
torchvision-0.15.1-cp38-cp38-linux_aarch64.whl
detectron2-0.6-cp38-cp38-linux_aarch64.whl
```

安装脚本会自动验证 wheel 哈希。也可以先在 `wheels/` 目录手工验证：

```bash
sha256sum -c ../basket_gdrnpp/requirements/jetpack5_py38_binary_wheels.sha256
```

任何哈希不一致都应停止部署。

## 6. 安装隔离环境

在 Basket Vision 根目录执行。默认环境位于 `basket_gdrnpp/.venv`，标准 Python 依赖从客户配置的 pip 源下载：

```bash
bash basket_vision_module/scripts/install_jetpack5_py38_env.sh
```

如需把环境放到独立数据盘，显式传入环境和 wheel 目录：

```bash
export BASKET_VISION_ENV=/media/data/basket_vision_envs/gdrn38
bash basket_vision_module/scripts/install_jetpack5_py38_env.sh \
  "$BASKET_VISION_ENV" \
  /path/to/wheels
```

脚本首先核对 Ubuntu、JetPack/L4T、Python 3.8 和 `cv_bridge` ABI，再校验三个 wheel 的 SHA256。目标目录不存在时才创建；如果目录已存在但不是隔离的 Python 3.8 venv，脚本会停止，不会覆盖或删除它。重复执行可补齐同一个有效环境。

Detectron2 的 wheel 元数据包含若干训练工具依赖；当前运行时 profile 不安装 `black`、`future`、`hydra-core`、`pydot` 和 TensorBoard，并沿用实机验证过的 `iopath==0.1.10`。因此 `pip check` 会报告这些训练工具缺失及 Detectron2 声明的 `iopath<0.1.10` 约束。最终判定以第 9 节的运行时验收为准。

## 7. 编译 ROS Service 消息

catkin 的 `build/`、`devel/` 和 `install/` 含绝对路径，不能从开发机复制给客户。必须在客户机的最终工程路径重新生成：

```bash
source /opt/ros/noetic/setup.bash
cd basket_vision_ws
catkin_make \
  -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
```

确认消息接口：

```bash
rossrv show basket_vision_msgs/InferBasketPose
```

## 8. 部署模型资产

模型和 CAD 文件不进入普通 Git。将交付物复制到以下固定相对路径：

```text
basket_gdrnpp/output_basket_5/model_final_5.pth.1
basket_gdrnpp/yolo_basket_5_weights/best_5.pt
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/models_info.json
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_000001.ply
...
basket_gdrnpp/datasets/BOP_DATASETS/basket/models/obj_000005.ply
```

从 Basket Vision 根目录验证：

```bash
sha256sum -c basket_gdrnpp/requirements/basket_model_assets.sha256
```

## 9. 严格运行时验收

```bash
export BASKET_VISION_ENV="${BASKET_VISION_ENV:-$PWD/basket_gdrnpp/.venv}"
bash basket_vision_module/scripts/verify_jetpack5_py38_runtime.sh \
  "$BASKET_VISION_ENV/bin/python"
```

需要同时校验大模型哈希时：

```bash
BASKET_VERIFY_HASHES=1 \
  bash basket_vision_module/scripts/verify_jetpack5_py38_runtime.sh \
  "$BASKET_VISION_ENV/bin/python"
```

交付前做一次完整权重加载验收（会占用 GPU 显存，但不会启动 ROS 节点或读取相机）：

```bash
BASKET_VERIFY_MODEL_LOAD=1 \
  bash basket_vision_module/scripts/verify_jetpack5_py38_runtime.sh \
  "$BASKET_VISION_ENV/bin/python"
```

验收脚本检查：

- Python 和关键包精确版本；
- CUDA、cuDNN、CXX11 ABI 和 Orin 8.7 架构；
- TorchVision CUDA NMS；
- Detectron2 运行接口；
- ROS、TF、AprilTag 和自定义 Service 消息；
- Python 环境与用户 `~/.local` 隔离；
- 自定义 Service 消息确实来自当前工程的 catkin workspace；
- `cv_bridge` BGR8 内存往返；
- Basket Vision 主节点导入；
- 模型/CAD 文件完整性。

默认模式不加载权重。两种模式都不连接 ROS master、不订阅相机、不启动节点。

## 10. 启动与实机验收

严格验收通过后，在独立终端只启动 Gemini 335L 相机。这个 launch 默认不启动手腕相机，也不包含机器人运动组件：

```bash
cd /path/to/kuavo_ros_application
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch dynamic_biped orbbec_sensor_only_enable.launch \
  with_wrist_camera:=false
```

停止测试时在该终端按 `Ctrl-C`。确认三个输入各能收到一条消息：

```bash
timeout 10 rostopic echo -n 1 /camera/color/image_raw >/dev/null
timeout 10 rostopic echo -n 1 /camera/depth/image_raw >/dev/null
timeout 10 rostopic echo -n 1 /camera/color/camera_info >/dev/null
```

然后在另一个终端设置工程根目录和虚拟环境：

```bash
export KUAVO_STUDIO_DIR=/absolute/path/to/LeTools
export BASKET_VISION_ENV=/absolute/path/to/basket_vision_envs/gdrn38

bash basket_vision_module/scripts/start_gdrn_inference.sh
```

启动脚本会等待相机彩色图像，然后加载 YOLO、GDRN 和服务节点。实机验收仍需确认：

- 彩色图、深度图和相机内参持续发布；
- `/infer_basket_pose`、`/infer_top_basket_ids` 可调用；
- Torch/GDRN 推理日志无 CUDA 错误；
- `base_link <- camera_color_optical_frame` 来自真实标定；
- 未标定的 TF fallback 不用于抓取精度验收。

## 11. 已验证但不应复制的内容

- 不复制开发机已有虚拟环境目录；重新创建并安装 wheel。
- 不复制 catkin 的 `build/`、`devel/`、`install/`。
- 不复用客户用户目录中的 `~/.local` 包；始终设置 `PYTHONNOUSERSITE=1`。
- 不使用系统 Python 中可能存在的 CPU PyTorch。
- 不把模型权重、wheel 或运行日志提交到普通 Git。
- 回滚 Python 安装时只移除本模块的 venv；系统 Python、CUDA 和 ROS 不需要回滚。

## 12. 从零复验记录

2026-07-29 在目标 AGX Orin 上使用本安装脚本新建了独立 Python 3.8 venv，而不是复用此前跑通的环境。复验结果：

- 三个 Jetson wheel SHA256 通过；
- 90 个精确锁定的 Python 包可从实机配置的 pip 镜像安装；
- catkin Service 消息在最终工程路径重新编译，生成路径检查通过；
- CUDA 11.4、cuDNN 8.6、算力 8.7 和 TorchVision CUDA NMS 通过；
- Detectron2、ROS、TF、cv_bridge 和 Basket Vision 主节点导入通过；
- GDRN 102,585,703 个参数和 YOLO detect 权重均成功加载；
- 模型及 CAD 资产 SHA256 全部通过。

随后使用 `kuavo_ros_application` 的 `dynamic_biped/orbbec_sensor_only_enable.launch` 做了端到端复验。该 launch 只启动 Gemini 335L，相机彩色图、深度图和内参均为正确的 ROS 消息类型；两个 Basket Vision Service 成功注册，`/infer_basket_pose` 实际完成 YOLO 与 GDRN 推理，并输出 `basket_4622` 的相机坐标 6D 位姿。由于 sensor-only 模式没有真实的 `base_link <- camera_color_optical_frame` 标定 TF，Service 按设计返回 `success: False`，但相机坐标推理日志有效。本次未启动伪 TF 或任何运动组件。

复验确认 `python3.8-venv` 是从零创建环境的必需系统包；安装器会在缺少 `ensurepip` 时提前停止并提示安装该包。
