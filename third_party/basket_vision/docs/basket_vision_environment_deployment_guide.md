# GDRNPP + YOLO 机器人端 Python 环境部署指南

> **状态：历史 Python 3.10 构建方案，不作为当前客户默认部署流程。**
> JetPack 5.1.4 / ROS Noetic 客户机统一使用已实机验证的
> [Python 3.8 客户部署基线](basket_vision_python38_deployment.md)。只有为其他平台建立独立 profile 时才参考本文的源码构建方法。

本文用于在 Kuavo Jetson Orin 机器人上，从零创建料箱视觉所需的 Python 运行环境并安装依赖。

本文只处理以下内容：

- 在机器人上拉取 LeTools。
- 创建独立的 Python 3.10 环境。
- 安装 CUDA 版 PyTorch、TorchVision、MMCV、Ultralytics 和 GDRNPP 运行依赖。
- 编译 `basket_vision_msgs`。
- 验证 Python、CUDA、ROS 和关键 Python 包能否正常导入。

本文不包含相机、TF、推理服务和 SDK 的测试流程。所有命令都在机器人 Ubuntu 终端中执行；通过 SSH 登录机器人后，直接从第 1 节开始即可。

## 1. 先理解这套视觉系统

### 1.1 它解决什么问题

机器人看到料箱后，需要完成两类不同任务：

1. **找到料箱**：判断图像中哪些区域是料箱，用矩形框表示位置。
2. **估计料箱姿态**：计算料箱相对相机和机器人的三维位置、朝向。

YOLO 和 GDRNPP 分别负责这两个阶段：

| 组件 | 输入 | 输出 | 通俗解释 |
|---|---|---|---|
| YOLO | 相机彩色图像 | 类别、置信度、二维框 `bbox` | 先在画面中圈出料箱 |
| GDRNPP | 原图、YOLO 框、相机内参 | 三维平移和三维旋转 | 再判断料箱离相机多远、朝向如何 |
| 深度图 | 每个像素的距离 | 可见表面的实际距离 | 用真实测距修正网络估计的深度 |
| TF | 坐标系之间的实时关系 | 相机坐标转换到 `base_link` 等坐标系 | 把“相机看到的位置”变成“机器人能使用的位置” |
| ROS 服务 | 识别请求 | 结构化识别结果 | 让 SDK 和机器人任务调用视觉能力 |

常见输出含义：

| 输出 | 含义 |
|---|---|
| `bbox_xyxy` | 图像框的左上角和右下角像素坐标 |
| `poses_camera_link` | 料箱在相机光学坐标系中的三维姿态 |
| `poses_base_link` | 料箱在机器人底盘坐标系中的三维姿态 |
| 
| `yaw` | 料箱绕竖直方向的旋转角 |
| `num_instances` | 本次服务最终返回的料箱数量 |

相机光学坐标系通常是 `X` 向右、`Y` 向下、`Z` 向前；机器人 `base_link` 通常是 `X` 向前、`Y` 向左、`Z` 向上。两者方向不同，所以视觉结果必须经过 TF 转换，不能把相机坐标直接当成底盘坐标。

### 1.2 一次识别是怎样完成的

完整数据流如下：

```text
Orbbec 相机
  ├─ 彩色图像 /camera/color/image_raw
  ├─ 深度图   /camera/depth/image_raw
  └─ 相机内参 /camera/color/camera_info
          ↓
cv_bridge + OpenCV 把 ROS 图像转换为 Python 图像
          ↓
Ultralytics YOLO 检测料箱并输出二维框
          ↓
框筛选：置信度、尺寸、中心列、候选数量
          ↓
GDRNPP + Detectron2 估计每个候选的 6D 位姿
          ↓
深度图修正可见表面距离，并换算料箱中心
          ↓
TF 将 camera_color_optical_frame 转换到 base_link 
          ↓
ROS 服务返回位置、姿态、二维框和筛选结果
          ↓
LeTools SDK 将 ROS 消息整理成 Python 字典和 Pose6D
```

这条链路解释了为什么环境不能只安装 YOLO：

- YOLO 依赖 PyTorch 和 TorchVision NMS。
- GDRNPP 依赖 PyTorch、MMCV、Detectron2 和科学计算库。
- 相机图像依赖 ROS、`cv_bridge` 和 OpenCV。
- 坐标转换依赖 ROS TF。
- 任意一层版本不匹配，都可能表现为“服务启动失败”“检测框正常但姿态失败”或“坐标无法转换”。

## 2. 从视觉功能理解运行环境

### 2.1 依赖分为哪几层

先按从底层到上层理解环境：

| 层次 | 主要内容 | 上一层为什么需要它 |
|---|---|---|
| 机器人系统层 | Ubuntu、GCC、CUDA、CuDNN、系统动态库 | 提供 GPU、编译器和底层运行库 |
| ROS 层 | ROS Noetic、消息类型、`rospy`、`cv_bridge`、TF | 接收相机数据并提供坐标转换与服务通信 |
| Python 环境层 | Python 3.10 虚拟环境 | 隔离料箱视觉依赖，避免污染系统 Python |
| 深度学习基础层 | PyTorch、TorchVision | 执行 CUDA 网络推理和 NMS |
| 视觉框架层 | Ultralytics、MMCV、Detectron2 | 分别支持 YOLO 检测和 GDRNPP 网络 |
| 项目层 | LeTools 料箱视觉代码 | 组合图像、检测、位姿、TF 和 ROS 服务 |

安装顺序必须从下往上：

```text
检查系统和 CUDA
  -> 安装 apt 系统库与 ROS 包
  -> 创建 Python 3.10 虚拟环境
  -> 安装 PyTorch 和 TorchVision
  -> 安装 GDRNPP / YOLO Python 依赖
  -> 编译 basket_vision_msgs
  -> 逐层验证
```

如果 PyTorch 的 CUDA 检查没有通过，不要继续安装 Detectron2；否则后面的错误会掩盖真正原因。

### 2.2 固定目录

本文统一使用以下目录，不要随意改名：

| 内容 | 目录 |
|---|---|
| LeTools | `/media/data/LeTools` |
| Python 虚拟环境 | `/media/data/basket_vision_envs/gdrn` |
| 构建源码和临时文件 | `/media/data/basket_vision_build` |

先定义变量：

```bash
export LETOOLS_ROOT=/media/data/LeTools
export GDRN_ENV=/media/data/basket_vision_envs/gdrn
export BUILD_ROOT=/media/data/basket_vision_build
```

### 2.3 已验证的核心版本

当前机器人上验证通过的核心组合如下：

| 组件 | 版本或要求 |
|---|---|
| Ubuntu | 20.04 |
| 架构 | `aarch64` |
| GPU | Jetson Orin，CUDA 架构 8.7 |
| ROS | Noetic |
| CUDA | 11.4 |
| CuDNN | 8.6 |
| Python | 3.10.18 |
| PyTorch | 2.2.0，CUDA 11.4，CXX11 ABI |
| TorchVision | 0.17.2 |
| NumPy | 1.26.4 |
| OpenCV | 4.11.0 |
| Pillow | 12.2.0 |
| fvcore | 0.1.5.post20221221 |
| iopath | 0.1.10 |
| Detectron2 | 0.6 |
| MMCV | 1.x，不能使用 2.x |

PyTorch 与 TorchVision 必须是为 Jetson `aarch64` 和 CUDA 11.4 构建的版本。不要安装 x86 wheel，也不要使用会拉取 CUDA 12/13 依赖的普通 PyPI CUDA 包。

### 2.4 检查系统、架构和 ROS

```bash
lsb_release -a
uname -m
test -f /opt/ros/noetic/setup.bash && echo "[OK] ROS Noetic"
nvcc --version
cat /usr/local/cuda/version.json 2>/dev/null || true
```

预期结果：

```text
Ubuntu 20.04
aarch64
[OK] ROS Noetic
CUDA 11.4
```

确认 GPU 和 CuDNN：

```bash
ls -l /dev/nvhost-gpu
dpkg -l | grep -E 'cuda|cudnn' | head -30
```

如果不是 `aarch64`、没有 `/opt/ros/noetic` 或 CUDA 主版本不是 11.4，应先修复机器人基础系统，不要继续安装 Python 包。

### 2.5 检查磁盘和内存

从源码构建 PyTorch 和 TorchVision 会占用较多空间：

```bash
df -h /media/data
free -h
```

建议 `/media/data` 至少保留 30 GB 空间。内存不足时降低后续命令中的 `MAX_JOBS`，不要同时执行多个编译任务。

## 3. 在机器人上拉取 LeTools

### 3.1 首次拉取

```bash
cd /media/data
git clone -b dev ssh://git@www.lejuhub.com:10026/highlydynamic/LeTools.git LeTools
cd /media/data/LeTools
git status --short
git branch --show-current
```

预期当前分支为 `dev`，项目目录为 `/media/data/LeTools`。

### 3.2 已经存在 LeTools 时更新

如果 `/media/data/LeTools` 已经存在，不要重复克隆：

```bash
cd /media/data/LeTools
git status --short
git fetch origin dev
git switch dev
git pull --ff-only origin dev
```

如果 `git status --short` 有本地修改，先确认这些修改是否需要保留，再执行更新。不要使用 `git reset --hard`。

## 4. 安装机器人系统依赖

### 4.1 安装编译和图像依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  git \
  curl \
  ca-certificates \
  pkg-config \
  patchelf \
  ccache \
  gfortran \
  libopenblas-dev \
  liblapack-dev \
  libeigen3-dev \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  zlib1g-dev \
  libffi-dev \
  libssl-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libomp-dev \
  libopenmpi-dev \
  openmpi-bin \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libgtk-3-dev \
  libgl1 \
  libglib2.0-0
```

### 4.2 安装 ROS Python 和消息依赖

```bash
sudo apt install -y \
  python3-catkin-tools \
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

确认编译器和 CUDA：

```bash
gcc --version | head -1
g++ --version | head -1
nvcc --version | tail -4
```

已验证环境使用 GCC 9.4 和 CUDA 11.4。

## 5. 创建 Python 3.10 虚拟环境

这一节按照之前机器人上验证成功的运行方式，从零创建同一路径的环境。历史启动日志显示：

```text
Python 3.10.18: /usr/bin/python3.10
运行环境: /media/data/basket_vision_envs/gdrn
启动方式: source .../gdrn/bin/activate 后执行 python
```

历史上也出现过下面这条日志：

```text
activate script not found, fallback to uv run
Creating virtual environment at: .../basket_gdrnpp/.venv
Downloading nvidia-cudnn-cu13
```

这条回退流程不是成功环境的安装步骤。它会在项目目录创建另一套 `.venv`，并可能下载与机器人 CUDA 11.4 不匹配的 CUDA 13 依赖。因此本节不安装 uv，也不使用 `uv run`、`uv venv` 或项目目录中的 `.venv`。

Python 包必须与系统 Python 隔离。不要使用 `sudo pip install`，也不要把包安装到 `~/.local/lib/python*`。

### 5.1 检查系统 Python 3.10

```bash
command -v python3.10 || true
python3.10 -V 2>/dev/null || true
readlink -f "$(command -v python3.10)" 2>/dev/null || true
```

已验证机器人应输出类似：

```text
/usr/bin/python3.10
Python 3.10.18
```

小版本可以不同，但必须是 Python 3.10。如果 `python3.10` 不存在，先检查机器人软件源是否提供：

```bash
apt-cache policy python3.10 python3.10-venv python3.10-dev
```

能够看到 `Candidate` 版本时安装：

```bash
sudo apt update
sudo apt install -y python3.10 python3.10-venv python3.10-dev
```

如果 `Candidate` 显示 `(none)`，说明当前机器人镜像的软件源不包含 Python 3.10。此时应先补齐机器人官方软件源或使用包含 `/usr/bin/python3.10` 的机器人镜像，不要改用 Python 3.8，也不要通过 uv 临时下载另一套解释器。

### 5.2 检查 venv 支持

```bash
python3.10 -m venv --help >/dev/null 2>&1 \
  && echo "[OK] python3.10 venv available"
```

如果没有输出 `[OK]`，执行：

```bash
sudo apt update
sudo apt install -y python3.10-venv python3.10-dev
```

再次检查，确认出现 `[OK]` 后继续。

### 5.3 准备环境目录

```bash
sudo mkdir -p /media/data/basket_vision_envs
sudo chown -R "$USER":"$USER" /media/data/basket_vision_envs

test -w /media/data/basket_vision_envs \
  && echo "[OK] environment directory is writable"
```

必须看到 `[OK]`。后续环境固定放在 `/media/data/basket_vision_envs/gdrn`，不要创建在 LeTools 源码目录里。

### 5.4 创建 gdrn 虚拟环境

```bash
python3.10 -m venv /media/data/basket_vision_envs/gdrn
```

检查生成结果：

```bash
test -f /media/data/basket_vision_envs/gdrn/bin/activate \
  && echo "[OK] activate script"
test -x /media/data/basket_vision_envs/gdrn/bin/python \
  && echo "[OK] python executable"
test -x /media/data/basket_vision_envs/gdrn/bin/pip \
  && echo "[OK] pip executable"
```

三行都显示 `[OK]` 才继续。

### 5.5 激活环境并隔离旧包

```bash
source /media/data/basket_vision_envs/gdrn/bin/activate
export PYTHONNOUSERSITE=1
export PIP_REQUIRE_VIRTUALENV=true
hash -r
```

这些设置分别用于切换解释器、禁止加载 `~/.local/lib/python*` 中的旧包，以及防止未激活环境时误用 pip。

初始化安装工具：

```bash
python -m pip install --upgrade \
  "pip<25" \
  "setuptools<81" \
  wheel \
  packaging \
  ninja
```

固定 `setuptools<81` 是因为已验证环境中的旧版 PyTorch Lightning 仍使用 `pkg_resources`。

### 5.6 严格验证环境路径

```bash
which python
which python3
which pip
python -V
python -m pip --version
echo "$VIRTUAL_ENV"
```

`python`、`python3` 和 `pip` 都应指向：

```text
/media/data/basket_vision_envs/gdrn/bin/
```

再执行自动检查：

```bash
python - <<'PY'
import site
import sys

expected = "/media/data/basket_vision_envs/gdrn"

print("executable:", sys.executable)
print("version:", sys.version)
print("prefix:", sys.prefix)
print("site-packages:", site.getsitepackages())
print("user site enabled:", site.ENABLE_USER_SITE)

assert sys.executable.startswith(expected)
assert sys.prefix == expected
assert sys.prefix != sys.base_prefix
assert sys.version_info[:2] == (3, 10)
assert all(path.startswith(expected) for path in site.getsitepackages())
assert site.ENABLE_USER_SITE is False
print("[OK] clean Python 3.10 gdrn environment")
PY
```

### 5.7 告诉启动脚本使用该环境

之前成功启动时实际使用的是：

```bash
export UV_ACTIVATE_SCRIPT=/media/data/basket_vision_envs/gdrn/bin/activate
test -f "$UV_ACTIVATE_SCRIPT" \
  && echo "[OK] GDRN activate script is available"
```

`UV_ACTIVATE_SCRIPT` 只是现有启动脚本沿用的变量名，其值是普通 `venv` 的激活文件，不代表需要安装 uv。启动脚本会优先 `source` 这个文件，只有找不到它时才会尝试错误的 uv 回退路径。

后续每次打开新终端，先执行：

```bash
source /media/data/basket_vision_envs/gdrn/bin/activate
export PYTHONNOUSERSITE=1
export PIP_REQUIRE_VIRTUALENV=true
export UV_ACTIVATE_SCRIPT=/media/data/basket_vision_envs/gdrn/bin/activate
```

后面的包安装统一使用：

```bash
python -m pip install <包名>
```

需要退出环境时执行 `deactivate`。

## 6. 安装 PyTorch 2.2.0 和 TorchVision 0.17.2

之前成功服务的启动日志明确显示：

```text
PyTorch       2.2.0
TorchVision   0.17.2+c1d70fe
CUDA Runtime  11.4
CuDNN         8.6
GPU           Orin (arch=8.7)
CXX11 ABI     True
```

因此不能直接执行不带版本和平台限制的 `pip install torch torchvision`。下面按这一组合在 Jetson Orin 上构建 CUDA 11.4 版本。此步骤耗时最长，建议保持机器人供电和网络稳定。

### 6.1 准备构建目录和基础 Python 包

```bash
source /media/data/basket_vision_envs/gdrn/bin/activate
mkdir -p /media/data/basket_vision_build
cd /media/data/basket_vision_build

python -m pip install \
  numpy==1.26.4 \
  pyyaml \
  typing-extensions \
  requests \
  sympy \
  networkx \
  jinja2 \
  fsspec
```

### 6.2 从源码构建 PyTorch

```bash
cd /media/data/basket_vision_build
git clone --recursive --branch v2.2.0 \
  https://github.com/pytorch/pytorch.git pytorch-2.2.0
cd pytorch-2.2.0
git submodule sync
git submodule update --init --recursive
python -m pip install -r requirements.txt
```

设置 Jetson Orin 编译参数：

```bash
export CUDA_HOME=/usr/local/cuda-11.4
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="$VIRTUAL_ENV"
export TORCH_CUDA_ARCH_LIST="7.2;8.7"
export USE_CUDA=1
export USE_CUDNN=1
export USE_NCCL=0
export USE_DISTRIBUTED=1
export USE_MPI=1
export USE_KINETO=0
export BUILD_TEST=0
export MAX_JOBS=4
export CXXFLAGS="-D_GLIBCXX_USE_CXX11_ABI=1"
```

开始构建并安装：

```bash
python setup.py bdist_wheel
python -m pip install dist/torch-2.2.0*.whl
```

内存不足或编译进程被系统杀死时，重新执行前先降低并发：

```bash
export MAX_JOBS=2
```

### 6.3 从源码构建 TorchVision

```bash
cd /media/data/basket_vision_build
git clone --branch v0.17.2 \
  https://github.com/pytorch/vision.git torchvision-0.17.2
cd torchvision-0.17.2

export CUDA_HOME=/usr/local/cuda-11.4
export FORCE_CUDA=1
export TORCH_CUDA_ARCH_LIST="7.2;8.7"
export MAX_JOBS=4
export BUILD_VERSION=0.17.2

python setup.py bdist_wheel
python -m pip install dist/torchvision-0.17.2*.whl
```

### 6.4 立即验证 CUDA 和 NMS

```bash
python - <<'PY'
import torch
import torchvision
from torchvision.ops import nms

print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("torch CUDA:", torch.version.cuda)
print("CUDA available:", torch.cuda.is_available())
print("CuDNN:", torch.backends.cudnn.version())
print("CXX11 ABI:", torch._C._GLIBCXX_USE_CXX11_ABI)

boxes = torch.tensor([[0.0, 0.0, 10.0, 10.0]], device="cuda")
scores = torch.tensor([0.9], device="cuda")
print("CUDA NMS:", nms(boxes, scores, 0.5))
PY
```

必须满足：

- `torch` 为 2.2.0。
- `torchvision` 为 0.17.2。
- `torch CUDA` 为 11.4。
- `CUDA available` 为 `True`。
- `CXX11 ABI` 为 `True`。
- CUDA NMS 返回 `tensor([0], device='cuda:0')`。

出现 `Couldn't load custom C++ ops`，说明 TorchVision 与 PyTorch/CUDA 不匹配；不要继续安装后续依赖。

## 7. 安装 GDRNPP 和 YOLO Python 依赖

本节只安装当前推理服务实际需要的依赖。训练工具、TensorBoard、数据分析和视频处理包不作为默认依赖安装；以后只有在明确出现对应的 `ModuleNotFoundError` 时才补装。

### 7.1 安装固定基础版本

```bash
source /media/data/basket_vision_envs/gdrn/bin/activate

python -m pip install \
  numpy==1.26.4 \
  opencv-python==4.11.0.86 \
  Pillow==12.2.0 \
  fvcore==0.1.5.post20221221 \
  iopath==0.1.10
```

### 7.2 安装通用运行依赖

```bash
python -m pip install \
  scipy \
  PyYAML \
  tqdm \
  tabulate \
  termcolor \
  transforms3d \
  pycocotools \
  cloudpickle \
  yacs \
  easydict \
  einops \
  timm \
  plyfile \
  setproctitle \
  imageio \
  pypng \
  numba \
  omegaconf \
  matplotlib \
  "pytorch-lightning<2"
```

这些包供 GDRNPP 的配置读取、模型构建、几何运算和推理入口使用。`pandas`、`tensorboard`、`scikit-image`、`sklearn` 等训练或离线工具不在默认列表中。

### 7.3 安装 MMCV 1.x

GDRNPP 当前只使用 MMCV 1.x 的配置和基础接口，先安装不带 CUDA 扩展编译的 1.x 版本：

```bash
python -m pip install mmcv==1.7.1
```

验证：

```bash
python - <<'PY'
import mmcv
print("MMCV:", mmcv.__version__)
assert mmcv.__version__.startswith("1.")
PY
```

### 7.4 使用 GDRNPP 自带的 Detectron2

Detectron2 不是多余依赖。`predictor_gdrn.py` 最终会导入 `detectron2.data.MetadataCatalog`，缺少它时服务会报 `No module named 'detectron2.data'`。

但 LeTools 的完整 GDRNPP 目录已经包含兼容源码，因此不要再从 GitHub 克隆或执行 `pip install detectron2`。先检查源码：

```bash
export GDRN_ROOT=/media/data/LeTools/third_party/basket_vision/basket_gdrnpp
test -d "$GDRN_ROOT/detectron2/detectron2" \
  && echo "[OK] bundled Detectron2 source"
export PYTHONPATH="$GDRN_ROOT:$GDRN_ROOT/detectron2:${PYTHONPATH:-}"
```

验证当前推理链路需要的 Python 接口：

```bash
python - <<'PY'
import detectron2
from detectron2.data import MetadataCatalog

print("Detectron2 source:", detectron2.__file__)
print("MetadataCatalog:", MetadataCatalog)
print("[OK] Detectron2")
PY
```

本服务的已验证环境即使没有 `detectron2._C` 也能运行，因此不把 `_C` 编译作为部署前提。若 `detectron2.data` 不存在，先确认完整的 GDRNPP 运行源码已经放入 `$GDRN_ROOT`，再检查 `PYTHONPATH`，不要安装 PyPI 上的同名包顶替。

### 7.5 安装 Ultralytics

Ultralytics 不能删除。当前服务脚本直接执行 `from ultralytics import YOLO`，并用 `YOLO(yolo_weights)` 加载二维检测模型；没有它就无法产生传给 GDRNPP 的料箱框。

```bash
python -m pip install ultralytics
```

安装后验证 YOLO 和 CUDA NMS：

```bash
python - <<'PY'
import ultralytics
from ultralytics import YOLO
from torchvision.ops import nms

print("Ultralytics:", ultralytics.__version__)
print("YOLO:", YOLO)
print("[OK] Ultralytics")
PY
```

不要执行以下命令：

```bash
pip install -U torch torchvision mmcv detectron2
```

它可能覆盖刚刚构建的 Jetson CUDA 版本。

### 7.6 安装 empy（ROS catkin_make 代码生成依赖）

```bash
python -m pip install empy==3.3.4
```

必须安装 `3.3.4` 版本。4.x 版本的 `empy` 移除了 `em.RAW_OPT` 等 API，与 ROS noetic 的 `gencpp` 不兼容，会导致 `catkin_make` 报错 `AttributeError: module 'em' has no attribute 'RAW_OPT'`。

## 8. 编译 ROS 消息、配置环境并验收

### 8.1 编译 basket_vision_msgs

```bash
cd /media/data/LeTools/third_party/basket_vision/basket_vision_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
```

> 如果 CMake 版本 ≥ 3.5，需加 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` 以避免 `Compatibility with CMake < 3.5 has been removed` 报错。
```

检查消息包：

```bash
rospack find basket_vision_msgs
rossrv show basket_vision_msgs/InferBasketPose
```

### 8.2 创建统一激活脚本

```bash
mkdir -p /media/data/LeTools/third_party/basket_vision/env

cat > /media/data/LeTools/third_party/basket_vision/env/activate_gdrn_runtime.sh <<'EOF'
#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

if [ -f /media/data/LeTools/infrastructure/ros_packages/devel/setup.bash ]; then
  source /media/data/LeTools/infrastructure/ros_packages/devel/setup.bash
fi

source /media/data/LeTools/third_party/basket_vision/basket_vision_ws/devel/setup.bash
source /media/data/basket_vision_envs/gdrn/bin/activate

export PYTHONNOUSERSITE=1
export PIP_REQUIRE_VIRTUALENV=true
export LETOOLS_ROOT=/media/data/LeTools
export KUAVO_STUDIO_DIR=$LETOOLS_ROOT
export BASKET_ROOT=$LETOOLS_ROOT/third_party/basket_vision
export GDRN_ROOT=$BASKET_ROOT/basket_gdrnpp
export UV_ACTIVATE_SCRIPT=/media/data/basket_vision_envs/gdrn/bin/activate
export CUDA_HOME=/usr/local/cuda-11.4
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=/lib/aarch64-linux-gnu:$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}
export BASKET_INFERENCE_IMAGE_WIDTH="${BASKET_INFERENCE_IMAGE_WIDTH:-640}"
export BASKET_INFERENCE_IMAGE_HEIGHT="${BASKET_INFERENCE_IMAGE_HEIGHT:-480}"
export PYTHONPATH=$LETOOLS_ROOT:/opt/ros/noetic/lib/python3/dist-packages:$BASKET_ROOT/basket_vision_ws/devel/lib/python3/dist-packages:$GDRN_ROOT:$GDRN_ROOT/detectron2:${PYTHONPATH:-}
EOF

chmod +x /media/data/LeTools/third_party/basket_vision/env/activate_gdrn_runtime.sh
```

以后打开新终端，只需要执行：

```bash
source /media/data/LeTools/third_party/basket_vision/env/activate_gdrn_runtime.sh
```

### 8.3 检查 Python、CUDA 和核心版本

```bash
source /media/data/LeTools/third_party/basket_vision/env/activate_gdrn_runtime.sh

python - <<'PY'
import sys
import numpy
import cv2
import torch
import torchvision

print("Python:", sys.version)
print("Executable:", sys.executable)
print("NumPy:", numpy.__version__)
print("OpenCV:", cv2.__version__)
print("PyTorch:", torch.__version__)
print("TorchVision:", torchvision.__version__)
print("CUDA runtime:", torch.version.cuda)
print("CUDA available:", torch.cuda.is_available())
print("CuDNN:", torch.backends.cudnn.version())
print("GPU:", torch.cuda.get_device_name(0))
PY
```

### 8.4 检查 GDRNPP 和 YOLO 依赖

```bash
python - <<'PY'
import PIL
import scipy
import yaml
import mmcv
import fvcore
import iopath
import transforms3d
import ultralytics
import detectron2

from detectron2.data import MetadataCatalog
from ultralytics import YOLO

print("Pillow:", PIL.__version__)
print("MMCV:", mmcv.__version__)
print("fvcore:", fvcore.__version__)
print("iopath:", iopath.__version__)
print("Ultralytics:", ultralytics.__version__)
print("Detectron2 source:", detectron2.__file__)
print("MetadataCatalog:", MetadataCatalog)
print("[OK] GDRNPP and YOLO Python dependencies")
PY
```

### 8.5 检查 ROS Python 依赖

```bash
python - <<'PY'
import rospy
import cv_bridge
import tf
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from basket_vision_msgs.srv import InferBasketPose

print("rospy:", rospy.__file__)
print("cv_bridge:", cv_bridge.__file__)
print("[OK] ROS Python dependencies")
PY
```

如果 Python 3.10 找不到 `rospy`，先检查：

```bash
echo "$PYTHONPATH" | tr ':' '\n'
```

必须包含：

```text
/opt/ros/noetic/lib/python3/dist-packages
/media/data/LeTools/third_party/basket_vision/basket_vision_ws/devel/lib/python3/dist-packages
```

### 8.6 检查依赖冲突并保存版本

```bash
python -m pip check
mkdir -p /media/data/basket_vision_envs
python -m pip freeze > /media/data/basket_vision_envs/gdrn_requirements.lock
```

`pip check` 应输出：

```text
No broken requirements found.
```

最后确认激活脚本和版本记录存在：

```bash
test -x /media/data/LeTools/third_party/basket_vision/env/activate_gdrn_runtime.sh \
  && echo "[OK] activation script"
test -s /media/data/basket_vision_envs/gdrn_requirements.lock \
  && echo "[OK] requirements lock"
```

以上检查全部通过，机器人上的 GDRNPP + YOLO Python 环境即搭建完成。
