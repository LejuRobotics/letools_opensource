# Basket Vision Python 3.8 完整交付报告（2026-07-29）

## 1. 交付结论

Basket Vision 已按目标 AGX Orin 实机固定为 Python 3.8 独立部署方案。推理代码、ROS 接口、隔离环境安装、依赖锁定、catkin 重建、严格验收、模型权重加载和 sensor-only 相机端到端服务调用均已验证。

本次没有改变算法接口或机器人运动行为。部署只依赖实机已有的 JetPack、CUDA、cuDNN、ROS Noetic 和 `cv_bridge`；全部 pip 包安装在模块独立 venv 中，不复用或污染客户已有 Python 包。

## 2. 固定支持平台

| 项目 | 已验证值 |
|---|---|
| 设备 | NVIDIA Jetson AGX Orin，aarch64，GPU 算力 8.7 |
| Ubuntu | 20.04.6 |
| JetPack / L4T | 5.1.4-b17 / 35.6.0 |
| CUDA / cuDNN | 11.4.315 / 8.6 |
| TensorRT | 8.5.2 |
| ROS | ROS 1 Noetic |
| Python | 3.8.10 |
| `cv_bridge` ABI | `libpython3.8`、`libboost_python38` |

其他 JetPack、CUDA、架构或 Python 版本不属于本交付 profile，不能直接复用这里的二进制 wheel。

## 3. 代码与配置交付

### 3.1 推理兼容代码

| 文件 | 交付内容 |
|---|---|
| `basket_gdrnpp/core/gdrn_modeling/demo/predictor_gdrn.py` | Python 3.8 类型标注兼容；推理期关闭 backbone 预训练下载；可选渲染依赖按需加载 |
| `basket_gdrnpp/core/utils/my_checkpoint.py` | 兼容 Lightning 1.9 `_LiteModule` 与新版 Fabric wrapper；可选 FairScale 不再阻塞推理导入 |
| `basket_gdrnpp/core/gdrn_modeling/demo/inference_service_vis_2_mult_inst_10_shared.py` | 从 YAML 解析运行时模型路径；输出可审计的 `[CAM_POSE_6D]` 相机坐标日志；保留两个 ROS Service 契约 |
| `basket_gdrnpp/core/gdrn_modeling/demo/box_configs/basket_5.yaml` | 固定 GDRN config/checkpoint、YOLO 权重和五类 CAD 模型相对路径 |

这些改动已经随实机推理通过。部署整理阶段没有继续修改模型结构、检测阈值、位姿算法或 ROS 消息定义。

### 3.2 部署与验收代码

| 文件 | 作用 |
|---|---|
| `basket_vision_module/scripts/collect_jetson_env.sh` | 只读采集 OS、JetPack、CUDA、ROS、Python 和 `cv_bridge` ABI，不启动节点或相机 |
| `basket_vision_module/scripts/install_jetpack5_py38_env.sh` | 校验固定平台及 wheel 哈希，创建/修复独立 Python 3.8 venv，安装精确锁定依赖；不执行 apt、不修改系统 Python |
| `basket_vision_module/scripts/verify_jetpack5_py38_runtime.sh` | 校验环境隔离、关键版本、CUDA NMS、Detectron2、ROS、cv_bridge、当前 catkin overlay 和模型资产；可选实际加载 GDRN/YOLO 权重 |
| `basket_vision_module/scripts/start_gdrn_inference.sh` | 从脚本位置解析模块路径，只接受隔离 Python 3.8 venv，禁用用户 site-packages，并保留原有话题等待、服务就绪和停止清理逻辑 |
| `.gitignore` | 排除 wheel、模型权重、catkin 生成物、venv、日志、缓存和运行输出 |

安装器经过了“从零创建”“半成品 venv 修复”和“重复执行”三种路径验证。缺少 `python3.8-venv/ensurepip` 时会在安装 pip 包前明确停止；已有 venv 缺少标准 activation scripts 时会在不清空 site-packages 的前提下补齐。

## 4. 依赖与资产清单

| 文件 | 内容 |
|---|---|
| `basket_gdrnpp/requirements/jetpack5_py38_runtime.in` | 人工维护的直接运行依赖 |
| `basket_gdrnpp/requirements/jetpack5_py38_runtime.txt` | 客户实际安装的 90 个精确版本包 |
| `basket_gdrnpp/requirements/jetpack5_py38_binary_wheels.sha256` | PyTorch、TorchVision、Detectron2 三个 aarch64 wheel 的 SHA256 |
| `basket_gdrnpp/requirements/basket_model_assets.sha256` | GDRN、YOLO 和五类 CAD 资产 SHA256 |

ABI 敏感 wheel 固定为：

```text
torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
torchvision-0.15.1-cp38-cp38-linux_aarch64.whl
detectron2-0.6-cp38-cp38-linux_aarch64.whl
```

Detectron2 wheel 的元数据会让 `pip check` 报告训练工具缺失以及 `iopath<0.1.10` 约束。本交付保留实机验证通过的 runtime-only 组合；最终判定使用严格运行时验收，不使用 `pip check` 代替 CUDA、ROS 和模型加载测试。

## 5. 文档交付

| 文档 | 定位 |
|---|---|
| `README.md` | 模块结构、ROS 接口、配置、启动与调用总览 |
| `docs/basket_vision_python38_deployment.md` | 客户从零部署的唯一主流程 |
| `docs/basket_vision_delivery_report_2026-07-29.md` | 本次代码、依赖、验证、边界与交付清单 |
| `docs/agx_orin_field_deployment_record_2026-07-28.md` | 首次实机推理与物体识别记录 |
| `docs/basket_vision_migration_and_test_guide.md` | ROS 接口、迁移测试和故障排查补充说明 |
| `docs/basket_vision_environment_deployment_guide.md` | 历史 Python 3.10 构建方案，仅供其他平台参考 |

客户部署以 `basket_vision_python38_deployment.md` 为准，历史 Python 3.10 文档不是当前安装入口。

## 6. ROS 运行契约

### 输入话题

| 名称 | 类型 | 用途 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |

### 服务和输出

| 名称 | 类型 | 用途 |
|---|---|---|
| `/infer_basket_pose` | `basket_vision_msgs/InferBasketPose` | 单箱识别与位姿估计 |
| `/infer_top_basket_ids` | `basket_vision_msgs/InferBasketPose` | 顶层多箱识别 |
| `/tag_detections` | `apriltag_ros/AprilTagDetectionArray` | 兼容既有 AprilTag 工作流 |
| `/basket_vision/viz_image` | `sensor_msgs/Image` | 推理可视化图像 |

相机坐标结果使用 `camera_color_optical_frame`。要返回可用于机器人作业的 `base_link` 位姿，必须由集成侧提供真实标定的 `base_link <- camera_color_optical_frame` TF。TF 缺失时 Service 按设计失败，不得用未标定的静态 fallback 通过抓取验收。

## 7. 实机复验结果

### 7.1 从零环境与静态运行时

- 在不同于原跑通环境的新 venv 中安装 90 个精确锁定包成功；
- 安装器重复执行成功，不覆盖其他目录；
- 三个 Jetson wheel 和全部模型/CAD SHA256 通过；
- catkin `build/devel` 在最终工程路径重新生成，不再引用旧路径；
- Python 3.8 venv 与 `~/.local` 完全隔离；
- PyTorch `2.0.0+nv23.05`、TorchVision `0.15.1`、Detectron2 `0.6` 通过；
- CUDA 11.4、cuDNN 8.6、CXX11 ABI、算力 8.7 和 CUDA NMS 通过；
- ROS、TF、AprilTag、自定义 Service、`cv_bridge` BGR8 往返和主节点导入通过；
- GDRN 102,585,703 个参数和 YOLO detect 权重实际加载成功。

### 7.2 sensor-only 端到端验证

验证使用 `kuavo_ros_application` 中的：

```bash
roslaunch dynamic_biped orbbec_sensor_only_enable.launch \
  with_wrist_camera:=false
```

该入口只启动 Gemini 335L，不启动底盘、机械臂、规划或控制组件。实测结果：

- 三个相机输入话题均持续发布且消息类型正确；
- 两个 Basket Vision Service 均成功注册；
- `/infer_basket_pose` 实际执行 YOLO 与 GDRN；
- 检出 `basket_4622`，置信度 `0.9584`；
- 相机坐标位置为 `(-0.349830, -0.209605, 1.136168) m`；
- 相机坐标四元数 XYZW 为 `(-0.041711, -0.398000, 0.915621, 0.038665)`；
- 因 sensor-only 模式没有真实 `base_link` 标定 TF，Service 返回 `success: False`，同时 `[CAM_POSE_6D]` 日志保留有效相机坐标结果；
- 测试完成后 roscore、Orbbec 和 Basket Vision 临时进程全部停止，无残留节点。

### 7.3 非阻塞告警

验证日志中可见 Horovod 缺失提示，以及 MMCV、Numba、Lightning 的弃用告警。它们不影响当前固定版本推理，不能据此升级这些核心包；任何升级必须重新执行本报告全部验收。

## 8. 客户部署顺序

1. 运行 `collect_jetson_env.sh`，确认平台指纹完全匹配。
2. 安装文档列出的缺失 apt/ROS 包，尤其是 `python3.8-venv`。
3. 将三个已校验 wheel 放入 `wheels/`。
4. 将 GDRN、YOLO 和 CAD 资产放入固定相对路径并验证 SHA256。
5. 运行 `install_jetpack5_py38_env.sh` 创建独立 venv。
6. 在客户最终工程路径重新执行 `catkin_make`。
7. 运行默认严格验收，再运行一次 `BASKET_VERIFY_MODEL_LOAD=1` 权重加载验收。
8. 使用 sensor-only 相机 launch 启动相机，再启动 Basket Vision。
9. 验证服务、日志、相机话题和真实标定 TF；测试结束用 `Ctrl-C` 停止相机和感知节点。

完整命令见 `docs/basket_vision_python38_deployment.md`。

## 9. 实机状态变更与可恢复内容

- 安装了缺失的 `python3.8-venv`；apt 同步将 Ubuntu Python 3.8 系统包从 `20.04.15` 修订更新到 `20.04.18`，语言版本仍为 Python 3.8.10；
- 保留了从零验证的新候选 venv：`/home/leju_kuavo/basket_vision_envs/gdrn38_candidate_20260729`；
- 旧 catkin `build/devel` 已改名备份，新目录在当前工程路径重新生成；
- `/tmp` 中保留安装与端到端验证日志，便于复核；
- 原先跑通的 `gdrn38` 环境和推理源码未被覆盖；
- 测试完成后没有残留 ROS、相机或推理进程。

## 10. 仓库外交付物与未关闭项

以下大文件受 `.gitignore` 保护，不进入普通 Git，必须通过最终选定的制品渠道交付：

- 三个 Jetson aarch64 wheel；
- GDRN checkpoint；
- YOLO 权重；
- 五类 CAD `.ply` 与 `models_info.json`。

交付前仍需完成：

- 决定 Git LFS、制品服务器或离线包中的一种大文件分发方式；
- 在真正的 Git 仓库工作区执行 `git status`、审查 diff 并提交。本工作目录的 `.git` 是空目录，当前无法生成可信的 Git diff；
- 由机器人集成侧提供真实相机外参 TF，并完成 `base_link` 坐标下的抓取精度验收。

除上述仓库外制品、Git 操作和真实 TF 集成外，Python 3.8 客户部署内容已经完整验证。
