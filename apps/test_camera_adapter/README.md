# 相机适配器测试目录

## 概述

本目录包含相机适配器（CameraAdapter）和感知适配器（PerceptionAdapter）的测试脚本，用于验证适配器层的功能正确性。

## 目录结构

```
apps/test_camera_adapter/
├── README.md                       # 本文件
├── e2e_camera_perception.py        # 端到端验证（CameraAdapter + PerceptionAdapter 完整链路）
├── test_perception_adapter.py      # 感知适配器综合测试
│
│   # CameraAdapter 单项测试（按功能拆分，可独立运行）
├── test_camera_init.py             # 初始化 + launch 启动
├── test_camera_tf.py               # TF 静态变换节点检查
├── test_camera_rviz.py             # rviz 启动/不启动
├── test_camera_frame.py            # RGB 帧获取
├── test_camera_depth.py            # 深度图获取 + 话题数据校验
├── test_camera_pointcloud.py       # 点云获取 + 话题数据校验
├── test_camera_status.py           # 相机状态查询
├── test_camera_shutdown.py         # 资源清理
│
│   # PerceptionAdapter 单项测试
├── test_perception_init.py         # 初始化 + 依赖注入
├── test_perception_frame.py        # 相机数据委托（get_camera_frame 等）
├── test_perception_apriltag.py     # AprilTag 检测 + 话题 publisher 校验
└── test_perception_shutdown.py     # 资源清理
```

## 测试内容

| 测试项 | 说明 |
|--------|------|
| 适配器初始化 | 测试 CameraAdapter / PerceptionAdapter 的初始化流程（依赖注入） |
| TF 静态变换 | 检查 `static_transform_publisher` 节点（camera→head_camera_link, base_link→torso） |
| rviz 启动 | 验证 rviz 启动/不启动行为 |
| 相机帧获取 | 测试 RGB 图像获取 |
| 深度数据获取 | 测试深度图获取 + `rostopic echo` 话题数据校验（防假成功） |
| 点云数据获取 | 测试点云数据订阅 + 话题数据校验 |
| 相机状态查询 | 测试 `get_camera_status()` / `check_health()` / `get_performance_metrics()` |
| AprilTag 检测 | 测试二维码识别 + `/tag_detections` `/robot_tag_info` 话题 publisher 校验 |
| 相机数据委托 | 验证 PerceptionAdapter 通过注入的 ICamera 获取帧/深度/点云 |
| 资源清理 | 测试 `shutdown()` 子进程终止（terminate→wait→kill 带重试） |
| 适配器初始化 | 测试 CameraAdapter / PerceptionAdapter 的初始化流程（依赖注入） |
| TF 静态变换 | 检查 `static_transform_publisher` 节点（camera→head_camera_link, base_link→torso） |
| rviz 启动 | 验证 rviz 启动/不启动行为 |
| 相机帧获取 | 测试 RGB 图像获取 |
| 深度数据获取 | 测试深度图获取 + `rostopic echo` 话题数据校验（防假成功） |
| 点云数据获取 | 测试点云数据订阅 + 话题数据校验 |
| 相机状态查询 | 测试 `get_camera_status()` / `check_health()` / `get_performance_metrics()` |
| AprilTag 检测 | 测试二维码识别 + `/tag_detections` `/robot_tag_info` 话题 publisher 校验 |
| 相机数据委托 | 验证 PerceptionAdapter 通过注入的 ICamera 获取帧/深度/点云 |
| 资源清理 | 测试 `shutdown()` 子进程终止（terminate→wait→kill 带重试） |
| 端到端验证 | 完整链路：相机启动 → 感知注入 → 检测 → 清理 |

## 运行方式

### 方式一：重用模式（--reuse，推荐）

先启动一次相机，然后反复运行不同测试脚本，无需每次等 3s launch：

```bash
# 终端 1: 先启动相机（保持运行）
# 默认读取 config/camera_config.yaml
python3 apps/test_camera_adapter/test_camera_init.py --keep-alive

# 也可以指定其他相机配置
python3 apps/test_camera_adapter/test_camera_init.py --config path/to/camera_config.yaml --keep-alive

# 如需 rviz 可视化，加 --rviz
python3 apps/test_camera_adapter/test_camera_init.py --keep-alive --rviz

# 终端 2: 逐个跑 --reuse 测试（秒级，跳过 launch/TF/rviz）
# CameraAdapter
python3 apps/test_camera_adapter/test_camera_frame.py --reuse
python3 apps/test_camera_adapter/test_camera_depth.py --reuse
python3 apps/test_camera_adapter/test_camera_pointcloud.py --reuse
python3 apps/test_camera_adapter/test_camera_status.py --reuse
python3 apps/test_camera_adapter/test_camera_tf.py --reuse

# PerceptionAdapter（仅 camera 数据委托部分）
python3 apps/test_camera_adapter/test_perception_frame.py --reuse
```

> `--reuse` 跳过 roslaunch/TF/rviz 启动和 shutdown，仅订阅话题 + 执行测试。适用脚本：`test_camera_{frame,depth,pointcloud,status,tf}.py` + `test_perception_frame.py`。
>
> **rviz 验证**：通过 `test_camera_init.py --keep-alive --rviz` 在启动相机时同时打开 rviz（使用 `biped_s4_head.rviz` 配置）。在机器人显示器上应能看到 RGB/深度/点云话题的实时可视化。也可单独验证：`python3 apps/test_camera_adapter/test_camera_init.py --rviz`（3s 后自动退出，快速确认 rviz 能正常弹出）。
>
> `test_camera_{init,shutdown}.py` 不支持 `--reuse`（它们测试的就是初始化和清理本身）。`test_camera_rviz.py` 也不推荐 `--reuse`（它测试 launch 期 rviz 开关行为，真机建议直接用 `--rviz` 标志验证）。

### 方式二：独立运行（需完整 init→shutdown 周期的脚本）

以下脚本内部有自己的 CameraAdapter 初始化，**必须先 Ctrl+C 停掉 `--keep-alive`**，否则 roslaunch 会冲突：

```bash
# CameraAdapter 资源清理测试（3s init → shutdown，约 5s）
python3 apps/test_camera_adapter/test_camera_shutdown.py

# PerceptionAdapter 初始化和依赖注入
python3 apps/test_camera_adapter/test_perception_init.py

# PerceptionAdapter 资源清理（init → 惰性启动 AprilTag → shutdown）
python3 apps/test_camera_adapter/test_perception_shutdown.py

# AprilTag 检测（需要二维码对准相机视野）
python3 apps/test_camera_adapter/test_perception_apriltag.py

# 端到端验证（CameraAdapter + PerceptionAdapter 完整链路）
python3 apps/test_camera_adapter/e2e_camera_perception.py

# 感知适配器综合测试
python3 apps/test_camera_adapter/test_perception_adapter.py
```

> **推荐测试顺序**：先方式一覆盖 CameraAdapter 全部功能 → 停 `--keep-alive` → 方式二逐个跑 PerceptionAdapter + shutdown 测试。

### 方式三：pytest 集成测试（无需硬件）

```bash
cd LeTools
PYTHONPATH=. pytest tests/test_camera_adapter_integration.py -v
PYTHONPATH=. pytest tests/test_interface_contract.py -v
```

## 前置条件与依赖

| 脚本 | 依赖硬件 | 前置 ROS 节点 | 特殊准备 | 支持 --reuse |
|------|----------|--------------|----------|-------------|
| `test_camera_init.py` | Orbbec 相机 | roscore | 无 | ❌（测的就是 init） |
| `test_camera_tf.py` | Orbbec 相机 | roscore | 无 | ✅ |
| `test_camera_rviz.py` | Orbbec 相机 | roscore | 显示器（rviz=true 时） | ✅ |
| `test_camera_frame.py` | Orbbec 相机 | roscore | 无 | ✅ |
| `test_camera_depth.py` | Orbbec 相机 | roscore | 无 | ✅ |
| `test_camera_pointcloud.py` | Orbbec 相机 | roscore | 无 | ✅ |
| `test_camera_status.py` | Orbbec 相机 | roscore | 先跑过 frame（有帧数据） | ✅ |
| `test_camera_shutdown.py` | Orbbec 相机 | roscore | 无 | ❌（测的就是 shutdown） |
| `test_perception_init.py` | Orbbec 相机 | roscore | 无 | ❌ |
| `test_perception_frame.py` | Orbbec 相机 | roscore | 无 | ✅（仅 camera 部分） |
| `test_perception_apriltag.py` | Orbbec 相机 **+ AprilTag 二维码** | roscore | 二维码对准相机视野 | ❌ |
| `test_perception_shutdown.py` | Orbbec 相机 | roscore | 无 | ❌ |

## 测试结果说明

测试脚本会输出各项测试的通过/失败状态，包括：
- ✅ 通过
- ❌ 失败
- ⚠️ 未测试（需要特定硬件）

## 依赖要求

1. ROS Noetic 环境已配置
2. `dynamic_biped` 功能包已编译（包含 `orbbec_sensor_only_enable.launch`、`biped_s4_head.rviz`）
2. `dynamic_biped` 功能包已编译（包含 `orbbec_sensor_only_enable.launch`、`biped_s4_head.rviz`）
3. `apriltag_ros` 和 `ar_control` 功能包已编译（包含 `continuous_detection.launch` 和 `ar_control_node.py`）
4. Orbbec 相机硬件已连接（可选但推荐）
5. AprilTag 二维码（用于测试二维码检测）

## 配置参数

### CameraAdapter（`camera_config.yaml`）

| 参数 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `has_head` | bool | 是否启用头部 Orbbec 相机 | `true` |
| `enable_wrist_camera` | bool | 是否启用手腕 RealSense 相机 | `false` |
| `has_left_wrist` | bool | 是否启用左手腕相机 | `true` |
| `has_right_wrist` | bool | 是否启用右手腕相机 | `true` |
| `left_wrist_camera_sn` | str | 左手腕相机序列号 | `""` |
| `right_wrist_camera_sn` | str | 右手腕相机序列号 | `""` |
| `rviz` | bool | 是否启动 rviz（使用 `biped_s4_head.rviz`） | `false` |
| `color_width` | int | RGB 图像宽度（0=驱动默认） | `1280` |
| `color_height` | int | RGB 图像高度（0=驱动默认） | `720` |
| `color_fps` | int | RGB 帧率（0=驱动默认） | `30` |
| `depth_width` | int | 深度图宽度（0=驱动默认） | `640` |
| `depth_height` | int | 深度图高度（0=驱动默认） | `400` |
| `depth_fps` | int | 深度图帧率（0=驱动默认） | `30` |

### PerceptionAdapter（`camera_config.yaml`）

| 参数 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `launch_apriltag` | bool | 是否启动 AprilTag 检测 | `true` |

### 底层参数修改指引

以下参数**不在** `camera_config.yaml` 中暴露，需直接编辑对应文件：

| 参数 | 位置 | 说明 |
|------|------|------|
| RGB/深度分辨率、帧率 | `config/camera_config.yaml`（`color_width/height/fps`、`depth_width/height/fps`）→ 通过 `config/camera_orbbec.launch` 传入 `gemini_330_series.launch` | 修改 yaml 配置值即可，无需动 launch 文件 |
| 二维码家族、检测参数 | `config/apriltag_settings.yaml`（`tag_family`、`tag_threads`、`tag_decimate` 等） | 由 `config/apriltag_continuous.launch` 加载 |
| 二维码尺寸、ID 列表 | `config/apriltag_tags.yaml`（`standalone_tags` 列表） | 同上 |

> **设计原则**：Adapters 层只管理自身行为（启动哪个 launch、是否需要 TF/rviz/AprilTag），底层驱动/算法的参数由各自的 configuration file 管理，保持 Infrastructure 层原始文件零修改可控。

## API 参考

### CameraAdapter（`camera_config.yaml`）

| 参数 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `has_head` | bool | 是否启用头部 Orbbec 相机 | `true` |
| `enable_wrist_camera` | bool | 是否启用手腕 RealSense 相机 | `false` |
| `has_left_wrist` | bool | 是否启用左手腕相机 | `true` |
| `has_right_wrist` | bool | 是否启用右手腕相机 | `true` |
| `left_wrist_camera_sn` | str | 左手腕相机序列号 | `""` |
| `right_wrist_camera_sn` | str | 右手腕相机序列号 | `""` |
| `rviz` | bool | 是否启动 rviz（使用 `biped_s4_head.rviz`） | `false` |
| `color_width` | int | RGB 图像宽度（0=驱动默认） | `1280` |
| `color_height` | int | RGB 图像高度（0=驱动默认） | `720` |
| `color_fps` | int | RGB 帧率（0=驱动默认） | `30` |
| `depth_width` | int | 深度图宽度（0=驱动默认） | `640` |
| `depth_height` | int | 深度图高度（0=驱动默认） | `400` |
| `depth_fps` | int | 深度图帧率（0=驱动默认） | `30` |

### PerceptionAdapter（`camera_config.yaml`）

| 参数 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `launch_apriltag` | bool | 是否启动 AprilTag 检测 | `true` |

### 底层参数修改指引

以下参数**不在** `camera_config.yaml` 中暴露，需直接编辑对应文件：

| 参数 | 位置 | 说明 |
|------|------|------|
| RGB/深度分辨率、帧率 | `config/camera_config.yaml`（`color_width/height/fps`、`depth_width/height/fps`）→ 通过 `config/camera_orbbec.launch` 传入 `gemini_330_series.launch` | 修改 yaml 配置值即可，无需动 launch 文件 |
| 二维码家族、检测参数 | `config/apriltag_settings.yaml`（`tag_family`、`tag_threads`、`tag_decimate` 等） | 由 `config/apriltag_continuous.launch` 加载 |
| 二维码尺寸、ID 列表 | `config/apriltag_tags.yaml`（`standalone_tags` 列表） | 同上 |

> **设计原则**：Adapters 层只管理自身行为（启动哪个 launch、是否需要 TF/rviz/AprilTag），底层驱动/算法的参数由各自的 configuration file 管理，保持 Infrastructure 层原始文件零修改可控。

## API 参考

### CameraAdapter

#### 生命周期

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `initialize` | `(config: Dict[str, Any]) -> Result` | `Result(success=True/False, message=...)` | 启动相机 launch、TF 节点、订阅话题、可选启动 rviz |
| `shutdown` | `() -> Result` | `Result(success=True/False, message=...)` | 终止所有子进程（terminate→wait→kill 两轮重试）、清理订阅者与缓存 |
| `is_connected` | `() -> bool` | `True` / `False` | 适配器是否已成功初始化 |

#### 数据获取

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `get_camera_frame` | `(camera_name="camera") -> Optional[CameraFrame]` | `CameraFrame` 或 `None` | 获取最新 RGB + 深度帧 |
| `get_depth_data` | `(camera_name="camera") -> Optional[DepthData]` | `DepthData` 或 `None` | 获取深度图 + 内参（scale=0.001, 毫米→米） |
| `get_point_cloud` | `(camera_name="camera") -> Optional[PointCloudData]` | `PointCloudData` 或 `None` | 获取点云（仅头部相机） |
| `get_camera_info` | `(camera_name="camera") -> Optional[CameraInfo]` | `CameraInfo` 或 `None` | 获取相机内参/外参/分辨率/帧率 |
| `get_camera_status` | `(camera_name="camera") -> Optional[CameraStatus]` | `CameraStatus` 或 `None` | 获取运行状态（is_running/frame_count/fps） |

#### 相机控制

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `start_camera` | `(camera_name="camera") -> bool` | `True` | 标记相机为运行中 |
| `stop_camera` | `(camera_name="camera") -> bool` | `True` | 标记相机为停止 |

#### 扩展方法（非 ICamera 接口）

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `check_health` | `(camera_name="camera") -> Dict[str, Any]` | `{healthy, reason, frame_count, last_frame_time, time_since_last_frame}` | 健康检查：5 秒内有新帧视为健康 |
| `get_performance_metrics` | `(camera_name="camera") -> Dict[str, Any]` | `{frame_count, fps, avg_latency_ms, max_latency_ms, error_counts, time_since_last_frame}` | 按消息类型（color/depth/pointcloud）分别统计 |
#### 生命周期

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `initialize` | `(config: Dict[str, Any]) -> Result` | `Result(success=True/False, message=...)` | 启动相机 launch、TF 节点、订阅话题、可选启动 rviz |
| `shutdown` | `() -> Result` | `Result(success=True/False, message=...)` | 终止所有子进程（terminate→wait→kill 两轮重试）、清理订阅者与缓存 |
| `is_connected` | `() -> bool` | `True` / `False` | 适配器是否已成功初始化 |

#### 数据获取

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `get_camera_frame` | `(camera_name="camera") -> Optional[CameraFrame]` | `CameraFrame` 或 `None` | 获取最新 RGB + 深度帧 |
| `get_depth_data` | `(camera_name="camera") -> Optional[DepthData]` | `DepthData` 或 `None` | 获取深度图 + 内参（scale=0.001, 毫米→米） |
| `get_point_cloud` | `(camera_name="camera") -> Optional[PointCloudData]` | `PointCloudData` 或 `None` | 获取点云（仅头部相机） |
| `get_camera_info` | `(camera_name="camera") -> Optional[CameraInfo]` | `CameraInfo` 或 `None` | 获取相机内参/外参/分辨率/帧率 |
| `get_camera_status` | `(camera_name="camera") -> Optional[CameraStatus]` | `CameraStatus` 或 `None` | 获取运行状态（is_running/frame_count/fps） |

#### 相机控制

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `start_camera` | `(camera_name="camera") -> bool` | `True` | 标记相机为运行中 |
| `stop_camera` | `(camera_name="camera") -> bool` | `True` | 标记相机为停止 |

#### 扩展方法（非 ICamera 接口）

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `check_health` | `(camera_name="camera") -> Dict[str, Any]` | `{healthy, reason, frame_count, last_frame_time, time_since_last_frame}` | 健康检查：5 秒内有新帧视为健康 |
| `get_performance_metrics` | `(camera_name="camera") -> Dict[str, Any]` | `{frame_count, fps, avg_latency_ms, max_latency_ms, error_counts, time_since_last_frame}` | 按消息类型（color/depth/pointcloud）分别统计 |

### PerceptionAdapter

#### 生命周期

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `initialize` | `(camera: ICamera, config: Dict[str, Any]=None) -> bool` | `True` / `False` | 依赖注入 ICamera 实例；AprilTag 节点惰性启动（首次 `get_tag_detections()` 时触发） |
| `shutdown` | `() -> None` | 无 | 终止 apriltag/ar_control 子进程 |

#### 算法输出

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `get_tag_detections` | `() -> List[TagDetection]` | `List[TagDetection]`（可能为空） | 获取最新 AprilTag 检测结果（首次调用时惰性启动 AprilTag 节点） |
| `get_object_detections` | `() -> List[ObjectDetection]` | `[]`（空列表） | 预留接口，未实现 |
| `get_latest_result` | `() -> Optional[PerceptionResult]` | `PerceptionResult(success, tags=...)` 或 `PerceptionResult(success=False)` | 获取最近一次完整感知结果 |

#### 委托查询（委托给注入的 ICamera）

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `get_camera_frame` | `(camera_name="camera") -> Optional[CameraFrame]` | 同 CameraAdapter | 委托 |
| `get_depth_data` | `(camera_name="camera") -> Optional[DepthData]` | 同 CameraAdapter | 委托 |
| `get_point_cloud` | `(camera_name="camera") -> Optional[PointCloudData]` | 同 CameraAdapter | 委托 |
| `get_camera_info` | `(camera_name="camera") -> Optional[CameraInfo]` | 同 CameraAdapter | 委托 |
| `get_camera_status` | `(camera_name="camera") -> Optional[CameraStatus]` | 同 CameraAdapter | 委托 |

#### 扩展方法（非 IPerception 接口）

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `check_health` | `() -> Dict[str, Any]` | `{healthy, reason, initialized, apriltag_started, latest_detection_count, camera_available}` | 感知链路健康检查 |
| `get_performance_metrics` | `() -> Dict[str, Any]` | `{total_detections, latest_detection_count, avg_latency_ms, max_latency_ms, time_since_last_detection, detections_per_second}` | 感知性能统计 |

## 使用示例

### CameraAdapter 基本用法

```python
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter

adapter = CameraAdapter()

# 初始化（启动相机 launch、TF、话题订阅）
result = adapter.initialize({
    "has_head": True,
    "enable_wrist_camera": False,
    "rviz": False,
})

if not result.success:
    print(f"初始化失败: {result.message}")
    exit(1)

# 获取相机数据
frame = adapter.get_camera_frame("camera")
if frame:
    print(f"RGB 图像: {frame.color_image.shape}")

# 健康检查
health = adapter.check_health("camera")
print(f"相机健康: {health['healthy']}, 帧数: {health['frame_count']}")

# 关闭
adapter.shutdown()
```

### PerceptionAdapter 基本用法

```python
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter

# 先初始化相机
camera = CameraAdapter()
camera.initialize({"has_head": True})

# 依赖注入
perception = PerceptionAdapter()
perception.initialize(camera=camera, config={"launch_apriltag": True})

# 检测 AprilTag（首次调用时惰性启动 AprilTag 节点）
tags = perception.get_tag_detections()
for tag in tags:
    print(f"Tag {tag.tag_id}: ({tag.pose_in_world.x:.2f}, {tag.pose_in_world.y:.2f}, {tag.pose_in_world.z:.2f})")

# 也支持通过 perception 委托访问相机数据（可选）
frame = perception.get_camera_frame("camera")

# 清理
perception.shutdown()
camera.shutdown()
```

### 通过 LifecycleMixin 自动初始化（推荐）

```python
# LejuWheeledArmHardware 的 LifecycleMixin 自动处理相机初始化：
# 1. rospy.init_node('leju_wheeled_arm_hardware')
# 2. 读取 config/camera_config.yaml
# 3. camera.initialize(cam_cfg)          ← CameraAdapter
# 4. perception.initialize(camera, config) ← PerceptionAdapter（如果 launch_apriltag=true）

hardware = LejuWheeledArmHardware()
hardware.initialize()  # 相机自动随硬件一起初始化

# 访问相机（通过 hardware 的属性）
frame = hardware.camera.get_camera_frame()
tags = hardware.perception.get_tag_detections()
```
#### 生命周期

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `initialize` | `(camera: ICamera, config: Dict[str, Any]=None) -> bool` | `True` / `False` | 依赖注入 ICamera 实例；AprilTag 节点惰性启动（首次 `get_tag_detections()` 时触发） |
| `shutdown` | `() -> None` | 无 | 终止 apriltag/ar_control 子进程 |

#### 算法输出

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `get_tag_detections` | `() -> List[TagDetection]` | `List[TagDetection]`（可能为空） | 获取最新 AprilTag 检测结果（首次调用时惰性启动 AprilTag 节点） |
| `get_object_detections` | `() -> List[ObjectDetection]` | `[]`（空列表） | 预留接口，未实现 |
| `get_latest_result` | `() -> Optional[PerceptionResult]` | `PerceptionResult(success, tags=...)` 或 `PerceptionResult(success=False)` | 获取最近一次完整感知结果 |

#### 委托查询（委托给注入的 ICamera）

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `get_camera_frame` | `(camera_name="camera") -> Optional[CameraFrame]` | 同 CameraAdapter | 委托 |
| `get_depth_data` | `(camera_name="camera") -> Optional[DepthData]` | 同 CameraAdapter | 委托 |
| `get_point_cloud` | `(camera_name="camera") -> Optional[PointCloudData]` | 同 CameraAdapter | 委托 |
| `get_camera_info` | `(camera_name="camera") -> Optional[CameraInfo]` | 同 CameraAdapter | 委托 |
| `get_camera_status` | `(camera_name="camera") -> Optional[CameraStatus]` | 同 CameraAdapter | 委托 |

#### 扩展方法（非 IPerception 接口）

| 方法 | 签名 | 返回值 | 说明 |
|------|------|--------|------|
| `check_health` | `() -> Dict[str, Any]` | `{healthy, reason, initialized, apriltag_started, latest_detection_count, camera_available}` | 感知链路健康检查 |
| `get_performance_metrics` | `() -> Dict[str, Any]` | `{total_detections, latest_detection_count, avg_latency_ms, max_latency_ms, time_since_last_detection, detections_per_second}` | 感知性能统计 |

## 使用示例

### CameraAdapter 基本用法

```python
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter

adapter = CameraAdapter()

# 初始化（启动相机 launch、TF、话题订阅）
result = adapter.initialize({
    "has_head": True,
    "enable_wrist_camera": False,
    "rviz": False,
})

if not result.success:
    print(f"初始化失败: {result.message}")
    exit(1)

# 获取相机数据
frame = adapter.get_camera_frame("camera")
if frame:
    print(f"RGB 图像: {frame.color_image.shape}")

# 健康检查
health = adapter.check_health("camera")
print(f"相机健康: {health['healthy']}, 帧数: {health['frame_count']}")

# 关闭
adapter.shutdown()
```

### PerceptionAdapter 基本用法

```python
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter

# 先初始化相机
camera = CameraAdapter()
camera.initialize({"has_head": True})

# 依赖注入
perception = PerceptionAdapter()
perception.initialize(camera=camera, config={"launch_apriltag": True})

# 检测 AprilTag（首次调用时惰性启动 AprilTag 节点）
tags = perception.get_tag_detections()
for tag in tags:
    print(f"Tag {tag.tag_id}: ({tag.pose_in_world.x:.2f}, {tag.pose_in_world.y:.2f}, {tag.pose_in_world.z:.2f})")

# 也支持通过 perception 委托访问相机数据（可选）
frame = perception.get_camera_frame("camera")

# 清理
perception.shutdown()
camera.shutdown()
```

### 通过 LifecycleMixin 自动初始化（推荐）

```python
# LejuWheeledArmHardware 的 LifecycleMixin 自动处理相机初始化：
# 1. rospy.init_node('leju_wheeled_arm_hardware')
# 2. 读取 config/camera_config.yaml
# 3. camera.initialize(cam_cfg)          ← CameraAdapter
# 4. perception.initialize(camera, config) ← PerceptionAdapter（如果 launch_apriltag=true）

hardware = LejuWheeledArmHardware()
hardware.initialize()  # 相机自动随硬件一起初始化

# 访问相机（通过 hardware 的属性）
frame = hardware.camera.get_camera_frame()
tags = hardware.perception.get_tag_detections()
```

## 架构说明

- **CameraAdapter**: 负责通过 `config/camera_orbbec.launch` 启动相机 ROS 节点（Orbbec `gemini_330_series.launch`），订阅原始图像/深度/点云话题，启动 TF 静态变换（`static_transform_publisher`），可选启动 rviz。分辨率等参数在 `config/camera_config.yaml` 中配置。
- **PerceptionAdapter**: 通过依赖注入使用 CameraAdapter 获取相机数据，专注于算法检测。AprilTag 通过 `config/apriltag_continuous.launch` 加载 `config/apriltag_settings.yaml` + `config/apriltag_tags.yaml` 启动（惰性启动：首次 `get_tag_detections()` 时触发），`ar_control_node.py` 负责坐标变换。
- **ROS 节点初始化**（`rospy.init_node`）由 `LifecycleMixin` 统一负责，在 camera/perception 初始化之前最先执行，两个 adapter 不需自行 init_node。
- **启动顺序**（AprilTag 链路）：`apriltag_ros continuous_detection.launch` → `ar_control_node.py` → 订阅 `/robot_tag_info`。`time.sleep(3.0)` 作为就绪等待。

## 附录：常见坑点（Lessons Learned）

### A1. subprocess.Popen 管道阻塞

`stdout=subprocess.PIPE` / `stderr=subprocess.PIPE` 会开辟内核管道缓冲区（约 64KB）。roslaunch 输出量大，缓冲区填满后进程阻塞，相机驱动无法正常启动。

**解决**：用 `stdout=subprocess.DEVNULL` / `stderr=subprocess.DEVNULL`。

### A2. --reuse 模式缺少 rospy.init_node()

`--reuse` 跳过 `CameraAdapter.initialize()` 的完整流程，手动调用 `_setup_subscribers()` 创建 ROS subscriber。但 `rospy.Subscriber` 需要所属节点已初始化，否则静默失效（无报错，但永远收不到消息）。

完整链路中 `LifecycleMixin` 负责 `rospy.init_node()`，独立测试时 7 个 `--reuse` 脚本各自缺少，需在 `if reuse:` 分支开头补充：
```python
import rospy
if not rospy.core.is_initialized():
    rospy.init_node('test_xxx', anonymous=True)
```

### A3. Orbbec 点云话题被 remap

`gemini_330_series.launch` 末尾将点云话题从 `/camera/depth/color/points` remap 为 `/camera/depth_registered/points`。订阅者和 rostopic echo 需使用后一个话题名。

### A4. 点云 callback 逐点循环性能

Python `for` 循环逐点遍历 `pc2.read_points()` 处理 92 万点（1280×720），单帧耗时远超 33ms，在 30Hz 帧率下 callback 永远无法完成积压。

**解决**：用 numpy 矢量化 `np.frombuffer() → reshape → mask`，处理 92 万点 < 50ms。

### A5. AprilTag TF 链路

`ar_control_node.py` 执行 `lookup_transform('base_link', source_frame, ...)` 将 tag 坐标转换到机器人坐标系。完整 TF 链路：

```
base_link → ... → zhead_2_link → head_camera_base   ← 下位机 URDF
head_camera_base → camera_link                       ← camera_adapter
camera_link → camera_color_optical_frame             ← Orbbec 驱动 publish_tf
```

缺任意一环都会导致 `/robot_tag_info` 为空（`ar_control_node` 静默跳过 TF 转换失败的 tag）。

### A6. ar_control_node source_frame 与相机 namespace

`ar_control_node` 默认 `source_frame=camera_color_optical_frame`。Orbbec 相机 namespace 为 `/camera`，optical frame 即为 `camera_color_optical_frame`。换用 RealSense（namespace `/head_camera`）时需改为 `head_camera_color_optical_frame`。`perception_adapter.py` 通过 `config.get('apriltag_source_frame', 'camera_color_optical_frame')` 支持覆盖。

### A7. 不修改 infrastructure/ 文件

遵循 LeTools 架构规范，所有适配器自有配置和 launch 文件放在 `config/` 下，通过绝对路径引用。`infrastructure/ros_packages/` 内的文件保持原样。

| 功能 | 基础设施文件 | LeTools 自有文件 |
|------|-------------|----------------------|
| 相机启动 | `gemini_330_series.launch` | `config/camera_orbbec.launch` |
| 相机分辨率 | 无 | `config/camera_config.yaml` |
| AprilTag 检测 | `continuous_detection.launch` | `config/apriltag_continuous.launch` |
| AprilTag 参数 | `apriltag_ros/config/settings.yaml` | `config/apriltag_settings.yaml` |
| AprilTag 标签 | `apriltag_ros/config/tags.yaml` | `config/apriltag_tags.yaml` |

### A8. tag_family 和 tag_size 必须匹配实物

`tag_family` 决定二维码编码方式（`tag16h5` / `tag36h11` 等），`size` 是黑色边框物理边长（米）。两者中任一项不匹配实际打印的二维码，AprilTag 检测器将完全无法识别。
