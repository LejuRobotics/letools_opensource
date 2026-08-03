<a id="api-reference"></a>

# 📷 相机与底盘控制接口文档

| 📁 代码路径 | 🔧 主要类 | 🤖 适用机器人 |
|:---|:---|:---|
| `adapters/hardware/leju_wheeled/` | `CameraAdapter` / `SDKControlMixin` / `JibotChassisMixin` | 乐聚轮式机器人 |

> 📘 本文整理 `LeTools` 项目中和“相机”“底盘”相关的 Python SDK/API 接口。

#### ⚠️ WARNING

在运行任何代码示例之前，请确认机器人相关服务已经启动，否则相机和底盘 SDK/API 无法正常工作。

- 相机接口依赖 ROS 图像、深度、点云等 topic。如果没有启动相机节点，`get_camera_frame()`、`get_depth_data()`、`get_point_cloud()` 可能返回 `None`。
- 底盘接口依赖机器人底层控制服务、SDK 管理器或 ROS topic/service。如果机器人未启动，速度、位置、导航命令会发送失败。
- 真机上测试底盘运动前，请确认周围安全，并优先使用较小速度，例如 `vx=0.1` 或 `vx=0.2`。
- 速度控制接口通常需要主动发送零速度停止，例如 `vx=0.0, vy=0.0, vyaw=0.0`。


这里先把几个概念说清楚：

| 名称 | 含义 |
| --- | --- |
| SDK 接口 | 项目封装好、给外部调用的能力入口，通常是 Python 类的方法，例如 `send_base_velocity_sdk()`。 |
| API | 调用这个能力的具体规则，包括函数名、参数、返回值和调用方式。 |
| 入参 | 调用函数时括号里传进去的参数，例如 `vx=0.2`。 |
| 返回值 | 函数执行后返回的对象，例如 `Result`、`CameraFrame`、`bool`。 |
| 示例 | 告诉你在脚本里应该怎样真正调用这个接口。 |

本文按两大块组织：

- **相机 SDK/API**：负责读取 RGB 图、深度图、点云、相机状态，以及基于相机做感知。
- **底盘 SDK/API**：负责发送底盘速度、位置、定时运动、ROS topic 控制和 JiBot/Jarvis 导航命令。

常见返回值：

| 返回值类型 | 含义 | 常见读取方式 |
| --- | --- | --- |
| `Result` | 项目统一结果对象，通常有 `success`、`message`、`data`。 | `if result.success: ... else: print(result.message)` |
| `bool` | 成功/失败，或者状态真假。 | `if ok: ...` |
| `CameraFrame` | 相机图像帧，通常包含 RGB 图和可选深度图。 | `frame.color_image`、`frame.depth_image` |
| `DepthData` | 深度图、相机内参和深度缩放系数。 | `depth.depth_image`、`depth.intrinsics` |
| `PointCloudData` | 点云数据，通常包含三维点和可选颜色。 | `pc.points`、`pc.colors` |
| `CameraInfo` | 相机类型、分辨率、帧率、内参。 | `info.intrinsics.matrix` |
| `CameraStatus` | 相机运行状态、帧数、FPS 等。 | `status.is_running`、`status.fps` |

## 🚀 快速选择

| 你想做什么 | 优先使用哪个接口 |
| --- | --- |
| 读取头部相机图像 | `camera.get_camera_frame("camera")` |
| 读取深度图 | `camera.get_depth_data("camera")` |
| 读取点云 | `camera.get_point_cloud("camera")` |
| 让底盘按速度运动 | `hardware.send_base_velocity_sdk(vx, vy, vyaw)` |
| 让底盘相对当前位置移动 | `hardware.send_base_position_local_sdk(x, y, yaw)` |
| 让底盘按世界坐标移动 | `hardware.send_base_position_world_sdk(x, y, yaw)` |
| 让底盘按指定时间完成移动 | `hardware.send_base_pose_timed(x, y, yaw, duration, frame)` |
| 使用 JiBot/Jarvis 导航 | `hardware.base_move_to_target_jibot(...)` |

---

## 1. 📷 相机接口

相机相关接口分三层：

1. `ICamera`：抽象接口，定义“相机应该有哪些能力”。
2. `CameraAdapter`：真实实现，负责订阅 ROS topic、缓存图像/深度/点云数据。
3. `PerceptionAdapter` 和 `CameraCaptureSkill`：更高层封装，用相机数据做感知或技能执行。

### 🏗️ *class* core.interfaces.i_camera.ICamera

Bases: `ABC`

相机抽象接口。它不是直接控制硬件的实现类，而是规定所有相机实现都应该提供哪些方法。

如果你只是写业务脚本，一般不会直接实例化 `ICamera`，而是使用 `CameraAdapter`。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`initialize`](#icamera_initialize) | 初始化相机模块。 |
| [`get_camera_frame`](#icamera_get_camera_frame) | 获取指定相机的图像帧。 |
| [`get_depth_data`](#icamera_get_depth_data) | 获取指定相机的深度数据。 |
| [`get_point_cloud`](#icamera_get_point_cloud) | 获取指定相机的点云数据。 |
| [`get_camera_info`](#icamera_get_camera_info) | 获取相机基础信息。 |
| [`get_camera_status`](#icamera_get_camera_status) | 获取相机运行状态。 |

---


<details>
<summary id="icamera_initialize">🔧 <code>initialize(config: dict) -> Result</code></summary>

初始化相机模块。

📥 **入参**
  * **config** (*dict*) - 相机配置，例如是否启用头部相机、腕部相机、RViz 可视化等。

📤 **出参**
  初始化结果。成功时 `result.success=True`，失败时可从 `result.message` 查看原因。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter

camera = CameraAdapter()
result = camera.initialize({
    "enable_head": True,
    "enable_wrist_camera": False,
    "rviz": False,
})

if not result.success:
    raise RuntimeError(result.message)
```

</details>

<details>
<summary id="icamera_get_camera_frame">🔧 <code>get_camera_frame(camera_name: str = "camera") -> CameraFrame | None</code></summary>

获取指定相机的图像帧。

`camera_name` 是相机名字，不是关键字。项目里常用 `"camera"` 表示头部主相机，也可能有 `"left_wrist_camera"`、`"right_wrist_camera"` 这类名字，具体取决于配置和 ROS topic。

📥 **入参**
  * **camera_name** (*str*) - 相机名称，默认 `"camera"`。

📤 **出参**
  成功时返回 `CameraFrame`；如果相机未启动、没有收到图像或名字不存在，返回 `None`。

🏷️ **返回类型**
  `CameraFrame | None`
* **Example:**

```python
frame = camera.get_camera_frame("camera")

if frame is None:
    print("没有读到相机画面")
else:
    image = frame.color_image
    print("RGB 图像尺寸:", image.shape)
```

</details>

<details>
<summary id="icamera_get_depth_data">🔧 <code>get_depth_data(camera_name: str = "camera") -> DepthData | None</code></summary>

获取指定相机的深度数据。

📥 **入参**
  * **camera_name** (*str*) - 相机名称，默认 `"camera"`。

📤 **出参**
  成功时返回 `DepthData`，里面通常包含深度图、内参、深度缩放系数；失败返回 `None`。

🏷️ **返回类型**
  `DepthData | None`
* **Example:**

```python
depth = camera.get_depth_data("camera")

if depth is not None:
    depth_image = depth.depth_image
    intrinsics = depth.intrinsics
    print("深度图尺寸:", depth_image.shape)
    print("相机内参:", intrinsics)
```

</details>

<details>
<summary id="icamera_get_point_cloud">🔧 <code>get_point_cloud(camera_name: str = "camera") -> PointCloudData | None</code></summary>

获取指定相机的点云数据。

📥 **入参**
  * **camera_name** (*str*) - 相机名称，默认 `"camera"`。

📤 **出参**
  成功时返回 `PointCloudData`；如果没有点云 topic 或还没收到点云，返回 `None`。

🏷️ **返回类型**
  `PointCloudData | None`
* **Example:**

```python
point_cloud = camera.get_point_cloud("camera")

if point_cloud is not None:
    points = point_cloud.points
    print("点云点数:", len(points))
```

</details>

<details>
<summary id="icamera_get_camera_info">🔧 <code>get_camera_info(camera_name: str = "camera") -> CameraInfo | None</code></summary>

获取相机基础信息。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  相机信息，包括分辨率、帧率、相机内参等；失败返回 `None`。

🏷️ **返回类型**
  `CameraInfo | None`
* **Example:**

```python
info = camera.get_camera_info("camera")

if info is not None:
    print("相机信息:", info)
```

</details>

<details>
<summary id="icamera_get_camera_status">🔧 <code>get_camera_status(camera_name: str = "camera") -> CameraStatus | None</code></summary>

获取相机运行状态。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  相机状态，例如是否运行、FPS、收到的帧数；失败返回 `None`。

🏷️ **返回类型**
  `CameraStatus | None`
* **Example:**

```python
status = camera.get_camera_status("camera")

if status is not None:
    print("相机是否运行:", status.is_running)
    print("FPS:", status.fps)
```

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.camera_adapter.CameraAdapter

Bases: `ICamera`

轮式机器人相机适配器。它是相机 SDK/API 的主要实现类，内部负责启动或连接相机相关 ROS 节点，并订阅图像、深度、点云、相机信息等 topic。

实际使用中，你通常创建 `CameraAdapter()`，然后调用它的 `initialize()`、`get_camera_frame()`、`shutdown()` 等方法。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`initialize`](#cameraadapter_initialize) | 初始化相机适配器。 |
| [`start_camera`](#cameraadapter_start_camera) | 启动指定相机。 |
| [`stop_camera`](#cameraadapter_stop_camera) | 停止指定相机。 |
| [`is_connected`](#cameraadapter_is_connected) | 判断相机适配器是否已经连接并初始化。 |
| [`check_health`](#cameraadapter_check_health) | 检查指定相机是否健康。 |
| [`get_performance_metrics`](#cameraadapter_get_performance_metrics) | 获取相机性能指标。 |
| [`shutdown`](#cameraadapter_shutdown) | 关闭相机适配器并释放资源。 |

---


<details>
<summary id="cameraadapter_initialize">🔧 <code>initialize(config: dict) -> Result</code></summary>

初始化相机适配器。

📥 **入参**
  * **config** (*dict*) - 相机配置。
    - `enable_head`：是否启动头部相机。
    - `enable_wrist_camera`：腕部相机总开关。
    - `has_left_wrist` / `has_right_wrist`：硬件是否实际安装对应腕部相机。
      某侧只有在总开关和对应安装标记均为 `true` 时才会启动；完全未安装
      腕部相机时三项均设为 `false`。
    - `rviz`：是否启动 RViz 可视化。

📤 **出参**
  初始化结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter

camera = CameraAdapter()

try:
    result = camera.initialize({
        "enable_head": True,
        "enable_wrist_camera": False,
        "rviz": False,
    })
    if not result.success:
        raise RuntimeError(f"相机初始化失败: {result.message}")

    frame = camera.get_camera_frame("camera")
    if frame is not None:
        print("读取到相机画面")
finally:
    camera.shutdown()
```

</details>

<details>
<summary id="cameraadapter_start_camera">🔧 <code>start_camera(camera_name: str = "camera") -> Result</code></summary>

启动指定相机。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  启动结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = camera.start_camera("camera")
if not result.success:
    print("启动相机失败:", result.message)
```

</details>

<details>
<summary id="cameraadapter_stop_camera">🔧 <code>stop_camera(camera_name: str = "camera") -> Result</code></summary>

停止指定相机。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  停止结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = camera.stop_camera("camera")
if not result.success:
    print("停止相机失败:", result.message)
```

</details>

<details>
<summary id="cameraadapter_is_connected">🔧 <code>is_connected() -> bool</code></summary>

判断相机适配器是否已经连接并初始化。

📤 **出参**
  已连接返回 `True`，否则返回 `False`。

🏷️ **返回类型**
  `bool`
* **Example:**

```python
if camera.is_connected():
    print("相机已连接")
```

</details>

<details>
<summary id="cameraadapter_check_health">🔧 <code>check_health(camera_name: str = "camera") -> dict</code></summary>

检查指定相机是否健康。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  健康状态字典，通常包含是否运行、是否收到图像、错误信息等。

🏷️ **返回类型**
  `dict`
* **Example:**

```python
health = camera.check_health("camera")
print(health)
```

</details>

<details>
<summary id="cameraadapter_get_performance_metrics">🔧 <code>get_performance_metrics(camera_name: str = "camera") -> dict</code></summary>

获取相机性能指标。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  性能指标字典，例如帧数、延迟、错误计数等。

🏷️ **返回类型**
  `dict`
* **Example:**

```python
metrics = camera.get_performance_metrics("camera")
print("相机性能:", metrics)
```

</details>

<details>
<summary id="cameraadapter_shutdown">🔧 <code>shutdown() -> Result</code></summary>

关闭相机适配器并释放资源。

📤 **出参**
  关闭结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = camera.shutdown()
if not result.success:
    print("关闭相机失败:", result.message)
```

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.perception_adapter.PerceptionAdapter

Bases: `IPerception`

感知适配器。它使用相机接口提供的数据，进一步做 AprilTag 等感知处理。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`initialize`](#perceptionadapter_initialize) | 初始化感知模块，并注入相机对象。 |
| [`get_tag_detections`](#perceptionadapter_get_tag_detections) | 获取 AprilTag 检测结果。 |
| [`get_latest_result`](#perceptionadapter_get_latest_result) | 获取最近一次完整感知结果。 |
| [`get_camera_frame`](#perceptionadapter_get_camera_frame) | 通过感知适配器转发调用相机帧接口。 |
| [`get_depth_data`](#perceptionadapter_get_depth_data) | 通过感知适配器转发调用深度数据接口。 |
| [`get_point_cloud`](#perceptionadapter_get_point_cloud) | 通过感知适配器转发调用点云接口。 |

---


<details>
<summary id="perceptionadapter_initialize">🔧 <code>initialize(camera: ICamera, config: dict | None = None) -> bool</code></summary>

初始化感知模块，并注入相机对象。

📥 **入参**
  * **camera** (*ICamera*) - 已初始化的相机对象。
  * **config** (*dict | None*) - 感知配置。

📤 **出参**
  初始化成功返回 `True`，失败返回 `False`。

🏷️ **返回类型**
  `bool`
* **Example:**

```python
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter
from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter

camera = CameraAdapter()
camera.initialize({"enable_head": True})

perception = PerceptionAdapter()
ok = perception.initialize(camera, {})

if not ok:
    raise RuntimeError("感知模块初始化失败")
```

</details>

<details>
<summary id="perceptionadapter_get_tag_detections">🔧 <code>get_tag_detections() -> list</code></summary>

获取 AprilTag 检测结果。

📤 **出参**
  检测结果列表。

🏷️ **返回类型**
  `list`
* **Example:**

```python
detections = perception.get_tag_detections()
for tag in detections:
    print(tag)
```

</details>

<details>
<summary id="perceptionadapter_get_latest_result">🔧 <code>get_latest_result() -> PerceptionResult</code></summary>

获取最近一次完整感知结果。

📤 **出参**
  最近一次感知结果。

🏷️ **返回类型**
  `PerceptionResult`
* **Example:**

```python
result = perception.get_latest_result()
print(result)
```

</details>

<details>
<summary id="perceptionadapter_get_camera_frame">🔧 <code>get_camera_frame(camera_name: str = "camera") -> CameraFrame | None</code></summary>

通过感知适配器转发调用相机帧接口。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  相机图像帧，失败返回 `None`。

🏷️ **返回类型**
  `CameraFrame | None`

</details>

<details>
<summary id="perceptionadapter_get_depth_data">🔧 <code>get_depth_data(camera_name: str = "camera") -> DepthData | None</code></summary>

通过感知适配器转发调用深度数据接口。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  深度数据，失败返回 `None`。

🏷️ **返回类型**
  `DepthData | None`

</details>

<details>
<summary id="perceptionadapter_get_point_cloud">🔧 <code>get_point_cloud(camera_name: str = "camera") -> PointCloudData | None</code></summary>

通过感知适配器转发调用点云接口。

📥 **入参**
  * **camera_name** (*str*) - 相机名称。

📤 **出参**
  点云数据，失败返回 `None`。

🏷️ **返回类型**
  `PointCloudData | None`

</details>

### 🏗️ *class* skills.atomic.perception.camera_capture.skill.CameraCaptureSkill

Bases: `ISkill`

相机捕获技能层封装。它不是最底层相机驱动，而是把“拍一次/采集一次相机数据”包装成技能执行流程。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`initialize`](#cameracaptureskill_initialize) | 初始化相机捕获技能参数。 |
| [`execute`](#cameracaptureskill_execute) | 执行一次相机捕获。 |

---


<details>
<summary id="cameracaptureskill_initialize">🔧 <code>initialize(params: CameraCaptureParams) -> Result</code></summary>

初始化相机捕获技能参数。

📥 **入参**
  * **params** (*CameraCaptureParams*) - 相机捕获参数，例如使用哪个相机、是否保存图片等。

📤 **出参**
  初始化结果。

🏷️ **返回类型**
  `Result`

</details>

<details>
<summary id="cameracaptureskill_execute">🔧 <code>execute() -> Result</code></summary>

执行一次相机捕获。

📤 **出参**
  执行结果，结果数据通常放在 `result.data`。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
# 伪代码：具体参数类以项目实际定义为准
skill = CameraCaptureSkill(camera)
skill.initialize(params)
result = skill.execute()

if result.success:
    print("相机捕获成功:", result.data)
```

</details>

---

## 2. 🚗 底盘接口

底盘相关接口分五层：

1. `SDKControlMixin`：高层硬件对象上的 SDK 直调接口，名字里通常带 `_sdk`。
2. `LowLevelSDKManager`：SDK 管理器，真正调用底层 `robot_sdk.control`。
3. `TimedCmdManager` / `TimedCommandMixin`：定时命令接口，适合要求“几秒内完成某个底盘动作”的场景。
4. `BaseControlMixin`：ROS topic 风格底盘接口，常用于发布 `cmd_vel` 或位姿命令。
5. `JibotChassisMixin`：JiBot/Jarvis 导航封装，适合目标点导航、到达判断、速度控制状态查询。

### 🏗️ *class* adapters.hardware.factory.HardwareFactory

Bases: `object`

硬件对象工厂。它根据配置创建机器人硬件对象。相机、底盘、手臂等接口通常都通过这个硬件对象调用。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`create_hardware`](#hardwarefactory_create_hardware) | 创建硬件对象。 |

---


<details>
<summary id="hardwarefactory_create_hardware">🔧 <code>*static* create_hardware(config: dict) -> object</code></summary>

创建硬件对象。

📥 **入参**
  * **config** (*dict*) - 机器人硬件配置。
    - `robot_type`：机器人类型，例如 `"leju_wheeled"`。
    - `angle_unit`：角度单位，例如 `"rad"`。
    - `sdk_managers_whitelist`：只初始化哪些 SDK 管理器，例如 `["low"]`。
    - `skip_camera`：是否跳过相机。
    - `skip_end_effector`：是否跳过末端执行器。
    - `skip_state_manager`：是否跳过状态管理器。
    - `skip_force_publishers`：是否跳过力控 publisher。

📤 **出参**
  硬件对象。后续通过这个对象调用 `initialize()`、`send_base_velocity_sdk()`、`shutdown()` 等接口。

🏷️ **返回类型**
  `object`
* **Example:**

```python
import time

from adapters.hardware.factory import HardwareFactory

# 创建机器人硬件对象。这里选择轮式机器人，并且只初始化底盘速度控制需要的 low SDK 管理器。
hardware = HardwareFactory.create_hardware(config={
    "robot_type": "leju_wheeled",
    "angle_unit": "rad",
    "sdk_managers_whitelist": ["low"],
    "skip_camera": True,
    "skip_end_effector": True,
    "skip_state_manager": True,
    "skip_force_publishers": True,
})

try:
    # 初始化硬件和 SDK。失败时 result.success 会是 False。
    result = hardware.initialize()
    if not result.success:
        raise RuntimeError(f"初始化失败: {result.message}")

    # 调用底盘 SDK 接口，让机器人以 0.2 m/s 向前运动。
    result = hardware.send_base_velocity_sdk(
        vx=0.2,   # 前后方向速度，正数通常表示前进，单位 m/s。
        vy=0.0,   # 左右方向速度，0 表示不横移，单位 m/s。
        vyaw=0.0, # 旋转角速度，0 表示不旋转，单位 rad/s。
    )
    if not result.success:
        raise RuntimeError(f"底盘运动失败: {result.message}")

    # 保持当前速度 2 秒。速度命令发出后，底盘会继续按这个速度运动。
    time.sleep(2.0)

    # 再次调用同一个 SDK 接口，但速度全部给 0，用来停止底盘。
    result = hardware.send_base_velocity_sdk(
        vx=0.0,
        vy=0.0,
        vyaw=0.0,
    )
    if not result.success:
        raise RuntimeError(f"底盘停止失败: {result.message}")

finally:
    # 无论前面成功还是报错，都关闭资源。
    hardware.shutdown()
```

上面代码里，真正用于“控制底盘”的 SDK/API 调用是：

```python
hardware.send_base_velocity_sdk(vx=0.2, vy=0.0, vyaw=0.0)
```

`HardwareFactory.create_hardware()` 和 `hardware.initialize()` 是使用接口前的准备步骤，`hardware.shutdown()` 是用完后的资源清理步骤。

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.mixins.sdk_control_mixin.SDKControlMixin

Bases: `object`

底盘 SDK 直调接口。业务脚本一般不会单独创建这个 mixin，而是通过 `hardware` 对象调用它的方法。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`send_base_velocity_sdk`](#sdkcontrolmixin_send_base_velocity_sdk) | 发送底盘速度控制命令。 |
| [`send_base_position_local_sdk`](#sdkcontrolmixin_send_base_position_local_sdk) | 发送底盘局部坐标系位置命令。 |
| [`send_base_position_world_sdk`](#sdkcontrolmixin_send_base_position_world_sdk) | 发送底盘世界坐标系位置命令。 |

---


<details>
<summary id="sdkcontrolmixin_send_base_velocity_sdk">🔧 <code>send_base_velocity_sdk(vx: float, vy: float, vyaw: float) -> Result</code></summary>

发送底盘速度控制命令。

这是最常用的底盘 SDK 接口之一。

📥 **入参**
  * **vx** (*float*) - 前后方向速度，单位 `m/s`。正数通常表示前进。
  * **vy** (*float*) - 左右方向速度，单位 `m/s`。是否支持横移取决于底盘类型。
  * **vyaw** (*float*) - 旋转角速度，单位 `rad/s`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
# 前进
result = hardware.send_base_velocity_sdk(vx=0.2, vy=0.0, vyaw=0.0)

# 停止
stop_result = hardware.send_base_velocity_sdk(vx=0.0, vy=0.0, vyaw=0.0)
```

</details>

<details>
<summary id="sdkcontrolmixin_send_base_position_local_sdk">🔧 <code>send_base_position_local_sdk(x: float, y: float, yaw: float) -> Result</code></summary>

发送底盘局部坐标系位置命令。

局部坐标系可以理解成“以机器人当前所在位置为参考”。例如 `x=1.0` 通常表示相对当前位置向前移动 1 米。

📥 **入参**
  * **x** (*float*) - 局部坐标系 x 方向位移，单位 `m`。
  * **y** (*float*) - 局部坐标系 y 方向位移，单位 `m`。
  * **yaw** (*float*) - 相对旋转角，单位通常是 `rad`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
# 相对当前位置前进 0.5 米，不旋转
result = hardware.send_base_position_local_sdk(x=0.5, y=0.0, yaw=0.0)
```

</details>

<details>
<summary id="sdkcontrolmixin_send_base_position_world_sdk">🔧 <code>send_base_position_world_sdk(x: float, y: float, yaw: float) -> Result</code></summary>

发送底盘世界坐标系位置命令。

世界坐标系可以理解成地图/全局坐标系。它适合让机器人移动到某个全局目标位姿。

📥 **入参**
  * **x** (*float*) - 世界坐标 x，单位 `m`。
  * **y** (*float*) - 世界坐标 y，单位 `m`。
  * **yaw** (*float*) - 世界坐标 yaw 朝向，单位通常是 `rad`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
# 移动到世界坐标 (1.0, 0.5)，朝向 0 rad
result = hardware.send_base_position_world_sdk(x=1.0, y=0.5, yaw=0.0)
```

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.services.sdk_manager.low_level_sdk_manager.LowLevelSDKManager

Bases: `BaseSDKManager`

低层 SDK 管理器。`SDKControlMixin` 的底盘 `_sdk` 接口通常会进一步调用这里的方法，然后再调用底层 `robot_sdk.control`。

一般业务脚本优先使用 `hardware.send_base_velocity_sdk()` 这类高层接口；只有在维护 SDK 管理器时才直接看这个类。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`control_base_velocity`](#lowlevelsdkmanager_control_base_velocity) | 控制底盘速度。 |
| [`control_base_position_local`](#lowlevelsdkmanager_control_base_position_local) | 控制底盘移动到局部坐标目标。 |
| [`control_base_position`](#lowlevelsdkmanager_control_base_position) | 控制底盘移动到世界坐标目标。 |

---


<details>
<summary id="lowlevelsdkmanager_control_base_velocity">🔧 <code>control_base_velocity(vel_cmd: tuple[float, float, float]) -> Result</code></summary>

控制底盘速度。

📥 **入参**
  * **vel_cmd** (*tuple[float, float, float]*) - 三元组 `(vx, vy, vyaw)`。

📤 **出参**
  控制结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
# 通常由 hardware.send_base_velocity_sdk(...) 间接调用
result = low_level_sdk_manager.control_base_velocity((0.2, 0.0, 0.0))
```

</details>

<details>
<summary id="lowlevelsdkmanager_control_base_position_local">🔧 <code>control_base_position_local(target_pos: tuple[float, float, float]) -> Result</code></summary>

控制底盘移动到局部坐标目标。

📥 **入参**
  * **target_pos** (*tuple[float, float, float]*) - 三元组 `(x, y, yaw)`。

📤 **出参**
  控制结果。

🏷️ **返回类型**
  `Result`

</details>

<details>
<summary id="lowlevelsdkmanager_control_base_position">🔧 <code>control_base_position(target_pos: tuple[float, float, float]) -> Result</code></summary>

控制底盘移动到世界坐标目标。

📥 **入参**
  * **target_pos** (*tuple[float, float, float]*) - 三元组 `(x, y, yaw)`。

📤 **出参**
  控制结果。

🏷️ **返回类型**
  `Result`

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.services.sdk_manager.timed_cmd_manager.TimedCmdManager

Bases: `BaseSDKManager`

TimedCmd 底盘命令管理器。它适合“在指定时间内完成底盘动作”的场景。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`send_chassis_world`](#timedcmdmanager_send_chassis_world) | 发送世界坐标系定时底盘命令。 |
| [`send_chassis_local`](#timedcmdmanager_send_chassis_local) | 发送局部坐标系定时底盘命令。 |
| [`send_multi_commands`](#timedcmdmanager_send_multi_commands) | 发送多条定时命令。 |

---


<details>
<summary id="timedcmdmanager_send_chassis_world">🔧 <code>send_chassis_world(x: float, y: float, yaw: float, desire_time: float) -> Result</code></summary>

发送世界坐标系定时底盘命令。

📥 **入参**
  * **x** (*float*) - 世界坐标 x，单位 `m`。
  * **y** (*float*) - 世界坐标 y，单位 `m`。
  * **yaw** (*float*) - 世界坐标 yaw；`send_world_position()` 内部已按弧度处理，传入值应为 `rad`。
  * **desire_time** (*float*) - 期望执行时间，单位 `s`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = timed_cmd_manager.send_chassis_world(
    x=1.0,
    y=0.0,
    yaw=0.0,
    desire_time=3.0,
)
```

</details>

<details>
<summary id="timedcmdmanager_send_chassis_local">🔧 <code>send_chassis_local(x: float, y: float, yaw: float, desire_time: float) -> Result</code></summary>

发送局部坐标系定时底盘命令。

📥 **入参**
  * **x** (*float*) - 局部 x 位移，单位 `m`。
  * **y** (*float*) - 局部 y 位移，单位 `m`。
  * **yaw** (*float*) - 相对旋转角，单位通常是 `rad`。
  * **desire_time** (*float*) - 期望执行时间，单位 `s`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = timed_cmd_manager.send_chassis_local(
    x=0.5,
    y=0.0,
    yaw=0.0,
    desire_time=2.0,
)
```

</details>

<details>
<summary id="timedcmdmanager_send_multi_commands">🔧 <code>send_multi_commands(commands: list, is_sync: bool = True) -> Result</code></summary>

发送多条定时命令。

📥 **入参**
  * **commands** (*list*) - 多条命令组成的列表。
  * **is_sync** (*bool*) - 是否同步执行。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.mixins.timed_command_mixin.TimedCommandMixin

Bases: `object`

高层 TimedCmd 硬件封装。业务脚本通常通过 `hardware` 对象调用这些方法。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`send_base_velocity_timed`](#timedcommandmixin_send_base_velocity_timed) | 发送 TimedCmd 底盘速度命令。 |
| [`send_base_pose_timed`](#timedcommandmixin_send_base_pose_timed) | 发送指定时间内完成的底盘位姿命令。 |
| [`send_timed_base_pose`](#timedcommandmixin_send_timed_base_pose) | 标准硬件接口形式的定时底盘位姿命令。 |
| [`send_timed_multi_commands`](#timedcommandmixin_send_timed_multi_commands) | 发送多条 TimedCmd 命令。 |

---


<details>
<summary id="timedcommandmixin_send_base_velocity_timed">🔧 <code>send_base_velocity_timed(vx: float, vy: float, vyaw: float, frame: FrameType = FrameType.LOCAL) -> Result</code></summary>

发送 TimedCmd 底盘速度命令。

📥 **入参**
  * **vx** (*float*) - 前后速度，单位 `m/s`。
  * **vy** (*float*) - 左右速度，单位 `m/s`。
  * **vyaw** (*float*) - 旋转角速度，单位 `rad/s`。
  * **frame** (*FrameType*) - 坐标系，默认 `FrameType.LOCAL`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.send_base_velocity_timed(
    vx=0.2,
    vy=0.0,
    vyaw=0.0,
)
```

</details>

<details>
<summary id="timedcommandmixin_send_base_pose_timed">🔧 <code>send_base_pose_timed(x: float, y: float, yaw: float, frame: FrameType = FrameType.WORLD, desire_time: float = 3.0) -> Result</code></summary>

发送指定时间内完成的底盘位姿命令。

📥 **入参**
  * **x** (*float*) - x 方向位置或位移，单位 `m`。
  * **y** (*float*) - y 方向位置或位移，单位 `m`。
  * **yaw** (*float*) - yaw 角，单位通常是 `rad`。
  * **frame** (*FrameType*) - 坐标系，常见值为 `FrameType.LOCAL` 或 `FrameType.WORLD`。
  * **desire_time** (*float*) - 期望执行时间，单位 `s`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
# 2 秒内相对当前位置前进 0.5 米
result = hardware.send_base_pose_timed(
    x=0.5,
    y=0.0,
    yaw=0.0,
    frame=FrameType.LOCAL,
    desire_time=2.0,
)
```

</details>

<details>
<summary id="timedcommandmixin_send_timed_base_pose">🔧 <code>send_timed_base_pose(x: float, y: float, yaw: float, desire_time: float, frame: FrameType = FrameType.WORLD) -> Result</code></summary>

标准硬件接口形式的定时底盘位姿命令。

它内部会委托给 `send_base_pose_timed()`，只是参数顺序更符合 `IHardware` 标准接口。

📥 **入参**
  * **x** (*float*) - x 方向位置或位移，单位 `m`。
  * **y** (*float*) - y 方向位置或位移，单位 `m`。
  * **yaw** (*float*) - yaw 角，单位通常是 `rad`。
  * **desire_time** (*float*) - 期望执行时间，单位 `s`。
  * **frame** (*FrameType*) - 坐标系，默认 `FrameType.WORLD`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
from core.domain.enums import FrameType

result = hardware.send_timed_base_pose(
    x=0.5,
    y=0.0,
    yaw=0.0,
    desire_time=2.0,
    frame=FrameType.LOCAL,
)
```

</details>

<details>
<summary id="timedcommandmixin_send_timed_multi_commands">🔧 <code>send_timed_multi_commands(commands: list[dict], is_sync: bool = False) -> Result</code></summary>

发送多条 TimedCmd 命令。

这个接口不只支持底盘，也可能包含躯干、腿、手臂等命令。本文只把它作为“底盘相关定时命令入口”列出。

📥 **入参**
  * **commands** (*list[dict]*) - 命令列表，每条命令通常包含 `planner_index`、`desire_time`、`cmd_vec`。
  * **is_sync** (*bool*) - 是否同步等待执行完成，默认 `False`。

📤 **出参**
  命令发送结果，成功时 `result.data` 可能包含实际执行时间。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
commands = [
    {
        "planner_index": 1,      # 具体 planner_index 以项目 TimedCmd 约定为准。
        "desire_time": 2.0,
        "cmd_vec": [0.5, 0.0, 0.0],
    }
]

result = hardware.send_timed_multi_commands(commands, is_sync=True)
```

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.mixins.base_control_mixin.BaseControlMixin

Bases: `object`

ROS topic 风格的底盘控制封装。它更接近 ROS 控制习惯，常用于发布速度或位姿命令。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`send_base_velocity`](#basecontrolmixin_send_base_velocity) | 发送底盘速度命令。 |
| [`send_base_pose`](#basecontrolmixin_send_base_pose) | 发送底盘位姿命令。 |
| [`send_base_position`](#basecontrolmixin_send_base_position) | 发送底盘位置命令。 |
| [`publish_cmd_vel`](#basecontrolmixin_publish_cmd_vel) | 直接发布 `/cmd_vel` 速度命令。 |
| [`send_world_position`](#basecontrolmixin_send_world_position) | 发送世界坐标系底盘位置命令。 |

---


#### 💡 NOTE

`BaseControlMixin` 里的 yaw 单位需要分接口看：`send_base_pose()` 接收用户单位并在内部调用 `_to_rad` 转换；`send_base_position()` 和 `send_world_position()` 的 yaw 在接口内部已按弧度处理，文档中分别标注，不能统一写成“单位通常是 rad”。

<details>
<summary id="basecontrolmixin_send_base_velocity">🔧 <code>send_base_velocity(vx: float, vy: float, vyaw: float, frame: FrameType = FrameType.LOCAL) -> Result</code></summary>

发送底盘速度命令。

📥 **入参**
  * **vx** (*float*) - 前后速度，单位 `m/s`。
  * **vy** (*float*) - 左右速度，单位 `m/s`。
  * **vyaw** (*float*) - 旋转角速度，单位 `rad/s`。
  * **frame** (*FrameType*) - 坐标系，默认局部坐标系。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.send_base_velocity(vx=0.2, vy=0.0, vyaw=0.0)
```

</details>

<details>
<summary id="basecontrolmixin_send_base_pose">🔧 <code>send_base_pose(x: float, y: float, yaw: float, frame: FrameType = FrameType.WORLD) -> Result</code></summary>

发送底盘位姿命令。

📥 **入参**
  * **x** (*float*) - x 坐标或位移，单位 `m`。
  * **y** (*float*) - y 坐标或位移，单位 `m`。
  * **yaw** (*float*) - yaw 角，使用用户单位；该接口内部会调用 `_to_rad` 转成弧度，实际按 `rad` 还是 `deg` 取决于硬件对象的 `angle_unit` 配置。
  * **frame** (*FrameType*) - 坐标系，默认世界坐标系。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.send_base_pose(x=1.0, y=0.0, yaw=0.0)
```

</details>

<details>
<summary id="basecontrolmixin_send_base_position">🔧 <code>send_base_position(x: float, y: float, yaw: float) -> Result</code></summary>

发送底盘位置命令。

📥 **入参**
  * **x** (*float*) - x 坐标，单位 `m`。
  * **y** (*float*) - y 坐标，单位 `m`。
  * **yaw** (*float*) - yaw 角；`send_base_position()` 内部已按弧度处理，传入值应为 `rad`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`

</details>

<details>
<summary id="basecontrolmixin_publish_cmd_vel">🔧 <code>publish_cmd_vel(linear_x: float, linear_y: float, angular_z: float, duration: float = 0.0) -> Result</code></summary>

直接发布 `/cmd_vel` 速度命令。

这是 ROS topic 风格接口，适合调试或需要直接发速度 topic 的场景。

📥 **入参**
  * **linear_x** (*float*) - x 方向线速度，单位 `m/s`。
  * **linear_y** (*float*) - y 方向线速度，单位 `m/s`。
  * **angular_z** (*float*) - z 轴角速度，单位 `rad/s`。
  * **duration** (*float*) - 持续发布时间，单位 `s`。如果为 `0.0`，只发布一次。

📤 **出参**
  发布结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.publish_cmd_vel(
    linear_x=0.2,
    linear_y=0.0,
    angular_z=0.0,
    duration=2.0,
)
```

</details>

<details>
<summary id="basecontrolmixin_send_world_position">🔧 <code>send_world_position(x: float, y: float, yaw: float) -> Result</code></summary>

发送世界坐标系底盘位置命令。

📥 **入参**
  * **x** (*float*) - 世界坐标 x，单位 `m`。
  * **y** (*float*) - 世界坐标 y，单位 `m`。
  * **yaw** (*float*) - 世界坐标 yaw，单位通常是 `rad`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.send_world_position(x=1.0, y=0.0, yaw=0.0)
```

</details>

### 🏗️ *class* adapters.hardware.leju_wheeled.mixins.jibot.chassis_mixin.JibotChassisMixin

Bases: `object`

JiBot/Jarvis 底盘导航封装。它用于相对移动、目标点移动、到达判断和速度控制状态查询。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`base_move_relative_jibot`](#jibotchassismixin_base_move_relative_jibot) | 通过 JiBot/Jarvis 让底盘做相对移动。 |
| [`base_move_to_target_jibot`](#jibotchassismixin_base_move_to_target_jibot) | 通过 JiBot/Jarvis 让底盘移动到目标位姿。 |
| [`check_arrived_jibot`](#jibotchassismixin_check_arrived_jibot) | 检查 JiBot/Jarvis 导航是否到达。 |
| [`enable_vel_control_jibot`](#jibotchassismixin_enable_vel_control_jibot) | 开启或关闭 JiBot/Jarvis 速度控制模式。 |
| [`get_vel_control_state_jibot`](#jibotchassismixin_get_vel_control_state_jibot) | 获取 JiBot/Jarvis 速度控制模式状态。 |

---


<details>
<summary id="jibotchassismixin_base_move_relative_jibot">🔧 <code>base_move_relative_jibot(x: float, y: float, yaw: float, timeout: float = 10.0) -> Result</code></summary>

通过 JiBot/Jarvis 让底盘做相对移动。

📥 **入参**
  * **x** (*float*) - 相对 x 位移，单位 `m`。
  * **y** (*float*) - 相对 y 位移，单位 `m`。
  * **yaw** (*float*) - 相对 yaw，单位通常是 `rad`。
  * **timeout** (*float*) - 超时时间，单位 `s`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.base_move_relative_jibot(
    x=0.5,
    y=0.0,
    yaw=0.0,
    timeout=10.0,
)
```

</details>

<details>
<summary id="jibotchassismixin_base_move_to_target_jibot">🔧 <code>base_move_to_target_jibot(x: float, y: float, yaw: float, timeout: float = 10.0) -> Result</code></summary>

通过 JiBot/Jarvis 让底盘移动到目标位姿。

📥 **入参**
  * **x** (*float*) - 目标 x 坐标，单位 `m`。
  * **y** (*float*) - 目标 y 坐标，单位 `m`。
  * **yaw** (*float*) - 目标 yaw，单位通常是 `rad`。
  * **timeout** (*float*) - 超时时间，单位 `s`。

📤 **出参**
  命令发送结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.base_move_to_target_jibot(
    x=1.0,
    y=0.5,
    yaw=0.0,
    timeout=15.0,
)
```

</details>

<details>
<summary id="jibotchassismixin_check_arrived_jibot">🔧 <code>check_arrived_jibot(target_id: str | None = None, timeout: float = 3.0) -> Result</code></summary>

检查 JiBot/Jarvis 导航是否到达。

📥 **入参**
  * **target_id** (*str | None*) - 目标点 ID，没有时按当前导航任务判断。
  * **timeout** (*float*) - 超时时间，单位 `s`。

📤 **出参**
  到达状态结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.check_arrived_jibot(timeout=3.0)
if result.success:
    print("到达状态:", result.data)
```

</details>

<details>
<summary id="jibotchassismixin_enable_vel_control_jibot">🔧 <code>enable_vel_control_jibot(enable: bool) -> Result</code></summary>

开启或关闭 JiBot/Jarvis 速度控制模式。

📥 **入参**
  * **enable** (*bool*) - `True` 表示开启，`False` 表示关闭。

📤 **出参**
  设置结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.enable_vel_control_jibot(True)
```

</details>

<details>
<summary id="jibotchassismixin_get_vel_control_state_jibot">🔧 <code>get_vel_control_state_jibot(timeout: float = 3.0) -> Result</code></summary>

获取 JiBot/Jarvis 速度控制模式状态。

📥 **入参**
  * **timeout** (*float*) - 超时时间，单位 `s`。

📤 **出参**
  查询结果。

🏷️ **返回类型**
  `Result`
* **Example:**

```python
result = hardware.get_vel_control_state_jibot(timeout=3.0)
print(result.data)
```

</details>

### 🏗️ *class* skills.atomic.motion.chassis_velocity.skill.ChassisVelocitySkill

Bases: `ISkill`

底盘速度技能封装。它把底盘速度控制包装成技能流程。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`initialize`](#chassisvelocityskill_initialize) | 初始化速度技能参数。 |
| [`execute`](#chassisvelocityskill_execute) | 执行底盘速度技能。 |

---


<details>
<summary id="chassisvelocityskill_initialize">🔧 <code>initialize(params: ChassisVelocityParams) -> Result</code></summary>

初始化速度技能参数。

📥 **入参**
  * **params** (*ChassisVelocityParams*) - 速度控制参数，例如 `vx`、`vy`、`vyaw`、持续时间等。

📤 **出参**
  初始化结果。

🏷️ **返回类型**
  `Result`

</details>

<details>
<summary id="chassisvelocityskill_execute">🔧 <code>execute() -> Result</code></summary>

执行底盘速度技能。

📤 **出参**
  执行结果。

🏷️ **返回类型**
  `Result`

</details>

### 🏗️ *class* skills.atomic.manipulation.pos_base_control.skill.PosBaseControlSkill

Bases: `ISkill`

底盘位置控制技能封装。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`initialize`](#posbasecontrolskill_initialize) | 初始化底盘位置控制参数。 |
| [`execute`](#posbasecontrolskill_execute) | 执行底盘位置控制技能。 |

---


<details>
<summary id="posbasecontrolskill_initialize">🔧 <code>initialize(params: PosBaseControlParams) -> Result</code></summary>

初始化底盘位置控制参数。

📥 **入参**
  * **params** (*PosBaseControlParams*) - 目标位置、坐标系、执行时间等参数。

📤 **出参**
  初始化结果。

🏷️ **返回类型**
  `Result`

</details>

<details>
<summary id="posbasecontrolskill_execute">🔧 <code>execute() -> Result</code></summary>

执行底盘位置控制技能。

📤 **出参**
  执行结果。

🏷️ **返回类型**
  `Result`

</details>

### 🏗️ *class* skills.atomic.manipulation.vel_control.skill.VelControlSkill

Bases: `ISkill`

底盘速度控制技能封装。

#### 📋 接口总览

| 接口 | 说明 |
|:---|:---|
| [`initialize`](#velcontrolskill_initialize) | 初始化速度控制技能参数。 |
| [`execute`](#velcontrolskill_execute) | 执行速度控制技能。 |

---


<details>
<summary id="velcontrolskill_initialize">🔧 <code>initialize(params: VelControlParams) -> Result</code></summary>

初始化速度控制技能参数。

📥 **入参**
  * **params** (*VelControlParams*) - 速度控制参数。

📤 **出参**
  初始化结果。

🏷️ **返回类型**
  `Result`

</details>

<details>
<summary id="velcontrolskill_execute">🔧 <code>execute() -> Result</code></summary>

执行速度控制技能。

📤 **出参**
  执行结果。

🏷️ **返回类型**
  `Result`

</details>

---

## 3. 🔍 接口识别指南

不是所有函数都是给你直接调用的 SDK/API。可以这样判断：

| 判断方式 | 说明 |
| --- | --- |
| 是否在 `interfaces`、`adapter`、`manager`、`mixin`、`skill` 这类文件中公开定义 | 这些通常是项目希望外部或上层调用的接口。 |
| 方法名是否表达一个完整能力 | 例如 `get_camera_frame()`、`send_base_velocity_sdk()` 就是完整能力。 |
| 是否有明确入参和返回值 | API 文档通常会说明参数、返回值和失败情况。 |
| 是否以下划线开头 | 例如 `_setup_subscribers()`、`_callback()` 通常是内部方法，不建议当作 SDK 接口直接调用。 |
| 是否只是内部变量或临时辅助函数 | 例如 `_timed` 如果只是名字片段或内部字段，不一定是 SDK 接口。 |

`_timed` 和 `jibot` 的判断：

- 名字里带 `_timed` 的方法，例如 `send_base_pose_timed()`，是项目封装出来的定时命令 API，可以当作底盘接口使用。
- `jibot` 不是一个单独函数，而是一组 JiBot/Jarvis 底盘导航接口的命名后缀，例如 `base_move_to_target_jibot()`。
- 只有像 `hardware.base_move_to_target_jibot(...)` 这样能被外部对象调用、有明确参数和返回值的方法，才整理为 SDK/API 接口。

## 4. 📷 相机名称说明

在 `get_camera_frame("camera")` 里，`"camera"` 是相机名称。

它的作用是告诉接口：“我要读取哪一个相机的数据。”

常见理解：

```python
frame = camera.get_camera_frame("camera")
```

这句话可以读成：

> 从名字叫 `"camera"` 的相机里读取一帧图像。

在这个项目里，`"camera"` 通常代表头部主相机。其他相机名称可能包括腕部相机等，具体要看机器人配置和 ROS topic 配置。

## 5. 🚀 完整示例

下面这个脚本展示了如何在一个大脚本里同时使用相机和底盘接口。每行关键代码旁边都写了说明。

```python
import time  # 导入 Python 自带的 time 模块，后面用 time.sleep() 控制等待时间。

from adapters.hardware.factory import HardwareFactory  # 导入硬件工厂，用它创建机器人硬件对象。
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter  # 导入相机适配器，用它读取相机数据。

# 创建硬件对象。这个对象后面用来调用底盘 SDK 接口。
hardware = HardwareFactory.create_hardware(config={
    "robot_type": "leju_wheeled",          # 指定机器人类型为乐聚轮式机器人。
    "angle_unit": "rad",                   # 指定角度单位为弧度，yaw/vyaw 等角度参数按 rad 理解。
    "sdk_managers_whitelist": ["low"],     # 只启用 low SDK 管理器，底盘 SDK 速度控制会用到它。
    "skip_camera": True,                   # 硬件对象里跳过相机，因为下面单独创建 CameraAdapter。
    "skip_end_effector": True,             # 跳过夹爪/末端执行器。
    "skip_state_manager": True,            # 跳过状态管理器，简化示例。
    "skip_force_publishers": True,         # 跳过力控 publisher。
})

# 创建相机对象。这个对象后面用来调用 get_camera_frame()。
camera = CameraAdapter()

try:
    # 初始化底盘相关 SDK。初始化失败就抛出错误。
    result = hardware.initialize()
    if not result.success:
        raise RuntimeError(f"硬件初始化失败: {result.message}")

    # 初始化相机。这里启用头部相机，不启用腕部相机。
    result = camera.initialize({
        "enable_head": True,
        "enable_wrist_camera": False,
        "rviz": False,
    })
    if not result.success:
        raise RuntimeError(f"相机初始化失败: {result.message}")

    # 调用相机 API，从名字为 camera 的相机读取一帧图像。
    frame = camera.get_camera_frame("camera")
    if frame is None:
        print("没有读到相机图像")
    else:
        print("读到相机图像:", frame.color_image.shape)

    # 调用底盘 SDK/API，让底盘以 0.2 m/s 向前运动。
    result = hardware.send_base_velocity_sdk(
        vx=0.2,    # 前进速度，单位 m/s。
        vy=0.0,    # 左右速度，0 表示不横移。
        vyaw=0.0,  # 旋转速度，0 表示不旋转。
    )
    if not result.success:
        raise RuntimeError(f"底盘运动失败: {result.message}")

    # 保持运动 2 秒。
    time.sleep(2.0)

    # 再次调用底盘 SDK/API，速度全部置 0，用来停止底盘。
    result = hardware.send_base_velocity_sdk(
        vx=0.0,
        vy=0.0,
        vyaw=0.0,
    )
    if not result.success:
        raise RuntimeError(f"底盘停止失败: {result.message}")

finally:
    # 先关闭相机，释放相机 topic、进程和缓存资源。
    camera.shutdown()

    # 再关闭硬件对象，释放 SDK 管理器和底盘相关资源。
    hardware.shutdown()
```

这个示例里真正的 SDK/API 调用包括：

- `HardwareFactory.create_hardware(...)`：创建机器人硬件对象。
- `hardware.initialize()`：初始化硬件和 SDK 管理器。
- `camera.initialize(...)`：初始化相机。
- `camera.get_camera_frame("camera")`：读取相机图像。
- `hardware.send_base_velocity_sdk(...)`：控制底盘速度。
- `camera.shutdown()`：关闭相机资源。
- `hardware.shutdown()`：关闭硬件资源。

<style>
summary:target ~ * {
  display: block !important;
}
</style>

<script>
function expandAndCenter() {
  var hash = decodeURIComponent(location.hash.slice(1));
  if (!hash) return;
  var el = document.getElementById(hash);
  if (!el) return;
  var details = el.closest('details');
  if (details) {
    if (!details.open) {
      details.open = true;
    }
    setTimeout(function() {
      details.scrollIntoView({ block: 'center', behavior: 'smooth' });
    }, 100);
  }
}
if (document.readyState !== 'loading') {
  expandAndCenter();
} else {
  document.addEventListener('DOMContentLoaded', expandAndCenter);
}
window.addEventListener('hashchange', expandAndCenter);
document.addEventListener('click', function(e) {
  if (e.target.closest('a[href^="#"]')) {
    setTimeout(expandAndCenter, 50);
  }
}, true);
</script>
