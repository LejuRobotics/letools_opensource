# Core 层 - 核心领域层

## 📋 概述

Core 层是 LeTools 框架的**核心领域层**，定义了机器人控制的抽象概念、数据结构和接口规范。

### 核心职责

- ✅ **定义接口**：提供硬件、技能、相机等模块的标准接口（`IHardware`, `ISkill`, `ICamera`）
- ✅ **数据结构**：定义通用的数据类型（`Pose6D`, `Result`, `JointState` 等）
- ✅ **业务逻辑**：实现与具体硬件无关的核心算法（坐标变换、插值器等）
- ✅ **枚举类型**：定义统一的枚举常量（`FrameType`, `MPCControlMode`, `ArmSide` 等）

---

## 新手理解：Core 层为什么重要

`core/` 可以理解成 LeTools 的“共同语言层”。它本身不应该直接控制机器人，而是定义大家都要遵守的接口、数据结构和返回格式。

如果没有 Core 层，上层和底层可能会各说各话：

```text
测试脚本想传 x/y/yaw
适配器想要 Pose6D
SDK 需要 list[float]
ROS 需要某种 msg 类型
```

Core 层的作用就是把这些常用概念统一起来。

### 1. IHardware：机器人应该具备哪些能力

`IHardware` 是硬件能力清单。它规定一个机器人硬件对象至少应该提供哪些方法，例如：

```python
send_base_velocity(...)
send_base_pose(...)
send_arm_joint_trajectory(...)
send_ee_pose(...)
set_mpc_mode(...)
shutdown()
```

`IHardware` 自己不实现具体控制，它只是规定“必须有这些能力”。真正实现这些能力的是 `adapters/` 里的 `LejuWheeledArmHardware` 等具体类。

### 2. Result：统一表达成功或失败

`Result` 是统一返回值。很多方法不会直接返回 `True/False`，而是返回：

```python
Result(success=True, message="...", data=...)
```

这样上层可以统一判断：

```python
result = hardware.send_base_velocity(...)
if not result.success:
    print(result.message)
```

它的好处是：失败时不只是知道“失败了”，还能知道失败原因。

### 3. Pose6D：统一表示位置和姿态

`Pose6D` 用来表示机器人、手臂末端、视觉标签等在空间中的位姿：

```text
x, y, z      # 位置，单位通常是米
roll, pitch, yaw  # 姿态，单位通常是弧度
```

有了 `Pose6D`，上层就不用每个地方都临时约定“这个列表第 0 个是 x，第 1 个是 y”。

### 4. FrameType / ArmSide / MPCControlMode：统一枚举

枚举是为了避免魔法数字和字符串乱飞。例如：

```python
FrameType.LOCAL
FrameType.WORLD
ArmSide.LEFT
ArmSide.RIGHT
MPCControlMode.ARM_ONLY
MPCControlMode.BASE_ARM
```

这样代码可读性比 `0`、`1`、`"left"`、`"world"` 更好，也更不容易写错。

### 5. Core 和其它层的关系

```text
apps / orchestration / skills
└── 使用 Core 定义的数据结构和接口

adapters
└── 实现 Core 定义的 IHardware 接口

drivers / ROS / SDK
└── 被 adapters 调用，完成真正硬件通信
```

一句话总结：

```text
Core 不直接干活，但规定所有层怎么说话。
```

---
## 🏗️ 架构设计

### 分层结构

```
Core Layer
├── Interfaces (接口定义)
│   ├── IHardware - 硬件抽象接口
│   ├── ISkill - 技能接口
│   ├── ICamera - 相机接口
│   └── IPerception - 感知接口
│
├── Domain (领域模型)
│   ├── Pose6D - 六自由度位姿
│   ├── Result - 统一返回结果
│   ├── JointState - 关节状态
│   ├── EndEffector - 末端执行器
│   ├── Camera - 相机配置
│   ├── Perception - 感知数据
│   ├── ChassisOptions - 底盘导航选项
│   ├── RuckigParams - 规划器参数
│   └── Tag - 视觉标签
│
├── Common (通用工具)
│   ├── Transform - 坐标变换
│   ├── Interpolator - 轨迹插值
│   ├── MathUtils - 数学工具
│   ├── ConfigLoader - 配置加载
│   ├── AppConfig - 应用配置
│   └── Logger - 日志系统
│
└── Services (核心服务)
    ├── SDKManager - SDK 管理器
    └── StateManager - 状态管理器
```

### 设计原则

1. **依赖倒置**：高层模块定义接口，低层模块实现接口
2. **单一职责**：每个模块只负责一个明确的职责
3. **开闭原则**：对扩展开放，对修改关闭
4. **无外部依赖**：Core 层不依赖 ROS、SDK 等外部库

---

## 📂 目录结构

```
core/
├── __init__.py
│
├── interfaces/                    # 接口定义
│   ├── i_hardware.py             # 硬件抽象接口
│   ├── i_skill.py                # 技能接口
│   ├── i_camera.py               # 相机接口
│   └── i_perception.py           # 感知接口
│
├── domain/                        # 领域模型
│   ├── pose.py                   # Pose6D 位姿类
│   ├── result.py                 # Result 返回结果类
│   ├── enums.py                  # 枚举类型
│   ├── joint_state.py            # 关节状态
│   ├── end_effector.py           # 末端执行器
│   ├── camera.py                 # 相机配置
│   ├── perception.py             # 感知数据
│   ├── observation.py            # 观测数据
│   ├── trajectory.py             # 轨迹数据
│   ├── task.py                   # 任务定义
│   ├── skill_params.py           # 技能参数
│   ├── chassis_options.py        # 底盘导航选项
│   ├── ruckig_params.py          # Ruckig 规划器参数
│   └── tag.py                    # 视觉标签
│
├── common/                        # 通用工具
│   ├── transform.py              # 坐标变换工具
│   ├── interpolator.py           # 轨迹插值器
│   ├── math_utils.py             # 数学工具函数
│   ├── config_loader.py          # YAML 配置加载器
│   ├── app_config.py             # 应用配置加载
│   ├── logger.py                 # 统一日志系统
│   └── exceptions.py             # 自定义异常类
│
└── (services/ 已迁移至 adapters/hardware/leju_wheeled/services/)
```

---

## 🔧 核心组件详解

### 1. Interfaces - 接口定义

#### IHardware - 硬件抽象接口

**位置**：`core/interfaces/i_hardware.py`

**功能**：定义所有硬件适配器必须实现的标准接口。

**主要方法**：

| 类别 | 方法 | 说明 |
|------|------|------|
| **连接管理** | `initialize()` | 初始化硬件连接 |
| | `shutdown()` | 断开连接并释放资源 |
| | `is_connected` | 检查连接状态 |
| **底盘控制** | `send_base_velocity()` | 底盘速度控制 |
| | `send_base_pose()` | 底盘位置控制 |
| **躯干控制** | `send_torso_pose()` | 躯干位姿控制 |
| **手臂控制** | `send_ee_pose()` | 末端位姿控制 |
| | `send_arm_joint_trajectory()` | 关节轨迹控制 |
| **腿部控制** | `send_leg_joint_command()` | 腿部关节控制 |
| **模式切换** | `set_mpc_mode()` | 设置 MPC 模式 |
| | `enable_quick_mode()` | 启用快速模式 |
| **末端执行器** | `control_end_effector()` | 控制夹爪/灵巧手 |

**使用示例**：

```python
from core.interfaces.i_hardware import IHardware

class MyHardwareAdapter(IHardware):
    def initialize(self) -> Result:
        # 实现初始化逻辑
        return Result.ok("Initialized")
    
    def send_base_velocity(self, vx, vy, vyaw, frame):
        # 实现底盘速度控制
        pass
    
    # ... 实现其他方法
```

---

#### ISkill - 技能接口

**位置**：`core/interfaces/i_skill.py`

**功能**：定义原子技能的标准接口。

**主要方法**：
- `execute(params)` - 执行技能
- `cancel()` - 取消技能
- `get_status()` - 获取技能状态

---

#### ICamera - 相机接口

**位置**：`core/interfaces/i_camera.py`

**功能**：定义相机操作的标准接口。

**主要方法**：
- `capture_image()` - 捕获图像
- `start_streaming()` - 启动视频流
- `stop_streaming()` - 停止视频流

---

#### IPerception - 感知接口

**位置**：`core/interfaces/i_perception.py`

**功能**：定义感知算法的标准接口。

**主要方法**：
- `detect_qrcode()` - 二维码识别
- `detect_aruco()` - ArUco 标记识别
- `estimate_pose()` - 位姿估计

---

### 2. Domain - 领域模型

#### Pose6D - 六自由度位姿

**位置**：`core/domain/pose.py`

**功能**：表示三维空间中的位置和姿态。

**属性**：
- `x, y, z` - 位置（米）
- `roll, pitch, yaw` - 姿态（弧度）

**方法**：
- `to_matrix()` - 转换为 4x4 变换矩阵
- `transform(frame)` - 坐标系变换

**使用示例**：

```python
from core.domain.pose import Pose6D

# 创建位姿
pose = Pose6D(x=0.5, y=0.0, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)

# 转换为矩阵
matrix = pose.to_matrix()

# 坐标系变换
world_pose = pose.transform(FrameType.WORLD)
```

---

#### Result - 统一返回结果

**位置**：`core/domain/result.py`

**功能**：封装操作的成功/失败状态和消息。

**属性**：
- `success` - 是否成功（bool）
- `message` - 结果消息（str）
- `data` - 可选的返回数据

**工厂方法**：
- `Result.ok(message, data)` - 创建成功结果
- `Result.fail(message)` - 创建失败结果

**使用示例**：

```python
from core.domain.result import Result

def some_operation():
    try:
        # 业务逻辑
        return Result.ok("Operation successful", data=result)
    except Exception as e:
        return Result.fail(f"Operation failed: {e}")

# 检查结果
result = some_operation()
if result.success:
    print(f"Success: {result.message}")
else:
    print(f"Failed: {result.message}")
```

---

#### Enums - 枚举类型

**位置**：`core/domain/enums.py`

**定义的枚举**：

| 枚举 | 值 | 说明 |
|------|-----|------|
| **FrameType** | `LOCAL`, `WORLD` | 坐标系类型 |
| **MPCControlMode** | `STAND`, `WALK`, `ARM_CONTROL` | MPC 控制模式 |
| **ArmSide** | `LEFT`, `RIGHT`, `BOTH` | 手臂侧别 |
| **EndEffectorType** | `LEJU_CLAW`, `QIANGNAO_HAND` | 末端执行器类型 |

**使用示例**：

```python
from core.domain.enums import FrameType, ArmSide

# 使用枚举
hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0, frame=FrameType.LOCAL)
hardware.send_ee_pose(side=ArmSide.LEFT, pose=pose, frame=FrameType.WORLD)
```

---

#### EndEffector - 末端执行器

**位置**：`core/domain/end_effector.py`

**功能**：定义末端执行器的命令和状态。

**主要类**：
- `GripperCommand` - 夹爪命令（开合度、力度）
- `HandFingerCommand` - 灵巧手手指命令
- `EndEffectorState` - 末端执行器状态
- `GripperStatus` - 夹爪状态

**使用示例**：

```python
from core.domain.end_effector import GripperCommand

# 控制夹爪
cmd = GripperCommand(open_ratio=0.5, force=10.0)
hardware.control_end_effector(side=ArmSide.LEFT, cmd=cmd)
```

---

#### ChassisOptions - 底盘导航选项

**位置**：`core/domain/chassis_options.py`

**功能**：定义底盘导航（如 `MoveToTargetOptions`）的参数载体，与 ROS 消息包 `leju_mobile_base_msgs/MoveToTargetOptions` 对应，但在 Core 层保持零外部依赖。

**主要类**：
- `MoveToTargetOptions` - 底盘导航选项配置

**属性**：
- `avoid_enabled` - 是否启用避障
- `avoid_distance` - 避障距离（m）
- `linear_velocity` - 线速度（m/s）
- `angular_velocity` - 角速度（rad/s）
- `position_threshold` - 位置到达阈值（m）
- `angle_threshold` - 角度到达阈值（rad）
- `allow_rotation` - 是否允许旋转

**使用示例**：

```python
from core.domain.chassis_options import MoveToTargetOptions

options = MoveToTargetOptions(
    avoid_enabled=False,
    linear_velocity=0.15,
    angular_velocity=0.25,
    position_threshold=0.08
)
```

---

#### RuckigParams - Ruckig 规划器参数

**位置**：`core/domain/ruckig_params.py`

**功能**：封装 Ruckig 在线轨迹规划器的速度、加速度、急动度限制，用于底盘、手臂关节和末端笛卡尔空间规划。

**主要类**：
- `RuckigParams` - 规划器参数

**属性**：
- `velocity_max` - 最大速度列表
- `acceleration_max` - 最大加速度列表
- `jerk_max` - 最大急动度列表
- `velocity_min` - 最小速度列表（可选，默认为 `-velocity_max`）
- `acceleration_min` - 最小加速度列表（可选，默认为 `-acceleration_max`）

**工厂方法**：
- `create_chassis_params(...)` - 创建底盘规划器参数
- `create_arm_joint_params(...)` - 创建手臂关节规划器参数
- `create_ee_cartesian_params(...)` - 创建末端笛卡尔规划器参数

**使用示例**：

```python
from core.domain.ruckig_params import RuckigParams

# 底盘规划器参数
params = RuckigParams.create_chassis_params(
    vel_xy=0.2,
    vel_yaw=0.6,
    acc_xy=4.0,
    acc_yaw=4.0,
    jerk_xy=20.0,
    jerk_yaw=12.0
)
```

---

#### Tag - 视觉标签

**位置**：`core/domain/tag.py`

**功能**：描述 AprilTag 等视觉标记的信息，包括 ID、位姿和检测时间戳。

**主要类**：
- `Tag` - 视觉标签

**属性**：
- `id` - Tag ID
- `pose` - Tag 位姿（`Pose6D`）
- `timestamp` - 检测时间戳（秒），可选

**方法**：
- `validate()` - 验证 Tag 有效性
- `to_dict()` / `from_dict()` - 序列化与反序列化
- `get_distance_to(pose)` - 计算到另一姿态的欧氏距离
- `is_fresh(current_time, max_age)` - 检查 Tag 是否未过期

**使用示例**：

```python
from core.domain.tag import Tag
from core.domain.pose import Pose6D

tag = Tag(
    id=1,
    pose=Pose6D(x=0.5, y=0.0, z=0.8, roll=0.0, pitch=-0.5, yaw=0.0),
    timestamp=1234567890.123
)
```

---

### 3. Common - 通用工具

#### Transform - 坐标变换

**位置**：`core/common/transform.py`

**功能**：提供坐标系变换工具函数。

**主要函数**：
- `pose6d_to_matrix(pose)` - Pose6D 转 4x4 矩阵
- `matrix_to_pose6d(matrix)` - 4x4 矩阵转 Pose6D
- `transform_pose(pose, source_frame, target_frame)` - 坐标系变换

**使用示例**：

```python
from core.common.transform import pose6d_to_matrix, transform_pose

# 转换位姿到矩阵
matrix = pose6d_to_matrix(pose)

# 坐标系变换
world_pose = transform_pose(local_pose, FrameType.LOCAL, FrameType.WORLD)
```

---

#### Interpolator - 轨迹插值器

**位置**：`core/common/interpolator.py`

**功能**：生成平滑的运动轨迹。

**支持的插值方式**：
- 线性插值
- 多项式插值
- 梯形速度规划

**使用示例**：

```python
from core.common.interpolator import LinearInterpolator

# 创建插值器
interpolator = LinearInterpolator(start_pose, end_pose, duration=2.0)

# 生成轨迹点
for t in np.linspace(0, 2.0, 100):
    pose = interpolator.interpolate(t)
```

---

#### Logger - 统一日志系统

**位置**：`core/common/logger.py`

**功能**：提供统一的日志记录功能。

**特性**：
- ✅ Trace ID 自动生成
- ✅ 相对路径显示
- ✅ 精确到秒的文件命名
- ✅ 分级格式策略

**使用示例**：

```python
from core.common.logger import init_logging, get_logger

# 初始化日志
init_logging()

# 获取日志器
logger = get_logger(__name__)

# 记录日志
logger.info("这是一条信息")
logger.error(f"发生错误: {e}", exc_info=True)
```

**详细文档**：[docs/LOGGING_COMPLETE_GUIDE.md](../../docs/LOGGING_COMPLETE_GUIDE.md)

---

#### ConfigLoader - 配置加载器

**位置**：`core/common/config_loader.py`

**功能**：加载 YAML 配置文件。

**使用示例**：

```python
from core.common.config_loader import ConfigLoader

# 加载配置
config = ConfigLoader.load('config/app_config.yaml')
robot_type = config.get('robot_type')
```

---

#### AppConfig - 应用配置

**位置**：`core/common/app_config.py`

**功能**：加载并提供 `config/app_config.yaml` 的访问接口，供 `HardwareFactory` 等组件读取应用级配置。

**主要函数**：
- `get_app_config_path()` - 返回应用配置文件路径
- `load_app_config(reload=False)` - 加载并缓存配置
- `get_hardware_factory_config()` - 返回 `HardwareFactory.create_hardware()` 所需的配置 dict

**使用示例**：

```python
from core.common.app_config import load_app_config, get_hardware_factory_config

# 获取完整配置
cfg = load_app_config()

# 获取硬件工厂配置（包含 robot_type 等字段）
factory_cfg = get_hardware_factory_config()
```

---

### 4. Services - 核心服务

> **架构调整**：`services` 已从 Core 层迁移到 Adapters 层，现位于 `adapters/hardware/leju_wheeled/services/`。这些实现为了与 ROS 状态话题交互而引入了 `rospy` 等依赖，放在 Adapters 层更符合 Core 层“零外部依赖”的设计原则。

#### SDKManager - SDK 管理器

**位置**：`adapters/hardware/leju_wheeled/services/sdk_manager/`

**功能**：统一封装乐聚官方 SDK (`kuavo_humanoid_sdk`)，为上层提供清晰、易用的接口。

**主要组件**：
- `BaseSDKManager` - SDK 管理器基类，提供初始化、关闭、MPC 模式管理
- `TimedCmdManager` - 封装 `TimedCmdAPI`，提供 7 种简单控制模式
- `ArmSDKManager` - 封装 `ArmAPI`，支持连续轨迹控制
- `LowLevelSDKManager` - 直接调用底层 SDK，用于研究和调试

**使用示例**：

```python
from adapters.hardware.leju_wheeled.services.sdk_manager import TimedCmdManager

manager = TimedCmdManager()
manager.initialize()
result = manager.send_chassis_world(x=0.5, y=0.0, yaw=0.0, desire_time=3.0)
manager.shutdown()
```

**详细文档**：[adapters/hardware/leju_wheeled/services/sdk_manager/README.md](../adapters/hardware/leju_wheeled/services/sdk_manager/README.md)

---

#### StateManager - 状态管理器

**位置**：`adapters/hardware/leju_wheeled/services/state_manager.py`

**功能**：统一管理机器人状态反馈，订阅 ROS 状态话题，维护最新状态的缓存，并提供线程安全的查询接口。

**⚠️ 架构说明**：该模块依赖 `rospy` 和 ROS 消息类型（如 `std_msgs`、`geometry_msgs`、`sensor_msgs`、`ocs2_msgs`），已迁移至 Adapters 层。

**主要能力**：
- 订阅 ROS 状态话题
- 维护状态缓存
- 提供线程安全的状态查询
- 支持状态更新回调通知

---

## 🎯 设计原则

### 1. 接口隔离

应用层只依赖 Core 层定义的接口，不关心具体实现。

### 2. 无外部依赖

Core 层不依赖 ROS、SDK 等外部库，保证可移植性。

### 3. 类型安全

使用 Python 类型注解，提高代码可读性和可维护性。

### 4. 错误处理

统一使用 `Result` 对象返回操作结果，避免异常滥用。

---

## 📊 与其他层的关系

```
┌─────────────────────────────────────┐
│     Application Layer (Apps)        │
│  - 使用 Core 层接口和数据结构       │
└──────────────┬──────────────────────┘
               │ 依赖
               ▼
┌─────────────────────────────────────┐
│     Adapters Layer                  │
│  - 实现 Core 层接口                 │
└──────────────┬──────────────────────┘
               │ 依赖
               ▼
┌─────────────────────────────────────┐
│     Core Layer                      │
│  - 定义接口和数据结构               │
│  - 无外部依赖                       │
└─────────────────────────────────────┘
```

**关键点**：
- Core 层是整个框架的基础
- 所有其他层都依赖 Core 层
- Core 层不依赖任何其他层

---

## 🚀 快速开始

### 1. 定义新的接口

```python
from abc import ABC, abstractmethod
from core.domain.result import Result

class INewInterface(ABC):
    @abstractmethod
    def do_something(self) -> Result:
        pass
```

### 2. 创建新的领域模型

```python
from dataclasses import dataclass

@dataclass
class NewDomainModel:
    field1: str
    field2: int
    
    def validate(self) -> bool:
        return len(self.field1) > 0
```

### 3. 添加工具函数

```python
def new_util_function(param: int) -> float:
    """工具函数说明"""
    return param * 2.0
```

---

## 🧪 测试

Core 层的测试应该：
- ✅ 不依赖 ROS 或外部硬件
- ✅ 使用单元测试框架（pytest）
- ✅ 覆盖所有公共方法

**测试位置**：`tests/core/`

---

## 📝 开发规范

### 1. 类型注解

所有公共方法必须添加类型注解：

```python
def some_method(self, param: int) -> Result:
    pass
```

### 2. 文档字符串

所有公共类和函数必须包含 docstring：

```python
class MyClass:
    """类的简短说明
    
    详细说明...
    """
    pass
```

### 3. 错误处理

使用 `Result` 对象返回结果，而不是抛出异常：

```python
def some_operation() -> Result:
    try:
        # 业务逻辑
        return Result.ok("Success")
    except Exception as e:
        return Result.fail(str(e))
```

---

## 🔗 相关文档

- **Adapters 层**：[adapters/README.md](../../adapters/README.md)
- **Drivers 层**：[drivers/README.md](../../drivers/README.md)
- **日志系统**：[docs/LOGGING_COMPLETE_GUIDE.md](../../docs/LOGGING_COMPLETE_GUIDE.md)

---

## 📈 当前状态

| 模块 | 状态 | 完成度 | 备注 |
|------|------|--------|------|
| Interfaces | ✅ 已完成 | 100% | IHardware, ISkill, ICamera, IPerception |
| Domain | ✅ 已完成 | 95% | 核心数据结构已完善，包含新增底盘/规划器/标签模型 |
| Common | ✅ 已完成 | 95% | 日志系统、坐标变换、配置加载已完成 |
| Services | 🔄 进行中 | 70% | SDKManager 已完善；StateManager 含 ROS 依赖，需后续架构优化 |

---

**最后更新**: 2026-06-17  
**维护者**: Kuavo Studio Team  
**版本**: v1.1

