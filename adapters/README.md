# Adapters 层 - 硬件适配器

## 📋 概述

Adapters 层是 LeTools 框架中的**硬件抽象层**，负责将 Core 层的标准接口转换为具体机器人平台的底层实现。

### 核心职责

- ✅ **硬件抽象**：提供统一的 `IHardware` 接口，屏蔽不同机器人平台的差异
- ✅ **协议转换**：将高层指令转换为 ROS 话题、服务或 SDK 调用
- ✅ **资源管理**：管理硬件连接、订阅者、发布者的生命周期
- ✅ **状态同步**：维护机器人实时状态（位姿、关节角等）

---

## 新手理解：Adapter、Hardware 和 Mixin 的关系

可以先把这一层理解成三层关系：

```text
Adapter（适配器层）
└── Hardware（具体机器人硬件对象）
    └── Mixin（按功能拆开的能力模块）
```

### 1. Adapter 是什么

`Adapter` 是“翻译官/中间层”。上层代码，比如 `apps` 测试脚本、`skills` 原子技能、`orchestration` 行为树，不应该直接关心 ROS 话题名、服务名、SDK 函数名这些底层细节。

Adapter 的作用就是把上层统一的调用：

```python
hardware.send_base_velocity(...)
hardware.send_arm_joint_trajectory(...)
hardware.get_robot_state(...)
```

翻译成底层真正能执行的 ROS、SDK 或服务调用。

### 2. Hardware 是什么

`Hardware` 是“某一类机器人在代码里的代表”。例如：

```text
LejuWheeledArmHardware = 乐聚轮臂机器人在 LeTools 里的硬件对象
LejuBipedalHardware    = 乐聚足式机器人在 LeTools 里的硬件对象
```

上层拿到 `hardware` 对象以后，就通过它控制机器人，而不是每次都自己写 ROS 发布器或 SDK 初始化逻辑。

### 3. Mixin 是什么

`Mixin` 是把 `Hardware` 的能力拆成一块一块的小模块。如果所有底盘、手臂、躯干、末端、力控、状态反馈、SDK 控制都写进一个 `hardware.py`，文件会很大，也很难维护。

所以 `LejuWheeledArmHardware` 通过多个 Mixin 组合能力：

```text
LejuWheeledArmHardware
├── lifecycle_mixin         # 生命周期管理
├── base_control_mixin      # 底盘控制
├── arm_control_mixin       # 手臂控制
├── torso_control_mixin     # 躯干控制
├── end_effector_mixin      # 末端执行器
├── force_control_mixin     # 力控
├── mode_service_mixin      # MPC、快速模式等服务
├── sdk_control_mixin       # SDK 直调
├── state_feedback_mixin    # 状态反馈
├── timed_command_mixin     # 时序指令
└── jibot/chassis_mixin     # JiBot 底盘协议
```

一句话总结：

```text
Adapter 是这一层的职责名称，负责隔离上层业务和底层机器人。
Hardware 是 Adapter 层里真正被创建和调用的机器人对象。
Mixin 是组成 Hardware 的一块块功能模块。
```

### 4. 为什么要这样设计

这样做的好处是：

```text
上层代码只关心“我要做什么”
↓
hardware 提供统一方法
↓
Mixin 负责具体功能实现
↓
底层 ROS/SDK/服务真正执行
```

比如控制底盘时，上层只调用：

```python
hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)
```

至于底层到底是发 `/cmd_vel`，还是走 SDK，还是调用某个服务，由 Adapter/Hardware/Mixin 负责处理。

---
## 🏗️ 架构设计

### 分层结构

```
Adapters Layer
├── Hardware Factory (工厂模式)
│   └── 根据配置动态创建适配器实例
│
├── LejuWheeledArmHardware (乐聚轮臂)
│   ├── 由多个 Mixin 组合而成
│   │   ├── lifecycle_mixin - 生命周期管理
│   │   ├── base_control_mixin - 底盘控制
│   │   ├── arm_control_mixin - 手臂控制
│   │   ├── torso_control_mixin - 躯干控制
│   │   ├── end_effector_mixin - 末端执行器
│   │   ├── force_control_mixin - 力控
│   │   ├── mode_service_mixin - 模式服务
│   │   ├── sdk_control_mixin - SDK 控制
│   │   ├── state_feedback_mixin - 状态反馈
│   │   ├── timed_command_mixin - 时序指令
│   │   └── jibot/chassis_mixin - 底盘移动（JiBot 协议）
│   ├── 相机适配器 (camera_adapter)
│   ├── 感知适配器 (perception_adapter)
│   └── 内部服务 (services/)
│       ├── state_manager - 状态管理器
│       └── sdk_manager/ - SDK 管理器（base/arm/low_level/timed_cmd）
│
└── LejuBipedalHardware (乐聚足式)
    └── (待完善)
```

### 设计模式

1. **工厂模式** (`HardwareFactory`)
   - 根据 `robot_type` 配置自动选择适配器
   - 支持运行时动态切换

2. **适配器模式** (`IHardware` 接口)
   - 统一的应用层接口
   - 不同的底层实现

3. **组合模式**
   - 硬件适配器内部组合多个子适配器（相机、感知等）

---

## 📂 目录结构

```
adapters/
├── __init__.py                    # 导出 HardwareFactory
├── hardware/
│   ├── __init__.py
│   ├── factory.py                 # 硬件工厂类
│   │
│   ├── leju_wheeled/              # 乐聚轮臂适配器
│   │   ├── __init__.py
│   │   ├── hardware.py            # 主适配器（LejuWheeledArmHardware）
│   │   ├── camera_adapter.py      # 相机适配器
│   │   ├── perception_adapter.py  # 感知适配器
│   │   ├── README.md              # 轮臂适配器详细文档
│   │   └── mixins/                # Mixin 组合
│   │       ├── lifecycle_mixin.py
│   │       ├── base_control_mixin.py
│   │       ├── arm_control_mixin.py
│   │       ├── torso_control_mixin.py
│   │       ├── end_effector_mixin.py
│   │       ├── force_control_mixin.py
│   │       ├── mode_service_mixin.py
│   │       ├── sdk_control_mixin.py
│   │       ├── state_feedback_mixin.py
│   │       ├── timed_command_mixin.py
│   │       ├── _logging_setup.py   # 日志初始化辅助（非 Mixin）
│   │       └── jibot/
│   │           └── chassis_mixin.py
│   │
│   │   └── services/              # 内部服务
│   │       ├── state_manager.py   # 状态管理器
│   │       └── sdk_manager/       # SDK 管理器（含测试）
│   │
│   └── leju_bipedal/              # 乐聚足式适配器
│       ├── __init__.py
│       └── hardware.py            # 主适配器（LejuBipedalHardware）
│
└── (未来扩展)
    ├── mock/                      # Mock 适配器（用于测试，暂未实现）
    └── universal_robots/          # UR 机械臂适配器（规划中）
```

---

## 🔧 核心组件

### 1. HardwareFactory - 硬件工厂

**位置**：`adapters/hardware/factory.py`

**功能**：根据配置文件中的 `robot_type` 创建对应的硬件适配器实例。

**支持的机器人类型**：
- `leju_wheeled` - 乐聚轮臂机器人
- `leju_bipedal` - 乐聚足式机器人
- `mock` - Mock 适配器（用于单元测试，`factory.py` 有引用但目录尚未创建）

**使用示例**：
```python
from adapters import HardwareFactory

config = {
    'robot_type': 'leju_wheeled',
    # ... 其他配置
}

hardware = HardwareFactory.create_hardware(config)
hardware.initialize()
```

---

### 2. LejuWheeledArmHardware - 乐聚轮臂适配器

**位置**：`adapters/hardware/leju_wheeled/hardware.py`

**功能**：实现 `IHardware` 接口，提供乐聚轮臂机器人的完整控制能力。

#### 主要功能模块

| 模块 | 方法 | 说明 |
|------|------|------|
| **初始化** | `initialize()` | 连接 ROS、初始化末端执行器、启动相机 |
| **关闭** | `shutdown()` | 断开连接、释放资源 |
| **底盘控制** | `send_base_velocity()` | 发送底盘速度指令 |
| | `send_base_pose()` | 发送底盘位置指令 |
| **手臂控制** | `send_ee_pose()` | 发送末端位姿指令 |
| | `set_mpc_mode()` | 设置 MPC 控制模式 |
| **躯干控制** | `send_torso_pose()` | 发送躯干位姿指令 |
| **末端执行器** | `control_end_effector()` | 控制夹爪/灵巧手 |
| **头部控制** | `control_head()` | 发送头部关节指令 |
| **力控** | `set_ee_force()` | 设置末端力控参数 |
| | `set_external_wrench()` | 设置外部力矩 |
| **时序指令** | `send_timed_base_pose()` | 定时底盘位姿 |
| | `send_timed_left_arm_joint()` | 定时左臂关节 |
| **相机** | `camera.capture_image()` | 捕获图像 |
| **感知** | `perception.detect_qrcode()` | 二维码识别 |

#### 技术特点

- ✅ **ROS 集成**：直接使用 `rospy.Publisher` 和 `rospy.ServiceProxy`
- ✅ **日志系统**：集成统一日志系统，支持 Trace ID 追踪
- ✅ **线程安全**：使用 `threading.Lock` 保护共享状态
- ✅ **坐标系转换**：支持本体坐标系和世界坐标系的自动转换
- ✅ **错误处理**：所有方法返回 `Result` 对象，统一错误处理

#### 使用示例

```python
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from core.domain.pose import Pose6D
from core.domain.enums import FrameType, ArmSide

# 初始化
hardware = LejuWheeledArmHardware()
result = hardware.initialize()
if not result.success:
    print(f"初始化失败: {result.message}")
    exit(1)

# 底盘速度控制
hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)

# 手臂末端位姿控制
pose = Pose6D(x=0.5, y=0.0, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)
hardware.send_ee_pose(side=ArmSide.LEFT, pose=pose, frame=FrameType.WORLD)

# 关闭
hardware.shutdown()
```


---

### 3. Mixin 组合架构

**位置**：`adapters/hardware/leju_wheeled/mixins/`

`LejuWheeledArmHardware` 通过 Mixin 组合实现不同控制域的职责分离，每个 Mixin 只负责单一硬件能力。

| Mixin 文件 | 控制域 |
|------------|--------|
| `lifecycle_mixin.py` | 生命周期管理 |
| `base_control_mixin.py` | 底盘控制 |
| `arm_control_mixin.py` | 手臂控制 |
| `torso_control_mixin.py` | 躯干控制 |
| `end_effector_mixin.py` | 末端执行器 |
| `force_control_mixin.py` | 力控 |
| `mode_service_mixin.py` | 模式服务 |
| `sdk_control_mixin.py` | SDK 控制 |
| `state_feedback_mixin.py` | 状态反馈 |
| `timed_command_mixin.py` | 时序指令 |
| `jibot/chassis_mixin.py` | 底盘移动（JiBot 协议） |
| `_logging_setup.py` | 日志初始化辅助（非 Mixin） |

**设计优势**：
- 单一职责：每个 Mixin 只处理一个控制域
- 可组合性：通过多重继承组合成完整适配器
- 可测试性：可以单独测试每个 Mixin 的行为
- 可扩展性：新增硬件能力只需添加新的 Mixin

---

### 4. CameraAdapter - 相机适配器

**位置**：`adapters/hardware/leju_wheeled/camera_adapter.py`

**功能**：封装相机操作，提供统一的图像捕获接口。

**主要方法**：
- `capture_image()` - 捕获单帧图像
- `start_streaming()` - 启动视频流
- `stop_streaming()` - 停止视频流

---

### 5. PerceptionAdapter - 感知适配器

**位置**：`adapters/hardware/leju_wheeled/perception_adapter.py`

**功能**：封装感知算法，提供高级感知功能。

**主要方法**：
- `detect_qrcode()` - 二维码识别
- `detect_aruco()` - ArUco 标记识别
- `estimate_pose()` - 位姿估计

---

## 🎯 设计原则

### 1. 单一职责

每个适配器只负责一种机器人平台的硬件交互，不包含业务逻辑。

### 2. 接口隔离

应用层只依赖 `IHardware` 接口，不关心具体实现细节。

### 3. 依赖倒置

- 高层模块（应用层）定义接口（`IHardware`）
- 低层模块（适配器）实现接口

### 4. 开闭原则

新增机器人平台时，只需添加新的适配器类，无需修改现有代码。

---

## 📊 与 Core 层的关系

```
┌─────────────────────────────────────┐
│     Application Layer (Apps)        │
│  - test_kuavo_5w_adapter/           │
│  - test_kuavo_5w_sdk_adapter/       │
└──────────────┬──────────────────────┘
               │ 使用 IHardware 接口
               ▼
┌─────────────────────────────────────┐
│     Adapters Layer                  │
│  - LejuWheeledArmHardware           │
│  - LejuBipedalHardware              │
└──────────────┬──────────────────────┘
               │ 调用 ROS/SDK
               ▼
┌─────────────────────────────────────┐
│     Drivers / ROS Infrastructure    │
│  - rospy.Publisher                  │
│  - rospy.ServiceProxy               │
│  - kuavo_msgs                       │
└─────────────────────────────────────┘
```

**关键点**：
- Core 层定义接口（`core/interfaces/i_hardware.py`）
- Adapters 层实现接口
- Apps 层使用接口

---

## 🚀 快速开始

### 1. 创建自定义适配器

```python
from core.interfaces.i_hardware import IHardware
from core.domain.result import Result

class MyRobotHardware(IHardware):
    def initialize(self) -> Result:
        # 初始化逻辑
        return Result.ok("Initialized")
    
    def shutdown(self) -> Result:
        # 关闭逻辑
        return Result.ok("Shutdown")
    
    def send_base_velocity(self, vx, vy, vyaw, frame=FrameType.LOCAL):
        # 实现底盘控制
        pass
    
    # ... 实现其他 IHardware 方法
```

### 2. 注册到工厂

编辑 `adapters/hardware/factory.py`：

```python
elif robot_type == 'my_robot':
    from adapters.hardware.my_robot.hardware import MyRobotHardware
    return MyRobotHardware(config=config)
```

### 3. 在配置中使用

```yaml
# config/app_config.yaml
robot_type: my_robot
```

---

## 🧪 测试

### 测试目录

适配器层的测试主要在 `apps/` 下的以下目录：

- `test_kuavo_5w_adapter/` — 适配器层硬件测试（按控制域分子目录：底盘、手臂、躯干等）
- `test_kuavo_5w_sdk_adapter/` — SDK 适配器层测试
- `test_kuavo_5w_internal/` — 底层 ROS 接口测试
- `test_kuavo_5w_sdk_internal/` — SDK 内部接口测试

### 运行测试

```bash
# 适配器层测试
cd apps/test_kuavo_5w_adapter
python3 01_base_control/test_cmd_pose_base.py

# SDK 适配器测试
cd apps/test_kuavo_5w_sdk_adapter/sdk/01_head
python3 test_head_control.py
```

---

## 📝 开发规范

### 1. 日志记录

所有适配器必须使用统一日志系统：

```python
from core.common.logger import get_logger
logger = get_logger(__name__)

logger.info("初始化适配器...")
logger.error(f"连接失败: {e}", exc_info=True)
```

### 2. 错误处理

所有公共方法必须返回 `Result` 对象：

```python
def some_method(self) -> Result:
    try:
        # 业务逻辑
        return Result.ok("Success")
    except Exception as e:
        logger.error(f"操作失败: {e}", exc_info=True)
        return Result.fail(str(e))
```

### 3. 资源管理

确保在 `shutdown()` 中释放所有资源：

```python
def shutdown(self) -> Result:
    if self.camera:
        self.camera.shutdown()
    if self._sub:
        self._sub.unregister()
    return Result.ok("Shutdown complete")
```

---

## 🔗 相关文档

- **Core 层接口**：[core/interfaces/i_hardware.py](../../core/interfaces/i_hardware.py)
- **适配器层测试**：[apps/test_kuavo_5w_adapter/](../../apps/test_kuavo_5w_adapter/)
- **SDK 适配器测试**：[apps/test_kuavo_5w_sdk_adapter/](../../apps/test_kuavo_5w_sdk_adapter/)
- **底层接口测试**：[apps/test_kuavo_5w_internal/](../../apps/test_kuavo_5w_internal/)

---

## 📈 当前状态

| 适配器 | 状态 | 完成度 | 备注 |
|--------|------|--------|------|
| LejuWheeledArmHardware | ✅ 已完成 | 90% | 基于 11 个 Mixin 组合，底盘、手臂、躯干、末端、力控、时序指令已完成 |
| LejuBipedalHardware | ⚠️ 进行中 | 30% | 基础框架已搭建 |
| MockHardware | ❌ 未实现 | 0% | `factory.py` 有引用但 `adapters/hardware/mock/` 目录不存在 |

---

**最后更新**: 2026-06-30  
**维护者**: Kuavo Studio Team  
**版本**: v1.2


