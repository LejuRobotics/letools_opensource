# Drivers 层 - 硬件驱动层

## 📋 概述

Drivers 层是 LeTools 框架的**硬件驱动层**，负责与具体的硬件设备进行底层通信。

### 核心职责

- ✅ **硬件通信**：实现与机器人硬件的底层通信（ROS 话题、服务、SDK）
- ✅ **协议封装**：将硬件特定的协议封装为统一的接口
- ✅ **状态管理**：维护硬件设备的实时状态
- ✅ **错误处理**：处理硬件通信中的异常和错误

---

## 🏗️ 架构设计

### 分层结构

```
Drivers Layer
├── Leju (乐聚机器人驱动)
│   ├── LejuEndEffector - 末端执行器驱动
│   └── Kuavo Humanoid SDK - 乐聚人形机器人 SDK（Git Submodule）
│
└── Camera (相机驱动，暂未实现)
    └── (待开发)
```

**注意**：`kuavo_humanoid_sdk` 以 Git Submodule 引入，实际路径为 `drivers/leju/kuavo_humanoid_sdk/src/kuavo_humanoid_sdk/`。使用前请确保已执行 `git submodule update --init`。

### 设计原则

1. **单一职责**：每个驱动只负责一种硬件设备
2. **协议透明**：暴露硬件的原始能力，不做过多抽象
3. **错误隔离**：驱动层错误不应影响上层逻辑
4. **可替换性**：同一类硬件可以有多个驱动实现

---

## 📂 目录结构

```
drivers/
├── __init__.py
│
├── leju/                          # 乐聚机器人驱动
│   ├── __init__.py
│   ├── end_effector.py           # 末端执行器驱动
│   │
│   └── kuavo_humanoid_sdk/        # 乐聚人形机器人 SDK（Git Submodule）
│       ├── src/kuavo_humanoid_sdk/  # SDK 实际根目录（submodule）
│       │   ├── README.md         # SDK 文档
│       │   ├── install.sh        # SDK 安装脚本
│       │   ├── setup.py          # Python 包配置
│       │   ├── kuavo_humanoid_sdk/ # SDK 核心代码
│       │   ├── examples/         # SDK 示例代码
│       │   ├── docs/             # SDK 文档
│       │   └── test/             # SDK 测试
│       └── .git                  # submodule 元数据
│
└── camera/                        # 相机驱动（暂未实现）
    └── (待开发)
```

**注意**：`kuavo_humanoid_sdk` 为 Git Submodule，首次使用需执行：

```bash
git submodule update --init --recursive
```

Submodule 中的具体文件结构以 SDK 仓库为准，本 README 不再逐一枚举内部模块。

---

## 🔧 核心组件详解

### 1. LejuEndEffector - 末端执行器驱动

**位置**：`drivers/leju/end_effector.py`

**功能**：驱动乐聚机器人的末端执行器（夹爪或灵巧手）。

#### 支持的末端执行器类型

| 类型 | 说明 | 通信方式 |
|------|------|---------|
| **LEJU_CLAW** | 乐聚二指夹爪 | ROS Service (`/control_robot_leju_claw`) |
| **QIANGNAO_HAND** | 强脑灵巧手 | ROS Topic (`/control_robot_hand_position`) |

#### 主要方法

| 方法 | 说明 | 参数 |
|------|------|------|
| `connect()` | 连接末端执行器 | - |
| `disconnect()` | 断开连接 | - |
| `control_gripper(cmd)` | 控制夹爪 | `GripperCommand` |
| `control_hand(cmd)` | 控制灵巧手 | `HandFingerCommand` |
| `get_state()` | 获取当前状态 | - |

#### 使用示例

```python
from drivers.leju.end_effector import LejuEndEffector
from core.domain.end_effector import GripperCommand, EndEffectorType

# 配置
config = {
    'type': 'leju_claw'  # 或 'qiangnao_hand'
}

# 创建驱动实例
end_effector = LejuEndEffector(config)

# 连接
if not end_effector.connect():
    print("连接失败")
    exit(1)

# 控制夹爪（50% 开合度，10N 力度）
cmd = GripperCommand(open_ratio=0.5, force=10.0)
result = end_effector.control_gripper(cmd)

if result.success:
    print("夹爪控制成功")
else:
    print(f"夹爪控制失败: {result.message}")

# 断开连接
end_effector.disconnect()
```

#### 技术特点

- ✅ **自动初始化 ROS**：如果 ROS 未初始化，自动调用 `rospy.init_node()`
- ✅ **超时保护**：服务调用带有超时机制（默认 5 秒）
- ✅ **状态缓存**：内部维护末端执行器的最新状态
- ✅ **日志记录**：集成统一日志系统

---

### 2. Kuavo Humanoid SDK - 乐聚人形机器人 SDK

**位置**：`drivers/leju/kuavo_humanoid_sdk/`

**说明**：这是乐聚官方提供的 Python SDK，通过 Git Submodule 引入。

#### SDK 功能模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **机器人控制** | `robot.py` | 机器人主控制类 |
| **手臂控制** | `robot_arm.py` | 手臂关节和末端控制 |
| **轮式底盘** | `robot_wheel_control.py` | 轮式底盘运动控制 |
| **灵巧手** | `dexterous_hand.py` | 多指灵巧手控制 |
| **夹爪** | `leju_claw.py` | 二指夹爪控制 |
| **视觉感知** | `robot_vision.py` | 相机、二维码识别 |
| **音频交互** | `robot_audio.py` | 语音识别和合成 |
| **导航** | `robot_navigation.py` | 自主导航 |
| **观测数据** | `robot_observation.py` | 机器人状态观测 |

#### 安装 SDK

```bash
cd drivers/leju/kuavo_humanoid_sdk
./install.sh
```

#### 使用示例

```python
from kuavo_humanoid_sdk import Robot

# 创建机器人实例
robot = Robot()

# 初始化
robot.initialize()

# 控制手臂
robot.arm.move_to_pose(x=0.5, y=0.0, z=0.3)

# 控制夹爪
robot.claw.open()

# 关闭
robot.shutdown()
```

#### 详细文档

- **SDK README**：[drivers/leju/kuavo_humanoid_sdk/README.md](leju/kuavo_humanoid_sdk/README.md)
- **安装指南**：[drivers/leju/kuavo_humanoid_sdk/docs/installation.md](leju/kuavo_humanoid_sdk/docs/installation.md)
- **快速开始**：[drivers/leju/kuavo_humanoid_sdk/docs/quickstart.md](leju/kuavo_humanoid_sdk/docs/quickstart.md)
- **API 参考**：[drivers/leju/kuavo_humanoid_sdk/docs/api_reference.md](leju/kuavo_humanoid_sdk/docs/api_reference.md)

---

## 🎯 设计原则

### 1. 最小抽象

驱动层尽量保持硬件的原始能力，不做过多的业务逻辑抽象。

### 2. 错误隔离

驱动层的错误应该被捕获并转换为 `Result` 对象，避免影响上层逻辑。

### 3. 资源管理

驱动层负责管理硬件资源的生命周期（连接、断开、重连）。

### 4. 可测试性

驱动层应该支持 Mock，便于单元测试。

---

## 📊 与其他层的关系

```
┌─────────────────────────────────────┐
│     Adapters Layer                  │
│  - 调用 Drivers 层                  │
│  - 实现 Core 层接口                 │
└──────────────┬──────────────────────┘
               │ 依赖
               ▼
┌─────────────────────────────────────┐
│     Drivers Layer                   │
│  - 直接操作硬件                     │
│  - ROS 话题/服务/SDK                │
└──────────────┬──────────────────────┘
               │ 依赖
               ▼
┌─────────────────────────────────────┐
│     Hardware / ROS Infrastructure   │
│  - 物理硬件                         │
│  - ROS Master                       │
└─────────────────────────────────────┘
```

**关键点**：
- Drivers 层直接与硬件通信
- Adapters 层调用 Drivers 层
- Core 层不依赖 Drivers 层

---

## 🚀 快速开始

### 1. 创建新的驱动

```python
from core.domain.result import Result
from core.common.logger import get_logger

logger = get_logger(__name__)

class NewHardwareDriver:
    """新硬件驱动"""
    
    def __init__(self, config: dict):
        self.config = config
        self._connected = False
    
    def connect(self) -> bool:
        """连接硬件"""
        try:
            # 连接逻辑
            self._connected = True
            logger.info("Connected to hardware")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self._connected = False
        logger.info("Disconnected from hardware")
    
    def do_something(self, param: int) -> Result:
        """执行操作"""
        if not self._connected:
            return Result.fail("Not connected")
        
        try:
            # 业务逻辑
            return Result.ok("Success")
        except Exception as e:
            return Result.fail(str(e))
```

### 2. 在适配器中使用

```python
from drivers.leju.new_driver import NewHardwareDriver

class MyHardwareAdapter(IHardware):
    def __init__(self, config: dict):
        self.driver = NewHardwareDriver(config)
    
    def initialize(self) -> Result:
        if not self.driver.connect():
            return Result.fail("Failed to connect")
        return Result.ok("Initialized")
    
    def shutdown(self) -> Result:
        self.driver.disconnect()
        return Result.ok("Shutdown")
```

---

## 🧪 测试

### 单元测试

驱动层的测试应该：
- ✅ 使用 Mock 模拟硬件响应
- ✅ 测试连接和断开逻辑
- ✅ 测试错误处理

**测试位置**：`tests/drivers/`

### 集成测试

需要真实硬件或仿真环境：
- ✅ 测试实际硬件通信
- ✅ 验证协议正确性
- ✅ 性能测试

---

## 📝 开发规范

### 1. 日志记录

所有驱动必须使用统一日志系统：

```python
from core.common.logger import get_logger
logger = get_logger(__name__)

logger.info("Connecting to hardware...")
logger.error(f"Connection failed: {e}", exc_info=True)
```

### 2. 错误处理

所有公共方法必须返回 `Result` 对象或布尔值：

```python
def some_operation(self) -> Result:
    if not self._connected:
        return Result.fail("Not connected")
    
    try:
        # 业务逻辑
        return Result.ok("Success")
    except Exception as e:
        logger.error(f"Operation failed: {e}", exc_info=True)
        return Result.fail(str(e))
```

### 3. 资源管理

确保在 `disconnect()` 中释放所有资源：

```python
def disconnect(self):
    if self._publisher:
        self._publisher.unregister()
    if self._subscriber:
        self._subscriber.unregister()
    self._connected = False
```

### 4. 线程安全

如果驱动涉及多线程，必须使用锁保护共享状态：

```python
import threading

class ThreadSafeDriver:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = None
    
    def update_state(self, new_state):
        with self._lock:
            self._state = new_state
```

---

## 🔗 相关文档

- **Core 层**：[core/README.md](../../core/README.md)
- **Adapters 层**：[adapters/README.md](../../adapters/README.md)
- **SDK 文档**：[drivers/leju/kuavo_humanoid_sdk/README.md](leju/kuavo_humanoid_sdk/README.md)
- **项目迁移规划**：[docs/PROJECT_MIGRATION_MASTER_PLAN.md](../../docs/PROJECT_MIGRATION_MASTER_PLAN.md)

---

## 📈 当前状态

| 驱动 | 状态 | 完成度 | 备注 |
|------|------|--------|------|
| LejuEndEffector | ✅ 已完成 | 90% | 夹爪和灵巧手已支持 |
| Kuavo Humanoid SDK | ✅ 已完成 | 100% | 官方 SDK，以 Git Submodule 引入，需 `git submodule update --init` |
| Camera Driver | 📋 待开发 | 0% | 当前仓库不存在 `drivers/camera/` 目录，需后续实现 |

---

## 💡 常见问题

### Q1: 如何调试驱动层代码？

**A**: 
1. 查看日志文件：`log/kuavo_studio_*.log`
2. 使用 `rostopic echo` 查看 ROS 话题
3. 使用 `rosservice call` 测试服务

### Q2: 驱动连接失败怎么办？

**A**:
1. 检查 roscore 是否运行
2. 检查硬件是否正常启动
3. 查看日志中的错误信息
4. 确认 ROS 环境变量已正确设置

### Q3: 如何添加新的硬件驱动？

**A**:
1. 在 `drivers/` 下创建新目录
2. 实现驱动类，遵循开发规范
3. 编写单元测试
4. 在 Adapters 层中集成

---

**最后更新**: 2026-06-17  
**维护者**: Kuavo Studio Team  
**版本**: v1.1
