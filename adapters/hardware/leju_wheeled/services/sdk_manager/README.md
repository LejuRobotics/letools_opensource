# SDK 管理器（Core 层）

**版本**: v1.0  
**状态**: ✅ 已完成  
**最后更新**: 2026-05-18

---

## 📋 概述

`adapters/hardware/leju_wheeled/services/sdk_manager/` 是 Kuavo Studio 的 **SDK 管理器**，统一封装乐聚官方 SDK (`kuavo_humanoid_sdk`)，为上层提供清晰、易用的接口。

### 设计目标

1. **统一管理** - 集中处理 SDK 初始化、MPC 模式管理、错误处理
2. **双模式支持** - 同时提供自动管理和手动管理 MPC 模式
3. **单位转换** - 自动处理角度单位转换（度 ↔ 弧度）
4. **解耦依赖** - Adapter 层不直接依赖 SDK，便于测试和维护

---

## 🏗️ 架构设计

### 组件结构

```
sdk_manager/
├── base_sdk_manager.py        # 基类：通用功能
├── timed_cmd_manager.py       # TimedCmdAPI 管理器
├── arm_sdk_manager.py         # ArmAPI 管理器
└── low_level_sdk_manager.py   # 底层 SDK 管理器
```

### 继承关系

```
BaseSDKManager (基类)
    ├── TimedCmdManager        # 时序指令控制
    ├── ArmSDKManager          # 手臂高级控制
    └── LowLevelSDKManager     # 底层直接控制
```

---

## 🚀 快速开始

### 1. 安装依赖

确保已安装 `kuavo_humanoid_sdk`:

```bash
cd ~/LeTools
./scripts/install_sdk.sh
```

### 2. 导入管理器

```python
from adapters.hardware.leju_wheeled.services.sdk_manager import (
    TimedCmdManager,
    ArmSDKManager,
    LowLevelSDKManager
)
```

### 3. 初始化和使用

```python
# 创建管理器实例
manager = TimedCmdManager()

# 初始化 SDK
result = manager.initialize()
if not result.success:
    print(f"初始化失败: {result.message}")
    exit(1)

# 使用 API
result = manager.send_chassis_world(x=0.5, y=0.0, yaw=0.0, desire_time=3.0)
if result.success:
    print(f"执行成功，实际时间: {result.data['actual_time']}s")
else:
    print(f"执行失败: {result.message}")

# 关闭 SDK
manager.shutdown()
```

---

## 📚 管理器说明

### 1. BaseSDKManager（基类）

**职责**: 提供通用的 SDK 管理功能

#### 核心方法

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `initialize()` | 初始化 SDK | `Result` |
| `shutdown()` | 关闭 SDK | `Result` |
| `is_initialized()` | 检查是否已初始化 | `bool` |
| `setup_mpc_mode()` | 配置并应用 MPC 模式 | `Result` |

#### MPC 模式管理（手动）

```python
# Step 1: 切换到外部控制模式
manager.set_external_control_mode()

# Step 2: 设置 MPC 模式和流
manager.set_mpc_mode(
    mode=KuavoManipulationMpcCtrlMode.ArmOnly,
    flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
)

# ... 执行控制 ...

# Step 3: 恢复默认模式
manager.restore_mpc_mode()
```

#### MPC 模式管理（自动）

```python
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoManipulationMpcCtrlMode,
    KuavoManipulationMpcControlFlow
)

# 使用模式管理器自动管理
with manager.mpc_mode_context(
    mode=KuavoManipulationMpcCtrlMode.ArmOnly,
    flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
):
    # 在这里执行控制
    result = manager.some_control_method(...)
# 退出时自动恢复模式
```

---

### 2. TimedCmdManager

**职责**: 封装 `TimedCmdAPI`，提供 7 种控制模式

**特点**:
- ✅ 无需手动管理 MPC 模式
- ✅ 自动处理单位转换（度 → 弧度）
- ✅ 简单易用，适合初学者

#### 控制模式

| 方法 | 说明 | 参数 | 返回值 |
|------|------|------|--------|
| `send_chassis_world()` | 底盘世界系控制 | x, y, yaw, desire_time | `Result` |
| `send_chassis_local()` | 底盘局部系控制 | x, y, yaw, desire_time | `Result` |
| `send_torso_pose()` | 躯干位姿控制 | x, z, yaw, pitch, desire_time | `Result` |
| `send_leg_joint()` | 下肢关节控制 | joint_angles_deg[4], desire_time | `Result` |
| `send_arm_joint()` | 手臂关节控制 | joint_angles_deg[14], desire_time | `Result` |
| `send_arm_ee_world()` | 手臂末端世界系 | left_pose[7], right_pose[7], desire_time | `Result` |
| `send_arm_ee_local()` | 手臂末端局部系 | left_pose[7], right_pose[7], desire_time | `Result` |
| `send_left_arm_ee_world()` | 左臂末端世界系 | pose[7], desire_time | `Result` |
| `send_right_arm_ee_world()` | 右臂末端世界系 | pose[7], desire_time | `Result` |
| `send_left_arm_ee_local()` | 左臂末端局部系 | pose[7], desire_time | `Result` |
| `send_right_arm_ee_local()` | 右臂末端局部系 | pose[7], desire_time | `Result` |
| `send_left_arm_joint()` | 左臂关节控制 | joint_angles_deg[7], desire_time | `Result` |
| `send_right_arm_joint()` | 右臂关节控制 | joint_angles_deg[7], desire_time | `Result` |
| `send_arm_force()` | 手臂力控 | force[6], desire_time | `Result` |
| `send_multi_commands()` | 批量发送多种指令 | cmds（命令列表） | `Result` |
| `set_ruckig_planner_params()` | 设置 Ruckig 规划器参数 | params（`RuckigParams`） | `Result` |
| `set_offline_trajectory()` | 设置离线轨迹 | trajectory | `Result` |
| `enable_offline_trajectory()` | 启用离线轨迹 | enable（bool） | `Result` |
| `check_ik_accessibility()` | 检查 IK 可达性 | pose, frame | `Result` |
| `set_offline_trajectory()` | 设置离线轨迹 | trajectory | `Result` |
| `enable_offline_trajectory()` | 启用离线轨迹 | enable（bool） | `Result` |
| `check_ik_accessibility()` | 检查 IK 可达性 | pose, frame | `Result` |

#### 使用示例

```python
from adapters.hardware.leju_wheeled.services.sdk_manager import TimedCmdManager

# 创建并初始化
manager = TimedCmdManager()
manager.initialize()

# 示例 1: 底盘世界系控制
result = manager.send_chassis_world(
    x=0.5,      # 前进 0.5 米
    y=0.0,
    yaw=0.0,
    desire_time=3.0
)
print(f"底盘移动完成，实际时间: {result.data['actual_time']}s")

# 示例 2: 手臂关节控制（自动转换单位）
joint_angles_deg = [0.0] * 14  # 14 个关节角度（度）
result = manager.send_arm_joint(
    joint_angles_deg=joint_angles_deg,
    desire_time=2.0
)

# 示例 3: 手臂末端世界系控制
left_pose = [0.5, 0.3, 0.3, 0.0, 0.707, 0.0, 0.707]   # [x,y,z,qx,qy,qz,qw]
right_pose = [0.5, -0.3, 0.3, 0.0, 0.707, 0.0, 0.707]
result = manager.send_arm_ee_world(
    left_pose=left_pose,
    right_pose=right_pose,
    desire_time=3.0
)

# 关闭
manager.shutdown()
```

---

### 3. ArmSDKManager

**职责**: 封装 `ArmAPI`，支持连续轨迹控制

**特点**:
- ✅ 支持自动和手动两种 MPC 管理模式
- ✅ 支持多关键点轨迹插值
- ✅ 适用于复杂的手臂运动

#### 双模式设计

##### 方式 1: 自动管理（推荐）

```python
from adapters.hardware.leju_wheeled.services.sdk_manager import ArmSDKManager

manager = ArmSDKManager()
manager.initialize()

# 定义轨迹（多个关键点）
left_traj = [
    [0.5, 0.3, 0.3, 0.0, 0.707, 0.0, 0.707],  # 起点
    [0.6, 0.3, 0.4, 0.0, 0.707, 0.0, 0.707],  # 中间点
    [0.7, 0.3, 0.5, 0.0, 0.707, 0.0, 0.707],  # 终点
]
right_traj = [
    [0.5, -0.3, 0.3, 0.0, 0.707, 0.0, 0.707],
    [0.6, -0.3, 0.4, 0.0, 0.707, 0.0, 0.707],
    [0.7, -0.3, 0.5, 0.0, 0.707, 0.0, 0.707],
]

# 自动管理 MPC 模式（设置 → 执行 → 恢复）
result = manager.move_eef_traj_auto(
    left_traj=left_traj,
    right_traj=right_traj,
    total_time=5.0,
    frame='world',          # 'world' 或 'base_link'
    back_default=False      # 是否恢复到默认模式
)

if result.success:
    print("轨迹执行成功")
else:
    print(f"轨迹执行失败: {result.message}")

manager.shutdown()
```

##### 方式 2: 手动管理（高级）

```python
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoManipulationMpcCtrlMode,
    KuavoManipulationMpcControlFlow
)

manager = ArmSDKManager()
manager.initialize()

try:
    # Step 1: 手动设置 MPC 模式
    manager.set_external_control_mode()
    manager.set_mpc_mode(
        mode=KuavoManipulationMpcCtrlMode.ArmOnly,
        flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
    )
    time.sleep(0.5)  # 等待模式切换生效
    
    # Step 2: 执行轨迹（不自动恢复模式）
    result = manager.move_eef_traj_manual(
        left_traj=left_traj,
        right_traj=right_traj,
        total_time=5.0,
        frame='world'
    )
    
    # Step 3: 可以在这里执行其他操作...
    
finally:
    # Step 4: 手动恢复模式
    manager.restore_mpc_mode()
    time.sleep(0.5)

manager.shutdown()
```

#### 主要方法

| 方法 | 模式 | 说明 |
|------|------|------|
| `move_eef_traj_auto()` | 自动 | 末端轨迹控制（自动管理 MPC） |
| `move_eef_traj_manual()` | 手动 | 末端轨迹控制（手动管理 MPC） |
| `move_joint_traj_auto()` | 自动 | 关节轨迹控制（自动管理 MPC） |
| `move_joint_traj_manual()` | 手动 | 关节轨迹控制（手动管理 MPC） |
| `arm_reset()` | 自动 | 手臂归位（自动管理 MPC） |

---

### 4. LowLevelSDKManager

**职责**: 直接调用 `robot_sdk.control.*`，用于研究和调试

**特点**:
- ⚠️ 需要手动管理 MPC 模式
- 🔬 适用于底层研究和问题诊断
- ❌ 不推荐在生产环境使用

#### 主要方法

| 方法 | 说明 | MPC 模式 |
|------|------|---------|
| `control_arm_joint_positions()` | 手臂关节控制 | ✅ 需要 |
| `control_robot_end_effector_pose()` | 手臂末端控制 | ✅ 需要 |
| `control_head()` | 头部控制 | ❌ 不需要 |
| `control_base_position()` | 底盘位置控制 | ❌ 不需要 |
| `control_base_position_local()` | 底盘局部位置控制 | ❌ 不需要 |
| `control_base_velocity()` | 底盘速度控制 | ❌ 不需要 |
| `control_torso_6dof()` | 躯干 6DOF 控制 | ❌ 不需要 |
| `control_wheel_lower_joint()` | 下肢关节控制 | ❌ 不需要 |
| `move_wheel_lower_joint_auto()` | 自动下肢关节控制 | ❌ 不需要 |

#### 使用示例

```python
from adapters.hardware.leju_wheeled.services.sdk_manager import LowLevelSDKManager
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoManipulationMpcCtrlMode,
    KuavoManipulationMpcControlFlow
)
import numpy as np

manager = LowLevelSDKManager()
manager.initialize()

try:
    # Step 1: 设置 MPC 模式
    manager.set_external_control_mode()
    manager.set_mpc_mode(
        mode=KuavoManipulationMpcCtrlMode.ArmOnly,
        flow=KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
    )
    time.sleep(0.5)
    
    # Step 2: 控制手臂关节（注意：需要传入弧度）
    joint_angles_deg = [0.0, 30.0, -20.0, 0.0, 0.0, -45.0, 0.0] * 2
    joint_angles_rad = [np.deg2rad(a) for a in joint_angles_deg]
    
    success = manager.robot_sdk.control.control_arm_joint_positions(
        joint_positions=joint_angles_rad
    )
    
    if success:
        print("关节控制成功")
    else:
        print("关节控制失败")
    
finally:
    # Step 3: 恢复模式
    manager.restore_mpc_mode()
    time.sleep(0.5)

manager.shutdown()
```

---

## 🔑 关键技术点

### 1. MPC 模式管理

#### 为什么需要 MPC 模式管理？

乐聚机器人的手臂控制需要通过 MPC（模型预测控制）优化器，必须先切换到外部控制模式，否则控制指令会被忽略。

#### 三步设置流程

```python
# Step 1: 切换到外部控制模式（必须）
robot_sdk.control.set_external_control_arm_mode()

# Step 2: 设置 MPC 控制模式（必须）
robot_sdk.control.set_manipulation_mpc_mode(
    KuavoManipulationMpcCtrlMode.ArmOnly
)

# Step 3: 设置控制流（建议）
robot_sdk.control.set_manipulation_mpc_control_flow(
    KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
)

time.sleep(0.5)  # 等待模式切换生效
```

#### 两步恢复流程

```python
# Step 1: 恢复到无控制模式
robot_sdk.control.set_manipulation_mpc_mode(
    KuavoManipulationMpcCtrlMode.NoControl
)

# Step 2: 恢复默认控制流
robot_sdk.control.set_manipulation_mpc_control_flow(
    KuavoManipulationMpcControlFlow.ThroughFullBodyMpc
)

time.sleep(0.5)  # 等待模式恢复生效
```

### 2. 单位转换

#### 关节角度：度 → 弧度

**API 要求**: 所有关节角度 API 接收**弧度**

**用户友好**: 测试脚本中使用**度**更直观

**解决方案**: 在管理器中自动转换

```python
# 用户传入（度）
joint_angles_deg = [0.0, 30.0, -20.0, 0.0, 0.0, -45.0, 0.0]

# 自动转换为弧度
import numpy as np
joint_angles_rad = [np.deg2rad(angle) for angle in joint_angles_deg]

# 传递给 API
robot_sdk.control.control_arm_joint_positions(joint_angles_rad)
```

#### 姿态表示：四元数 vs 欧拉角

**推荐使用**: `Pose6D` 对象统一表示

```python
from core.domain.pose import Pose6D

# 创建位姿对象
pose = Pose6D(
    x=0.5, y=0.3, z=0.3,
    roll=0.0, pitch=-1.57, yaw=0.0
)

# 转换为四元数（如果需要）
quaternion = pose.to_quaternion()
```

### 3. 错误处理

#### 统一使用 Result 对象

```python
from core.domain.result import Result

def send_chassis_world(self, x, y, yaw, desire_time) -> Result:
    """返回 Result 对象，统一错误处理"""
    try:
        # 验证参数
        if not self.is_initialized():
            return Result.failure("SDK 未初始化")
        
        # 执行控制
        success, actual_time = self.timed_cmd_api.send_timed_cmd(
            cmd_type='chassis_world',
            cmd_vec=[x, y, yaw],
            desire_time=desire_time
        )
        
        if success:
            return Result.success(data={"actual_time": actual_time})
        else:
            return Result.failure("指令执行失败")
            
    except Exception as e:
        return Result.failure(f"异常: {str(e)}")
```

#### 使用示例

```python
result = manager.send_chassis_world(x=0.5, y=0.0, yaw=0.0, desire_time=3.0)

if result.success:
    print(f"✅ 执行成功，实际时间: {result.data['actual_time']}s")
else:
    print(f"❌ 执行失败: {result.message}")
    # 可选：记录日志、重试等
```

---

## ⚠️ 注意事项

### 1. 线程安全

- ❌ **不要**在多个线程中同时使用同一个管理器实例
- ✅ **可以**为每个线程创建独立的管理器实例

### 2. 资源管理

- ✅ 使用完务必调用 `shutdown()` 释放资源
- ✅ 推荐使用 `try-finally` 确保资源释放

```python
manager = TimedCmdManager()
try:
    manager.initialize()
    # 使用...
finally:
    manager.shutdown()
```

### 3. MPC 模式冲突

- ❌ **不要**同时运行多个需要 MPC 模式的控制
- ✅ 确保前一个控制完成后才启动下一个

### 4. 轮臂机器人特殊要求

对于轮臂机器人（Wheeled Humanoid）：

- `direct_to_wbc = False` - 必须经过 MPC 优化
- `back_default = False` - 保持控制状态，避免频繁切换

---

## 🧪 测试

### 单元测试

SDK 管理器提供完整的单元测试套件，无需实机或仿真环境即可运行。

#### 运行所有单元测试

```bash
cd ~/LeTools
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/ -v
```

#### 运行特定管理器的测试

```bash
# BaseSDKManager 测试
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/test_base_sdk_manager.py -v

# TimedCmdManager 测试
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/test_timed_cmd_manager.py -v

# ArmSDKManager 测试
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/test_arm_sdk_manager.py -v

# LowLevelSDKManager 测试
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/test_low_level_sdk_manager.py -v
```

#### 运行特定测试类

```bash
# 只运行初始化相关测试
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/test_base_sdk_manager.py::TestBaseSDKManagerInit -v

# 只运行 MPC 模式管理测试
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/test_base_sdk_manager.py::TestBaseSDKManagerMPCMode -v
```

#### 生成覆盖率报告

```bash
# 安装 pytest-cov
pip install pytest-cov

# 运行测试并生成覆盖率报告
pytest adapters/hardware/leju_wheeled/services/sdk_manager/tests/ --cov=adapters.hardware.leju_wheeled.services.sdk_manager --cov-report=html

# 查看报告
firefox htmlcov/index.html
```

### 测试覆盖范围

| 管理器 | 测试文件 | 测试数量 | 覆盖内容 |
|--------|---------|---------|----------|
| **BaseSDKManager** | `test_base_sdk_manager.py` | 15+ | 初始化、关闭、MPC 模式管理（手动/自动）、错误处理 |
| **TimedCmdManager** | `test_timed_cmd_manager.py` | 12+ | 7 种控制模式、单位转换、参数验证、错误处理 |
| **ArmSDKManager** | `test_arm_sdk_manager.py` | 10+ | 自动/手动 MPC 模式、末端轨迹、关节轨迹、手臂归位 |
| **LowLevelSDKManager** | `test_low_level_sdk_manager.py` | 10+ | 底层 API 访问、MPC 模式管理、单位转换、错误处理 |
| **总计** | 4 个文件 | **47+** | 完整覆盖 Core 层功能 |

### 集成测试

集成测试需要启动机器人仿真或实机环境。

```bash
# 1. 启动 Mujoco 仿真
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch 

# 2. 运行应用层测试（使用 SDK 管理器）
python3 apps/test_kuavo_5w_refactored/01_base_control/test_cmd_vel_world.py
```

**注意**: 集成测试在 `apps/test_kuavo_5w_refactored/` 目录中，不在本模块。

---

## 📖 相关文档

- [IHardware 接口定义](../../interfaces/i_hardware.py)
- [应用层测试 README](../../../apps/test_kuavo_5w_app/README.md)

---

## 🆘 常见问题

### Q1: 为什么手臂控制没有反应？

**可能原因**:
1. MPC 模式未正确设置
2. 关节角度超出限位
3. 机器人处于急停状态

**解决方法**:
```python
# 检查 MPC 模式是否正确设置
manager.set_external_control_mode()
manager.set_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly, ...)
time.sleep(0.5)

# 检查关节角度范围
print(f"关节角度: {joint_angles}")

# 检查机器人状态
print(f"机器人状态: {robot_sdk.get_robot_state()}")
```

### Q2: 自动管理和手动管理有什么区别？

| 特性 | 自动管理 | 手动管理 |
|------|---------|---------|
| **易用性** | ✅ 简单 | ⚠️ 复杂 |
| **灵活性** | ⚠️ 一般 | ✅ 高 |
| **适用场景** | 单次控制 | 多次连续控制 |
| **出错风险** | ✅ 低 | ⚠️ 高 |

**建议**: 初学者使用自动管理，专家使用手动管理

### Q3: 如何选择使用哪个管理器？

| 场景 | 推荐管理器 |
|------|-----------|
| 简单的点位控制 | `TimedCmdManager` |
| 复杂的轨迹控制 | `ArmSDKManager`（自动模式） |
| 底层研究和调试 | `LowLevelSDKManager` |
| 生产环境应用 | `TimedCmdManager` 或 `ArmSDKManager` |

---

**维护者**: Kuavo Studio Team  
**联系方式**: 
**最后更新**: 2026-06-17
