# MPC 模式使用指南

## 📋 概述

本文档说明了 Kuavo 5-W 机器人 **MPC 控制模式**的使用场景和注意事项。

---

## 新手理解：MPC 模式是什么

在这个项目里，**MPC 模式可以先理解成机器人底层控制器的“控制权限模式”**。

它不是一个具体动作，而是在告诉机器人：

```text
现在允许控制哪一部分？
├── 只控制手臂？
├── 只控制底盘？
├── 底盘和手臂一起控制？
└── 还是暂时不让 MPC 控制？
```

只有控制器进入了合适的模式，对应的外部指令才会真正生效。特别是手臂相关指令，如果 MPC 模式没有切对，就可能出现“代码发了命令、甚至看起来返回成功，但机器人手臂不动”的情况。

可以把它和普通控制指令这样区分：

```text
MPC 模式
└── 决定当前允许哪一类控制生效，是控制权限/控制状态。

控制指令
└── 真正让机器人运动，比如底盘速度、手臂关节、末端位姿。
```

常见模式可以这样记：

```python
MPCControlMode.NO_CONTROL   # 不控制
MPCControlMode.ARM_ONLY     # 只控制手臂
MPCControlMode.BASE_ONLY    # 只控制底盘
MPCControlMode.BASE_ARM     # 底盘 + 手臂一起控制
MPCControlMode.ARM_EE_ONLY  # 只控制手臂末端
```

最重要的使用原则：

```text
底盘速度控制：一般不需要设置 MPC 模式
底盘位置控制：纯底盘时一般不需要设置 MPC 模式
手臂控制：必须设置 MPC 模式
底盘 + 手臂协同：必须设置 BASE_ARM 模式
躯干/腿部控制：通常不需要设置 MPC 模式
```

例如，只让底盘走，一般可以直接发速度：

```python
hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)
```

但如果要控制手臂，通常需要先切模式，再切手臂控制模式，最后发手臂指令：

```python
hardware.set_mpc_mode(MPCControlMode.ARM_ONLY)
hardware.set_arm_control_mode(1)  # 先重置/准备，避免奇异点等问题
hardware.set_arm_control_mode(2)  # 切到外部控制器，开始接受外部指令
hardware.send_ee_pose(...)
```

所以，阅读和编写 `test_kuavo_5w_app/` 里的测试脚本时，可以先问自己一个问题：

```text
这个测试是在控制底盘、手臂，还是底盘和手臂一起控制？
```

如果是手臂或底盘+手臂协同，就要重点检查 MPC 模式和手臂控制模式是否设置正确。

---


## 🎯 核心原则

### 1. 底盘速度控制（`/cmd_vel`）- **无需设置 MPC 模式**

**原因**：
- ✅ `/cmd_vel` 在底盘指令中**优先级最高**
- ✅ 直接发布即可控制底盘，不受 MPC 模式影响
- ✅ 底层测试脚本均未设置 MPC 模式

**示例**：
```python
# ✅ 正确：直接发送速度命令
hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0, frame=FrameType.LOCAL)

# ❌ 错误：不需要设置 MPC 模式
# hardware.set_mpc_mode(MPCControlMode.BASE_ONLY)  # 多余！
```

**参考**：
- 接口文档：`Kuavo 5-W 接口使用文档.md` - "在底盘指令中优先级最高"
- 底层测试：`cmd_vel_base_test.py` - 未设置 MPC 模式

---

### 2. 底盘位置控制（`/cmd_pose`, `/cmd_pose_world`）- **建议不设置 MPC 模式**

**原因**：
- ✅ 位置控制也是高优先级指令
- ✅ 底层测试脚本均未设置 MPC 模式
- ⚠️ 如果同时控制手臂，可能需要设置 `BASE_ARM` 模式

**示例**：
```python
# ✅ 纯底盘位置控制：无需设置 MPC 模式
hardware.send_base_pose(x=1.0, y=0.0, yaw=0.0, frame=FrameType.WORLD)

# ⚠️ 底盘+手臂协同：需要设置 BASE_ARM 模式
hardware.set_mpc_mode(MPCControlMode.BASE_ARM)
hardware.send_base_pose(...)
hardware.send_ee_pose(...)
```

---

### 3. 手臂控制 - **必须设置 MPC 模式**

**原因**：
- ❌ 手臂控制受 MPC 模式严格限制
- ✅ 必须先切换到 `ARM_ONLY` 或 `BASE_ARM` 模式
- ✅ 还需要调用 `set_arm_control_mode(2)` 切换到外部控制器

**示例**：
```python
# ✅ 正确：先设置 MPC 模式，再设置手臂控制模式
hardware.set_mpc_mode(MPCControlMode.ARM_ONLY)  # MPC 模式
hardware.set_arm_control_mode(2)  # 手臂控制模式（外部控制器）
hardware.send_ee_pose(side=ArmSide.LEFT, pose=pose, frame=FrameType.WORLD)

# ❌ 错误：未设置 MPC 模式
# hardware.send_ee_pose(...)  # 会失败！
```

**关键步骤**：
1. 设置 MPC 模式：`set_mpc_mode(MPCControlMode.ARM_ONLY)`
2. 重置手臂：`set_arm_control_mode(1)` （避免奇异点）
3. 切换到外部控制：`set_arm_control_mode(2)` （接受外部指令）
4. 发送位姿指令：`send_ee_pose(...)`

**参考**：
- 底层测试：`cmd_arm_ee_world_test.py` - 完整展示了这4个步骤

---

### 4. 躯干控制 - **通常无需设置 MPC 模式**

**原因**：
- ✅ 躯干控制是独立的高优先级指令
- ✅ 底层测试脚本未设置 MPC 模式

**示例**：
```python
# ✅ 直接发送躯干位姿命令
from core.domain.pose import Pose6D
pose = Pose6D(x=0.2, y=0.0, z=0.8, roll=0.0, pitch=0.0, yaw=0.0)
hardware.send_torso_pose(pose)
```

---

### 5. 腿部关节控制 - **通常无需设置 MPC 模式**

**原因**：
- ✅ 腿部关节控制是独立指令
- ✅ 底层测试脚本未设置 MPC 模式

**示例**：
```python
# ✅ 直接发送腿部关节命令
positions = [14.90, -32.01, 18.03, 0.0]  # 4个关节角度（度）
hardware.send_leg_joint_command(positions)
```

---

## 📊 MPC 模式使用场景总结

| 控制类型 | ROS 话题 | 是否需要 MPC 模式 | 推荐模式 | 备注 |
|---------|---------|------------------|---------|------|
| **底盘速度** | `/cmd_vel` | ❌ 不需要 | - | 优先级最高 |
| **底盘位置** | `/cmd_pose` | ❌ 不需要 | - | 纯底盘控制时无需设置 |
| **底盘位置** | `/cmd_pose_world` | ❌ 不需要 | - | 纯底盘控制时无需设置 |
| **手臂末端** | `/mm/two_arm_hand_pose_cmd` | ✅ 必须 | `ARM_ONLY` 或 `BASE_ARM` | 还需设置手臂控制模式 |
| **手臂关节** | `/kuavo_arm_traj` | ✅ 必须 | `ARM_ONLY` 或 `BASE_ARM` | 还需设置手臂控制模式 |
| **躯干位姿** | `/cmd_lb_torso_pose` | ❌ 不需要 | - | 独立高优先级指令 |
| **腿部关节** | `/lb_leg_traj` | ❌ 不需要 | - | 独立指令 |
| **底盘+手臂协同** | 多个话题 | ✅ 必须 | `BASE_ARM` | 同时控制时需要 |

---

## 🔧 MPC 模式枚举值

```python
from core.domain.enums import MPCControlMode

MPCControlMode.NO_CONTROL     # 0: 无控制
MPCControlMode.ARM_ONLY       # 1: 仅控制手臂，基座固定
MPCControlMode.BASE_ONLY      # 2: 仅控制基座，手臂固定
MPCControlMode.BASE_ARM       # 3: 同时控制基座和手臂
MPCControlMode.ARM_EE_ONLY    # 4: 仅控制手臂末端
```

---

## 💡 最佳实践

### 场景 1：纯底盘运动测试

```python
# ✅ 推荐：无需设置 MPC 模式
hardware = LejuWheeledArmHardware()
hardware.initialize()

# 直接发送速度命令
hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)

hardware.shutdown()
```

### 场景 2：纯手臂控制测试

```python
# ✅ 推荐：必须设置 MPC 模式和手臂控制模式
hardware = LejuWheeledArmHardware()
hardware.initialize()

# 1. 设置 MPC 模式
hardware.set_mpc_mode(MPCControlMode.ARM_ONLY)

# 2. 重置手臂（避免奇异点）
hardware.set_arm_control_mode(1)
time.sleep(1.0)

# 3. 切换到外部控制器
hardware.set_arm_control_mode(2)

# 4. 发送位姿指令
pose = Pose6D(x=0.5, y=0.0, z=0.3, roll=0.0, pitch=-1.5708, yaw=0.0)
hardware.send_ee_pose(side=ArmSide.LEFT, pose=pose, frame=FrameType.WORLD)

hardware.shutdown()
```

### 场景 3：底盘+手臂协同控制

```python
# ✅ 推荐：使用 BASE_ARM 模式
hardware = LejuWheeledArmHardware()
hardware.initialize()

# 1. 设置 MPC 模式为 BASE_ARM
hardware.set_mpc_mode(MPCControlMode.BASE_ARM)

# 2. 设置手臂控制模式
hardware.set_arm_control_mode(1)
time.sleep(1.0)
hardware.set_arm_control_mode(2)

# 3. 同时发送底盘和手臂指令
hardware.send_base_velocity(vx=0.1, vy=0.0, vyaw=0.0)
pose = Pose6D(x=0.5, y=0.0, z=0.3, roll=0.0, pitch=-1.5708, yaw=0.0)
hardware.send_ee_pose(side=ArmSide.LEFT, pose=pose, frame=FrameType.WORLD)

hardware.shutdown()
```

---

## ❓ 常见问题

### Q1: 为什么底盘速度控制不需要设置 MPC 模式？

**A**: 因为 `/cmd_vel` 话题在底盘指令中**优先级最高**，可以直接控制底盘电机，不受 MPC 模式限制。这是设计上的特殊处理，确保紧急情况下可以快速控制底盘。

### Q2: 如果不设置 MPC 模式就发送手臂指令，会发生什么？

**A**: 手臂指令会被忽略或报错。因为手臂控制严格受 MPC 模式管理，必须先切换到 `ARM_ONLY` 或 `BASE_ARM` 模式，并且还要调用 `set_arm_control_mode(2)` 切换到外部控制器。

### Q3: `BASE_ONLY` 模式和 `NO_CONTROL` 模式有什么区别？

**A**: 
- `BASE_ONLY (2)`: 仅控制基座，手臂固定在当前位置
- `NO_CONTROL (0)`: 关闭所有 MPC 控制，机器人进入被动状态

对于纯底盘测试，两者都可以，但通常不需要设置任何模式，因为 `/cmd_vel` 优先级最高。

### Q4: 什么时候必须设置 MPC 模式？

**A**: 
- ✅ **必须设置**：手臂控制（末端位姿、关节控制）
- ✅ **建议设置**：底盘+手臂协同控制（使用 `BASE_ARM`）
- ❌ **无需设置**：纯底盘速度/位置控制、躯干控制、腿部控制

---

## 📝 测试脚本规范

根据以上分析，测试脚本应遵循以下规范：

### 底盘控制测试

```python
class TestBaseVelocity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hardware = LejuWheeledArmHardware()
        cls.hardware.initialize()
        # ❌ 不要设置 MPC 模式
    
    def test_forward(self):
        result = cls.hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)
        self.assertTrue(result.success)
```

### 手臂控制测试

```python
class TestArmEEWorld(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hardware = LejuWheeledArmHardware()
        cls.hardware.initialize()
        
        # ✅ 必须设置 MPC 模式
        cls.hardware.set_mpc_mode(MPCControlMode.ARM_ONLY)
        
        # ✅ 必须设置手臂控制模式
        cls.hardware.set_arm_control_mode(1)  # 重置
        time.sleep(1.0)
        cls.hardware.set_arm_control_mode(2)  # 外部控制
    
    def test_move_forward(self):
        pose = Pose6D(x=0.5, y=0.0, z=0.3, roll=0.0, pitch=-1.5708, yaw=0.0)
        result = cls.hardware.send_ee_pose(side=ArmSide.LEFT, pose=pose)
        self.assertTrue(result.success)
```

---

## 🔗 相关文档

- **接口文档**: [Kuavo 5-W 接口使用文档.md](../../../kuavo-ros-opensource/docs/4开发接口/Kuavo%205-W%20接口使用文档.md)
- **适配器实现**: [hardware.py](../../adapters/hardware/leju_wheeled/hardware.py)
- **底层测试**: [test_kuavo_wheel_real](../../../kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/)

---

**最后更新**: 2026-05-16  
**维护者**: Kuavo Studio Team

