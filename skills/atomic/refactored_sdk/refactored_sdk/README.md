# refactored_sdk 原子技能集合

从可跑通测试脚本抽取的 Adapter 原子技能，每个技能封装了 **一个** `hardware.xxx()` 调用。

## 架构约定

```
Skill (本层)         →  Node (orchestration/nodes/)  →  JSON 行为树
    │
    └── hardware.xxx()     ← IHardware 接口
         └── _xxx_manager  ← Adapter 内部 SDK 管理服务
              └── robot_sdk.control.xxx()  ← Kuavo 底层 SDK
```

所有需硬件依赖的技能通过 `get_shared_hardware()` 获取 `IHardware` 单例注入；纯编排工具技能无硬件依赖。

---

## 技能清单

### 1. 手臂控制

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `arm_ee_traj_local_sdk.py` | 手臂末端轨迹（本体坐标系 `base_link`） | `hardware.send_arm_ee_traj_sdk(frame='base_link')` | `sdk_control_mixin.py` → `ArmSDKManager.move_eef_traj_auto()` → `robot_sdk.control.control_robot_end_effector_pose()` |
| `arm_ee_traj_world_sdk.py` | 手臂末端轨迹（世界坐标系 `world`） | `hardware.send_arm_ee_traj_sdk(frame='world')` | 同上 |
| `arm_joint_traj_sdk.py` | 手臂 14 关节轨迹（多关键点，自动 MPC） | `hardware.send_arm_joint_traj_sdk()` | `sdk_control_mixin.py` → `ArmSDKManager.move_joint_traj_auto()` → `robot_sdk.control.control_arm_joint_positions()` |
| `arm_reset_sdk.py` | 双臂归位到初始姿态 | `hardware.arm_reset()` | `sdk_control_mixin.py` → `ArmSDKManager.arm_reset()` → `robot_sdk.control.arm_reset()` |

**参数说明**：
- 末端轨迹点格式：`[x, y, z, qx, qy, qz, qw]`（位置 + 四元数）
- 关节角度格式：14 个 float，顺序 `[左臂J0-J6, 右臂J0-J6]`（用户角度单位），镜像规则 J0/J3/J6 保持原值，J1/J2/J4/J5 取反

### 2. 底盘控制

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `base_pose_local.py` | 底盘本体坐标系相对位姿移动 | `hardware.send_base_pose(x, y, yaw, frame=LOCAL)` | `base_control_mixin.py` → 适配器内部等待到达时间反馈 |

### 3. 头部与下肢

| 技能文件 | 功能 | 调用的 Adapter 方法 | 底层链路 |
|---------|------|-------------------|---------|
| `head_control_sdk.py` | 头部偏航/俯仰控制 | `hardware.control_head_sdk(yaw, pitch)` | `sdk_control_mixin.py` → `LowLevelSDKManager.control_head()` → `robot_sdk.control.control_head()` |
| `leg_joint_sdk.py` | 下肢 4 关节控制 | `hardware.send_leg_joint_sdk()` | `sdk_control_mixin.py` → `LowLevelSDKManager.move_wheel_lower_joint_auto()` → `robot_sdk.control.control_leg_joint_positions()` |

### 4. 编排工具（无硬件依赖）

| 技能文件 | 功能 | 实现方式 |
|---------|------|---------|
| `wait_for_enter.py` | 等待用户按 Enter 后继续 | `input()` 阻塞，DRY_RUN 环境自动跳过 |
| `wait_seconds.py` | 阻塞等待指定秒数 | `time.sleep()`，DRY_RUN 环境自动跳过 |

---

## 对应测试脚本

| 技能 | 测试脚本 |
|------|---------|
| `arm_ee_traj_local_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_ee_traj_local.py` |
| `arm_ee_traj_world_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_ee_traj_world.py` |
| `arm_joint_traj_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_joint_traj.py` |
| `arm_reset_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_arm/test_arm_reset.py` |
| `base_pose_local` | `apps/test_kuavo_5w_refactored/sdk/01_base_control/test_cmd_pose_base.py` |
| `head_control_sdk` | `apps/test_kuavo_5w_refactored/sdk/06_services/test_head_control.py` |
| `leg_joint_sdk` | `apps/test_kuavo_5w_refactored/sdk/02_lower_body/test_leg_joint.py` |

---

## 关键源码位置

| 层级 | 路径 |
|------|------|
| 硬件适配器接口 | `adapters/hardware/leju_wheeled/mixins/sdk_control_mixin.py` |
| 底盘适配器 | `adapters/hardware/leju_wheeled/mixins/base_control_mixin.py` |
| Arm SDK 管理器 | `adapters/hardware/leju_wheeled/services/sdk_manager/arm_sdk_manager.py` |
| LowLevel SDK 管理器 | `adapters/hardware/leju_wheeled/services/sdk_manager/low_level_sdk_manager.py` |
| 硬件工厂 | `adapters/hardware/factory.py` |
| 共享硬件单例 | `orchestration/shared_hardware.py` |
| 编排节点（薄封装） | `orchestration/nodes/` |
| 行为树场景 | `orchestration/scenarios/refactored_sdk_arm_v1/` |
