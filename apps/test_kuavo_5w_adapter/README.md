# Kuavo 5-W 适配器层标准接口测试 (Tier 2)

> 📋 [apps/ 测试套件总览](../README.md) · [源脚本 → T1 → T2 → T3 → T4 映射表](../TEST_SCRIPT_MAPPING.md)

## 定位

**T2 是适配器层标准接口验证层**，使用 `LejuWheeledArmHardware` 的**标准接口方法（无后缀）**验证硬件适配器功能。

在 LeTools 的分层测试架构中：

| 层级 | 目录 | 接口方式 | 目的 |
|------|------|---------|------|
| T1 | `test_kuavo_5w/` | rospy 直调 ROS 话题/服务 | 底层基准：ROS 通信正确性 |
| **T2 (本目录)** | `test_kuavo_5w_app/` | `LejuWheeledArmHardware` 标准接口 | 适配器层：标准方法验证 |
| T3 | `test_kuavo_5w_sdk/` | KuavoHumanoid SDK 原生 API | SDK 可用性验证 |
| T4 | `test_kuavo_5w_refactored/` | `HardwareFactory` → `_sdk`/`_timed` 方法 | Factory 封装验证 |

### 职责边界

| ✅ 本目录测试 | ❌ 不在本目录测试 |
|--------------|------------------|
| `hardware.send_base_velocity()` 等标准接口方法 | `_timed` 后缀方法 → 由 T3＋T4 覆盖 |
| `hardware.set_mpc_mode()` 等服务方法 | `_sdk` 后缀方法 → 由 T3＋T4 覆盖 |
| `hardware.arm_reset()` 等辅助方法 | 原生 rospy API → 由 T1 覆盖 |

**T2 只测试 `LejuWheeledArmHardware` 的标准接口方法（无后缀）**，不涉及 `_timed`/`_sdk` 路径。TimedCmd 和 SDK 直调路径由 T3（SDK 原生验证）+ T4（Factory 封装验证）覆盖。

**源脚本路径**：`kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/`（与 T1 同源，Path A）

**测试框架**：使用 `unittest.TestCase` + `adapter_setup/adapter_teardown` 脚手架。

## 新手推荐验证顺序

本目录是新手最推荐先看的测试目录，因为它使用的是 `hardware.send_xxx()` 这类标准接口，最接近后续开发 Skill 和行为树时的调用方式。

建议不要一上来就全量跑测试，而是按风险从低到高逐步验证：

```text
1. 无 ROS 预检
   └── 先确认 Python import、标准方法名、目录结构没有明显问题。

2. 底盘控制
   └── 先测 send_base_velocity / send_base_pose，验证基础运动链路。

3. 服务模式
   └── 再测 set_mpc_mode / enable_quick_mode，确认服务调用正常。

4. 躯干和腿部
   └── 验证 send_torso_pose / send_leg_joint_command。

5. 手臂控制
   └── 最后测手臂，因为它通常涉及 MPC 模式、手臂控制模式、超时等待等问题。

6. 状态反馈
   └── 用 07_debug_feedback 查看命令发出后有没有对应状态返回。
```

对应命令示例：

```bash
# 1. 无 ROS 预检
python3 apps/test_kuavo_5w_app/verify_phase1_standard_methods.py

# 2. 底盘速度，本体系
python3 apps/test_kuavo_5w_app/01_base_control/test_cmd_vel_base.py

# 3. MPC 模式服务
python3 apps/test_kuavo_5w_app/06_services/test_set_mpc_mode.py

# 4. 腿部关节
python3 apps/test_kuavo_5w_app/02_lower_body/test_leg_joint.py

# 5. 手臂关节
python3 apps/test_kuavo_5w_app/03_arm_control/test_arm_joint.py

# 6. 状态反馈
python3 apps/test_kuavo_5w_app/07_debug_feedback/verify_state_data.py
```

### 什么时候看哪个文档

| 你遇到的问题 | 建议先看 |
|--------------|----------|
| 不知道 T1/T2/T3/T4 区别 | `../README.md` |
| 不知道某个脚本来自哪里 | `../TEST_SCRIPT_MAPPING.md` |
| 手臂命令发了但不动 | `MPC_MODE_GUIDE.md` |
| TimedCmd、Ruckig、离线轨迹不理解 | `04_timed_commands/README.md` 和对应完成报告 |
| 想看状态反馈话题有没有数据 | `07_debug_feedback/README.md` |

### 如何判断问题在哪一层

```text
T2 脚本失败时，不要马上怀疑上层业务逻辑，可以按下面顺序排查：

1. 同类 T1 脚本是否能跑？
   └── 如果 T1 也失败，优先查 ROS 话题/服务/控制器。

2. T1 成功但 T2 失败？
   └── 优先查 LejuWheeledArmHardware 或对应 Mixin 封装。

3. 手臂相关失败？
   └── 重点查 MPC 模式、手臂控制模式、外部控制器是否切换成功。

4. 命令成功但机器人没动？
   └── 查看状态反馈、reach_time、MPC 模式反馈和 ROS 日志。
```

---
## 目录结构

```
apps/test_kuavo_5w_app/
├── README.md
├── __init__.py
├── _scaffold.py                          # 适配器层脚手架 (adapter_setup/teardown)
├── MPC_MODE_GUIDE.md                     # MPC 模式使用指南
│
├── test_base_control.py                  # ⚠️ 旧版，已拆分为 01_base_control/
├── test_arm_control.py                   # ⚠️ 旧版，已拆分为 03_arm_control/
├── verify_phase1_standard_methods.py     # 预检脚本 (无ROS)
│
├── 01_base_control/                      # 底盘控制 (4) ✅
│   ├── test_cmd_vel_base.py
│   ├── test_cmd_vel_world.py
│   ├── test_cmd_pose_base.py
│   └── test_cmd_pose_world.py
│
├── 02_lower_body/                        # 下肢 + 躯干 (2) ✅
│   ├── test_leg_joint.py
│   └── test_torso_pose.py
│
├── 03_arm_control/                       # 手臂控制 (4) ⚠️
│   ├── test_arm_joint.py
│   ├── test_arm_ee_world.py
│   ├── test_arm_ee_local.py
│   └── test_arm_ee_joint.py
│
├── 04_timed_commands/                    # 标准接口的时序验证 (10) ⚠️
│   ├── test_cmd_vel_sequence.py
│   ├── test_cmd_pose_sequence.py
│   ├── test_leg_joint_sequence.py
│   ├── test_arm_joint_sequence.py
│   ├── test_mixed_commands.py
│   ├── test_multi_cmd_sequence.py
│   ├── test_ik_accessibility.py
│   ├── test_offline_trajectory.py
│   ├── test_ruckig_params.py
│   └── test_ruckig_simple.py
│
├── 05_force_control/                     # ❌ 空目录 (仅 __init__.py)
│
├── 06_services/                          # 服务调用 (2) ✅
│   ├── test_set_mpc_mode.py
│   └── test_enable_quick_mode.py
│
├── 07_debug_feedback/                    # 调试反馈 (1 test + 2 诊断)
│   ├── diagnose_ros_topics.py
│   ├── test_complete_state_feedback.py
│   └── verify_state_data.py
│
└── config/                               # 配置文件
    └── backend_config.yaml
```

## 完成状态

| 模块 | 脚本数 | 状态 | 说明 |
|------|:------:|:----:|------|
| 01_base_control | 4 | ✅ | 底盘控制全部实现 |
| 02_lower_body | 2 | ✅ | 下肢+躯干全部实现 |
| 03_arm_control | 4 | ⚠️ | 已实现，存在超时问题待修复 |
| 04_timed_commands | 10 | 🚫 | **架构违规**（全部使用 `_timed` 方法，违反 T2 约束），保留不动，由 T3+T4 覆盖 |
| 05_force_control | 0 | ❌ | 唯一未实现的模块 |
| 06_services | 2 | ✅ | 服务调用全部实现 |
| 07_debug_feedback | 1 test + 2 诊断 | ✅ | 反馈订阅全部实现 |
| **维护脚本** | **13** | | 参与覆盖率、baseline、修复范围 |
| 根目录 legacy 旧版 | 2 | ⚠️ | 已拆分为子目录，保留兼容，不参与覆盖率 |
| **架构违规** | **10** | 🚫 | `04_timed_commands/`，由 T3+T4 覆盖 |

### `04_timed_commands/` 架构违规说明

T2 的 `04_timed_commands/` 下全部 10 个脚本均使用 `_timed` 后缀方法（`send_*_timed()`），违反 T2 层 "只测试标准接口方法（无后缀），不涉及 `_timed`/`_sdk` 方法" 的架构约束。

这些脚本保留不动，**不纳入 T2 的修复范围**。对应的 TimedCmd 功能由 T3（SDK 原生验证）+ T4（Factory 封装验证）完整覆盖。

### 根目录旧版脚本

| 文件 | 说明 |
|------|------|
| `test_base_control.py` | 旧版底盘控制（已拆分为 `01_base_control/` 4 个细粒度脚本） |
| `test_arm_control.py` | 旧版手臂控制（已拆分为 `03_arm_control/` 4 个细粒度脚本） |
| `verify_phase1_standard_methods.py` | 预检脚本，无 ROS 环境可用 |

这些旧版脚本保留用于兼容，新开发和测试应使用子目录中的细粒度脚本。

## 运行方式

```bash
# 单个脚本
python3 apps/test_kuavo_5w_app/01_base_control/test_cmd_vel_base.py

# pytest 运行
pytest apps/test_kuavo_5w_app/01_base_control/test_cmd_vel_base.py -v

# 模块批量
python3 -m unittest discover -s apps/test_kuavo_5w_app/01_base_control -p "test_*.py"
```

## 测试脚本模板

```python
import unittest
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.test_kuavo_5w_app._scaffold import adapter_setup, adapter_teardown
from core.domain.enums import MPCControlMode

class TestExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hardware = LejuWheeledArmHardware()
        cls.hardware.initialize()
        adapter_setup(cls.hardware, need_arm=True, mpc_mode=MPCControlMode.ARM_ONLY)

    @classmethod
    def tearDownClass(cls):
        adapter_teardown(cls.hardware, need_arm=True)
        cls.hardware.shutdown()

    def test_01_example(self):
        result = self.hardware.send_base_velocity(vx=0.3, vy=0.0, vyaw=0.0)
        self.assertTrue(result.success, f"失败: {result.message}")
```

## 环境要求

- ROS Noetic + Python 3
- LeTools 框架已正确安装
- 机器人控制器已启动（仿真或实机）

---

**最后更新**: 2026-05-30
**状态**: 25/25 子目录脚本已实现 🔄，唯一缺口：`05_force_control/` (空目录)

