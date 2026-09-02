# Kuavo 5-W Factory 封装验证测试 (Tier 4)

> 📋 [apps/ 测试套件总览](../README.md) · [源脚本 → T1 → T2 → T3 → T4 映射表](../TEST_SCRIPT_MAPPING.md)

## 定位

**T4 Factory 模式封装验证层**，使用 `HardwareFactory.create_hardware()` 创建 `LejuWheeledArmHardware` 实例，通过 `_sdk` / `_timed` 后缀方法验证 Adapter 内部 SDK 管理服务的封装正确性。

在 LeTools 的分层测试架构中：

| 层级 | 目录 | 接口方式 | 目的 |
|------|------|---------|------|
| T1 | `test_kuavo_5w_internal/` | rospy 直调 ROS 话题/服务 | 底层基准：ROS 通信正确性 |
| T2 | `test_kuavo_5w_adapter/` | `LejuWheeledArmHardware` 标准接口 | 适配器层验证 |
| T3 | `test_kuavo_5w_sdk_internal/` | KuavoHumanoid SDK 原生 API | SDK 可用性验证 |
| **T4 (本目录)** | `test_kuavo_5w_sdk_adapter/` | `HardwareFactory` → `_sdk`/`_timed` 方法 | Factory 封装验证 |

### 与 T2 的关系

T2 和 T4 都测试同一个 `LejuWheeledArmHardware`，但测试不同的方法族：

| | T2 (标准接口) | T4 本目录 |
|------|-------------|----------|
| **测试方法** | 无后缀标准方法 | `_sdk` / `_timed` 后缀方法 |
| **调用示例** | `hw.send_base_velocity()` | `hw.send_base_velocity_sdk()` / `hw.send_base_velocity_timed()` |
| **底层走** | Mixin 直发 ROS 话题 | Mixin → SDKManager → SDK API |

### 与 T3 的关系

T4 是 T3 的封装验证层：

| | T3 (SDK 原生) | T4 本目录 |
|------|--------------|----------|
| **调用方式** | 直接 `TimedCmdAPI().send_timed_cmd()` | `hw.send_*_timed()`（封装 T3） |
| **直接 `robot_sdk.control.*`** | `hw.send_*_sdk()`（封装 T3） |
| **目的** | 验证 SDK API 本身可用 | 验证 Adapter 封装没调错 SDK API |

T4 的 `_timed` 方法封装了 T3 `TimedCmdAPI` 调用，T4 的 `_sdk` 方法封装了 T3 `robot_sdk.control.*` 调用。T3 是 T4 的参考实现。

### 职责边界

| ✅ 本目录测试 | ❌ 不在本目录测试 |
|--------------|------------------|
| `hw.send_*_timed()` 封装正确性 | SDK API 本身可用性 → T3 |
| `hw.send_*_sdk()` 封装正确性 | 标准接口方法 → T2 |
| `HardwareFactory` 创建链路 | ROS 原生话题 → T1 |

---

<!-- AUTO-GENERATED:START directory-tree -->
## 目录结构

```
apps/test_kuavo_5w_sdk_adapter/
├── README.md
├── __init__.py
├── _scaffold.py                          # Factory 脚手架 (factory_setup/teardown)
│
├── sdk/                                  # _sdk 路径 (15) ── 封装 robot_sdk.control.*
│   ├── 01_head/
│   │   └── test_head_control.py          # set_head_pose_sdk()
│   ├── 02_arm/
│   │   ├── test_arm_joint_traj.py        # send_arm_joint_traj_sdk()
│   │   ├── test_arm_reset.py             # arm_reset_sdk()
│   │   ├── test_arm_ee_traj_world.py     # send_arm_ee_traj_world_sdk()
│   │   ├── test_arm_ee_traj_local.py     # send_arm_ee_traj_local_sdk()
│   │   └── test_arm_ee_pose_sdk.py       # 末端位姿 30Hz 循环直调
│   ├── 03_lower_body/
│   │   ├── test_leg_joint.py             # send_leg_joint_sdk()
│   │   └── test_torso_6dof.py            # send_torso_6dof_sdk()
│   ├── 04_base/
│   │   ├── test_base_position_world.py   # send_base_position_world_sdk()
│   │   ├── test_base_position_local.py   # send_base_position_local_sdk()
│   │   └── test_base_velocity.py         # send_base_velocity_sdk()
│   ├── 05_mode/
│   │   ├── test_mpc_mode.py              # set_mpc_mode_sdk()
│   │   ├── test_quick_mode.py            # 快速模式切换
│   │   └── test_arm_ctrl_mode.py         # 手臂控制模式切换
│   └── 06_feedback/
│       └── test_state_feedback.py        # 状态反馈
│
└── timed/                                # _timed 路径 (23) ── 封装 TimedCmdAPI
    ├── 01_chassis/
    │   ├── test_chassis_local.py         # send_base_velocity_timed(frame=LOCAL)
    │   └── test_chassis_world.py         # send_base_velocity_timed(frame=WORLD)
    ├── 02_torso/
    │   └── test_torso_pose.py            # send_torso_pose_timed()
    ├── 03_leg/
    │   └── test_leg_joint.py             # send_leg_joint_timed()
    ├── 04_arm/
    │   ├── test_arm_joint.py             # send_left_arm_joint_timed()
    │   ├── test_arm_ee_world.py          # send_arm_ee_world_timed(frame=WORLD)
    │   ├── test_arm_ee_local.py          # send_arm_ee_world_timed(frame=LOCAL)
    │   ├── test_arm_force.py            # 力控 timed 路径
    │   ├── test_left_arm_joint.py       # 单左臂关节
    │   ├── test_right_arm_joint.py      # 单右臂关节
    │   ├── test_left_arm_ee_world.py    # 单左臂世界系
    │   ├── test_right_arm_ee_world.py   # 单右臂世界系
    │   ├── test_arm_ee_single_timed.py  # 单臂末端单点位姿
    │   ├── test_arm_ee_dual_timed.py    # 双臂末端位姿(躯干不回零)
    │   ├── test_arm_ee_burst_timed.py   # 在线航点连发
    │   ├── test_arm_ee_offline_traj.py  # 离线时间最优轨迹
    │   ├── test_arm_ee_local_relative_servo.py  # 相对补偿(TF)
    │   ├── test_get_arm_ee_current_pose.py      # 只读反馈
    │   └── behavior_tree_factory.py     # 辅助模块(行为树工厂,非测试)
    └── 05_advanced/
        ├── test_multi_cmd.py            # 多指令并发
        ├── test_offline_trajectory.py   # 离线轨迹
        ├── test_ik_accessibility.py     # IK 可达性
        └── test_ruckig_params.py        # Ruckig 参数
```
<!-- AUTO-GENERATED:END directory-tree -->

<!-- AUTO-GENERATED:START completion-table -->
## 完成状态

| 路径 | 子模块 | 脚本数 | 状态 |
|------|--------|:------:|:----:|
| `sdk/` | 01_head | 1 | 🔄 |
| | 02_arm | 5 | 🔄 |
| | 03_lower_body | 2 | 🔄 |
| | 04_base | 3 | 🔄 |
| | 05_mode | 3 | 🔄 |
| | 06_feedback | 1 | 🔄 |
| `timed/` | 01_chassis | 2 | 🔄 |
| | 02_torso | 1 | 🔄 |
| | 03_leg | 1 | 🔄 |
| | 04_arm | 16 | 🔄 |
| | 05_advanced | 4 | 🔄 |
| **总计** | | **38** | 🔄 待完整验证 |

> 统计口径：排除 `__init__.py` 和 `_scaffold.py`，含 `behavior_tree_factory.py` 辅助模块。
> `timed/` 路径 16 个 04_arm 脚本中含 1 个辅助模块（`behavior_tree_factory.py`，非测试脚本），实际测试脚本 37 个。
<!-- AUTO-GENERATED:END completion-table -->

### T4 新增脚本说明

T4 有 7 个 `timed/04_arm/` 脚本无对应 T3，无直接源脚本，是 Factory 封装层独有的功能扩展：

| 脚本 | 说明 |
|------|------|
| `test_arm_ee_single_timed.py` | 单臂末端单次位姿（`send_timed_single_command` 单点） |
| `test_arm_ee_dual_timed.py` | 双臂末端单次位姿（planner 4+5/6+7，躯干不回零） |
| `test_arm_ee_burst_timed.py` | 单臂末端在线航点连发（循环 `send_timed_single_command`） |
| `test_arm_ee_offline_traj.py` | 单臂末端离线整体时间最优轨迹 |
| `test_arm_ee_local_relative_servo.py` | local/base_link 相对补偿（订阅 eePoses + TF + 叠加 offset） |
| `test_get_arm_ee_current_pose.py` | 读取当前双臂 local 位姿（只读反馈） |
| `behavior_tree_factory.py` | 辅助模块（行为树工厂，非测试脚本） |

`sdk/02_arm/test_arm_ee_pose_sdk.py` 也是 T4 独有：SDK 单次直调，30Hz 循环调用 `control_arm_eef`，验证 SDK 端位姿控制的实时性。

详见 [源脚本映射表](../TEST_SCRIPT_MAPPING.md) 的 T4 新增脚本章节。

## 运行方式

```bash
# _sdk 路径
python3 apps/test_kuavo_5w_sdk_adapter/sdk/04_base/test_base_velocity.py

# _timed 路径
python3 apps/test_kuavo_5w_sdk_adapter/timed/01_chassis/test_chassis_world.py

# 末端位姿 30Hz 循环（T4 独有）
python3 apps/test_kuavo_5w_sdk_adapter/sdk/02_arm/test_arm_ee_pose_sdk.py
```

### 调用示例

```python
from adapters.hardware.factory import HardwareFactory
from apps.test_kuavo_5w_sdk_adapter._scaffold import factory_setup, factory_teardown

# _sdk 方法
hw = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})
factory_setup(hw, need_arm=True)
hw.send_base_velocity_sdk(vx=0.2, vy=0.0, vyaw=0.0)
factory_teardown(hw, need_arm=True)
hw.shutdown()

# _timed 方法
hw = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})
factory_setup(hw, need_arm=True)
hw.send_arm_ee_world_timed(
    left_pose=[0.1, 0.4, 0.7, 0.0, 0.0, 0.0],
    right_pose=[0.1, -0.4, 0.7, 0.0, 0.0, 0.0],
    desire_time=3.0
)
factory_teardown(hw, need_arm=True)
```

## 环境要求

- ROS Noetic + Python 3
- LeTools 框架已正确安装
- `kuavo_humanoid_sdk` 已安装（子模块位于 `drivers/leju/kuavo_humanoid_sdk`）
- 机器人控制器已启动（仿真或实机）

---

**最后更新**: 2026-08-11
**状态**: 38/38 脚本已实现 🔄，待完整验证
