# Kuavo 5-W Factory 封装验证测试 (Tier 4)

> 📋 [apps/ 测试套件总览](../README.md) · [源脚本 → T1 → T2 → T3 → T4 映射表](../TEST_SCRIPT_MAPPING.md)

## 定位

**T4 是 Factory 模式封装验证层**，使用 `HardwareFactory.create_hardware()` 创建 `LejuWheeledArmHardware` 实例，通过 `_sdk` / `_timed` 后缀方法验证 Core 层 SDK 管理器的封装正确性。

在 LeTools 的分层测试架构中：

| 层级 | 目录 | 接口方式 | 目的 |
|------|------|---------|------|
| T1 | `test_kuavo_5w/` | rospy 直调 ROS 话题/服务 | 底层基准：ROS 通信正确性 |
| T2 | `test_kuavo_5w_app/` | `LejuWheeledArmHardware` 标准接口 | 适配器层：标准方法验证 |
| T3 | `test_kuavo_5w_sdk/` | KuavoHumanoid SDK 原生 API | SDK 可用性验证 |
| **T4 (本目录)** | `test_kuavo_5w_refactored/` | `HardwareFactory` → `_sdk`/`_timed` 方法 | Factory 封装验证 |

### 与 T2 的关系

T2 和 T4 都测试同一个 `LejuWheeledArmHardware`，但测试不同的方法族：

| | T2 (标准接口) | T4 本目录 |
|------|-------------|----------|
| **测试方法** | 无后缀标准方法 | `_sdk` + `_timed` 后缀方法 |
| **工厂创建** | 直接 `LejuWheeledArmHardware()` | `HardwareFactory.create_hardware()` |
| **底层路径** | ROS 话题/服务 | Core 层 SDKManager → KuavoHumanoid SDK |
| **脚手架** | `adapter_setup/adapter_teardown` | `factory_setup/factory_teardown` |

### 两种封装路径

| 子目录 | 方法后缀 | 底层 Mixin | 调用链 |
|--------|---------|-----------|--------|
| `sdk/` | `_sdk` | `SDKControlMixin` | Factory → Core SDKMgr → `robot_sdk.control.*` |
| `timed/` | `_timed` | `TimedCommandMixin` | Factory → Core TimedCmdMgr → `TimedCmdAPI` → ROS 服务 |

**源脚本路径**：
- Path A：`kuavo-ros-opensource/src/demo/test_kuavo_wheel_real/`
- Path B：`kuavo-ros-opensource/src/kuavo_humanoid_sdk/.../pick_place_box/`

**权威参考**：`kuavo-ros-opensource/docs/轮臂V1.4开发文档/`

## 目录结构

```
apps/test_kuavo_5w_refactored/
├── README.md
├── __init__.py
├── _scaffold.py                          # Factory层脚手架 (factory_setup/teardown)
├── config/
│   └── backend_config.yaml               # 后端配置
│
├── sdk/                                  # _sdk 路径 (13)
│   ├── 01_head/
│   │   └── test_head_control.py         # control_head_sdk()
│   ├── 02_arm/
│   │   ├── test_arm_joint_traj.py       # send_arm_joint_sdk()
│   │   ├── test_arm_ee_traj_local.py    # send_arm_ee_traj_sdk(frame=LOCAL)
│   │   ├── test_arm_ee_traj_world.py    # send_arm_ee_traj_sdk(frame=WORLD)
│   │   └── test_arm_reset.py            # arm_reset() 辅助
│   ├── 03_lower_body/
│   │   ├── test_leg_joint.py            # send_leg_joint_sdk()
│   │   └── test_torso_6dof.py           # send_torso_pose_sdk()
│   ├── 04_base/
│   │   ├── test_base_position_world.py  # send_base_position_world_sdk()
│   │   ├── test_base_position_local.py  # send_base_position_local_sdk()
│   │   └── test_base_velocity.py        # send_base_velocity_sdk()
│   ├── 05_mode/
│   │   ├── test_mpc_mode.py             # set_mpc_mode_sdk()
│   │   └── test_quick_mode.py           # 快速模式切换
│   └── 06_feedback/
│       └── test_state_feedback.py        # 状态反馈
│
└── timed/                                # _timed 路径 (16)
    ├── 01_chassis/
    │   ├── test_chassis_local.py         # send_base_velocity_timed(frame=LOCAL)
    │   └── test_chassis_world.py         # send_base_velocity_timed(frame=WORLD)
    ├── 02_torso/
    │   └── test_torso_pose.py            # send_torso_pose_timed()
    ├── 03_leg/
    │   └── test_leg_joint.py             # send_leg_joint_timed()
    ├── 04_arm/
    │   ├── test_arm_ee_local.py          # send_arm_ee_world_timed(frame=LOCAL)
    │   ├── test_arm_ee_world.py          # send_arm_ee_world_timed(frame=WORLD)
    │   ├── test_arm_joint.py             # send_left_arm_joint_timed()
    │   ├── test_arm_force.py             # 力控 timed 路径
    │   ├── test_left_arm_ee_world.py     # 单左臂世界系
    │   ├── test_left_arm_joint.py        # 单左臂关节
    │   ├── test_right_arm_ee_world.py    # 单右臂世界系
    │   └── test_right_arm_joint.py       # 单右臂关节
    └── 05_advanced/
        ├── test_ik_accessibility.py      # check_ik_accessibility_timed()
        ├── test_multi_cmd.py             # 多指令组合
        ├── test_offline_trajectory.py    # 离线轨迹
        └── test_ruckig_params.py         # set_ruckig_params_timed()
```

## 完成状态

| 模块 | 脚本数 | 状态 |
|------|:------:|:----:|
| sdk/01_head | 1 | ✅ |
| sdk/02_arm | 4 | ⚠️ 部分通过基线 |
| sdk/03_lower_body | 2 | ⚠️ 待验证 |
| sdk/04_base | 3 | ⚠️ 待验证 |
| sdk/05_mode | 2 | ⚠️ 待验证 |
| sdk/06_feedback | 1 | ⚠️ 待验证 |
| timed/01_chassis | 2 | ⚠️ 待验证 |
| timed/02_torso | 1 | ⚠️ 待验证 |
| timed/03_leg | 1 | ⚠️ 待验证 |
| timed/04_arm | 8 | ⚠️ 部分通过基线 |
| timed/05_advanced | 4 | ⚠️ 待验证 |
| **总计** | **29** | 🔄 全部已实现，待完整验证 |

## 运行方式

```bash
# _sdk 路径
python3 apps/test_kuavo_5w_refactored/sdk/04_base/test_base_velocity.py

# _timed 路径
python3 apps/test_kuavo_5w_refactored/timed/01_chassis/test_chassis_world.py
```

### 调用示例

```python
from adapters.hardware.factory import HardwareFactory

# _sdk 方法
hw = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})
hw.initialize()
hw.send_base_velocity_sdk(vx=0.2, vy=0.0, vyaw=0.0)
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
- kuavo_humanoid_sdk 已安装
- 机器人控制器已启动（仿真或实机）

---

**最后更新**: 2026-05-30
**状态**: 29/29 脚本已实现 🔄，待完整验证
