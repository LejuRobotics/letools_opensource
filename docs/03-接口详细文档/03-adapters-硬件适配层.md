# 03 adapters 硬件适配层

> ⚠ **本篇是开发者主要调用入口。** 绝大多数机器人动作都是在 `LejuWheeledArmHardware` 这个对象上调方法完成的。

adapters 层把 core 层的标准接口翻译成具体的 ROS 话题 / 服务 / SDK 调用。开发者不需要记话题名，只需要 `HardwareFactory` 造一个对象、`initialize()`、然后调方法。

## 03.01 命名空间与类索引

| 点分路径 | 实体 | 说明 |
|---|---|---|
| `adapters.hardware.factory` | `HardwareFactory` | 按 `robot_type` 创建硬件对象（唯一推荐入口） |
| `adapters.hardware.leju_wheeled.hardware` | `LejuWheeledArmHardware` | 乐聚轮臂适配器主类，实现 `IHardware` |
| `adapters.hardware.leju_wheeled.mixins` | `lifecycle` / `base_control` / `arm_control` / `torso_control` / `end_effector` / `force_control` / `mode_service` / `sdk_control` / `state_feedback` / `timed_command` / `ik_service` / `navigation_chassis` | 按控制域拆开的能力模块（Mixin） |
| `adapters.hardware.leju_wheeled.mixins.jibot.chassis_mixin` | `JibotChassisMixin` | JiBot 协议底盘移动 |
| `adapters.hardware.leju_wheeled.mixins.jiateng` | 嘉腾底盘 Mixin | 嘉腾协议底盘移动（与 JiBot 并列） |
| `adapters.hardware.leju_wheeled.camera_adapter` | `CameraAdapter` | 相机适配器，实现 `ICamera` |
| `adapters.hardware.leju_wheeled.perception_adapter` | `PerceptionAdapter` | 感知适配器，实现 `IPerception` |
| `adapters.hardware.leju_wheeled.services.state_manager` | `StateManager` | 订阅状态话题、维护线程安全缓存 |
| `adapters.hardware.leju_wheeled.services.sdk_manager` | `BaseSDKManager` / `ArmSDKManager` / `LowLevelSDKManager` / `TimedCmdManager` | SDK 管理器（base/arm/low_level/timed_cmd） |
| `adapters.hardware.leju_wheeled.source_sdk_compat` | 兼容层 | 屏蔽不同 SDK 版本差异 |
| `adapters.vacuum_485` | `relay_pump_client` / `relay_pump_control` | 485 继电器气泵控制 |
| `adapters.vacuum_control` | `relay` / `pressure_sensor` | 继电器与压力传感器 |
| `adapters.hardware.leju_bipedal` | 乐聚足式适配器 | 待完善 |
| `adapters.hardware.mock` | — | `factory.py` 中有 `mock` 分支，但该目录不存在，使用 `robot_type: mock` 会报 `ModuleNotFoundError` |

**Adapter / Hardware / Mixin 的关系**：

```text
Adapter（适配器层，如 LejuWheeledArmHardware）
└── Hardware（具体机器人硬件对象）
    └── Mixin（按功能拆开的能力模块，多重继承组合）
```

`LejuWheeledArmHardware` 本身没有多少逻辑，它通过多重继承把各 Mixin 的方法拼装成一个完整对象；`control_head`、`send_ee_pose`、`send_timed_multi_commands` 分别来自不同的 Mixin。

## 03.02 类与方法详表

### 03.02.1 生命周期与工厂

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 工厂 | `HardwareFactory.create_hardware` | `create_hardware(config: dict) -> IHardware` | 按 `robot_type` 创建硬件对象（推荐入口）；缺 `robot_type` 抛 `ConfigurationError` | `adapters.hardware.factory` |
| 标准 | `initialize` | `initialize() -> Result` | 初始化 ROS/SDK/相机/状态管理器 | `IHardware` / `LifecycleMixin` |
| 标准 | `shutdown` | `shutdown() -> Result` | 释放连接与资源 | `IHardware` / `LifecycleMixin` |
| 查询 | `is_connected` | `is_connected -> bool` | 当前是否已连接 | `IHardware` |

支持的 `robot_type`：`leju_wheeled`（乐聚轮臂）、`leju_bipedal`（乐聚足式）、`mock`（`factory.py` 有引用但目录未创建）。`config` 还可带 `angle_unit`（`deg` / `rad`）。

```python
from adapters.hardware.factory import HardwareFactory

hw = HardwareFactory.create_hardware({"robot_type": "leju_wheeled"})
hw.initialize()
# ... 调 hw 上的方法 ...
hw.shutdown()
```

### 03.02.2 头部

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `control_head` | `control_head(yaw, pitch) -> Result` | 头部 yaw/pitch，**阻塞等待完成**，默认角度单位 deg | `SDKControlMixin` |
| SDK | `control_head_sdk` | `control_head_sdk(yaw, pitch) -> Result` | SDK 直调头部控制 | `SDKControlMixin` → `LowLevelSDKManager` |

头部 2 DOF：`yaw` 偏航（左正右负，范围约 ±80°），`pitch` 俯仰（低头为正，范围约 ±25°）。这是最简单的控制部位，适合用来验证适配器是否工作正常。

### 03.02.3 手臂

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `send_ee_pose` | `send_ee_pose(side, pose: Pose6D, frame=WORLD) -> Result` | 单臂笛卡尔末端位姿（另一臂填安全默认 x=0.1, y=±0.3, z=0.7） | `ArmControlMixin` |
| 标准 | `send_both_ee_poses` | `send_both_ee_poses(left_pose, right_pose, frame=WORLD) -> Result` | 双臂末端同时控制 → `/mm/two_arm_hand_pose_cmd` | `ArmControlMixin` |
| 标准 | `send_arm_ee_joint_space` | `send_arm_ee_joint_space(left_joints_7, right_joints_7) -> Result` | 双臂关节空间（`frame=5`，不做 IK） | `ArmControlMixin` |
| 标准 | `send_arm_joint_trajectory` | `send_arm_joint_trajectory(positions[14], time_sec=0.0) -> Result` | 双臂 14 关节轨迹，**阻塞等待到位** → `/kuavo_arm_traj` | `IHardware` / `ArmControlMixin` |
| 标准 | `publish_arm_joint_setpoint` | `publish_arm_joint_setpoint(positions_deg[14]) -> Result` | 双臂 14 关节设定点，**非阻塞**，只发布不等待到位 → `/kuavo_arm_traj` | `IHardware` / `ArmControlMixin` |
| 标准 | `send_joint_trajectory` | `send_joint_trajectory(joint_names, time_points, joint_angles_list) -> Result` | 自定义关节名/时间点的多段轨迹 | `ArmControlMixin` |
| 标准 | `arm_reset` | `arm_reset() -> Result` | 手臂归位，自动处理 MPC 模式设置与恢复 | `ArmSDKManager` |
| SDK | `send_ee_pose_sdk` | `send_ee_pose_sdk(left_pose=None, right_pose=None, frame='world') -> Result` | 单次末端位姿，**需上层约 100Hz 循环** | `SDKControlMixin` |
| SDK | `send_arm_joint_positions_sdk` | `send_arm_joint_positions_sdk(joint_angles[14]) -> Result` | 单次关节位置，需上层 100Hz 循环 | `SDKControlMixin` |
| SDK | `send_arm_ee_traj_sdk` | `send_arm_ee_traj_sdk(left_traj, right_traj, total_time=3.0, frame='world') -> Result` | 多关键点末端轨迹，自动 MPC 管理 | `ArmSDKManager.move_eef_traj_auto` |
| SDK | `send_arm_joint_traj_sdk` | `send_arm_joint_traj_sdk(joint_traj, total_time=3.0) -> Result` | 14 关节多关键点轨迹 | `SDKControlMixin` |
| Timed | `send_arm_ee_world_timed` | `send_arm_ee_world_timed(left_pose[6], right_pose[6], desire_time=3.0) -> Result` | 双臂末端世界系 TimedCmd（planner 4+5） | `TimedCommandMixin` |
| Timed | `send_arm_ee_local_timed` | `send_arm_ee_local_timed(left_pose[6], right_pose[6], desire_time=3.0) -> Result` | 双臂末端本体系 TimedCmd（planner 6+7） | `TimedCommandMixin` |
| Timed | `send_left_arm_ee_world_timed` / `send_right_arm_ee_world_timed` | `send_timed_*_arm_ee_world(pose[6], desire_time=3.0) -> Result` | 单臂末端世界系 | `TimedCommandMixin` |
| Timed | `send_left_arm_ee_local_timed` / `send_right_arm_ee_local_timed` | `send_timed_*_arm_ee_local(pose[6], desire_time=3.0) -> Result` | 单臂末端本体系 | `TimedCommandMixin` |
| Timed | `send_arm_joint_timed` | `send_arm_joint_timed(joint_angles[14], desire_time=2.0) -> Result` | 双臂 14 关节 TimedCmd | `TimedCommandMixin` |
| Timed | `send_timed_left_arm_joint` / `send_timed_right_arm_joint` | `send_timed_*_arm_joint(joint_angles[7], desire_time) -> Result` | 单臂 7 关节 TimedCmd | `IHardware` / `TimedCommandMixin` |
| Timed | `send_timed_multi_commands` | `send_timed_multi_commands(commands: List[dict], is_sync=False) -> Result` | 多规划器并发 TimedCmd，每项含 `planner_index` / `desire_time` / `cmd_vec` | `IHardware` / `TimedCommandMixin` |

`publish_arm_joint_setpoint(positions_deg)` 收 14 个关节位置，顺序为左臂 7 个、右臂 7 个，每臂依次是肩俯仰、肩横滚、肩偏航、肘俯仰、肘偏航、腕俯仰、腕横滚；单位度，没有 `time_sec` 参数。它只把这一帧目标组进 `JointState` 打到 `/kuavo_arm_traj` 就返回，不 sleep、不订阅到达时间、不等待到位。返回的 `Result.ok` 只表示 `publish()` 调用完成，不表示控制器收到该帧，也不表示关节已经到位。逐帧流式下发（如 IK 伺服）用这个方法；需要「下发并等到位」时仍用 `send_arm_joint_trajectory`。Publisher 在首次发布时懒创建，之后跨帧复用，当前实现不在初始化阶段预建。

### 03.02.4 躯干与下肢

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `send_torso_pose` | `send_torso_pose(pose: Pose6D) -> Result` | 躯干相对基座位姿 (x, z, pitch, yaw)，**阻塞等待**（忽略 pose.y / pose.roll）→ `/cmd_lb_torso_pose` | `IHardware` / `TorsoControlMixin` |
| 标准 | `get_torso_initial_pose` | `get_torso_initial_pose() -> Result` | 读躯干初始位姿（`data={position, euler}`） | `IHardware` |
| 标准 | `reset_torso_to_initial` | `reset_torso_to_initial() -> Result` | 躯干回到初始位姿 | `TorsoControlMixin` |
| 标准 | `send_leg_joint_command` | `send_leg_joint_command(positions[4]) -> Result` | 膝/腿/腰 pitch/腰 yaw 四关节（**默认 deg，不做弧度转换**）→ `/lb_leg_traj` | `IHardware` / `ArmControlMixin` |
| 标准 | `send_leg_joint_positions` | `send_leg_joint_positions(joint_names, positions) -> Result` | 内部方法，可自定义关节名 | `ArmControlMixin` |
| SDK | `send_torso_pose_sdk` | `send_torso_pose_sdk(x, z, yaw, pitch) -> Result` | 躯干位姿 SDK 单次 | `SDKControlMixin` |
| SDK | `send_torso_6dof_sdk` | `send_torso_6dof_sdk(x, y, z, roll, pitch, yaw) -> Result` | 躯干 6DoF SDK 直调 | `robot_sdk.control.control_torso_6dof` |
| SDK | `send_leg_joint_sdk` | `send_leg_joint_sdk(joint_angles[4], total_time=3.0) -> Result` | 下肢关节 SDK，自动插值与 MPC | `LowLevelSDKManager` |
| Timed | `send_torso_pose_timed` | `send_torso_pose_timed(x, z, yaw, pitch, desire_time=2.0) -> Result` | 躯干位姿 TimedCmd（planner_index=2） | `TimedCommandMixin` |
| Timed | `send_timed_torso_pose` | `send_timed_torso_pose(x, z, yaw, pitch, desire_time) -> Result` | 标准接口别名，委托 `send_torso_pose_timed` | `IHardware` |
| Timed | `send_leg_joint_timed` | `send_leg_joint_timed(joint_angles[4], desire_time=2.0) -> Result` | 躯干关节 TimedCmd（planner_index=3，服务端 Ruckig 时间同步） | `TimedCommandMixin` |
| Timed | `send_timed_leg_joint` | `send_timed_leg_joint(joint_angles[4], desire_time) -> Result` | 标准接口别名，委托 `send_leg_joint_timed` | `IHardware` |

> `send_leg_joint_command`、`send_arm_joint_trajectory` 与 `publish_arm_joint_setpoint` **不做** `_to_rad()`；`send_leg_joint_timed` / `send_leg_joint_sdk` 会内部把角度转弧度。控制器对 `/lb_leg_traj`、`/kuavo_arm_traj` 期望度数，因此即使配了 `angle_unit='rad'`，给这三个方法的仍应是度数。

### 03.02.5 底盘

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `send_base_velocity` | `send_base_velocity(vx, vy, vyaw, frame=LOCAL) -> Result` | 底盘速度（**需主动发零速停止**）→ `/cmd_vel` | `BaseControlMixin` |
| 标准 | `send_base_pose` | `send_base_pose(x, y, yaw, frame=WORLD) -> Result` | 底盘位姿目标 → `/cmd_pose*` | `BaseControlMixin` |
| 标准 | `send_base_position` | `send_base_position(x, y, yaw) -> Result` | 底盘位置（本体系便捷） | `BaseControlMixin` |
| 标准 | `send_world_position` | `send_world_position(x, y, yaw) -> Result` | 底盘世界系位置 | `BaseControlMixin` |
| 标准 | `publish_cmd_vel` | `publish_cmd_vel(linear_x, linear_y, angular_z, duration=0.0) -> Result` | 直接发布 `/cmd_vel` | `BaseControlMixin` |
| SDK | `send_base_velocity_sdk` | `send_base_velocity_sdk(vx, vy, vyaw) -> Result` | 底盘速度 SDK | `SDKControlMixin` → `LowLevelSDKManager` |
| SDK | `send_base_position_local_sdk` | `send_base_position_local_sdk(x, y, yaw) -> Result` | 底盘本体系位置 SDK | `SDKControlMixin` |
| SDK | `send_base_position_world_sdk` | `send_base_position_world_sdk(x, y, yaw) -> Result` | 底盘世界系位置 SDK | `SDKControlMixin` |
| Timed | `send_base_pose_timed` | `send_base_pose_timed(x, y, yaw, frame=WORLD, desire_time=3.0) -> Result` | 指定时长完成底盘位姿 | `TimedCommandMixin` |
| Timed | `send_timed_base_pose` | `send_timed_base_pose(x, y, yaw, desire_time, frame=WORLD) -> Result` | 标准 Timed 底盘 | `IHardware` |
| Timed | `send_base_velocity_timed` | `send_base_velocity_timed(vx, vy, vyaw, frame=LOCAL) -> Result` | TimedCmd 底盘速度 | `TimedCommandMixin` |

### 03.02.6 JiBot / 嘉腾导航

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `base_move_relative_jibot` | `base_move_relative_jibot(x, y, theta, options=None) -> Result` | JiBot 相对移动 | `JibotChassisMixin` |
| 标准 | `base_move_to_target_jibot` | `base_move_to_target_jibot(x, y, theta, options=None) -> Result` | JiBot 目标点导航 | `JibotChassisMixin` |
| 查询 | `check_arrived_jibot` | `check_arrived_jibot(task_id, blocking=True, timeout=20.0) -> Result` | 检查是否到达 | `JibotChassisMixin` |
| 标准 | `enable_vel_control_jibot` | `enable_vel_control_jibot(enable) -> Result` | 速度控制权限切换 | `JibotChassisMixin` |
| 查询 | `get_vel_control_state_jibot` | `get_vel_control_state_jibot(timeout=3.0) -> Result` | 查询速度控制状态 | `JibotChassisMixin` |
| 标准 | `send_base_cmd_vel_tick` | `send_base_cmd_vel_tick(vx, vy, wz, topic='/move_base/base_cmd_vel') -> Result` | 单帧发布底盘原生速度（`geometry_msgs/Twist`） | `JibotChassisMixin` |
| 标准 | `start_cmd_vel_stream` | `start_cmd_vel_stream(topic='/move_base/base_cmd_vel', rate_hz=100.0) -> Result` | 起后台速度流，按频率重发目标值 | `JibotChassisMixin` |
| 标准 | `update_cmd_vel_setpoint` | `update_cmd_vel_setpoint(vx, vy, wz) -> Result` | 线程安全地更新速度流的目标值 | `JibotChassisMixin` |
| 标准 | `stop_cmd_vel_stream` | `stop_cmd_vel_stream(zero_hold_s=0.2) -> Result` | 零速保持 → 停流 → 释放状态订阅 | `JibotChassisMixin` |
| 查询 | `get_motion_gate` | `get_motion_gate() -> dict` | 返回最近一条 `base_cmd_vel_status` 的事实，不判「是否允许运动」 | `JibotChassisMixin` |
| 标准 | `base_move_relative_jiateng` | `base_move_relative_jiateng(...) -> Result` | 嘉腾相对移动 | 嘉腾 Mixin |
| 标准 | `base_move_to_target_jiateng` | `base_move_to_target_jiateng(...) -> Result` | 嘉腾 map 绝对目标移动 | 嘉腾 Mixin |
| 查询 | `check_arrived_jiateng` | `check_arrived_jiateng(...) -> Result` | 嘉腾到达查询 | 嘉腾 Mixin |
| 标准 | `enable_vel_control_jiateng` | `enable_vel_control_jiateng(enable) -> Result` | 嘉腾导航/外部控制模式切换 | 嘉腾 Mixin |
| 查询 | `get_vel_control_state_jiateng` | `get_vel_control_state_jiateng(...) -> Result` | 嘉腾外部速度状态 | 嘉腾 Mixin |

`MoveToTargetOptions`（`avoid_enabled` / `avoid_distance` / `linear_velocity` / `angular_velocity` / `position_threshold` / `angle_threshold` / `allow_rotation`）作为 `options` 传入，用于覆盖避障与到达阈值。

> JiBot 与嘉腾对应同一组 `IHardware` 导航接口（`move_chassis_relative` / `move_chassis_to_target` / `check_chassis_arrived` / `set_chassis_external_control` / `get_chassis_external_control_state`），底层走 `/move_base/*` 系列话题，由 `app_config.yaml` 的 `navigation_chassis_backend`（`jibot` 等）选择。

**底盘原生速度通道**

上面五个方法操作 `/move_base/base_cmd_vel`（`geometry_msgs/Twist`，`linear.x` / `linear.y` / `angular.z`）。起流时顺带订阅 `/move_base/base_cmd_vel_status`（`leju_mobile_base_msgs/BaseCmdVelStatus`），消息包缺失或订阅失败只告警，不阻断速度通道。

- `send_base_cmd_vel_tick`：非阻塞单帧发布，流未启动也能用，供 apps 脚本与离线测试直接发一帧。
- `start_cmd_vel_stream`：起一条 daemon 线程按 `rate_hz` 重发当前目标值；已在运行时直接返回，不重复起线程。流线程异常退出前会先补一帧零速。
- `update_cmd_vel_setpoint`：只改目标值，不发布——真正的发布由流线程做。
- `stop_cmd_vel_stream`：先连续发零速保持 `zero_hold_s`，再停线程（`join(timeout=1.0)`）并释放状态订阅。未起流时是 no-op。
- `get_motion_gate`：返回最近一条状态消息的事实 `{"cmd_vel_effective", "reason", "description", "effective_age_s"}`。从未收到消息时 `cmd_vel_effective=True`、`effective_age_s=None`；`effective_age_s` 只作诊断，不参与「是否阻断」的判定，判定留给调用方。

状态只在 `start_cmd_vel_stream` 之后才会被订阅和缓存，直接用 `send_base_cmd_vel_tick` 时 `get_motion_gate` 一直返回「未收到状态」。

### 03.02.7 力控

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `set_ee_force` | `set_ee_force(side, force_kg=(0,0,0), torque=(0,0,0)) -> Result` | 末端期望力（kg，内部 ×9.8→N）→ `/desired_ee_force/*` | `ForceControlMixin` |
| 标准 | `set_ee_force_both` | `set_ee_force_both(left_force_kg, right_force_kg, ...) -> Result` | 双手分别设期望力 | `ForceControlMixin` |
| 标准 | `clear_ee_force` | `clear_ee_force(side=None) -> Result` | 清除期望力（None=双手） | `ForceControlMixin` |
| 标准 | `set_external_wrench` | `set_external_wrench(side, force_n=(0,0,0), torque=(0,0,0)) -> Result` | 仿真外力（N）→ `/external_wrench/*` | `ForceControlMixin` |
| 标准 | `clear_external_wrench` | `clear_external_wrench(side=None) -> Result` | 清除仿真外力 | `ForceControlMixin` |
| 标准 | `enable_force_empty_detect` | `enable_force_empty_detect(enable) -> Result` | 挥空检测开关 → `/enable_force_empty_detact`（latch） | `ForceControlMixin` |
| 标准 | `set_contact_force_params` | `set_contact_force_params(transition_time, interpolation_speed) -> Result` | 接触力插值参数 → `/set_contact_force_params` | `ForceControlMixin` |
| Timed | `send_arm_force_timed` | `send_arm_force_timed(force[6], desire_time=2.0) -> Result` | TimedCmd 力控 | `TimedCommandMixin` |
| Timed | `apply_arm_force_timed` | `apply_arm_force_timed(side, force_kg, enable, interpolation_speed=2000.0) -> Result` | 施加/撤销期望力（kg） | `TimedCommandMixin` |

### 03.02.8 模式切换

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `set_mpc_mode` | `set_mpc_mode(mode: MPCControlMode) -> Result` | 向 ROS 服务提交模式值；当前固件不按该值切换控制分支 | `IHardware` / `ModeServiceMixin` |
| SDK | `set_mpc_mode_sdk` | `set_mpc_mode_sdk(mode_name: str) -> Result` | 调用外部控制准备与 SDK 模式/控制流设置；模式值的控制效果见下方说明 | `SDKControlMixin` |
| 标准 | `enable_quick_mode` | `enable_quick_mode(enable: bool) -> Result` | 手臂/下肢快速模式（绕过 MPC） | `IHardware` / `ModeServiceMixin` |
| 标准 | `set_arm_quick_mode` | `set_arm_quick_mode(quick_mode: int) -> Result` | 手臂快速模式参数 | `ModeServiceMixin` |
| 标准 | `set_arm_control_mode` | `set_arm_control_mode(control_mode: int) -> Result` | 手臂控制模式切换 | `ModeServiceMixin` |
| 标准 | `set_focus_ee` / `set_focus_z` | `set_focus_ee(focus_ee=True) -> Result` | 笛卡尔 / Z 轴跟踪焦点（latch 话题） | `TorsoControlMixin` |

> **区分参数与调用**：当前固件会存储并回显 `MPCControlMode` 值，但控制主循环不按它选择分支。标准接口仍访问 `/mobile_manipulator_mpc_control`，现有流程依赖该兼容调用。`set_mpc_mode_sdk()` 还会调用外部控制准备及 SDK 的控制流设置；不能把整个方法理解成“无效”。实际控制方式应结合 `set_arm_control_mode`、快速模式及 planner 指令类型判断，见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)。

### 03.02.9 末端执行器

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `control_end_effector` | `control_end_effector(side: ArmSide, cmd: Union[GripperCommand, DualGripperCommand, HandFingerCommand, SG100HandCommand]) -> Result` | 夹爪/灵巧手统一控制 | `IHardware` / `EndEffectorMixin` → `drivers.leju` |

这是 D4 汇总表里**唯一一条**「末端执行器」API，但它的 `cmd` 参数可传四类指令，覆盖二指夹爪、双侧夹爪、灵巧手、SG100 手：

- **`GripperCommand`** → `LejuEndEffector.send_command`，走 ROS 服务 `/control_robot_leju_claw`；`side` 可为 LEFT/RIGHT/BOTH，BOTH 会在一次请求里给两侧发相同位置。

- **`DualGripperCommand`** → 与 `side=ArmSide.BOTH` 配合，一次请求发不同左右位置；位置为 `None` 的一侧会用当前夹爪位置反馈补齐，请求仍包含左右两侧。需要补齐但未取得反馈时返回失败；左右位置均为 `None` 时也返回失败。

- **`HandFingerCommand`** → `LejuEndEffector.send_hand_command(left, right)`，走 ROS 话题 `/control_robot_hand_position`；当前实现始终同时下发左右手。传 `side=ArmSide.LEFT` 时右侧填默认全零指令，传 `side=ArmSide.RIGHT` 时左侧填默认全零指令，另一侧不会保持原位；传 side=ArmSide.BOTH 时，左右两手均使用传入的 HandFingerCommand 指令。



- **`SG100HandCommand`** → SG100 灵巧手（11 关节），`enable_mask=0x07FF` 表示全部使能。

`side` 应传 `ArmSide` 枚举，`cmd` 应传上述命令对象；接口不支持 `action` 参数。例如，已初始化且配置为乐聚二指夹爪的 `hw` 可这样控制左夹爪闭合：

```python
from core.domain.enums import ArmSide
from core.domain.end_effector import GripperCommand

result = hw.control_end_effector(
    side=ArmSide.LEFT,
    cmd=GripperCommand(position=100.0),  # 0 为张开，100 为闭合
)
```

真空吸盘类末端不在此接口内，走 `adapters/vacuum_485`（`relay_pump_client.suck()` / `blow()` / `power_off()`）或 `adapters/vacuum_control`（`control_vacuum_pump(relay_index, action, duration)`、`read_pressure_kpa(sensor_id)`）。

### 03.02.10 规划 / IK / Rucking

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Timed | `set_ruckig_planner_params` | `set_ruckig_planner_params(planner_index, is_sync, velocity_max, acceleration_max, jerk_max, velocity_min=None, acceleration_min=None) -> Result` | Ruckig 速度/加速度/急动度限制 | `IHardware` / `TimedCommandMixin` |
| Timed | `set_offline_trajectory` | `set_offline_trajectory(trajectories: List[dict]) -> Result` | 预置离线定时轨迹（首帧 `desire_time` 必须为 0，时间严格递增） | `IHardware` |
| Timed | `enable_offline_trajectory` | `enable_offline_trajectory(enable: bool) -> Result` | 启用/禁用离线轨迹执行 | `IHardware` |
| Timed | `check_ik_accessibility` | `check_ik_accessibility(is_left, is_local, is_whole_body, pose_desired[6], total_time_desired=1.0, max_attempts=5, linear_error_max=0.005, angular_error_max=0.05) -> Result` | IK 可达性检查 | `IHardware` → `/mobile_manipulator_ik_accessibility_check` |
| Timed | `check_ik_accessibility_timed` | `check_ik_accessibility_timed(...) -> Result` | IK 检查的 timed 别名 | `TimedCommandMixin` |
| IK | `solve_ik` | `solve_ik(request: IkSolveRequest) -> Result` | 双臂 IK 求解，位姿为 IK 模型根坐标系（轮臂为 `waist_yaw_link`）下的四元数位姿；`data` 为 `core.domain.ik.IkSolution` | `IHardware`（`IkServiceMixin`） |
| 查询 | `get_transform` | `get_transform(target_frame, source_frame, timeout=0.2) -> Result` | TF 查询，`data` 为 `core.domain.transform.Transform`（位置米 + 四元数） | `IHardware`（`IkServiceMixin`） |
| 查询 | `check_joint_path` | `check_joint_path(start_q14, goal_q14, sampling_deg=3.0, timeout=5.0) -> Result` | 14 关节直线插值路径的 MoveIt 碰撞校验；关节角为**弧度** | `IHardware`（`IkServiceMixin`） |
| 查询 | `get_arm_joint_positions` | `get_arm_joint_positions() -> Result` | 当前 14 个手臂关节角，单位**弧度**（不随 `angle_unit` 变化） | `IHardware`（`SDKControlMixin`） |

`check_ik_accessibility` 的 `pose_desired` 顺序是 `[x, y, z, roll, pitch, yaw]`（注意与 `Pose6D.to_list()` 的 `yaw, pitch, roll` 不同）；`Result.data` 含 `success` / `best_linear_error` / `best_angular_error` / `q_best` 等字段。

**`TimedCmd` 规划器索引**（`planner_index` / `cmd_vec` 维度）：

| index | 规划器 | 维度 |
|---|---|---|
| 0 | 底盘世界系位置 | 3 (x, y, yaw) |
| 1 | 底盘本体系位置 / 速度 | 3 (x, y, yaw) |
| 2 | 躯干笛卡尔（局部系） | 4 (x, z, yaw, pitch) |
| 3 | 下肢关节 | 4 |
| 4 | 左臂笛卡尔世界系 | 6 |
| 5 | 右臂笛卡尔世界系 | 6 |
| 6 | 左臂笛卡尔本体系 | 6 |
| 7 | 右臂笛卡尔本体系 | 6 |
| 8 | 左臂上肢关节 | 7 |
| 9 | 右臂上肢关节 | 7 |

### 03.02.11 状态反馈

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 查询 | `get_reach_time` | `get_reach_time(topic_type) -> Optional[float]` | 预计到达时间；`topic_type` ∈ `cmd_pose` / `torso_pose` / `arm_joint` / `leg_joint` / `arm_ee` | `StateFeedbackMixin` |
| 查询 | `get_ee_poses` | `get_ee_poses() -> Optional[Dict]` | 末端实时位姿反馈 | `StateManager` 缓存 |
| 查询 | `get_ee_target_6d` | `get_ee_target_6d() -> Optional[List[Dict]]` | 末端目标 6D（指令目标） | `StateFeedbackMixin` |
| 查询 | `get_arm_joint_positions` | `get_arm_joint_positions() -> Result` | 当前 14 臂关节角 | `StateFeedbackMixin` |
| 查询 | `get_mpc_observation` | `get_mpc_observation() -> Optional[Dict]` | MPC 观测状态 | `IHardware` |
| 查询 | `get_mpc_control_mode` | `get_mpc_control_mode() -> Optional[int]` | 当前 MPC 模式枚举值 | `IHardware` |
| 查询 | `get_joint_torque` | `get_joint_torque() -> Optional[Dict]` | 关节力矩 | `IHardware` |
| 查询 | `get_body_acceleration` | `get_body_acceleration() -> Optional[Dict]` | 本体加速度 | `IHardware` |
| 查询 | `get_joint_acc` | `get_joint_acc() -> Optional[Dict]` | 关节加速度 | `StateFeedbackMixin` |
| 查询 | `get_wbc_observation` | `get_wbc_observation() -> Optional[Dict]` | WBC 观测 | `StateFeedbackMixin` |
| 查询 | `get_mpc_target_input` / `get_mpc_target_state` | `get_mpc_target_*() -> Optional[Dict]` | MPC 目标输入/状态 | `StateFeedbackMixin` |
| 查询 | `get_optimized_state_mrt` / `get_optimized_state_kinemic` / `get_optimized_input_mrt` / `get_optimized_input_kinemic` | `get_optimized_*() -> ...` | 优化器输出 | `StateFeedbackMixin` |
| 查询 | `get_torso_target_6d` | `get_torso_target_6d() -> Optional[...]` | 躯干目标 6D | `StateFeedbackMixin` |
| 查询 | `get_ee_pose_reach_error` | `get_ee_pose_reach_error(is_left: bool) -> Result` | 末端位姿静差（6 维 `err_vector`） | `IHardware` |
| 查询 | `robot_sdk` | `robot_sdk -> RobotSDK` | **已弃用**：底层 RobotSDK（property），仅为存量调用保留。新代码改用 `get_arm_joint_positions` / `solve_ik` / `get_transform`（`IHardware` 契约方法） | `SDKControlMixin` → `ArmSDKManager` |

### 03.02.12 相机

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `initialize` | `initialize(config: dict) -> Result` | 初始化相机适配器 | `ICamera` / `CameraAdapter` |
| 标准 | `start_camera` / `stop_camera` | `start_camera / stop_camera(camera_name="camera") -> bool` | 设置相机内部运行标记 | `CameraAdapter` |
| 查询 | `get_camera_frame` | `get_camera_frame(camera_name='camera') -> Optional[CameraFrame]` | RGB(+深度) 帧 | `ICamera` |
| 查询 | `get_synchronized_camera_frame` | `get_synchronized_camera_frame(camera_name='camera') -> Optional[CameraFrame]` | 按原始时间戳配对的 RGBD 帧（可选） | `ICamera` |
| 查询 | `wait_for_next_synchronized_camera_frame` | `wait_for_next_synchronized_camera_frame(camera_name='camera', timeout_sec=2.0) -> Result` | 等下一组同步 RGBD 帧 | `ICamera` |
| 查询 | `get_depth_data` | `get_depth_data(camera_name='camera') -> Optional[DepthData]` | 深度图与内参 | `ICamera` |
| 查询 | `get_point_cloud` | `get_point_cloud(camera_name='camera') -> Optional[PointCloudData]` | 点云 | `ICamera` |
| 查询 | `get_camera_info` / `get_camera_status` | `get_camera_info / status(name='camera') -> Optional[...]` | 相机参数与运行状态 | `ICamera` |
| 查询 | `is_connected` | `is_connected -> bool` | 适配器是否已连接初始化 | `CameraAdapter` |
| 查询 | `check_health` / `get_performance_metrics` | `check_health / get_performance_metrics(name='camera') -> dict` | 健康与 FPS 等指标 | `CameraAdapter` |
| 标准 | `shutdown` | `shutdown() -> Result` | 关闭相机适配器并释放资源 | `CameraAdapter` |

相机类型与分辨率由 `config/camera_config.yaml` 决定（头部 Orbbec / 手腕 RealSense）。

### 03.02.13 感知

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `initialize` | `initialize(camera: ICamera, config=None) -> bool` | 注入相机并初始化感知 | `IPerception` / `PerceptionAdapter` |
| 查询 | `get_tag_detections` | `get_tag_detections() -> List[TagDetection]` | AprilTag 检测结果 | `PerceptionAdapter` |
| 查询 | `get_object_detections` | `get_object_detections() -> List[ObjectDetection]` | 通用目标检测（预留） | `IPerception` |
| 查询 | `get_latest_result` | `get_latest_result() -> Optional[PerceptionResult]` | 最近一次完整感知结果 | `IPerception` |
| 查询 | `get_camera_frame` / `get_point_cloud` / `get_depth_data` / `get_camera_info` / `get_camera_status` | 同 `ICamera` | 委托注入的 `ICamera` | `IPerception` |

### 03.02.14 SDK 管理器

| 类 | 位置 | 职责 |
|---|---|---|
| `BaseSDKManager` | `services/sdk_manager/base_sdk_manager.py` | 管理器基类：连接、重连、通用调用 |
| `ArmSDKManager` | `services/sdk_manager/arm_sdk_manager.py` | 手臂相关：`arm_reset`、`move_eef_traj_auto`、`robot_sdk` |
| `LowLevelSDKManager` | `services/sdk_manager/low_level_sdk_manager.py` | 底层单次：头部、下肢、底盘位置/速度 |
| `TimedCmdManager` | `services/sdk_manager/timed_cmd_manager.py` | TimedCmd 与 Ruckig 规划 |
| `StateManager` | `services/state_manager.py` | 订阅状态话题、线程安全缓存、更新回调 |

## 03.03 参数说明

### 角度单位

- 默认**度（deg）**；创建时配 `angle_unit='rad'` 改为弧度。内部统一走 `_to_rad()`。
- **恒为度的例外**：`send_arm_joint_trajectory`、`send_leg_joint_command`、`publish_arm_joint_setpoint` 直接下发角度，不受 `angle_unit` 影响，始终传**度**。
- **`Pose6D` 例外**：其姿态字段**始终弧度**，与 `angle_unit` 无关。
- 关节序列：手臂 14 关节顺序 `[左肩俯仰, 左肩侧摆, 左肩偏航, 左肘俯仰, 左肘偏航, 左腕俯仰, 左腕滚转, 右臂同序 7 个]`（对应源码关节名 `*_shoulder_pitch/roll/yaw`、`*_elbow_pitch/yaw`、`*_wrist_pitch/roll`）；下肢 4 关节顺序 `[knee_joint, leg_joint, waist_pitch_joint, waist_yaw_joint]`。

### 关键默认值

| 参数 | 默认 | 备注 |
|---|---|---|
| `desire_time`（Timed 类） | `2.0`~`3.0`，随方法而异 | 期望执行时长（秒） |
| `total_time`（SDK 轨迹类） | `3.0` | 自动插值总时长 |
| `timeout`（JiBot 导航） | `20.0`（到达查询）/ `3.0`（状态查询） | 秒 |
| 单臂控制时另一臂 | `x=0.1, y=±0.3, z=0.7, yaw/pitch/roll=0` | 安全默认位姿 |
| `force_kg` 换算 | `×9.8` → N | 期望力内部换算 |

### 阻塞语义

标准接口（无后缀）大多**阻塞等待运动完成**：`send_ee_pose` 首次调用还会自动等待 1.0s 建立 Publisher 连接；`send_torso_pose` 订阅 `/lb_torso_pose_reach_time` 拿到达时间后阻塞。`_sdk` 方法**不**阻塞——只下发单次指令，需要上层约 100Hz 循环自行维持。`publish_arm_joint_setpoint` 是标准接口里的例外：只发布设定点就返回，不等到位。

## 03.04 返回值

- **`Result`**：动作类与部分查询类（`get_torso_initial_pose`、`get_arm_joint_positions`、`get_ee_pose_reach_error`）。`get_ee_pose_reach_error` 的 `success` 仅在末端模式且期望位姿不再更新时为 `True`，`data['err_vector']` 是 6 维 `[x, y, z, yaw, pitch, roll]`（位置米、姿态弧度）。
- **`Optional[...]`**：`get_ee_poses` / `get_mpc_observation` / `get_ee_target_6d` / `get_camera_frame` / `get_depth_data` / `get_point_cloud` / `get_camera_info` / `get_camera_status`。**调用前判空**。
- **`bool`**：`is_connected`、`start_camera`、`stop_camera`。

`Result` 的结构与判定写法见 `01-阅读说明.md`。

## 03.05 异常

| 异常 | 触发条件 |
|---|---|
| `ConfigurationError` | `create_hardware` 缺 `robot_type` 或 `robot_type` 不受支持 |
| `HardwareConnectionError` | 连接失败/断开（如 `initialize` 阶段） |
| `KinematicsError` | 逆解失败 |
| `Result.fail` | SDK 管理器未初始化（如 `TimedCommandMixin` 返回 `"TimedCmdManager 未初始化"`）、硬件 `Result` 失败 |

多数运行时问题通过 `Result.success=False` 返回，而不是抛异常；查询类在无数据时返回 `None`。

## 03.06 调用示例

```python
from adapters.hardware.factory import HardwareFactory
from core.domain.pose import Pose6D
from core.domain.enums import ArmSide, FrameType
from core.domain.end_effector import GripperCommand, HandFingerCommand

# ① 造对象
hw = HardwareFactory.create_hardware({"robot_type": "leju_wheeled"})
hw.initialize()

# ② 头部
hw.control_head(yaw=10, pitch=5)

# ③ 手臂末端（标准接口，Pose6D 用弧度）
hw.send_ee_pose(side=ArmSide.LEFT,
                pose=Pose6D(x=0.3, y=0.2, z=0.8, yaw=0.0, pitch=0.0, roll=0.0),
                frame=FrameType.WORLD)

# ④ 手臂关节轨迹（标准接口；角度恒为度，不受 angle_unit 影响）
#    time_sec 不决定运动时长，仅在收不到到达时间反馈时作为兜底等待时间
hw.send_arm_joint_trajectory([0]*14, time_sec=3.0)

# ⑤ TimedCmd：3 秒内双臂末端到位（列表用度）
hw.send_arm_ee_world_timed(
    left_pose=[0.3, 0.25, 0.5, 0, -90, 0],
    right_pose=[0.3, -0.25, 0.5, 0, -90, 0],
    desire_time=3.0)

# ⑥ 夹爪 / 灵巧手
hw.control_end_effector(side=ArmSide.LEFT, cmd=GripperCommand(position=100.0))
hw.control_end_effector(side=ArmSide.LEFT, cmd=HandFingerCommand(positions=[50]*6))

# ⑦ 查询（先判空）
poses = hw.get_ee_poses()
if poses:
    print(poses)

# ⑧ 收尾
hw.shutdown()
```

## 03.07 分发形式说明

本模块在**开源产物中为 `.pyc` 分发**：

- 编译后的 `.pyc` 被移到对应模块目录；构建过程产生的 `__pycache__/` 会被清理。
- 因此**不提供自动生成式 API 文档**，本篇为**人工维护版本**（依据仓库内源码接口与两份 API 参考整理）。
- 发布时只编译闭源清单命中的 `adapters/`；具体 `.pyc` 路径与原 `.py` 模块路径对应。

> 相关：面向读者的扩展规范见 `05-进阶开发指南/03-扩展-adapters-层.md`；三条控制路径（标准 / SDK / Timed）的选型见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)；类型与枚举定义见 [`02-core-接口与数据模型.md`](./02-core-接口与数据模型.md)。
