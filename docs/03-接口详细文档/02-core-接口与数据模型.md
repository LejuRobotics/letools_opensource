# 02 core 接口与数据模型

core 层是 LeTools 的**共同语言层**：它定义接口、领域数据模型和枚举，本身不直接控制机器人，也不依赖 ROS / SDK。

> **读法**：先看 `02.01` 的类索引建立全景，再按需查 `02.02` 的方法详表。`02.03`–`02.05` 是参数的坑（单位、枚举、返回形态、异常），`02.06` 汇总可复制示例。

## 02.01 命名空间与类索引

| 点分路径 | 实体 | 说明 |
|---|---|---|
| `core.interfaces.i_hardware` | `IHardware` | 硬件抽象接口（所有 Adapter 必须实现） |
| `core.interfaces.i_skill` | `ISkill` | 技能接口（生命周期契约） |
| `core.interfaces.i_camera` | `ICamera` | 相机接口 |
| `core.interfaces.i_perception` | `IPerception` | 感知接口（依赖注入 `ICamera`） |
| `core.domain.result` | `Result` | 统一操作结果封装 |
| `core.domain.pose` | `Pose6D`、`TransformMatrix` | 6D 位姿与 4×4 齐次变换矩阵 |
| `core.domain.enums` | `FrameType`、`MPCControlMode`、`ArmSide` | 坐标帧 / MPC 模式 / 手臂侧别 |
| `core.domain.skill_params` | `SkillParams`、`MoveToPoseParams`、`PickObjectParams` | 技能参数基类与常用子类 |
| `core.domain.end_effector` | `GripperCommand`、`DualGripperCommand`、`HandFingerCommand`、`SG100HandCommand`、`EndEffectorType` | 末端执行器指令与类型 |
| `core.domain.chassis_options` | `MoveToTargetOptions` | 底盘导航选项 |
| `core.domain.ruckig_params` | `RuckigParams` | Ruckig 在线轨迹规划器参数 |
| `core.domain.camera` | `CameraInfo`、`CameraFrame`、`DepthData`、`PointCloudData`、`CameraStatus`、`CameraIntrinsics`、`CameraExtrinsics` | 相机数据模型 |
| `core.domain.perception` | `TagDetection`、`ObjectDetection`、`PerceptionResult` | 感知数据模型 |
| `core.domain.joint_state` / `observation` / `tag` / `task` / `trajectory` | 关节状态、观测、标签、任务、轨迹模型 | 其余领域模型 |

## 02.02 类与方法详表

### `IHardware` —— 硬件抽象接口

`core.interfaces.i_hardware.IHardware` 是抽象基类（`ABC`）。它规定的不是「机器人有什么」，而是「Adapter 必须提供哪些方法」。方法分十组：

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `initialize` | `initialize() -> Result` | 初始化连接（ROS 节点 / SDK / 相机 / 状态管理器） | `IHardware` 抽象 |
| 标准 | `shutdown` | `shutdown() -> Result` | 断开连接并释放资源 | `IHardware` 抽象 |
| 查询 | `is_connected` | `is_connected -> bool` | 当前是否已连接（property） | `IHardware` 抽象 |
| 标准 | `send_base_velocity` | `send_base_velocity(vx, vy, vyaw, frame=LOCAL) -> Result` | 底盘速度控制 | `IHardware` 抽象 |
| 标准 | `send_base_pose` | `send_base_pose(x, y, yaw, frame=WORLD) -> Result` | 底盘位置控制 | `IHardware` 抽象 |
| 标准 | `move_chassis_relative` | `move_chassis_relative(x, y, theta, options=None) -> Result` | 本体系相对导航（可选能力，默认 `fail`） | `IHardware` 抽象 |
| 标准 | `move_chassis_to_target` | `move_chassis_to_target(x, y, theta, options=None) -> Result` | 地图系绝对导航（可选能力） | `IHardware` 抽象 |
| 标准 | `check_chassis_arrived` | `check_chassis_arrived(task_id, blocking=True, timeout=20.0) -> Result` | 导航到达查询 | `IHardware` 抽象 |
| 标准 | `set_chassis_external_control` | `set_chassis_external_control(enable) -> Result` | 导航 / 外部控制权切换 | `IHardware` 抽象 |
| 查询 | `get_chassis_external_control_state` | `get_chassis_external_control_state(timeout=3.0) -> Result` | 读当前控制权 | `IHardware` 抽象 |
| 标准 | `send_torso_pose` | `send_torso_pose(pose: Pose6D) -> Result` | 躯干相对基座位姿 (x, z, pitch, yaw) | `IHardware` 抽象 |
| 标准 | `get_torso_initial_pose` | `get_torso_initial_pose() -> Result` | 读躯干初始位姿（`data={position, euler}`） | `IHardware` 抽象 |
| 标准 | `send_ee_pose` | `send_ee_pose(side, pose: Pose6D, frame=WORLD) -> Result` | 单臂末端位姿 | `IHardware` 抽象 |
| 标准 | `send_both_ee_poses` | `send_both_ee_poses(left_pose, right_pose, frame=WORLD) -> Result` | 双臂末端同时控制 | `IHardware` 抽象 |
| 标准 | `send_arm_joint_trajectory` | `send_arm_joint_trajectory(positions[14], time_sec=0.0) -> Result` | 双臂 14 关节轨迹，阻塞等待到位 | `IHardware` 抽象 |
| 标准 | `publish_arm_joint_setpoint` | `publish_arm_joint_setpoint(positions_deg[14]) -> Result` | 双臂 14 关节设定点（左 7 右 7，单位度，无 `time_sec`），**非阻塞**：只发布设定点，不 sleep、不等到位；`success` 仅表示 publish 调用完成 | `IHardware` 抽象 |
| 查询 | `get_arm_joint_positions` | `get_arm_joint_positions() -> Result` | 当前 14 个手臂关节角，`data` 为 14 维列表，单位**弧度**（不随 `angle_unit` 变化） | `IHardware` 抽象 |
| 标准 | `send_leg_joint_command` | `send_leg_joint_command(positions[4]) -> Result` | 腿/躯干 4 关节 | `IHardware` 抽象 |
| Timed | `send_timed_base_pose` | `send_timed_base_pose(x, y, yaw, desire_time, frame=WORLD) -> Result` | 指定时长底盘位姿 | `IHardware` 抽象 |
| Timed | `send_timed_torso_pose` | `send_timed_torso_pose(x, z, yaw, pitch, desire_time) -> Result` | 指定时长躯干位姿 | `IHardware` 抽象 |
| Timed | `send_timed_leg_joint` | `send_timed_leg_joint(joint_angles[4], desire_time) -> Result` | 指定时长腿关节 | `IHardware` 抽象 |
| Timed | `send_timed_left_arm_joint` / `send_timed_right_arm_joint` | `send_timed_*_arm_joint(joint_angles[7], desire_time) -> Result` | 单臂 7 关节时序 | `IHardware` 抽象 |
| Timed | `send_timed_left_arm_ee_world` / `send_timed_right_arm_ee_world` / `send_timed_left_arm_ee_local` / `send_timed_right_arm_ee_local` | `send_timed_*_arm_ee_{world,local}(pose[6], desire_time=3.0) -> Result` | 单臂末端位姿 `[x, y, z, yaw, pitch, roll]`（planner_index 4/5/6/7；位置米，姿态按 `angle_unit`） | `IHardware` 抽象 |
| Timed | `send_timed_multi_commands` | `send_timed_multi_commands(commands: List[dict], is_sync=False) -> Result` | 多规划器并发（含 `planner_index`/`desire_time`/`cmd_vec`） | `IHardware` 抽象 |
| Timed | `set_ruckig_planner_params` | `set_ruckig_planner_params(planner_index, is_sync, vel/acc/jerk_max, ...) -> Result` | Ruckig 限值设置 | `IHardware` 抽象 |
| Timed | `set_offline_trajectory` / `enable_offline_trajectory` | `set_offline_trajectory(trajectories: List[dict]) -> Result` | 预置 / 启停离线定时轨迹 | `IHardware` 抽象 |
| Timed | `check_ik_accessibility` | `check_ik_accessibility(is_left, is_local, is_whole_body, pose_desired[6], ...) -> Result` | IK 可达性检查 | `IHardware` 抽象 |
| IK | `solve_ik` | `solve_ik(request: IkSolveRequest) -> Result` | 双臂 IK 求解（一次求左右臂），`data` 为 `core.domain.ik.IkSolution` | `IHardware` 抽象 |
| 查询 | `get_transform` | `get_transform(target_frame, source_frame, timeout=0.2) -> Result` | TF 位姿查询，`data` 为 `core.domain.transform.Transform` | `IHardware` 抽象 |
| 查询 | `check_joint_path` | `check_joint_path(start_q14, goal_q14, sampling_deg=3.0, timeout=5.0) -> Result` | 14 关节直线插值路径的碰撞校验（关节角为弧度） | `IHardware` 抽象 |
| 标准 | `set_mpc_mode` | `set_mpc_mode(mode: MPCControlMode) -> Result` | 向 ROS 服务提交模式值；当前固件不据此切换控制分支（见下方说明） | `IHardware` 抽象 |
| 标准 | `enable_quick_mode` | `enable_quick_mode(enable: bool) -> Result` | 手臂/下肢快速模式（绕过 MPC） | `IHardware` 抽象 |
| 标准 | `set_focus_ee` / `set_focus_z` | `set_focus_ee(focus_ee=True) -> Result` | 笛卡尔跟踪焦点 latch | `IHardware` 抽象 |
| 标准 | `control_end_effector` | `control_end_effector(side, cmd) -> Result` | 夹爪 / 灵巧手统一控制 | `IHardware` 抽象 |
| 标准 | `control_head` | `control_head(yaw, pitch) -> Result` | 头部姿态 | `IHardware` 抽象 |
| 标准 | `arm_reset` | `arm_reset() -> Result` | 手臂归位 | `IHardware` 抽象 |
| 标准 | `set_ee_force` / `set_ee_force_both` / `clear_ee_force` | `set_ee_force(side, force_kg=(0,0,0), torque=(0,0,0)) -> Result` | 末端期望力（内部 kg×9.8→N） | `IHardware` 抽象 |
| 标准 | `set_external_wrench` / `clear_external_wrench` | `set_external_wrench(side, force_n=(0,0,0), torque=(0,0,0)) -> Result` | 仿真外力（N） | `IHardware` 抽象 |
| 标准 | `enable_force_empty_detect` | `enable_force_empty_detect(enable) -> Result` | 挥空检测开关 | `IHardware` 抽象 |
| 标准 | `set_contact_force_params` | `set_contact_force_params(transition_time, interpolation_speed) -> Result` | 接触力插值参数 | `IHardware` 抽象 |
| 查询 | `get_reach_time` | `get_reach_time(topic_type: str) -> Optional[float]` | 指令预计到达时间 | `IHardware` 抽象 |
| 查询 | `get_mpc_observation` / `get_mpc_control_mode_cached` / `get_body_acceleration` / `get_joint_torque` / `get_ee_poses` | `get_*() -> Optional[...]` | 状态反馈 | `IHardware` 抽象 |
| 查询 | `get_ee_pose_reach_error` | `get_ee_pose_reach_error(is_left: bool) -> Result` | 可选兼容接口；ROS1 已退役，默认返回 `Result.fail` 表示不支持；ROS2 实现保持不变 | `IHardware` 默认实现（非抽象） |

> **实现新适配器**：当前 `IHardware` 共有 50 个 `@abstractmethod`（以 `core/interfaces/i_hardware.py` 为准），漏实现任一抽象方法都会在实例化时抛 `TypeError`。`get_ee_pose_reach_error` 已改为非抽象默认失败实现，适配器不必覆写。

> **「可选能力」的含义**：`move_chassis_relative` 等五个导航方法在 `IHardware` 里给的是**默认失败实现**（`return Result.fail("当前 Hardware Adapter 不支持底盘相对导航")`），而不是 `@abstractmethod`。没有导航底盘的 Adapter 不必实现它们。

> **`set_mpc_mode` 的当前固件行为**：服务会接收并回显 `MPCControlMode` 值，但控制主循环不按该值选择分支。调用本身仍会访问 ROS 服务，现有流程保留它用于兼容与软暂停守卫；不能凭返回成功判断控制分支已经切换。新流程应按目标能力使用 `enable_quick_mode` / `set_arm_quick_mode` / `set_arm_control_mode`，详见 `09-控制路径与选型.md`。

### `ISkill` —— 技能接口

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Skill | `name` | `name -> str` | 技能唯一标识（property） | `core.interfaces.i_skill` |
| Skill | `initialize` | `initialize(params: SkillParams) -> Result` | 注入并校验参数 | `core.interfaces.i_skill` |
| Skill | `execute` | `execute() -> Result` | 执行核心逻辑（可被多次 tick 调用） | `core.interfaces.i_skill` |
| Skill | `cancel` | `cancel() -> Result` | 中断技能，确保进入安全状态 | `core.interfaces.i_skill` |
| Skill | `is_finished` | `is_finished() -> bool` | 是否已结束（无论成功或失败） | `core.interfaces.i_skill` |

`ISkill` 是 `ABC`，五个成员全部 `@abstractmethod`。实际编写技能时通常继承 `skills.base.skill_base.SkillBase`（它实现了 `ISkill` 并额外提供超时检查），详见 `04-skills-原子技能.md`。

### `ICamera` / `IPerception`

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| 标准 | `initialize` | `ICamera.initialize(config: dict) -> Result` | 初始化相机 | `core.interfaces.i_camera` |
| 标准 | `shutdown` | `ICamera.shutdown() -> Result` | 关闭相机 | `core.interfaces.i_camera` |
| 查询 | `is_connected` | `ICamera.is_connected() -> bool` | 是否连接 | `core.interfaces.i_camera` |
| 查询 | `get_camera_frame` | `get_camera_frame(camera_name="camera") -> Optional[CameraFrame]` | RGB uint8 HWC + 深度 | `core.interfaces.i_camera` |
| 查询 | `get_synchronized_camera_frame` | `get_synchronized_camera_frame(camera_name="camera") -> Optional[CameraFrame]` | 时间戳配对的 RGBD（可选能力，默认 `None`） | `core.interfaces.i_camera` |
| 查询 | `wait_for_next_synchronized_camera_frame` | `wait_for_next_synchronized_camera_frame(camera_name="camera", timeout_sec=2.0) -> Result` | 等下一组同步 RGBD（不支持时 `fail(error_code="RGBD_SYNC_UNSUPPORTED")`） | `core.interfaces.i_camera` |
| 查询 | `get_depth_data` / `get_point_cloud` / `get_camera_info` / `get_camera_status` | `get_*(camera_name="camera") -> Optional[...]` | 深度 / 点云 / 参数 / 状态 | `core.interfaces.i_camera` |
| 标准 | `start_camera` / `stop_camera` | `start_camera(camera_name="camera") -> bool` | 启停指定相机 | `core.interfaces.i_camera` |
| 标准 | `initialize` | `IPerception.initialize(camera: ICamera, config=None) -> bool` | 注入相机并初始化感知 | `core.interfaces.i_perception` |
| 标准 | `shutdown` | `IPerception.shutdown() -> None` | 释放感知资源 | `core.interfaces.i_perception` |
| 查询 | `get_tag_detections` | `get_tag_detections() -> List[TagDetection]` | AprilTag 检测结果 | `core.interfaces.i_perception` |
| 查询 | `get_object_detections` | `get_object_detections() -> List[ObjectDetection]` | 通用目标检测（预留，当前返回空） | `core.interfaces.i_perception` |
| 查询 | `get_latest_result` | `get_latest_result() -> Optional[PerceptionResult]` | 最近一次完整感知结果 | `core.interfaces.i_perception` |
| 查询 | `get_camera_frame` / `get_point_cloud` / `get_depth_data` / `get_camera_info` / `get_camera_status` | 同 `ICamera` 同名方法 | 委托给注入的 `ICamera` | `core.interfaces.i_perception` |

### 领域数据模型

| 路径 | API | 签名 | 说明 | 所属层 |
|---|---|---|---|---|
| Core | `Result` | `Result(success, message, data=None, error_code=None)` | 统一返回；工厂 `Result.ok()` / `Result.fail()` | `core.domain.result` |
| Core | `Pose6D` | `Pose6D(x=0, y=0, z=0, yaw=0, pitch=0, roll=0)` | 6D 位姿；位置米、姿态**始终弧度** | `core.domain.pose` |
| Core | `TransformMatrix` | `TransformMatrix(matrix: np.ndarray)` | 4×4 齐次变换矩阵，非 4×4 抛 `ValueError` | `core.domain.pose` |
| Core | `SkillParams` | `SkillParams(skill_name="", timeout=30.0, retry_count=3, extra_kwargs={})` | 技能参数基类 | `core.domain.skill_params` |
| Core | `MoveToPoseParams` | 继承 `SkillParams`，加 `target_pose` / `speed_factor=0.5` / `tolerance_pos=0.01` | 移动到指定位姿 | `core.domain.skill_params` |
| Core | `PickObjectParams` | 继承 `SkillParams`，加 `object_label="box"` / `approach_distance=0.1` / `gripper_force=10.0` | 抓取物体 | `core.domain.skill_params` |
| Core | `GripperCommand` | `GripperCommand(position=0.0, velocity=50.0, effort=1.0)` | 二指夹爪：位置 [0,100]（0 张开 / 100 闭合） | `core.domain.end_effector` |
| Core | `DualGripperCommand` | `DualGripperCommand(left_position=None, right_position=None, velocity=50.0, effort=1.0)` | 双侧夹爪单次请求给不同位置 | `core.domain.end_effector` |
| Core | `HandFingerCommand` | `HandFingerCommand(positions=[0.0]*6)` | 灵巧手 6 指，各 [0,100] | `core.domain.end_effector` |
| Core | `MoveToTargetOptions` | `MoveToTargetOptions(avoid_enabled=False, avoid_distance=0.5, linear_velocity=0.15, angular_velocity=0.25, position_threshold=0.08, angle_threshold=0.1, allow_rotation=True)` | 底盘导航选项 | `core.domain.chassis_options` |
| Core | `RuckigParams` | `RuckigParams(velocity_max, acceleration_max, jerk_max, velocity_min=None, acceleration_min=None)`；工厂 `create_chassis_params` / `create_arm_joint_params` / `create_ee_cartesian_params` | 在线轨迹规划限值 | `core.domain.ruckig_params` |
| Core | `TagDetection` | `TagDetection(tag_id, pose_in_world, pose_in_camera=None, size=0.0, confidence=1.0, timestamp=0.0, corner_points=None, frame_id="base_link")` | AprilTag 检测 | `core.domain.perception` |
| Core | `ObjectDetection` | `ObjectDetection(label, confidence, bbox=None, pose_in_camera=None)` | 通用物体检测 | `core.domain.perception` |
| Core | `PerceptionResult` | `PerceptionResult(success, objects=[], tags=[], message="")` | 感知统一返回 | `core.domain.perception` |
| Core | `CameraFrame` / `DepthData` / `PointCloudData` / `CameraInfo` / `CameraStatus` / `CameraIntrinsics` / `CameraExtrinsics` | 相机数据模型 | 帧 / 深度 / 点云 / 参数 / 状态 / 内参 / 外参 | `core.domain.camera` |

## 02.03 参数说明

### `Pose6D` 的单位与顺序（最易错）

- 位置 `x/y/z`：**米**；姿态 `yaw/pitch/roll`：**始终弧度**——**不受** Adapter 的 `angle_unit` 配置影响，与「默认用度」的普通方法参数不同。
- 欧拉角顺序是 **`[x, y, z, yaw, pitch, roll]`**，即 yaw-pitch-roll（ZYX），与底层测试脚本一致。注意这与 `check_ik_accessibility` 期望的 `[x, y, z, roll, pitch, yaw]` 顺序不同。
- 内置方法：

```python
p = Pose6D(x=0.3, y=0.2, z=0.8, yaw=0.0, pitch=0.0, roll=0.0)
p.to_list()        # [0.3, 0.2, 0.8, 0.0, 0.0, 0.0]
p.to_quaternion()  # (qx, qy, qz, qw)

# 用「度」输入时显式声明
p2 = Pose6D.from_euler(0.3, 0.2, 0.8, yaw=0, pitch=-90, roll=0, degrees=True)
```

### `angle_unit` 与 `_to_rad()`

Adapter 创建时可通过 `config` 指定角度单位：

```python
hw = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled'})              # 默认：度
hw = HardwareFactory.create_hardware(config={'robot_type': 'leju_wheeled', 'angle_unit': 'rad'})
```

内部统一走 `_to_rad()` 转换（度→弧度 ×π/180，弧度→弧度不变）。**例外**：`send_arm_joint_trajectory`（发布到 `/kuavo_arm_traj`）、`publish_arm_joint_setpoint`（同样发布到 `/kuavo_arm_traj`）和 `send_leg_joint_command`（发布到 `/lb_leg_traj`）把角度直接下发，不做 `_to_rad()`，控制器期望度数，因此即使配了 `angle_unit='rad'`，传给这三个方法的仍应是**度数**。

### 枚举取值

| 枚举 | 取值 | 说明 |
|---|---|---|
| `FrameType` | `KEEP_CURRENT = 0`、`WORLD = 1`、`LOCAL = 2`、`JOINT_SPACE = 5` | 坐标系类型。`WORLD` 基于 odom，`LOCAL` 为 `base_link`，`JOINT_SPACE`=5 用于关节空间（不做 IK） |
| `ArmSide` | `LEFT = "left"`、`RIGHT = "right"`、`BOTH = "both"` | 手臂侧别 |
| `MPCControlMode` | `NO_CONTROL = 0`、`ARM_ONLY = 1`、`BASE_ONLY = 2`、`BASE_ARM = 3`、`ARM_EE_ONLY = 4` | 当前固件接收并回显该值，但不按其选择实际控制分支；`get_mpc_control_mode()` 读到值也不代表分支已切换 |
| `EndEffectorType` | `LEJU_CLAW = "leju_claw"`、`QIANGNAO_HAND = "qiangnao"`、`SG100_HAND = "sg100"`、`SUCTION_CUP = "suction_cup"`、`NONE = "none"` | 末端执行器类型 |
| `GripperStatus` | （见 `core.domain.end_effector`） | 夹爪状态枚举 |
| `SG100ControlMode` | `JOINT_POSITION` 等 | SG100 灵巧手控制模式，默认 `JOINT_POSITION`，`enable_mask=0x07FF` 表示全部 11 关节 |

> 以上描述针对仓库记录的固件行为；更换运控版本时应重新验证。`set_mpc_mode()` 调用并非空操作，但模式值不应作为实际控制分支的开关。三条控制路径见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)。

### `SkillParams` 的 `timeout`

`SkillParams.timeout` 默认 `30.0` 秒，`SkillBase.execute()` **每次**调用都会检查 `now - start > timeout`，超时返回 `Result.fail("Timeout")` 并打 ERROR 日志。因此对「等待」类技能要显式放大超时，例如 `WaitSeconds` 场景里 `duration_sec=86400` 必须同时给 `timeout=max(dur+60, 120)`，否则必然超时退化成 FAILURE。

## 02.04 返回值

- **`Result`**：绝大多数动作类方法返回它。判定只看 `.success`；`.message` 用于日志，`.error_code` 用于分支处理，`.data` 携带查询结果。
- **`Optional[...]`**：查询类方法在「还没数据」时返回 `None` 而不是 `fail`。典型：`get_ee_poses() -> Optional[Dict]`、`get_camera_frame() -> Optional[CameraFrame]`、`get_mpc_observation() -> Optional[Dict]`。**调用前必须判空**：

```python
poses = hw.get_ee_poses()
if poses is None:
    logger.warning("尚无末端位姿反馈")
else:
    left = poses.get("left")
```

- **`bool`**：`is_connected`、`is_finished`、`IPerception.initialize`、`ICamera.start_camera`。
- **`None`**：`IPerception.shutdown()`。

`Result.ok()` / `Result.fail()` 的结构与示例见 `01-阅读说明.md`。

## 02.05 异常

| 异常 | 触发条件 | 典型位置 |
|---|---|---|
| `KuavoFrameworkError` | 框架基础异常（父类） | `core.common.exceptions` |
| `HardwareConnectionError` | 硬件连接失败或断开 | Adapter 生命周期 |
| `SkillExecutionError` | 技能执行中逻辑错误或超时 | 技能层 |
| `ConfigurationError` | 配置缺失或格式错误 | `HardwareFactory.create_hardware` |
| `KinematicsError` | 逆解等运动学解算失败 | IK / 规划相关 |
| `ValueError` | 领域模型输入非法，如 `TransformMatrix` 非 4×4 | `core.domain.pose` |

## 02.06 调用示例

```python
from core.domain.result import Result
from core.domain.pose import Pose6D
from core.domain.enums import ArmSide, FrameType
from core.domain.end_effector import GripperCommand

# 1) 造一个位姿（姿态用弧度）
left_pose = Pose6D(x=0.3, y=0.2, z=0.8, yaw=0.0, pitch=0.0, roll=0.0)

# 2) 单臂末端位姿（标准接口，自动阻塞等待完成）
result = hw.send_ee_pose(side=ArmSide.LEFT, pose=left_pose, frame=FrameType.WORLD)
if not result.success:
    print(f"失败: {result.message}")

# 3) 夹爪闭合（position=100 闭合，effort=1.0A）
result = hw.control_end_effector(
    side=ArmSide.LEFT,
    cmd=GripperCommand(position=100.0, velocity=50.0, effort=1.0),
)

# 4) 查询类：先判空
poses = hw.get_ee_poses()
if poses is None:
    print("尚无末端反馈")
```

下一步：硬件实现与完整 API 清单见 [`03-adapters-硬件适配层.md`](./03-adapters-硬件适配层.md)；模式值与实际控制路径的区别见 [`09-控制路径与选型.md`](./09-控制路径与选型.md)。

## 躯干单帧控制补充

新增 `set_torso_joint(joint)` / `SetTorsoJoint` 和 `set_torso_pose(pose)` / `SetTorsoPose`：

| 参数 | 必填格式 | 单位 |
|---|---|---|
| `joint` | `[knee,leg,waist_pitch,waist_yaw]`，4维 | deg |
| `pose` | `[x,z,yaw,pitch]`，4维 | m/deg |

IHardware提供默认不支持的可选方法，其他Adapter无需新增抽象实现。轮式实现复用TorsoMixin和已有LowLevelSDKManager（需初始化low）；关节原样以deg交SDK，位姿填充y=0、roll=0并将yaw/pitch转rad，不受angle_unit影响。

Skill仅单次下发并缓存完整Result；SDK返回失败或异常时不得报告成功。节点支持运行期黑板输入与dry-run校验，无默认物理目标；不切模式/focus、不插值、不等待或检查到位。位姿参考语义沿用控制器配置，无frame参数/TF变换，y和roll由Adapter固定为0；单帧发布不保证运动完成。旧控制链和现有场景不变，目前没有场景迁移至这两个节点。

## 躯干 Timed 控制补充

| IHardware方法 | Skill / Node | 必填目标 | planner |
|---|---|---|---|
| `timed_torso_joint(joint, desire_time, is_sync=True)` | `TimedTorsoJointSkill` / `TimedTorsoJoint` | `[knee,leg,waist_pitch,waist_yaw]`，4维deg | 3 |
| `timed_torso_pose(pose, desire_time, is_sync=True)` | `TimedTorsoPoseSkill` / `TimedTorsoPose` | `[x,z,yaw,pitch]`，4维m/deg | 2 |

- 均由 `TimedCmdMixin` → `TimedRosManager.send_multi_commands()` 发一次MultiCmd请求（即便只有一个planner），不经旧的全局angle_unit换算路径。关节4维全部转rad，位姿只转换yaw/pitch；pose不暴露y/roll、不做TF变换。
- `desire_time` 必填大于0有限秒数，`is_sync` 必须为bool，仅表示规划时长同步；目标默认空。Adapter不切模式/focus，不等待，不使用LowLevelSDKManager发送运动目标。
- Skill与Timed手臂保持一致：`prepare_arm_control=True` 默认先通过现有模式接口请求模式2（需要low Manager），失败不发目标；False跳过准备。切2可能激活控制器残留手臂目标，操作者须先确认安全。不自动清空目标、改focus/MPC或恢复模式。
- Skill缓存完整Result及异常失败，不重复发令、不sleep、不检查到位；服务返回 `actual_time` 是规划时长，不是实际反馈或停止证明。
- Node支持运行期READ_BOARD与dry-run参数校验，可选 `actual_time_output_key` 将规划时长写黑板供独立WaitSeconds使用。初始化清空旧输出，只有缺字段时回退desire_time，显式null/bool/负数/非有限值失败；节点自身不等待。dry-run不获取硬件，输出请求时长仅供编排干跑，不伪造实测反馈。
- 人工脚本按Adapter/Skill/Node三层放在 `apps/test_internal/` 对应子目录，公共配置与结果校验统一为 `_timed_test.py`，说明见 `apps/test_internal/_timed_test.md`；两个手臂Timed入口也使用该公共模块。旧SDK/Timed接口和场景不删除、不迁移。

## 躯干反馈与SDK关节轨迹

- `get_torso_joint()`：经 `TorsoMixin → LowLevelSDKManager` 读取SDK `joint_state.position[:4]`（rad），Adapter转换为固定deg的 `[knee,leg,waist_pitch,waist_yaw]`。不受angle_unit影响。缺失、短数组或非法值失败，不补零；缓存不保证首帧/新鲜度，不用于证明到位。
- `set_torso_joint_traj(joint_traj, total_time)`：每帧4维deg，必须是嵌套列表，至少一个关键帧；total_time必填有限数且至少0.01秒（SDK 100Hz采样间隔，非物理限值）。没有默认目标或时长。
- 链路为 `TorsoMixin → LowLevelSDKManager → TorsoAPI.move_wheel_lower_joint(asynchronous=False)`。Manager先校验参数/反馈，再调用SDK。SDK会再次读取状态、自动插入当前起点、请求MPC ArmOnly、均匀分配关键帧并100Hz重采样发送。不额外切手臂模式或focus，也不自动恢复MPC。前置反馈检查不提供原子快照或新鲜度保证。
- SDK同步返回None，Result.success仅表示调用未抛异常，data=None。SDK内部不汇总模式切换和逐帧发送的bool，因此不能保证逐帧接收或实测到位。不提供actual_time；阻塞当前tick，Skill超时/cancel无法中断同步SDK循环，不适合要求可取消的Parallel。
- Skill缓存完整成功或失败结果，重新初始化后才再次读取/执行。`GetTorsoJoint` 的output_key默认torso_joint，初始化先清空旧输出，失败或dry-run保持None。`SetTorsoJointTraj` 支持运行期黑板轨迹/时长与dry-run校验；干跑不访问硬件。
- TorsoAPI使用原有low Manager的RobotSDK，惰性创建；shutdown释放其线程池，不与同步执行并行调用。新接口为IHardware可选方法，旧控制链和场景均保留。
- 人工入口在 `apps/test_internal/{adapters,skills,nodes}/get_torso_joint.py` 与 `set_torso_joint_traj.py`；只启用low Manager，具体见 `apps/test_internal/_torso_test.md`。自动化回归均使用Mock，不执行真机入口。

## 躯干当前姿态、初始姿态与复位

| 接口 | 后端 | 成功data |
|---|---|---|
| `get_torso_pose()` | TorsoMixin → LowLevelSDKManager → SDK TF查询 | `{pose:[x,z,yaw,pitch],frame:'base_link',link:'waist_yaw_link',source:'tf'}` |
| `get_torso_pose_initial()` | TorsoMixin → TorsoRosManager → 初始姿态服务 | `{pose:[x,z,yaw,pitch],frame:None,source:'initial_service'}` |
| `reset_torso()` | TorsoMixin → TorsoRosManager → 复位服务 | `{accepted:True,estimated_time:float或None}` |

两种pose公开单位m/deg。当前位姿只查waist_yaw_link相对base_link，不用目标或初始姿态作fallback，不保证TF缓存新鲜度；忽略反馈y/roll只返回四分量，并非将原始TF强制置零。初始服务响应无frame字段，保留来源信息，不伪称做过base_link变换或读取当前实测；服务angular按ROS角度rad转换为公共deg。

复位通过 `/mobile_manipulator_reset_torso` 的SetBool(True)提交，初始查询通过 `/mobile_manipulator_get_torso_initial_pose` 的getLbTorsoInitialPose(isNeed=True)。TorsoRosManager不依赖SDK、不自行init_node，服务发现5秒不代表整调用硬超时，无自动重试。复位成功只代表接受；服务message可解析为非负有限时间时填estimated_time，否则None，不将已接受误报为失败。

两个查询Node的output_key默认torso_pose/torso_pose_initial，初始化清空、失败/dry-run保持None；Skill单次缓存Result，重复tick不重复查询/复位。ResetTorso不切额外模式，不等待或检查到位。旧命名get_torso_initial_pose/reset_torso_to_initial及旧技能均保留。

配置：get_torso_pose需SDK low；初始查询/复位仅需 `ros_managers_whitelist=["torso"]` 且SDK白名单可为空。ROS管理器默认启用timed+torso+mpc，空白名单/skip_ros_managers可禁用。三层人工脚本在apps/test_internal对应目录，reset_torso.py默认CONFIRM_RESET=False，在初始化硬件前拦截，必须人工确认现场安全；关闭硬件不等于撤销已接受的复位。

## 躯干Ruckig持久规划限制

`timed_torso_ruckig_params(mode, velocity_max, acceleration_max, jerk_max, velocity_min=None, acceleration_min=None, is_sync=True)` 经 `TorsoMixin → TimedRosManager` 调用 `/mobile_manipulator_set_ruckig_planner_params`，不初始化SDK。

- mode必填：pose对应planner2，四维[x,z,yaw,pitch]；joint对应planner3，四维[knee,leg,waist_pitch,waist_yaw]。不接受其他planner，不按Timed手臂编号套用。
- 三个max均必填4维正有限数，两个min若显式填写必须为4维负有限数；省略时明确取对应-max，允许不对称幅值。拒绝bool、NaN/inf、零、错维度，不提供默认物理限制。
- 公开速度/加速度/jerk单位依次为m/s、m/s²、m/s³或deg/s、deg/s²、deg/s³；pose前两维为线量、后两维角量，joint全为角量。Adapter将所有角量导数统一转rad制，包括下界，不受angle_unit影响。
- is_sync为规划同步配置，不是Python等待运动完成。服务成功消息透传，data=None；失败/异常返回失败，不自动重试。服务发现5秒不代表整个调用硬超时。
- 参数会持续影响后续规划，不仅当前动作；这是规划限制，不是硬件安全限值。不发送目标、不切模式/focus、不自动恢复。Skill每次初始化仅设置一次并缓存成功或失败；Node支持运行期黑板和dry-run校验，不增加输出键。
- 三层人工入口：`apps/test_internal/{adapters,skills,nodes}/timed_torso_ruckig_params.py`。MODE默认空，三个max默认空，先校验再初始化硬件；仅启用ROS timed，SDK白名单为空。操作者确认物理限制后手动执行，不自动配置真机。旧Ruckig接口不改动。

## 独立MPC ROS接口

六接口通过MpcMixin → MpcRosManager，不依赖SDK：

| 设置/查询 | 输入/返回字段 | 值与名称 |
|---|---|---|
| set/get_mpc_control_mode | mode | 0 NoControl、1 ArmOnly、2 BaseOnly、3 BaseArm |
| set/get_mpc_control_flow | flow | 0 ThroughFullBodyMpc、1 DirectToWbc |
| set/get_mpc_frame | frame | 0 KeepCurrentFrame、1 WorldFrame、2 LocalFrame、3 VRFrame、4 ManipulationWorldFrame |

setter只接受严格整数（不接受bool/浮点/字符串，Node JSON解析后再校验），不接受旧ARM_EE_ONLY=4作为mode。旧Core枚举与旧set_mpc_mode保持不变。新setter成功data=None，仅表示服务接受，不额外查询、切手臂模式、改变其他项或恢复。frame=0表示保持当前，不要求回显为0。

getter每次Skill初始化只调用一次查询服务，成功data为对应字段与name的字典，如 `{mode:1,name:'ArmOnly'}`。服务拒绝/异常/非法返回值直接失败，不回退本地缓存。Node输出键默认mpc_control_mode/mpc_control_flow/mpc_frame，初始化清空，失败/dry-run保持None。重复execute/tick缓存完整Result，不重发；服务发现5秒不代表整个调用硬截止。

按用户确认，旧 `get_mpc_control_mode()->Optional[int]` 改名为 `get_mpc_control_mode_cached()`，保留旧缓存/ROS2查询行为并更新调试脚本。新的同名getter返回Result；ROS2目前明确返回不支持新ROS1服务契约，其他旧能力不变。外部未纳入仓库的旧调用者需自行改用cached名称。

ROS管理器默认启用timed、torso、mpc；可用ros_managers_whitelist=["mpc"]只启用本组，sdk_managers_whitelist=[]。不会自行init_node或初始化SDK。当前固件对模式的实际消费能力须现场确认，服务接受/回显不是安全隔离、停止或控制行为生效证明。SDK已有模式调用路径不被此次替换，旧场景保留。

人工入口在apps/test_internal的adapters/skills/nodes下，六个同名脚本。setter TARGET_MODE/TARGET_FLOW/TARGET_FRAME默认None，必须人工确认后填写；getter不发送设置请求；Node人工入口拒绝dry-run。公共配置与校验见apps/test_internal/_mpc_test.py。
